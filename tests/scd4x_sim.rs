#![deny(unexpected_cfgs)]
#![cfg(all(
    any(feature = "scd4x", feature = "scd41"),
    any(feature = "sync", feature = "async")
))]

//! Deterministic simulation of the SCD4x driver against a model of the sensor.
//!
//! The model follows the SCD4x datasheet (version D1): the measurement modes,
//! the execution time of each command, the commands allowed during a
//! measurement (table 9, column "During meas."), the data buffer, the NACK on a
//! read without data, sleep and `wake_up`. A seeded random sequence of driver
//! calls runs against the model, with random pauses and bus faults.
//!
//! The model panics when a command on the bus breaks the datasheet. After each
//! call, the test compares the result with what the model did. The decoders in
//! this file use the datasheet formulas and do not share code with the driver.
//!
//! Reproduce a failure: `SCD4X_SIM_SEED=<seed> cargo test --all-features --test scd4x_sim`.

use core::cell::RefCell;
#[cfg(feature = "sync")]
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
#[cfg(not(feature = "sync"))]
use embedded_hal_async::i2c::{ErrorKind, NoAcknowledgeSource};
use libscd::error::Error;
use libscd::measurement::Measurement;
use libscd::SensorVariant;

const SEEDS: u64 = 200;
const STEPS: usize = 400;

/// Commands that the sensor accepts during a periodic measurement (table 9)
const ALLOWED_DURING_MEASUREMENT: [u16; 4] = [
    0xec05, // read_measurement
    0x3f86, // stop_periodic_measurement
    0xe000, // set_ambient_pressure, get_ambient_pressure
    0xe4b8, // get_data_ready_status
];

const WAKE_UP: u16 = 0x36f6;
/// `get_sensor_variant` word of an SCD41 (section 3.10.6)
const SCD41_VARIANT: u16 = 0x1440;

const NACK: ErrorKind = ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data);

/// xorshift64*: deterministic and without dependencies
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }

    fn one_in(&mut self, n: u64) -> bool {
        self.below(n) == 0
    }

    fn word(&mut self) -> u16 {
        self.next() as u16
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    Idle,
    Periodic,
    LowPower,
    Sleep,
}

impl Mode {
    fn measuring(self) -> bool {
        matches!(self, Mode::Periodic | Mode::LowPower)
    }
}

/// What the sensor sends on the next read
#[derive(Debug)]
enum Response {
    Words([u16; 3], usize),
    /// `read_measurement` with an empty buffer: the sensor does not acknowledge the read
    NoData,
}

/// What the sensor did during one driver call
#[derive(Default, Debug)]
struct Served {
    executed: Option<u16>,
    written: Option<u16>,
    words: Option<[u16; 3]>,
    nack: bool,
    corrupted: bool,
}

struct Settings {
    temperature_offset: u16,
    altitude: u16,
    pressure: u16,
    asc_enabled: u16,
    asc_target: u16,
    asc_initial_period: u16,
    asc_standard_period: u16,
}

impl Settings {
    /// Factory defaults (sections 3.7 and 3.8)
    fn factory() -> Self {
        Self {
            temperature_offset: 1498, // 4 °C
            altitude: 0,
            pressure: 1013,
            asc_enabled: 1,
            asc_target: 400,
            asc_initial_period: 44,
            asc_standard_period: 156,
        }
    }
}

struct Model {
    rng: Rng,
    faults: bool,
    now_us: u64,
    busy_until_us: u64,
    mode: Mode,
    next_sample_us: u64,
    sample: Option<[u16; 3]>,
    response: Option<Response>,
    /// The sensor measured since power-up; precondition of the FRC (section 3.8.1)
    measured: bool,
    settings: Settings,
    served: Served,
}

impl Model {
    fn new(seed: u64) -> Self {
        Self {
            rng: Rng::new(seed),
            faults: true,
            now_us: 0,
            busy_until_us: 0,
            mode: Mode::Idle,
            next_sample_us: 0,
            sample: None,
            response: None,
            measured: false,
            settings: Settings::factory(),
            served: Served::default(),
        }
    }

    fn random_sample(&mut self) -> [u16; 3] {
        [
            400 + self.rng.below(4600) as u16,
            self.rng.word(),
            self.rng.word(),
        ]
    }

    /// Periodic modes put a new sample into the buffer once per interval
    fn tick(&mut self) {
        let interval_us = match self.mode {
            Mode::Periodic => 5_000_000,
            Mode::LowPower => 30_000_000,
            Mode::Idle | Mode::Sleep => return,
        };
        while self.now_us >= self.next_sample_us {
            self.sample = Some(self.random_sample());
            self.next_sample_us += interval_us;
        }
    }

    fn start_measurement(&mut self, mode: Mode, first_sample_us: u64) {
        self.mode = mode;
        self.measured = true;
        self.next_sample_us = self.now_us + first_sample_us;
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), ErrorKind> {
        self.tick();
        let op = u16::from_be_bytes([bytes[0], bytes[1]]);
        let data = match bytes.len() {
            2 => None,
            5 => {
                assert_eq!(
                    crc8(&bytes[2..4]),
                    bytes[4],
                    "bad CRC in the data of 0x{op:04x}"
                );
                Some(u16::from_be_bytes([bytes[2], bytes[3]]))
            }
            n => panic!("write of {n} bytes"),
        };
        assert!(
            self.now_us >= self.busy_until_us,
            "0x{op:04x} sent {} us before the previous command finished",
            self.busy_until_us - self.now_us
        );
        if self.mode == Mode::Sleep && op != WAKE_UP {
            self.served.nack = true;
            return Err(NACK);
        }
        assert!(
            !self.mode.measuring() || ALLOWED_DURING_MEASUREMENT.contains(&op),
            "0x{op:04x} sent during a measurement (datasheet table 9)"
        );
        if op != WAKE_UP && self.faults && self.rng.one_in(50) {
            self.served.nack = true;
            return Err(NACK);
        }

        self.served.executed = Some(op);
        self.served.written = data;
        let (exec_ms, response) = self.execute(op, data);
        self.busy_until_us = self.now_us + exec_ms * 1000;
        self.response = response;

        if op == WAKE_UP {
            // "the SCD4x does not acknowledge the wake_up command" (section 3.11.4)
            self.served.nack = true;
            return Err(NACK);
        }
        Ok(())
    }

    /// Executes one command. Returns the execution time in ms (table 9)
    /// and the response for a read command.
    fn execute(&mut self, op: u16, data: Option<u16>) -> (u64, Option<Response>) {
        let one = |word: u16| Some(Response::Words([word, 0, 0], 1));
        let settings = &mut self.settings;
        match (op, data) {
            (0x21b1, None) => {
                self.start_measurement(Mode::Periodic, 5_000_000);
                (0, None)
            }
            (0x21ac, None) => {
                self.start_measurement(Mode::LowPower, 30_000_000);
                (0, None)
            }
            (0x3f86, None) => {
                self.mode = Mode::Idle;
                (500, None)
            }
            (0xe4b8, None) => {
                // The upper five bits carry no information (section 3.9.2)
                let low = if self.sample.is_some() {
                    1 + self.rng.below(0x7FF)
                } else {
                    0
                };
                let word = (self.rng.word() & 0xF800) | low as u16;
                (1, one(word))
            }
            (0xec05, None) => {
                let response = match self.sample.take() {
                    Some(sample) => Response::Words(sample, 3),
                    None => Response::NoData,
                };
                (1, Some(response))
            }
            (0x241d, Some(word)) => {
                settings.temperature_offset = word;
                (1, None)
            }
            (0x2318, None) => (1, one(settings.temperature_offset)),
            (0x2427, Some(word)) => {
                settings.altitude = word;
                (1, None)
            }
            (0x2322, None) => (1, one(settings.altitude)),
            (0xe000, Some(word)) => {
                settings.pressure = word;
                (1, None)
            }
            (0xe000, None) => (1, one(settings.pressure)),
            (0x2416, Some(word)) => {
                settings.asc_enabled = word;
                (1, None)
            }
            (0x2313, None) => (1, one(settings.asc_enabled)),
            (0x243a, Some(word)) => {
                settings.asc_target = word;
                (1, None)
            }
            (0x233f, None) => (1, one(settings.asc_target)),
            (0x362f, Some(_)) => {
                let word = if self.measured {
                    (0x8000 + self.rng.below(1000) as i32 - 500) as u16
                } else {
                    0xFFFF
                };
                (400, one(word))
            }
            (0x3615, None) => (800, None),
            (0x3682, None) => (1, Some(Response::Words([0xF896, 0x9F07, 0x3BBE], 3))),
            (0x3639, None) => {
                let word = if self.rng.one_in(10) {
                    1 + self.rng.below(0xFFFF) as u16
                } else {
                    0
                };
                (10_000, one(word))
            }
            (0x3632, None) => {
                self.settings = Settings::factory();
                (1_200, None)
            }
            (0x3646, None) => (30, None),
            (0x202f, None) => (1, one(SCD41_VARIANT)),
            (0x219d, None) => {
                self.sample = Some(self.random_sample());
                self.measured = true;
                (5_000, None)
            }
            (0x2196, None) => {
                let [_, temperature, humidity] = self.random_sample();
                self.sample = Some([0, temperature, humidity]);
                (50, None)
            }
            (0x36e0, None) => {
                self.mode = Mode::Sleep;
                (1, None)
            }
            (WAKE_UP, None) => {
                if self.mode == Mode::Sleep {
                    self.mode = Mode::Idle;
                }
                (30, None)
            }
            (0x2445 | 0x244e, Some(hours)) => {
                // "Allowed values are integer multiples of 4 hours" (sections 3.11.5, 3.11.7)
                assert_eq!(hours % 4, 0, "ASC period of {hours} h sent as 0x{op:04x}");
                if op == 0x2445 {
                    self.settings.asc_initial_period = hours;
                } else {
                    self.settings.asc_standard_period = hours;
                }
                (1, None)
            }
            (0x2340, None) => (1, one(self.settings.asc_initial_period)),
            (0x234b, None) => (1, one(self.settings.asc_standard_period)),
            (op, data) => panic!("unknown command 0x{op:04x} with data {data:?}"),
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), ErrorKind> {
        assert!(
            self.now_us >= self.busy_until_us,
            "read {} us before the command finished",
            self.busy_until_us - self.now_us
        );
        let (words, len) = match self.response.take().expect("read without a read command") {
            Response::Words(words, len) => (words, len),
            Response::NoData => {
                self.served.nack = true;
                return Err(NACK);
            }
        };
        assert_eq!(buf.len(), len * 3, "read length");
        for (chunk, word) in buf.chunks_mut(3).zip(words) {
            chunk[..2].copy_from_slice(&word.to_be_bytes());
            chunk[2] = crc8(&word.to_be_bytes());
        }
        if self.faults && self.rng.one_in(50) {
            let byte = self.rng.below(buf.len() as u64) as usize;
            buf[byte] ^= 1 << self.rng.below(8);
            self.served.corrupted = true;
        }
        self.served.words = Some(words);
        Ok(())
    }

    fn delay_ns(&mut self, ns: u32) {
        self.now_us += u64::from(ns).div_ceil(1000);
    }
}

/// CRC-8 from section 3.12 of the datasheet: polynomial 0x31, init 0xFF
fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0xFF, |crc, byte| {
        (0..8).fold(crc ^ byte, |crc, _| {
            if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            }
        })
    })
}

#[derive(Clone, Copy, Debug)]
enum Op {
    StartPeriodic,
    StartLowPower,
    Stop,
    DataReady,
    ReadMeasurement,
    SetTemperatureOffset(f32),
    GetTemperatureOffset,
    SetAltitude(u16),
    GetAltitude,
    SetPressure(u16),
    GetPressure,
    SetAsc(bool),
    GetAsc,
    SetAscTarget(u16),
    GetAscTarget,
    ForcedRecalibration(u16),
    PersistSettings,
    SerialNumber,
    SensorVariant,
    SelfTest,
    FactoryReset,
    Reinit,
    #[cfg(feature = "scd41")]
    SingleShot,
    #[cfg(feature = "scd41")]
    SingleShotRhtOnly,
    #[cfg(feature = "scd41")]
    PowerDown,
    #[cfg(feature = "scd41")]
    WakeUp,
    #[cfg(feature = "scd41")]
    SetAscInitialPeriod(u16),
    #[cfg(feature = "scd41")]
    GetAscInitialPeriod,
    #[cfg(feature = "scd41")]
    SetAscStandardPeriod(u16),
    #[cfg(feature = "scd41")]
    GetAscStandardPeriod,
}

impl Op {
    fn random(rng: &mut Rng) -> Self {
        let ops: &[fn(&mut Rng) -> Op] = &[
            |_| Op::StartPeriodic,
            |_| Op::StartLowPower,
            |_| Op::Stop,
            |_| Op::DataReady,
            |_| Op::DataReady,
            |_| Op::ReadMeasurement,
            |_| Op::ReadMeasurement,
            |r| Op::SetTemperatureOffset(r.below(2500) as f32 / 100.0 - 2.0),
            |_| Op::GetTemperatureOffset,
            |r| Op::SetAltitude(r.below(3500) as u16),
            |_| Op::GetAltitude,
            |r| Op::SetPressure(600 + r.below(700) as u16),
            |_| Op::GetPressure,
            |r| Op::SetAsc(r.one_in(2)),
            |_| Op::GetAsc,
            |r| Op::SetAscTarget(400 + r.below(100) as u16),
            |_| Op::GetAscTarget,
            |r| Op::ForcedRecalibration(400 + r.below(1600) as u16),
            |_| Op::PersistSettings,
            |_| Op::SerialNumber,
            |_| Op::SensorVariant,
            |_| Op::SelfTest,
            |_| Op::FactoryReset,
            |_| Op::Reinit,
            #[cfg(feature = "scd41")]
            |_| Op::SingleShot,
            #[cfg(feature = "scd41")]
            |_| Op::SingleShotRhtOnly,
            #[cfg(feature = "scd41")]
            |_| Op::PowerDown,
            #[cfg(feature = "scd41")]
            |_| Op::WakeUp,
            #[cfg(feature = "scd41")]
            |r| Op::SetAscInitialPeriod(r.below(200) as u16),
            #[cfg(feature = "scd41")]
            |_| Op::GetAscInitialPeriod,
            #[cfg(feature = "scd41")]
            |r| Op::SetAscStandardPeriod(r.below(400) as u16),
            #[cfg(feature = "scd41")]
            |_| Op::GetAscStandardPeriod,
        ];
        ops[rng.below(ops.len() as u64) as usize](rng)
    }

    /// Opcode from table 9 of the datasheet
    fn opcode(self) -> u16 {
        match self {
            Op::StartPeriodic => 0x21b1,
            Op::StartLowPower => 0x21ac,
            Op::Stop => 0x3f86,
            Op::DataReady => 0xe4b8,
            Op::ReadMeasurement => 0xec05,
            Op::SetTemperatureOffset(_) => 0x241d,
            Op::GetTemperatureOffset => 0x2318,
            Op::SetAltitude(_) => 0x2427,
            Op::GetAltitude => 0x2322,
            Op::SetPressure(_) | Op::GetPressure => 0xe000,
            Op::SetAsc(_) => 0x2416,
            Op::GetAsc => 0x2313,
            Op::SetAscTarget(_) => 0x243a,
            Op::GetAscTarget => 0x233f,
            Op::ForcedRecalibration(_) => 0x362f,
            Op::PersistSettings => 0x3615,
            Op::SerialNumber => 0x3682,
            Op::SensorVariant => 0x202f,
            Op::SelfTest => 0x3639,
            Op::FactoryReset => 0x3632,
            Op::Reinit => 0x3646,
            #[cfg(feature = "scd41")]
            Op::SingleShot => 0x219d,
            #[cfg(feature = "scd41")]
            Op::SingleShotRhtOnly => 0x2196,
            #[cfg(feature = "scd41")]
            Op::PowerDown => 0x36e0,
            #[cfg(feature = "scd41")]
            Op::WakeUp => WAKE_UP,
            #[cfg(feature = "scd41")]
            Op::SetAscInitialPeriod(_) => 0x2445,
            #[cfg(feature = "scd41")]
            Op::GetAscInitialPeriod => 0x2340,
            #[cfg(feature = "scd41")]
            Op::SetAscStandardPeriod(_) => 0x244e,
            #[cfg(feature = "scd41")]
            Op::GetAscStandardPeriod => 0x234b,
        }
    }

    /// The input is outside of the range that the datasheet allows
    fn invalid_input(self) -> bool {
        match self {
            // The offset word is unsigned (section 3.7.1)
            Op::SetTemperatureOffset(celsius) => celsius < 0.0,
            Op::SetAltitude(meters) => meters > 3_000,
            Op::SetPressure(hpa) => !(700..=1200).contains(&hpa),
            #[cfg(feature = "scd41")]
            Op::SetAscInitialPeriod(hours) | Op::SetAscStandardPeriod(hours) => hours % 4 != 0,
            _ => false,
        }
    }
}

type Res<T> = Result<T, Error<ErrorKind>>;

#[derive(Debug)]
enum Outcome {
    Unit(Res<()>),
    Bool(Res<bool>),
    Word(Res<u16>),
    Celsius(Res<f32>),
    Measurement(Res<Measurement>),
    Frc(Res<Option<i16>>),
    Serial(Res<u64>),
    Variant(Res<Option<SensorVariant>>),
}

impl Outcome {
    fn error(&self) -> Option<&Error<ErrorKind>> {
        match self {
            Outcome::Unit(r) => r.as_ref().err(),
            Outcome::Bool(r) => r.as_ref().err(),
            Outcome::Word(r) => r.as_ref().err(),
            Outcome::Celsius(r) => r.as_ref().err(),
            Outcome::Measurement(r) => r.as_ref().err(),
            Outcome::Frc(r) => r.as_ref().err(),
            Outcome::Serial(r) => r.as_ref().err(),
            Outcome::Variant(r) => r.as_ref().err(),
        }
    }
}

/// Calls the driver method for `$op`. `$call` wraps each call: the sync
/// driver needs nothing, the async driver needs a poll.
macro_rules! dispatch {
    ($sensor:ident, $op:expr, $call:ident) => {
        match $op {
            Op::StartPeriodic => Outcome::Unit($call!($sensor.start_periodic_measurement())),
            Op::StartLowPower => {
                Outcome::Unit($call!($sensor.start_low_power_periodic_measurement()))
            }
            Op::Stop => Outcome::Unit($call!($sensor.stop_periodic_measurement())),
            Op::DataReady => Outcome::Bool($call!($sensor.data_ready())),
            Op::ReadMeasurement => Outcome::Measurement($call!($sensor.read_measurement())),
            Op::SetTemperatureOffset(c) => Outcome::Unit($call!($sensor.set_temperature_offset(c))),
            Op::GetTemperatureOffset => Outcome::Celsius($call!($sensor.get_temperature_offset())),
            Op::SetAltitude(m) => Outcome::Unit($call!($sensor.set_sensor_altitude(m))),
            Op::GetAltitude => Outcome::Word($call!($sensor.get_sensor_altitude())),
            Op::SetPressure(p) => Outcome::Unit($call!($sensor.set_ambient_pressure(p))),
            Op::GetPressure => Outcome::Word($call!($sensor.get_ambient_pressure())),
            Op::SetAsc(on) => Outcome::Unit($call!($sensor.enable_automatic_self_calibration(on))),
            Op::GetAsc => Outcome::Bool($call!($sensor.get_automatic_self_calibration())),
            Op::SetAscTarget(ppm) => {
                Outcome::Unit($call!($sensor.set_automatic_self_calibration_target(ppm)))
            }
            Op::GetAscTarget => {
                Outcome::Word($call!($sensor.get_automatic_self_calibration_target()))
            }
            Op::ForcedRecalibration(ppm) => {
                Outcome::Frc($call!($sensor.perform_forced_recalibration(ppm)))
            }
            Op::PersistSettings => Outcome::Unit($call!($sensor.persists_settings())),
            Op::SerialNumber => Outcome::Serial($call!($sensor.serial_number())),
            Op::SensorVariant => Outcome::Variant($call!($sensor.sensor_variant())),
            Op::SelfTest => Outcome::Bool($call!($sensor.perform_self_test())),
            Op::FactoryReset => Outcome::Unit($call!($sensor.perform_factory_reset())),
            Op::Reinit => Outcome::Unit($call!($sensor.reinit())),
            #[cfg(feature = "scd41")]
            Op::SingleShot => Outcome::Unit($call!($sensor.measure_single_shot())),
            #[cfg(feature = "scd41")]
            Op::SingleShotRhtOnly => Outcome::Unit($call!($sensor.measure_single_shot_rht_only())),
            #[cfg(feature = "scd41")]
            Op::PowerDown => Outcome::Unit($call!($sensor.power_down())),
            #[cfg(feature = "scd41")]
            Op::WakeUp => Outcome::Unit($call!($sensor.wake_up())),
            #[cfg(feature = "scd41")]
            Op::SetAscInitialPeriod(h) => Outcome::Unit($call!(
                $sensor.set_automatic_self_calibration_initial_period(h)
            )),
            #[cfg(feature = "scd41")]
            Op::GetAscInitialPeriod => Outcome::Word($call!(
                $sensor.get_automatic_self_calibration_initial_period()
            )),
            #[cfg(feature = "scd41")]
            Op::SetAscStandardPeriod(h) => Outcome::Unit($call!(
                $sensor.set_automatic_self_calibration_standard_period(h)
            )),
            #[cfg(feature = "scd41")]
            Op::GetAscStandardPeriod => Outcome::Word($call!(
                $sensor.get_automatic_self_calibration_standard_period()
            )),
        }
    };
}

/// The sensor state before a call
struct Before {
    mode: Mode,
}

/// Compares the result of one driver call with what the model did
fn check(op: Op, before: Before, outcome: &Outcome, served: &Served) {
    let opcode = op.opcode();
    let forbidden = before.mode.measuring() && !ALLOWED_DURING_MEASUREMENT.contains(&opcode);
    #[cfg(feature = "scd41")]
    let wake_up = matches!(op, Op::WakeUp);
    #[cfg(not(feature = "scd41"))]
    let wake_up = false;

    match outcome.error() {
        Some(Error::NotAllowed) => {
            assert!(
                forbidden,
                "the driver blocked a legal command in {:?}",
                before.mode
            );
            assert_eq!(
                None, served.executed,
                "a blocked command reached the sensor"
            );
            return;
        }
        Some(Error::InvalidInput) => {
            assert!(op.invalid_input(), "the driver rejected a valid input");
            assert_eq!(None, served.executed, "a rejected input reached the sensor");
            return;
        }
        Some(Error::I2C(_)) => {
            assert!(served.nack, "an I2C error without a NACK from the sensor");
            assert!(
                !wake_up,
                "wake_up returned the expected NACK as an error (section 3.11.4)"
            );
            return;
        }
        Some(Error::CRC) => {
            assert!(served.corrupted, "a CRC error on clean data");
            return;
        }
        None => {}
    }

    assert!(
        !forbidden,
        "unreachable: the model panics on a forbidden command"
    );
    assert!(!op.invalid_input(), "the driver accepted an invalid input");
    assert!(!served.corrupted, "the driver returned corrupted data");
    assert!(!served.nack || wake_up, "the driver hid a NACK");
    assert_eq!(Some(opcode), served.executed, "wrong command on the bus");

    let word = || served.words.expect("a read command without a response")[0];
    match (op, outcome) {
        (Op::SetTemperatureOffset(celsius), _) => {
            let written = f32::from(served.written.unwrap()) * 175.0 / 65_535.0;
            assert!(
                (written - celsius).abs() < 0.003,
                "offset {celsius} written as {written}"
            );
        }
        (Op::SetAltitude(v) | Op::SetPressure(v) | Op::SetAscTarget(v), _) => {
            assert_eq!(Some(v), served.written);
        }
        #[cfg(feature = "scd41")]
        (Op::SetAscInitialPeriod(v) | Op::SetAscStandardPeriod(v), _) => {
            assert_eq!(Some(v), served.written);
        }
        (Op::SetAsc(on), _) => assert_eq!(Some(u16::from(on)), served.written),
        (Op::ForcedRecalibration(ppm), Outcome::Frc(Ok(frc))) => {
            assert_eq!(Some(ppm), served.written);
            // section 3.8.1: 0xffff means failure, otherwise the correction is word - 0x8000
            let expected = (word() != 0xFFFF).then(|| (i32::from(word()) - 0x8000) as i16);
            assert_eq!(expected, *frc);
        }
        (Op::DataReady, Outcome::Bool(Ok(ready))) => {
            assert_eq!(
                word() & 0x07FF != 0,
                *ready,
                "data ready word 0x{:04x}",
                word()
            );
        }
        (Op::GetAsc, Outcome::Bool(Ok(on))) => assert_eq!(word() != 0, *on),
        (Op::SelfTest, Outcome::Bool(Ok(passed))) => assert_eq!(word() == 0, *passed),
        (_, Outcome::Word(Ok(value))) => assert_eq!(word(), *value),
        (Op::GetTemperatureOffset, Outcome::Celsius(Ok(celsius))) => {
            let expected = f32::from(word()) * 175.0 / 65_535.0;
            assert!((expected - celsius).abs() < 0.001);
        }
        (Op::ReadMeasurement, Outcome::Measurement(Ok(m))) => {
            let [co2, t, rh] = served.words.unwrap();
            assert_eq!(co2, m.co2);
            assert!((-45.0 + 175.0 * f32::from(t) / 65_535.0 - m.temperature).abs() < 0.001);
            assert!((100.0 * f32::from(rh) / 65_535.0 - m.humidity).abs() < 0.001);
        }
        (Op::SerialNumber, Outcome::Serial(Ok(serial))) => {
            let [a, b, c] = served.words.unwrap().map(u64::from);
            assert_eq!((a << 32) | (b << 16) | c, *serial);
        }
        (Op::SensorVariant, Outcome::Variant(Ok(variant))) => {
            assert_eq!(SCD41_VARIANT, word());
            assert_eq!(Some(SensorVariant::Scd41), *variant);
        }
        (_, Outcome::Unit(Ok(()))) => {}
        (op, outcome) => panic!("{op:?} returned {outcome:?}"),
    }
}

/// Proves that the random runs reach every command and every error path
#[derive(Default)]
struct Coverage {
    ok: std::collections::BTreeMap<u16, usize>,
    not_allowed: usize,
    invalid_input: usize,
    nack: usize,
    crc: usize,
}

impl Coverage {
    fn record(&mut self, op: Op, outcome: &Outcome) {
        match outcome.error() {
            None => *self.ok.entry(op.opcode()).or_default() += 1,
            Some(Error::NotAllowed) => self.not_allowed += 1,
            Some(Error::InvalidInput) => self.invalid_input += 1,
            Some(Error::I2C(_)) => self.nack += 1,
            Some(Error::CRC) => self.crc += 1,
        }
    }

    fn assert_complete(&self) {
        if std::env::var("SCD4X_SIM_SEED").is_ok() {
            return;
        }
        let mut rng = Rng::new(0);
        for _ in 0..10_000 {
            let opcode = Op::random(&mut rng).opcode();
            assert!(
                self.ok.contains_key(&opcode),
                "0x{opcode:04x} never succeeded"
            );
        }
        assert!(self.not_allowed > 0, "no NotAllowed");
        assert!(self.invalid_input > 0, "no InvalidInput");
        assert!(self.nack > 0, "no NACK");
        assert!(self.crc > 0, "no CRC error");
    }
}

/// Prints the seed if the simulation panics
struct SeedReport(u64);

impl Drop for SeedReport {
    fn drop(&mut self) {
        if std::thread::panicking() {
            eprintln!("failing seed: SCD4X_SIM_SEED={}", self.0);
        }
    }
}

fn seeds() -> core::ops::Range<u64> {
    match std::env::var("SCD4X_SIM_SEED") {
        Ok(seed) => {
            let seed = seed.parse().expect("SCD4X_SIM_SEED must be a number");
            seed..seed + 1
        }
        Err(_) => 0..SEEDS,
    }
}

/// Moves the clock between calls, so that measurements become ready
fn pause(model: &RefCell<Model>, rng: &mut Rng) {
    let us = match rng.below(4) {
        0 => 0,
        1 => rng.below(100_000),
        _ => rng.below(40_000_000),
    };
    model.borrow_mut().now_us += us;
}

/// Runs one call: snapshot, call, compare
fn step(model: &RefCell<Model>, coverage: &mut Coverage, op: Op, call: impl FnOnce() -> Outcome) {
    let before = {
        let mut model = model.borrow_mut();
        model.tick();
        model.served = Served::default();
        Before { mode: model.mode }
    };
    let outcome = call();
    check(op, before, &outcome, &model.borrow().served);
    coverage.record(op, &outcome);
}

#[cfg(feature = "sync")]
mod sync {
    use super::*;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{ErrorType, I2c, Operation};
    use libscd::synchronous::scd4x::Scd4x;

    pub struct Bus<'a>(pub &'a RefCell<Model>);
    pub struct Clock<'a>(pub &'a RefCell<Model>);

    impl ErrorType for Bus<'_> {
        type Error = ErrorKind;
    }

    impl I2c for Bus<'_> {
        fn transaction(
            &mut self,
            _: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), ErrorKind> {
            let mut model = self.0.borrow_mut();
            for op in operations {
                match op {
                    Operation::Write(bytes) => model.write(bytes)?,
                    Operation::Read(buf) => model.read(buf)?,
                }
            }
            Ok(())
        }
    }

    impl DelayNs for Clock<'_> {
        fn delay_ns(&mut self, ns: u32) {
            self.0.borrow_mut().delay_ns(ns);
        }
    }

    macro_rules! direct {
        ($e:expr) => {
            $e
        };
    }

    #[test]
    pub fn scd4x_sync_follows_the_datasheet() {
        let mut coverage = Coverage::default();
        for seed in seeds() {
            let _report = SeedReport(seed);
            let model = RefCell::new(Model::new(seed));
            let mut rng = Rng::new(!seed);
            let mut sensor = Scd4x::new(Bus(&model), Clock(&model));
            for _ in 0..STEPS {
                pause(&model, &mut rng);
                let op = Op::random(&mut rng);
                step(&model, &mut coverage, op, || dispatch!(sensor, op, direct));
            }
        }
        coverage.assert_complete();
    }
}

#[cfg(feature = "async")]
mod asynchronous {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::{ErrorType, I2c, Operation};
    use libscd::asynchronous::scd4x::Scd4x;

    pub struct Bus<'a>(pub &'a RefCell<Model>);
    pub struct Clock<'a>(pub &'a RefCell<Model>);

    impl ErrorType for Bus<'_> {
        type Error = ErrorKind;
    }

    impl I2c for Bus<'_> {
        async fn transaction(
            &mut self,
            _: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), ErrorKind> {
            let mut model = self.0.borrow_mut();
            for op in operations {
                match op {
                    Operation::Write(bytes) => model.write(bytes)?,
                    Operation::Read(buf) => model.read(buf)?,
                }
            }
            Ok(())
        }
    }

    impl DelayNs for Clock<'_> {
        async fn delay_ns(&mut self, ns: u32) {
            self.0.borrow_mut().delay_ns(ns);
        }
    }

    /// The bus and the clock above never return `Pending`, so one poll
    /// completes any driver future
    fn ready<F: Future>(future: F) -> F::Output {
        match pin!(future).poll(&mut Context::from_waker(Waker::noop())) {
            Poll::Ready(output) => output,
            Poll::Pending => unreachable!("the mocks never return Pending"),
        }
    }

    macro_rules! polled {
        ($e:expr) => {
            ready($e)
        };
    }

    #[test]
    pub fn scd4x_async_follows_the_datasheet() {
        let mut coverage = Coverage::default();
        for seed in seeds() {
            let _report = SeedReport(seed);
            let model = RefCell::new(Model::new(seed));
            let mut rng = Rng::new(!seed);
            let mut sensor = Scd4x::new(Bus(&model), Clock(&model));
            for _ in 0..STEPS {
                pause(&model, &mut rng);
                let op = Op::random(&mut rng);
                step(&model, &mut coverage, op, || dispatch!(sensor, op, polled));
            }
        }
        coverage.assert_complete();
    }
}
