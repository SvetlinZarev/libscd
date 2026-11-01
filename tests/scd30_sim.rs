#![deny(unexpected_cfgs)]
#![cfg(all(feature = "scd30", any(feature = "sync", feature = "async")))]

//! Deterministic simulation of the SCD30 driver against a model of the sensor.
//!
//! The model follows the SCD30 interface description (version 1.0, May 2020):
//! the command set of section 1.4 with its argument ranges, the CRC of the
//! arguments (section 1.1.3), the > 3 ms pause between a command and its read
//! header (sections 1.4.4 and 1.4.5), the boot time of < 2 s after a soft
//! reset (section 1.1), the settings kept in non-volatile memory and the FRC
//! reference that falls back to 400 ppm after a restart. A seeded random
//! sequence of driver calls runs against the model, with random pauses and
//! bus faults.
//!
//! The model panics when a command on the bus breaks the interface
//! description. After each call, the test compares the result with what the
//! model did. The decoders in this file follow section 1.5 and do not share
//! code with the driver.
//!
//! Reproduce a failure: `SCD30_SIM_SEED=<seed> cargo test --all-features --test scd30_sim`.

use core::cell::RefCell;
#[cfg(feature = "sync")]
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
#[cfg(not(feature = "sync"))]
use embedded_hal_async::i2c::{ErrorKind, NoAcknowledgeSource};
use libscd::error::Error;
use libscd::measurement::Measurement;

const SEEDS: u64 = 200;
const STEPS: usize = 400;

/// Section 1.1: "The boot-up time is < 2 s"
const BOOT_US: u64 = 2_000_000;
/// Sections 1.4.4 and 1.4.5: "the read header should be send with a delay
/// of > 3ms following the write sequence". The text names only these two
/// reads; the model applies the pause to every read, as the driver does.
const READ_PAUSE_US: u64 = 3_000;
/// Section 1.4.6, FRC: the reference after a restart
const FRC_DEFAULT_PPM: u16 = 400;
/// Section 1.4.9, example: firmware 3.66
const FIRMWARE: u16 = 0x0342;

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

/// What the sensor did during one driver call
#[derive(Default, Debug)]
struct Served {
    executed: Option<u16>,
    written: Option<u16>,
    words: Option<Vec<u16>>,
    nack: bool,
    corrupted: bool,
}

/// Settings in non-volatile memory: they survive a soft reset
struct Nvm {
    continuous: bool,
    interval_s: u16,
    asc: u16,
    temperature_offset: u16,
    altitude: u16,
}

impl Nvm {
    /// Defaults: interval 2 s (section 1.4.3), ASC off (section 1.4.6)
    fn factory() -> Self {
        Self {
            continuous: false,
            interval_s: 2,
            asc: 0,
            temperature_offset: 0,
            altitude: 0,
        }
    }
}

struct Model {
    rng: Rng,
    faults: bool,
    now_us: u64,
    /// The sensor does not answer before the end of its boot
    booted_at_us: u64,
    /// The last command, for the pause before its read header
    command_at_us: u64,
    next_sample_us: u64,
    /// CO2, T, RH as IEEE 754 words, big-endian (section 1.5)
    sample: Option<[u32; 3]>,
    response: Option<Vec<u16>>,
    nvm: Nvm,
    /// Volatile: back to 400 ppm after a restart (section 1.4.6, FRC)
    frc_ppm: u16,
    served: Served,
}

impl Model {
    fn new(seed: u64) -> Self {
        Self {
            rng: Rng::new(seed),
            faults: true,
            now_us: 0,
            booted_at_us: 0,
            command_at_us: 0,
            next_sample_us: 0,
            sample: None,
            response: None,
            nvm: Nvm::factory(),
            frc_ppm: FRC_DEFAULT_PPM,
            served: Served::default(),
        }
    }

    fn interval_us(&self) -> u64 {
        u64::from(self.nvm.interval_s) * 1_000_000
    }

    /// Ranges of table 4: CO2 0..10000 ppm, T -40..125 °C, RH 0..100 %
    fn random_sample(&mut self) -> [u32; 3] {
        let mut value = |low: f32, high: f32| {
            (low + (high - low) * self.rng.below(1 << 24) as f32 / (1 << 24) as f32).to_bits()
        };
        [value(0.0, 10_000.0), value(-40.0, 125.0), value(0.0, 100.0)]
    }

    /// In continuous mode a new sample replaces the unread one once per
    /// interval (section 1.4.1)
    fn tick(&mut self) {
        if !self.nvm.continuous {
            return;
        }
        while self.now_us >= self.next_sample_us {
            self.sample = Some(self.random_sample());
            self.next_sample_us += self.interval_us();
        }
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
                    "bad CRC in the data of 0x{op:04x} (section 1.1.2)"
                );
                Some(u16::from_be_bytes([bytes[2], bytes[3]]))
            }
            n => panic!("write of {n} bytes"),
        };
        assert!(
            self.now_us >= self.booted_at_us,
            "0x{op:04x} sent {} us before the end of the boot (section 1.1)",
            self.booted_at_us - self.now_us
        );
        if self.faults && self.rng.one_in(50) {
            self.served.nack = true;
            return Err(NACK);
        }

        self.served.executed = Some(op);
        self.served.written = data;
        self.command_at_us = self.now_us;
        self.response = self.execute(op, data);
        Ok(())
    }

    /// Executes one command of section 1.4. Returns the response of a read command.
    fn execute(&mut self, op: u16, data: Option<u16>) -> Option<Vec<u16>> {
        let nvm = &mut self.nvm;
        match (op, data) {
            (0x0010, Some(hpa)) => {
                assert!(
                    hpa == 0 || (700..=1400).contains(&hpa),
                    "ambient pressure {hpa} mbar (section 1.4.1)"
                );
                // Setting the pressure while the measurement runs does not restart it
                if !nvm.continuous {
                    nvm.continuous = true;
                    self.next_sample_us = self.now_us + self.interval_us();
                }
                None
            }
            (0x0104, None) => {
                nvm.continuous = false;
                None
            }
            (0x4600, Some(seconds)) => {
                assert!(
                    (2..=1800).contains(&seconds),
                    "measurement interval {seconds} s (section 1.4.3)"
                );
                nvm.interval_s = seconds;
                None
            }
            (0x4600, None) => Some(vec![nvm.interval_s]),
            (0x0202, None) => Some(vec![u16::from(self.sample.is_some())]),
            (0x0300, None) => {
                let sample = self
                    .sample
                    .take()
                    .expect("read measurement without data (section 1.4.5)");
                Some(
                    sample
                        .iter()
                        .flat_map(|word| [(word >> 16) as u16, *word as u16])
                        .collect(),
                )
            }
            (0x5306, Some(on)) => {
                assert!(on <= 1, "ASC argument {on} (section 1.4.6)");
                nvm.asc = on;
                None
            }
            (0x5306, None) => Some(vec![nvm.asc]),
            (0x5204, Some(ppm)) => {
                assert!(
                    (400..=2000).contains(&ppm),
                    "FRC reference {ppm} ppm (section 1.4.6)"
                );
                self.frc_ppm = ppm;
                None
            }
            (0x5204, None) => Some(vec![self.frc_ppm]),
            (0x5403, Some(offset)) => {
                nvm.temperature_offset = offset;
                None
            }
            (0x5403, None) => Some(vec![nvm.temperature_offset]),
            (0x5102, Some(meters)) => {
                nvm.altitude = meters;
                None
            }
            (0x5102, None) => Some(vec![nvm.altitude]),
            (0xD100, None) => Some(vec![FIRMWARE]),
            (0xD304, None) => {
                // "the same state as after powering up" (section 1.4.10): the
                // non-volatile settings stay, the measurement resumes if it ran
                self.booted_at_us = self.now_us + BOOT_US;
                self.frc_ppm = FRC_DEFAULT_PPM;
                self.sample = None;
                self.next_sample_us = self.booted_at_us + self.interval_us();
                None
            }
            (op, data) => panic!("unknown command 0x{op:04x} with data {data:?}"),
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), ErrorKind> {
        assert!(
            self.now_us >= self.booted_at_us,
            "read {} us before the end of the boot (section 1.1)",
            self.booted_at_us - self.now_us
        );
        let words = self.response.take().expect("read without a read command");
        assert!(
            self.now_us - self.command_at_us > READ_PAUSE_US,
            "read header {} us after the command (sections 1.4.4, 1.4.5)",
            self.now_us - self.command_at_us
        );
        if self.faults && self.rng.one_in(50) {
            self.served.nack = true;
            return Err(NACK);
        }
        assert_eq!(buf.len(), words.len() * 3, "read length");
        for (chunk, word) in buf.chunks_mut(3).zip(&words) {
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

/// CRC-8 from section 1.1.3: polynomial 0x31, init 0xFF
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
    Start(u16),
    Stop,
    SetInterval(u16),
    GetInterval,
    DataReady,
    ReadMeasurement,
    SetAsc(bool),
    GetAsc,
    SetFrc(u16),
    SetTemperatureOffset(u16),
    GetTemperatureOffset,
    SetAltitude(u16),
    GetAltitude,
    FirmwareVersion,
    SoftReset,
}

impl Op {
    fn random(rng: &mut Rng) -> Self {
        let ops: &[fn(&mut Rng) -> Op] = &[
            |r| {
                Op::Start(match r.below(4) {
                    0 => 0,
                    1 => r.word(),
                    _ => 650 + r.below(800) as u16,
                })
            },
            |_| Op::Stop,
            |r| {
                // Mostly short intervals, so that samples become ready
                Op::SetInterval(match r.below(4) {
                    0 => r.below(2000) as u16,
                    _ => r.below(12) as u16,
                })
            },
            |_| Op::GetInterval,
            |_| Op::DataReady,
            |_| Op::DataReady,
            |_| Op::ReadMeasurement,
            |_| Op::ReadMeasurement,
            |r| Op::SetAsc(r.one_in(2)),
            |_| Op::GetAsc,
            |r| Op::SetFrc(300 + r.below(1800) as u16),
            |r| Op::SetTemperatureOffset(r.word()),
            |_| Op::GetTemperatureOffset,
            |r| Op::SetAltitude(r.word()),
            |_| Op::GetAltitude,
            |_| Op::FirmwareVersion,
            |_| Op::SoftReset,
        ];
        ops[rng.below(ops.len() as u64) as usize](rng)
    }

    /// Command from section 1.4
    fn opcode(self) -> u16 {
        match self {
            Op::Start(_) => 0x0010,
            Op::Stop => 0x0104,
            Op::SetInterval(_) | Op::GetInterval => 0x4600,
            Op::DataReady => 0x0202,
            Op::ReadMeasurement => 0x0300,
            Op::SetAsc(_) | Op::GetAsc => 0x5306,
            Op::SetFrc(_) => 0x5204,
            Op::SetTemperatureOffset(_) | Op::GetTemperatureOffset => 0x5403,
            Op::SetAltitude(_) | Op::GetAltitude => 0x5102,
            Op::FirmwareVersion => 0xD100,
            Op::SoftReset => 0xD304,
        }
    }

    /// The input is outside of the range that the interface description allows
    fn invalid_input(self) -> bool {
        match self {
            Op::Start(hpa) => hpa != 0 && !(700..=1400).contains(&hpa),
            Op::SetInterval(seconds) => !(2..=1800).contains(&seconds),
            Op::SetFrc(ppm) => !(400..=2000).contains(&ppm),
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
    Measurement(Res<Measurement>),
    Firmware(Res<(u8, u8)>),
}

impl Outcome {
    fn error(&self) -> Option<&Error<ErrorKind>> {
        match self {
            Outcome::Unit(r) => r.as_ref().err(),
            Outcome::Bool(r) => r.as_ref().err(),
            Outcome::Word(r) => r.as_ref().err(),
            Outcome::Measurement(r) => r.as_ref().err(),
            Outcome::Firmware(r) => r.as_ref().err(),
        }
    }
}

/// Calls the driver method for `$op`. `$call` wraps each call: the sync
/// driver needs nothing, the async driver needs a poll.
macro_rules! dispatch {
    ($sensor:ident, $op:expr, $call:ident) => {
        match $op {
            Op::Start(hpa) => Outcome::Unit($call!($sensor.start_continuous_measurement(hpa))),
            Op::Stop => Outcome::Unit($call!($sensor.stop_continuous_measurement())),
            Op::SetInterval(s) => Outcome::Unit($call!($sensor.set_measurement_interval(s))),
            Op::GetInterval => Outcome::Word($call!($sensor.get_measurement_interval())),
            Op::DataReady => Outcome::Bool($call!($sensor.data_ready())),
            Op::ReadMeasurement => Outcome::Measurement($call!($sensor.read_measurement())),
            Op::SetAsc(on) => Outcome::Unit($call!($sensor.enable_automatic_self_calibration(on))),
            Op::GetAsc => Outcome::Bool($call!($sensor.get_automatic_self_calibration())),
            Op::SetFrc(ppm) => Outcome::Unit($call!($sensor.set_forced_recalibration_value(ppm))),
            Op::SetTemperatureOffset(t) => Outcome::Unit($call!($sensor.set_temperature_offset(t))),
            Op::GetTemperatureOffset => Outcome::Word($call!($sensor.get_temperature_offset())),
            Op::SetAltitude(m) => Outcome::Unit($call!($sensor.set_altitude_compensation(m))),
            Op::GetAltitude => Outcome::Word($call!($sensor.get_altitude_compensation())),
            Op::FirmwareVersion => Outcome::Firmware($call!($sensor.read_firmware_version())),
            Op::SoftReset => Outcome::Unit($call!($sensor.soft_reset())),
        }
    };
}

/// Compares the result of one driver call with what the model did
fn check(op: Op, outcome: &Outcome, served: &Served) {
    match outcome.error() {
        Some(Error::NotAllowed) => panic!("the SCD30 has no forbidden commands"),
        Some(Error::InvalidInput) => {
            assert!(op.invalid_input(), "the driver rejected a valid input");
            assert_eq!(None, served.executed, "a rejected input reached the sensor");
            return;
        }
        Some(Error::I2C(_)) => {
            assert!(served.nack, "an I2C error without a NACK from the sensor");
            return;
        }
        Some(Error::CRC) => {
            assert!(served.corrupted, "a CRC error on clean data");
            return;
        }
        None => {}
    }

    assert!(!op.invalid_input(), "the driver accepted an invalid input");
    assert!(!served.corrupted, "the driver returned corrupted data");
    assert!(!served.nack, "the driver hid a NACK");
    assert_eq!(
        Some(op.opcode()),
        served.executed,
        "wrong command on the bus"
    );

    let words = || {
        served
            .words
            .as_ref()
            .expect("a read command without a response")
    };
    match (op, outcome) {
        (
            Op::Start(v)
            | Op::SetInterval(v)
            | Op::SetFrc(v)
            | Op::SetTemperatureOffset(v)
            | Op::SetAltitude(v),
            _,
        ) => assert_eq!(Some(v), served.written),
        (Op::SetAsc(on), _) => assert_eq!(Some(u16::from(on)), served.written),
        (Op::DataReady | Op::GetAsc, Outcome::Bool(Ok(on))) => {
            assert_eq!(words()[0] == 1, *on, "status word {}", words()[0])
        }
        (_, Outcome::Word(Ok(value))) => assert_eq!(words()[0], *value),
        (Op::FirmwareVersion, Outcome::Firmware(Ok(version))) => {
            assert_eq!(words()[0].to_be_bytes(), [version.0, version.1]);
        }
        (Op::ReadMeasurement, Outcome::Measurement(Ok(m))) => {
            // Section 1.5: each value is an IEEE 754 float in big-endian
            let value = |i: usize| {
                f32::from_bits((u32::from(words()[2 * i]) << 16) | u32::from(words()[2 * i + 1]))
            };
            // The driver reports whole ppm: truncated or rounded
            assert!(
                [value(0).floor(), value(0).round()].contains(&f32::from(m.co2)),
                "CO2 {} reported as {}",
                value(0),
                m.co2
            );
            assert_eq!(value(1), m.temperature);
            assert_eq!(value(2), m.humidity);
        }
        (_, Outcome::Unit(Ok(()))) => {}
        (op, outcome) => panic!("{op:?} returned {outcome:?}"),
    }
}

/// Proves that the random runs reach every command and every error path
#[derive(Default)]
struct Coverage {
    ok: std::collections::BTreeSet<&'static str>,
    invalid_input: usize,
    nack: usize,
    crc: usize,
}

impl Coverage {
    fn name(op: Op) -> &'static str {
        match op {
            Op::Start(_) => "start",
            Op::Stop => "stop",
            Op::SetInterval(_) => "set interval",
            Op::GetInterval => "get interval",
            Op::DataReady => "data ready",
            Op::ReadMeasurement => "read measurement",
            Op::SetAsc(_) => "set ASC",
            Op::GetAsc => "get ASC",
            Op::SetFrc(_) => "set FRC",
            Op::SetTemperatureOffset(_) => "set temperature offset",
            Op::GetTemperatureOffset => "get temperature offset",
            Op::SetAltitude(_) => "set altitude",
            Op::GetAltitude => "get altitude",
            Op::FirmwareVersion => "firmware version",
            Op::SoftReset => "soft reset",
        }
    }

    fn record(&mut self, op: Op, outcome: &Outcome) {
        match outcome.error() {
            None => {
                self.ok.insert(Self::name(op));
            }
            Some(Error::InvalidInput) => self.invalid_input += 1,
            Some(Error::I2C(_)) => self.nack += 1,
            Some(Error::CRC) => self.crc += 1,
            Some(Error::NotAllowed) => unreachable!("check() panics on NotAllowed"),
        }
    }

    fn assert_complete(&self) {
        if std::env::var("SCD30_SIM_SEED").is_ok() {
            return;
        }
        let mut rng = Rng::new(0);
        for _ in 0..10_000 {
            let name = Self::name(Op::random(&mut rng));
            assert!(self.ok.contains(name), "{name} never succeeded");
        }
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
            eprintln!("failing seed: SCD30_SIM_SEED={}", self.0);
        }
    }
}

fn seeds() -> core::ops::Range<u64> {
    match std::env::var("SCD30_SIM_SEED") {
        Ok(seed) => {
            let seed = seed.parse().expect("SCD30_SIM_SEED must be a number");
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

/// Picks the next call. Section 1.4.5 makes the caller check the data ready
/// status before a read; the generator keeps that precondition by reading
/// only when the model holds a sample.
fn next_op(model: &RefCell<Model>, rng: &mut Rng) -> Op {
    let mut model = model.borrow_mut();
    model.tick();
    loop {
        let op = Op::random(rng);
        if !matches!(op, Op::ReadMeasurement) || model.sample.is_some() {
            return op;
        }
    }
}

/// Runs one call: reset the record, call, compare
fn step(model: &RefCell<Model>, coverage: &mut Coverage, op: Op, call: impl FnOnce() -> Outcome) {
    model.borrow_mut().served = Served::default();
    let outcome = call();
    check(op, &outcome, &model.borrow().served);
    coverage.record(op, &outcome);
}

#[cfg(feature = "sync")]
mod sync {
    use super::*;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{ErrorType, I2c, Operation};
    use libscd::synchronous::scd30::Scd30;

    pub struct Bus<'a>(pub &'a RefCell<Model>);
    pub struct Clock<'a>(pub &'a RefCell<Model>);

    impl ErrorType for Bus<'_> {
        type Error = ErrorKind;
    }

    impl I2c for Bus<'_> {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), ErrorKind> {
            // Section 1.1.1
            assert_eq!(address, 0x61, "I2C address");
            // Section 1.1: "SCD30 does not support repeated start condition"
            assert_eq!(operations.len(), 1, "repeated start");
            let mut model = self.0.borrow_mut();
            match &mut operations[0] {
                Operation::Write(bytes) => model.write(bytes),
                Operation::Read(buf) => model.read(buf),
            }
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
    pub fn scd30_sync_follows_the_interface_description() {
        let mut coverage = Coverage::default();
        for seed in seeds() {
            let _report = SeedReport(seed);
            let model = RefCell::new(Model::new(seed));
            let mut rng = Rng::new(!seed);
            let mut sensor = Scd30::new(Bus(&model), Clock(&model));
            for _ in 0..STEPS {
                pause(&model, &mut rng);
                let op = next_op(&model, &mut rng);
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
    use libscd::asynchronous::scd30::Scd30;

    pub struct Bus<'a>(pub &'a RefCell<Model>);
    pub struct Clock<'a>(pub &'a RefCell<Model>);

    impl ErrorType for Bus<'_> {
        type Error = ErrorKind;
    }

    impl I2c for Bus<'_> {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), ErrorKind> {
            // Section 1.1.1
            assert_eq!(address, 0x61, "I2C address");
            // Section 1.1: "SCD30 does not support repeated start condition"
            assert_eq!(operations.len(), 1, "repeated start");
            let mut model = self.0.borrow_mut();
            match &mut operations[0] {
                Operation::Write(bytes) => model.write(bytes),
                Operation::Read(buf) => model.read(buf),
            }
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
    pub fn scd30_async_follows_the_interface_description() {
        let mut coverage = Coverage::default();
        for seed in seeds() {
            let _report = SeedReport(seed);
            let model = RefCell::new(Model::new(seed));
            let mut rng = Rng::new(!seed);
            let mut sensor = Scd30::new(Bus(&model), Clock(&model));
            for _ in 0..STEPS {
                pause(&model, &mut rng);
                let op = next_op(&model, &mut rng);
                step(&model, &mut coverage, op, || dispatch!(sensor, op, polled));
            }
        }
        coverage.assert_complete();
    }
}
