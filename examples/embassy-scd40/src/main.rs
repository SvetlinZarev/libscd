#![no_std]
#![no_main]

use defmt::*;
#[allow(unused)]
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use libscd::synchronous::scd4x::Scd40;
#[allow(unused)]
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let i2c = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB3,
        Irqs,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );

    let mut scd = Scd40::new(i2c, Delay);

    // When re-programming, the controller will be restarted,
    // but not the sensor. We try to stop it in order to
    // prevent the rest of the commands failing.
    _ = scd.stop_periodic_measurement();

    info!("Sensor serial number: {:?}", scd.serial_number());
    if let Err(e) = scd.start_periodic_measurement() {
        defmt::panic!("Failed to start periodic measurement: {:?}", e);
    }

    loop {
        if scd.data_ready().unwrap() {
            let m = scd.read_measurement().unwrap();
            info!("CO2: {}\nHumidity: {}\nTemperature: {}", m.co2, m.humidity, m.temperature)
        }

        Delay.delay_ms(1000)
    }
}
