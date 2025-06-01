#![no_std]
#![no_main]

use axp192_dd::{Axp192, AxpError, ChargeCurrentValue, Gpio0FunctionSelect, LdoId};
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_hal::{
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    time::Rate,
};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);
    info!("Embassy initialized!");

    let config: I2cConfig = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(p.I2C1, config)
        .unwrap()
        .with_sda(p.GPIO21)
        .with_scl(p.GPIO22)
        .into_async();

    init_m5stickc_plus_pmic(i2c).await.unwrap();

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#[rustfmt::skip]
async fn init_m5stickc_plus_pmic(i2c: I2c<'_, Async>) -> Result<(), AxpError<I2cError>> {
    let mut axp = Axp192::new(i2c);
    axp.set_ldo_voltage_mv(LdoId::Ldo2, 3300).await?;
    axp.ll.adc_enable_1().write_async(|r| {
        r.set_battery_current_adc_enable(true);
        r.set_acin_voltage_adc_enable(true);
        r.set_acin_current_adc_enable(true);
        r.set_vbus_voltage_adc_enable(true);
        r.set_vbus_current_adc_enable(true);
        r.set_aps_voltage_adc_enable(true);
    }).await?;
    axp.ll.charge_control_1().write_async(|r| r.set_charge_current(ChargeCurrentValue::Ma100)).await?;
    axp.set_gpio0_ldo_voltage_mv(3300).await?;
    axp.ll.gpio_0_control().write_async(|r| {
        r.set_function_select(Gpio0FunctionSelect::LowNoiseLdoOutput);
    }).await?;
    axp.ll.power_output_control().modify_async(|r| {
        r.set_dcdc_1_output_enable(true);
        r.set_dcdc_3_output_enable(false);
        r.set_ldo_2_output_enable(true);
        r.set_ldo_3_output_enable(true);
        r.set_dcdc_2_output_enable(false);
        r.set_exten_output_enable(true);
    }).await?;
    axp.set_battery_charge_high_temp_threshold_mv(3226).await?;
    axp.ll.backup_battery_charge_control().write_async(|r| {
        r.set_backup_charge_enable(true);
    }).await?;

    info!("Battery voltage: {} mV", axp.get_battery_voltage_mv().await?);
    info!("Charge current: {} mA", axp.get_battery_charge_current_ma().await?);
    Ok(())
}
