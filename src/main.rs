#![no_std]
#![no_main]

mod fmt;

use cortex_m::Peripherals;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, can::{self, filter::{ExtendedFilter, ExtendedFilterSlot, Filter, StandardFilter, StandardFilterSlot}, Can, Instance}, gpio::{Level, Output, Speed}, peripherals::{FDCAN1, FDCAN2}, time::mhz, Config};
use embassy_time::{Timer};
use fmt::info;

bind_interrupts!(struct Irqs1 {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

bind_interrupts!(struct Irqs2 {
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
});

async fn read_dti_can<T>(can: &mut Can<'_,T>) 
    where
        T: Instance
{
    match can.read().await {
        Ok(envelope) => {
            let frame = envelope.frame;
            let id = {
                match frame.id() {
                    embedded_can::Id::Standard(id) => id.as_raw() as u32,
                    embedded_can::Id::Extended(id) => id.as_raw(),
                }
            };
            info!("Timestamp: {}ms, Frame id: 0x{:X}, data: {}", envelope.ts.as_millis(), id, frame.data());
        },
        Err(_) => {
            info!("Oh no")
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {

    let mut config: Config = Default::default();
    { // Setting clock timings
        use embassy_stm32::rcc::*;
        let pll = Pll { 
            source: PllSource::HSE, 
            prediv: PllPreDiv::DIV3, 
            mul: PllMul::MUL85, 
            divp: Some(embassy_stm32::rcc::PllPDiv::DIV6), 
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
            divr: Some(embassy_stm32::rcc::PllRDiv::DIV2) 
        };
        config.rcc.hse = Some(Hse {
            freq: mhz(12),
            mode: HseMode::Oscillator
        });
        config.rcc.hsi48 = Some(Hsi48Config {sync_from_usb: false});
        config.rcc.pll = Some(pll);
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ls = LsConfig { rtc: RtcClockSource::LSI, lsi: true, lse: None };
            
        // Clock for ADC
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.mux.adc345sel = mux::Adcsel::SYS;
        // Clock for CANBUS
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
    }

    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);
    
    
    let mut dti_can = {
        let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs2);
        can.properties().set_extended_filter(ExtendedFilterSlot::_0, ExtendedFilter::accept_all_into_fifo0());
        can.properties().set_extended_filter(ExtendedFilterSlot::_1, ExtendedFilter::accept_all_into_fifo1());
        can.properties().set_standard_filter(StandardFilterSlot::_1, StandardFilter::accept_all_into_fifo1());
        can.config().set_non_iso_mode(false);
        can.set_bitrate(1_000_000);
        can.into_normal_mode()
    
    };

    let mut led = Output::new(p.PB9, Level::High, Speed::Low);
    

    loop {
        read_dti_can(&mut dti_can).await;
        Timer::after_millis(10);
    }
}
