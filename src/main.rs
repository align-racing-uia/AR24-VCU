#![no_std]
#![no_main]

mod fmt;

use cortex_m::Peripherals;
use embassy_sync::{blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex}, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, can::{self, filter::{ExtendedFilter, ExtendedFilterSlot, Filter, StandardFilter, StandardFilterSlot}, frame, Can, Instance}, exti, gpio::{AnyPin, Level, Output, Pin, Speed}, peripherals::{FDCAN1, FDCAN2}, time::mhz, Config, Peripheral};
use embassy_time::{Instant, Timer};
use fmt::info;

bind_interrupts!(struct Irqs1 {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

bind_interrupts!(struct Irqs2 {
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
});


enum InverterCommand {
    SetCurrent,
    SetBrakeCurrent,
    SetERPM,
    SetPosition,
    SetRelativeCurrent,
    SetDriveEnable,
}

struct Inverter {
    node_id: u8,
    max_current: u16,
    dc_voltage: u16,
    erpm: u32,
    duty_cycle: u16,
    ac_current: u16,
    dc_current: u16,
    inverter_temp: u16,
    motor_temp: u16,
    drive_enable_feedback: bool,
    fault_code: u8,

}

impl Inverter {
    fn new() -> Inverter {
        Inverter {
            node_id: 30,
            max_current: 230,
            dc_voltage: 0,
            erpm: 0,
            duty_cycle: 0,
            ac_current: 0,
            dc_current: 0,
            inverter_temp: 0,
            motor_temp: 0,
            drive_enable_feedback: false,
            fault_code: 0,
        }
    }
}

struct APPS {
    throttle1: u8,
    throttle2: u8,
    brake_pressure1: u8,
    brake_pressure2: u8,

}

impl APPS {
    fn new() -> APPS {
        APPS { throttle1: 0, throttle2: 0, brake_pressure1: 0, brake_pressure2: 0 }
    }
}

struct VCU {
    ready_to_drive: bool,
    drive_enable: bool,
    applied_throttle: u8,
    error_code: u8,
}

impl VCU {
    fn new() -> VCU {
        VCU { ready_to_drive: false, drive_enable: false, applied_throttle: 0, error_code: 0 }
    }
}

struct DriverInterface {
    r2d: bool
}

impl DriverInterface {
    fn new() -> DriverInterface {
        DriverInterface { r2d: false }
    }
}

async fn drive_command<T>(command: InverterCommand, can: &mut Can<'_, T>, vcu: &mut VCU,  inverter: &mut Inverter, apps: &mut APPS, driver_interface: &mut DriverInterface)
where
    T: Instance
{
    match command {
        InverterCommand::SetCurrent => {
            let mut data = [0u8; 8];
            let mut current = inverter.max_current * apps.throttle1 as u16 / 100;
            data[1] = current as u8;
            data[0] = (current >> 8) as u8;
            send_can_packet(can, 0x01, 30, &data).await;

        },
        InverterCommand::SetBrakeCurrent => {},
        InverterCommand::SetERPM => {},
        InverterCommand::SetPosition => {},
        InverterCommand::SetRelativeCurrent => {},
        InverterCommand::SetDriveEnable => {
            let mut data = [0u8; 8];
            data[0] += vcu.drive_enable as u8;
            send_can_packet(can, 0x0C, inverter.node_id, &data).await;
        },
    }


}

async fn send_can_packet<T>(can: &mut Can<'_, T>, packet_id: u16, node_id: u8, data: &[u8; 8])
where
    T: Instance
{
    let id = (packet_id as u32) << 8 | node_id as u32;
    let frame = frame::Frame::new_extended(id, data).unwrap();
    can.write(&frame).await;
}

async fn read_drive_can<T>(can: &mut Can<'_, T>, vcu: &mut VCU,  inverter: &mut Inverter, apps: &mut APPS, driver_interface: &mut DriverInterface)
where
    T: Instance
{
    match can.read().await {
        Ok(envelope) => {
            let frame = envelope.frame;
            let data = frame.data();            
            let (packet_id, node_id) = {
                match frame.id() {
                    embedded_can::Id::Standard(id) => {
                        ((id.as_raw() >> 5) as u16, (id.as_raw() & 0x1F) as u8)
                    },
                    embedded_can::Id::Extended(id) => {
                        ((id.as_raw() >> 8) as u16, (id.as_raw() & 0xFF) as u8)
                    },
                }
            };
            match node_id {
                0x1E => { // ID = 30 Inverter
                    match packet_id {
                        0x20 => {
                            inverter.erpm = (data[3] as u32) | (data[2] as u32) << 8  | (data[1] as u32) << 16 | (data[0] as u32) << 24; // lol
                            inverter.duty_cycle = (data[5] as u16) | (data[4] as u16) << 8;
                            inverter.dc_voltage = (data[7] as u16) | (data[6] as u16) << 8;
                            
                        },
                        0x21 => {
                            inverter.ac_current = (data[1] as u16) | (data[0] as u16) << 8;
                            inverter.dc_current = (data[3] as u16) | (data[2] as u16) << 8;
                        },
                        0x22 => {
                            inverter.inverter_temp = (data[1] as u16) | (data[0] as u16) << 8;
                            inverter.motor_temp    = (data[3] as u16) | (data[2] as u16) << 8;
                            inverter.fault_code    = data[5];
                        },
                        0x23 => {
                            // No relevant messages
                        },
                        0x24 => {
                            inverter.drive_enable_feedback = (data[4] >> 7) != 0;
                        },
                        _ => {
                            info!("Unknown packet from inverter!")
                        }
                    }
                },
                0x0E => {
                    match packet_id {
                        0x20 => {
                            driver_interface.r2d = data[0] > 0;
                        },
                        _ => {}
                    }
                },
                _ => {
                    info!("Unknown sender! Checking against compatible basic IDs");
                }
            }
        },
        Err(_) => {
            info!("Oh no")
        }
    }
}

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>){ // Im well and alive script, wooo
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {

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
        // config.rcc.ls = LsConfig { rtc: RtcClockSource::LSI, lsi: true, lse: None };
            
        // Clock for ADC
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.mux.adc345sel = mux::Adcsel::SYS;
        // Clock for CANBUS
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
    }

    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);
    
    let mut brakelight = Output::new(p.PC13, Level::Low, Speed::Low);
    let mut buzzer = Output::new(p.PC14, Level::Low, Speed::Low);
    let mut extra = Output::new(p.PC15, Level::Low, Speed::Low);
    
    let mut can2 = {
        let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs2);
        can.properties().set_extended_filter(ExtendedFilterSlot::_0, ExtendedFilter::accept_all_into_fifo0());
        can.properties().set_extended_filter(ExtendedFilterSlot::_1, ExtendedFilter::accept_all_into_fifo1());
        can.properties().set_standard_filter(StandardFilterSlot::_1, StandardFilter::accept_all_into_fifo1());
        can.config().set_non_iso_mode(false);
        can.set_bitrate(1_000_000);
        can.into_normal_mode()
    
    };
    let led = Output::new(p.PB9, Level::High, Speed::Low);
    spawner.spawn(blinker(led)).unwrap();

    let mut vcu = VCU::new();
    let mut apps = APPS::new();
    let mut inverter = Inverter::new();
    let mut driver_interface = DriverInterface::new();

    let mut command_timestamp = Instant::now();

    loop {
        // Update all states from can
        read_drive_can(&mut can2, &mut vcu, &mut inverter, &mut apps, &mut driver_interface).await;

        if driver_interface.r2d && inverter.dc_voltage > 350 && inverter.fault_code == 0 { // This has to be programmed muuuch better
            vcu.ready_to_drive = true;
        }
        
        if apps.brake_pressure1 > 3 {
            brakelight.set_high();
        }else{
            brakelight.set_low();
        }

        if command_timestamp.elapsed().as_millis() >= 20 {
            drive_command(InverterCommand::SetDriveEnable, &mut can2, &mut vcu, &mut inverter, &mut apps, &mut driver_interface).await;

            if vcu.ready_to_drive {
                drive_command(InverterCommand::SetCurrent,  &mut can2, &mut vcu, &mut inverter, &mut apps, &mut driver_interface).await;
            }

            command_timestamp = Instant::now();
        }
        
    }
}
