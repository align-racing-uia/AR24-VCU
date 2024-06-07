#![no_std]
#![no_main]

mod fmt;

use cortex_m::Peripherals;
use embassy_sync::{blocking_mutex::{raw::{NoopRawMutex, ThreadModeRawMutex}, ThreadModeMutex}, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{adc, bind_interrupts, can::{self, filter::{ExtendedFilter, ExtendedFilterSlot, Filter, StandardFilter, StandardFilterSlot}, frame, BufferedCan, Can, CanRx, CanTx, Instance, RxBuf, TxBuf}, exti, gpio::{AnyPin, Level, Output, Pin, Speed}, peripherals::{FDCAN1, FDCAN2}, time::mhz, Config, Peripheral};
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

static MUT_APPS: Mutex<ThreadModeRawMutex, APPS> = Mutex::new(APPS::new());
static MUT_VCU: Mutex<ThreadModeRawMutex, VCU> = Mutex::new(VCU::new());
static MUT_INVERTER:  Mutex<ThreadModeRawMutex, Inverter> = Mutex::new(Inverter::new());
static MUT_DRIVER_INTERFACE: Mutex<ThreadModeRawMutex, DriverInterface> = Mutex::new(DriverInterface::new());

const INVERTER_NODE_ID: u8 = 30;
const MAX_CURRENT: u16 = 230;

enum InverterCommand {
    SetCurrent,
    SetBrakeCurrent,
    SetERPM,
    SetPosition,
    SetRelativeCurrent,
    SetDriveEnable,
}

struct Inverter {
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
    const fn new() -> Inverter {
        Inverter {
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
    const fn new() -> APPS {
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
    const fn new() -> VCU {
        VCU { ready_to_drive: false, drive_enable: false, applied_throttle: 0, error_code: 0 }
    }
}

struct DriverInterface {
    r2d: bool
}

impl DriverInterface {
    const fn new() -> DriverInterface {
        DriverInterface { r2d: false }
    }
}

async fn drive_command<T>(command: InverterCommand, can: &mut CanTx<'_, T>)
where
    T: Instance
{
    match command {
        InverterCommand::SetCurrent => {
            let apps = MUT_APPS.lock().await;
            let mut data = [0u8; 8];
            let current = MAX_CURRENT * apps.throttle1 as u16 / 100;
            data[1] = current as u8;
            data[0] = (current >> 8) as u8;    
            send_can_packet(can, 0x01, 30, &data).await;   
        },
        InverterCommand::SetBrakeCurrent => {},
        InverterCommand::SetERPM => {},
        InverterCommand::SetPosition => {},
        InverterCommand::SetRelativeCurrent => {},
        InverterCommand::SetDriveEnable => {
            let vcu = MUT_VCU.lock().await;
            let mut data = [0u8; 8];
            data[0] += vcu.drive_enable as u8;
            send_can_packet(can, 0x0C, INVERTER_NODE_ID, &data).await;
        },
    }


}

async fn send_can_packet<T>(can: &mut CanTx<'_, T>, packet_id: u16, node_id: u8, data: &[u8; 8])
where
    T: Instance
{
    let id = (packet_id as u32) << 8 | node_id as u32;
    let frame = frame::Frame::new_extended(id, data).unwrap();
    can.write(&frame).await;
}

#[embassy_executor::task]
async fn read_drive_can(mut can: CanRx<'static, FDCAN2>)
{
    loop {
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
                        let mut inverter = MUT_INVERTER.lock().await;
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
                    0x0A => {
                        let mut apps = MUT_APPS.lock().await;
                        match packet_id {
                            0x0A => {
                                apps.throttle1 = data[0];
                                apps.throttle2 = data[1];
                                apps.brake_pressure1 = data[2];
                                apps.brake_pressure2 = data[3];
                            }
                            _ => {}
                        }
                    },
                    0x0E => {
                        let mut driver_interface = MUT_DRIVER_INTERFACE.lock().await;
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
    
    let can2 = {
        let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs2);
        can.properties().set_extended_filter(ExtendedFilterSlot::_0, ExtendedFilter::accept_all_into_fifo0());
        can.config().set_non_iso_mode(false);
        can.set_bitrate(500_000);
        can.into_normal_mode()
    
    };

    let led = Output::new(p.PB9, Level::High, Speed::Low);
    spawner.spawn(blinker(led)).unwrap();

    let mut command_timestamp = Instant::now();
    let (mut can2_tx, mut can2_rx, mut _can2_properties) = can2.split();

    spawner.spawn(read_drive_can(can2_rx)).unwrap();

    loop {
        
        // APPS Section
        let brake_pressure: u8;
        { // Control brakelight
            let apps = MUT_APPS.lock().await;
            if apps.brake_pressure1 > 3 {
                brakelight.set_high();
            }else{
                brakelight.set_low();
            }
            brake_pressure = apps.brake_pressure1; // As of now we only care about 1 brake sensor
        }

        // Inverter section
        let dc_voltage: u16; let dc_current: u16; let ac_current: u16;
        let inverter_temp: u16; let motor_temp: u16; let erpm: u32;
        {
            let inverter = MUT_INVERTER.lock().await;
            dc_voltage = inverter.dc_voltage;
            dc_current = inverter.dc_current;
            ac_current = inverter.ac_current;
            inverter_temp = inverter.inverter_temp;
            motor_temp = inverter.motor_temp;
            erpm = inverter.erpm;


        }

        // Modify internal logic
        let ready_to_drive: bool;
        {
            let vcu = MUT_VCU.lock().await;
            ready_to_drive = vcu.ready_to_drive;

        }



        {   // Todo buzzer logic
            

        }


        if command_timestamp.elapsed().as_millis() >= 20 {
            drive_command(InverterCommand::SetDriveEnable, &mut can2_tx).await;

            command_timestamp = Instant::now();
        }
        Timer::after_micros(10).await;
        
    }
}
