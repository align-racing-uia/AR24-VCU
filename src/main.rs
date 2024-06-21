#![no_std]
#![no_main]

mod fmt;

use core::{cmp::{max, min}, ptr};

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{adc, bind_interrupts, can::{self, filter::{ExtendedFilter, ExtendedFilterSlot, Filter, StandardFilter, StandardFilterSlot}, frame, BufferedCan, Can, CanRx, CanTx, Instance, RxBuf, TxBuf}, dma::Priority, exti, gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed}, peripherals::{FDCAN1, FDCAN2}, time::mhz, Config, Peripheral};
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
static MUT_ACU_INTERFACE: Mutex<ThreadModeRawMutex, ACU> = Mutex::new(ACU::new());

const INVERTER_NODE_ID: u8 = 30;
const MAX_CURRENT: u16 = 230;
const BRAODCAST_ID: u32 = 0xC0C;

enum InverterCommand {
    SetCurrent(u8),
    SetBrakeCurrent(u8),
    SetERPM(u32),
    SetPosition(u16),
    SetRelativeCurrent(u8),
    SetDriveEnable(bool),
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
    watchdog: Instant,

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
            watchdog: Instant::MIN,
        }
    }
}

struct APPS {
    throttle: u8,
    brake_pressure1: u8,
    brake_pressure2: u8,
    plausability: bool,
    watchdog: Instant
}

impl APPS {
    const fn new() -> APPS {
        APPS { throttle: 0, brake_pressure1: 0, brake_pressure2: 0, plausability: false, watchdog: Instant::MIN }
    }
}

struct VCU {
    ready_to_drive: bool,
    buzzer: bool,
}

impl VCU {
    const fn new() -> VCU {
        VCU { ready_to_drive: false, buzzer: false }
    }
}

struct DriverInterface {
    r2d: bool,
    r2d_toggled: bool,
    watchdog: Instant
}

impl DriverInterface {
    const fn new() -> DriverInterface {
        DriverInterface { r2d: false, r2d_toggled: false, watchdog: Instant::MIN }
    }
}

struct ACU {
    sdc: bool,
    pre: bool,
    airp: bool, 
    airm: bool,
    watchdog: Instant
}
impl ACU {
    const fn new() -> ACU {
        ACU { sdc: false, pre: false, airp: false, airm: false, watchdog: Instant::MIN }
    }
}


async fn drive_command<T>(command: InverterCommand, can: &mut CanTx<'_, T>)
where
    T: Instance
{
    match command {
        InverterCommand::SetCurrent(throttle) => {
            let mut data = [0u8; 8];
            let mut current = 10 * MAX_CURRENT * throttle as u16 / 255;
            if throttle < 5 { // Safety limit
                current = 0;
            }
            if current > 230 {
                current = 230;
            }
            data[1] = current as u8;
            data[0] = (current >> 8) as u8;    
            send_can_packet(can, 0x01, INVERTER_NODE_ID, &data).await;   
        },
        InverterCommand::SetBrakeCurrent(throttle) => {},
        InverterCommand::SetERPM(rpm) => {},
        InverterCommand::SetPosition(pos) => {},
        InverterCommand::SetRelativeCurrent(relative) => {},
        InverterCommand::SetDriveEnable(r2d) => {
            let mut data = [0u8; 8];
            data[0] += r2d as u8;
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
                                inverter.fault_code    = data[4];
                            },
                            0x23 => {
                                // No relevant messages
                            },
                            0x24 => {
                                inverter.drive_enable_feedback = (data[4] >> 7) != 0;
                            },
                            _ => {
                                //info!("Unknown packet from inverter!")
                            }
                        }
                        inverter.watchdog = Instant::now();
                    },
                    0x0A => {
                        let mut apps = MUT_APPS.lock().await;
                        match packet_id {
                            0x0A => {
                                apps.throttle = data[0];
                                apps.plausability = data[1] != 0;
                                apps.brake_pressure1 = data[2];
                                apps.brake_pressure2 = data[3];
                            }
                            _ => {}
                        }
                        apps.watchdog = Instant::now();
                    },
                    0x0E => {
                        let mut driver_interface = MUT_DRIVER_INTERFACE.lock().await;
                        match packet_id {  
                            0x20 => {
                                let r2d = ( data[0] & 0x01 ) > 0 ;
                                
                                if r2d != driver_interface.r2d && r2d {
                                    
                                    driver_interface.r2d_toggled = true;
                                }else {
                                    driver_interface.r2d_toggled = false; // This might be too aggressive
                                }
                                //info!("R2D Read: {}, R2D_Toggle: {}", r2d, driver_interface.r2d_toggled);
                                driver_interface.r2d = r2d;
                            },
                            _ => {}
                        }
                        driver_interface.watchdog = Instant::now();
                    },
                    0x0B => {
                        let mut acu = MUT_ACU_INTERFACE.lock().await;
                        acu.watchdog = Instant::now();
                        match packet_id {
                            0x1 => {
                                acu.sdc = (data[0] & 1) > 0;
                                acu.airm = (data[0] & 2) > 0;
                                acu.airp = (data[0] & 4) > 0;
                                acu.pre = (data[0] & 8) > 0;
                            },
                            _ => {
                                acu.sdc = (data[0] & 1) > 0;
                                acu.airm = (data[0] & 2) > 0;
                                acu.airp = (data[0] & 4) > 0;
                                acu.pre = (data[0] & 8) > 0;
                            }
                        };
                        
                    },
                    _ => {
                        //info!("Unknown sender! Checking against compatible basic IDs");
                    }
                }
            },
            Err(_) => {
                //info!("Oh no")
            }
        }
        Timer::after_micros(20).await;
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
        config.rcc.ls = LsConfig { rtc: RtcClockSource::LSI, lsi: true, lse: None };        
        config.rcc.pll = Some(pll);
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.low_power_run = false;
            
        // Clock for ADC
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.mux.adc345sel = mux::Adcsel::SYS;
        // Clock for CANBUS
        config.rcc.mux.fdcansel = mux::Fdcansel::HSE;
    }
    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);
   
    
    let mut brakelight = Output::new(p.PC13, Level::Low, Speed::Low);
    let _buzzer_in = Input::new(p.PC14, Pull::Up);
    let mut buzzer = Output::new(p.PA4, Level::High, Speed::Low);
    let _extra_in = Input::new(p.PC15, Pull::Up); // LSI pins had to be a drain gpio, so it is jumped
    let mut extra = Output::new(p.PA5, Level::High, Speed::Low);

    
    let can2 = {
        let mut can = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs2);
        can.properties().set_extended_filter(ExtendedFilterSlot::_0, ExtendedFilter::accept_all_into_fifo0());
        can.config().set_non_iso_mode(false);
        can.set_bitrate(500_000);
        can.into_normal_mode()
    
    };

    let led = Output::new(p.PB9, Level::High, Speed::Low);
    spawner.spawn(blinker(led)).unwrap();

    
    let (mut can2_tx, can2_rx, mut _can2_properties) = can2.split();

    spawner.spawn(read_drive_can(can2_rx)).unwrap();

    let mut command_timestamp = Instant::now();
    let mut broadcast_timestamp = Instant::now(); 
    let mut buzzer_timestamp = Instant::now();

    loop {
        let mut timeout = false;

        // APPS Section
        let brake_pressure: u8; let throttle: u8; let plausability: bool;
        { // Control brakelight
            let apps = MUT_APPS.lock().await; 
            plausability = apps.plausability;
            timeout = timeout || apps.watchdog.elapsed().as_millis() > 100;
            if timeout {
                brake_pressure = 0;
                throttle = 0;
            }else{
                brake_pressure = max(apps.brake_pressure1, apps.brake_pressure2); // We only care about the highest pressure
                throttle = apps.throttle;
            }
        }

        // Inverter section
        let fault_code: u8;
        {
            let inverter = MUT_INVERTER.lock().await;
            fault_code = inverter.fault_code;
            timeout = timeout || inverter.watchdog.elapsed().as_millis() > 100;

        }

        // Driver Interface section
        let r2d: bool; let r2d_toggled: bool;
        {
            let interface = MUT_DRIVER_INTERFACE.lock().await;
            r2d = interface.r2d;
            r2d_toggled = interface.r2d_toggled;
            // timeout = timeout || interface.watchdog.elapsed().as_millis() > 500; // Not as critical

        }

        let sdc: bool; let pre: bool; let airm: bool; let airp: bool;
        {
            let acu = MUT_ACU_INTERFACE.lock().await;
            if acu.watchdog.elapsed().as_millis() > 500 {
                timeout = true;
            }
            sdc = acu.sdc;
            pre = acu.pre;
            airm = acu.airm;
            airp = acu.airp;
        }

        // Modify internal logic TODO: Buzzer
        let ready_to_drive: bool; let buzzer_state: bool;
        {
            let mut vcu = MUT_VCU.lock().await;
            info!("R2D: {}, SDC: {}, Timeout: {}, fault_code: {}", r2d, sdc, timeout, fault_code);
            
            if sdc && r2d_toggled && r2d && fault_code == 0 && !timeout {
                vcu.ready_to_drive = true;
                vcu.buzzer = true;
                buzzer_timestamp = Instant::now();
                
            }
            if !r2d || fault_code > 0 || !sdc || timeout {
                vcu.ready_to_drive = false;
            }
            if vcu.buzzer && buzzer_timestamp.elapsed().as_millis() >= 3000 {
                vcu.buzzer = false;
            }
            ready_to_drive = vcu.ready_to_drive;
            buzzer_state = vcu.buzzer;

        }

        if brake_pressure > 4 {
            brakelight.set_high();
        }else if brake_pressure < 2 {
            brakelight.set_low();
        }

        if buzzer_state {
            buzzer.set_high();
        }else{
            buzzer.set_low();
        }

        if command_timestamp.elapsed().as_millis() >= 20 {
            //It is very important to not use a Mutex Lock, and a canbus await at the same place, as this can cause mutex deadlocks
            drive_command(InverterCommand::SetDriveEnable(ready_to_drive), &mut can2_tx).await;

            if ready_to_drive {
                drive_command(InverterCommand::SetCurrent(throttle), &mut can2_tx).await;
            }

            command_timestamp = Instant::now();
        }

        if broadcast_timestamp.elapsed().as_millis() >= 100 {

            let mut data = [0u8; 8];
            data[0] = throttle;
            data[1] = brake_pressure;
            data[2] = plausability as u8;
            data[3] = ready_to_drive as u8;
            data[4] = timeout as u8;
            
            let frame = frame::Frame::new_extended(BRAODCAST_ID, &data).unwrap();
            can2_tx.write(&frame).await;
            
            broadcast_timestamp = Instant::now();

        }
        Timer::after_micros(10).await;
        
    }
}
