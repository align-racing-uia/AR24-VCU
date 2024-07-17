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
use embassy_stm32::{bind_interrupts, can::{self, filter::{ExtendedFilter, ExtendedFilterSlot, Filter, StandardFilter, StandardFilterSlot}, frame, BufferedCan, Can, CanRx, CanTx, Instance, RxBuf, TxBuf}, dma::Priority, exti, gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed}, peripherals::{FDCAN1, FDCAN2}, time::mhz, Config, Peripheral};
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
static MUT_INVERTER:  Mutex<ThreadModeRawMutex, Inverter> = Mutex::new(Inverter::new());
static MUT_DRIVER_INTERFACE: Mutex<ThreadModeRawMutex, DriverInterface> = Mutex::new(DriverInterface::new());
static MUT_ACU_INTERFACE: Mutex<ThreadModeRawMutex, ACU> = Mutex::new(ACU::new());
static MUT_BMS: Mutex<ThreadModeRawMutex, BMS> = Mutex::new(BMS::new());

const INVERTER_NODE_ID: u8 = 30;
const BRAODCAST_ID: u32 = 0xC0C;
const MAX_THROTTLE_CURRENT: u32 = 352 * 10;
const MAX_AC_CURRENT: u32 = MAX_THROTTLE_CURRENT * 110 / 100;
const MAX_DC_CURRENT: u32 = 200 * 10;
const MAX_DC_BRAKE_CURRENT: u32 = 42 * 10;
const MAX_AC_BRAKE_CURRENT: u32 = 160 * 10;

enum InverterCommand {
    SetCurrent(u32),
    SetBrakeCurrent(u16),
    SetMaxACCurrent(u32),
    SetMaxACBrakeCurrent(u32),
    SetMaxDCCurrent(u32),
    SetMaxDCBrakeCurrent(u32),
    SetDriveEnable(bool),
}

struct Inverter {
    dc_voltage: u16,
    erpm: u32,
    duty_cycle: i16,
    ac_current: i16,
    dc_current: i16,
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

#[derive(Clone, Copy, PartialEq)]
enum VCUFaultCode {
    None = 0,
    CanTimeout = 1,
    ACUTimeout = 2,
    APPSTimeout = 3,
    InverterTimeout = 4,
    InverterFault = 5,   
}

enum VCUMinorFault {
    None = 0,
    BSPDLite = 1,
    APPSPlaus = 2,
}

struct DriverInterface {
    r2d: bool,
    r2d_toggled: bool,
    clear_faults: bool,
    m1: bool,
    m2: bool,
    regen: u8,
    watchdog: Instant
}

impl DriverInterface {
    const fn new() -> DriverInterface {
        DriverInterface { r2d: false, r2d_toggled: false, clear_faults: false, m1: false, m2: false, regen: 0, watchdog: Instant::MIN }
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

struct BMS {
    pack_current: u16,
    pack_voltage: u16,
    pack_soc: u8,
    relay_state: u16,
    pack_dcl: u32,
    high_temp: u8,
    low_temp: u8,
    failsafe: u8,
    watchdog: Instant
}

impl BMS {
    const fn new() -> BMS {
        BMS {
            pack_current: 0,
            pack_voltage: 0,
            pack_soc: 0,
            relay_state: 0,
            pack_dcl: 0,
            high_temp: 0,
            low_temp: 0,
            failsafe: 0,
            watchdog: Instant::MIN,
        }
    }
}


async fn drive_command<T>(command: InverterCommand, can: &mut CanTx<'_, T>)
where
    T: Instance
{
    match command {
        InverterCommand::SetCurrent(current) => {
            // Throttle goes from 0-254
            let mut data = [0xFF; 8];
            data[1] = current as u8;
            data[0] = (current >> 8) as u8;    
            send_can_packet(can, 0x01, INVERTER_NODE_ID, &data).await;   
        },
        InverterCommand::SetBrakeCurrent(current) => {
            let mut data = [0xFF; 8];
            data[1] = current as u8;
            data[0] = (current >> 8) as u8;    
            send_can_packet(can, 0x02, INVERTER_NODE_ID, &data).await;   
        },
        InverterCommand::SetMaxACCurrent(max_current) => {
            let mut data = [0xFF; 8];
            data[1] = max_current as u8;
            data[0] = (max_current >> 8) as u8;    
            send_can_packet(can, 0x08, INVERTER_NODE_ID, &data).await;   
        },
        InverterCommand::SetMaxACBrakeCurrent(max_current) => {
            let mut data = [0xFF; 8];
            data[1] = (max_current) as u8;
            data[0] = ((max_current) >> 8) as u8;    
            send_can_packet(can, 0x09, INVERTER_NODE_ID, &data).await; 
        },
        InverterCommand::SetMaxDCCurrent(max_current) => {
            let mut data = [0xFF; 8];
            data[1] = (max_current) as u8;
            data[0] = ((max_current) >> 8) as u8;    
            send_can_packet(can, 0x0A, INVERTER_NODE_ID, &data).await;   
        },
        InverterCommand::SetMaxDCBrakeCurrent(max_current) => {
            let mut data = [0xFF; 8];
            data[1] = (max_current) as u8;
            data[0] = ((max_current) >> 8) as u8;    
            send_can_packet(can, 0x0B, INVERTER_NODE_ID, &data).await; 
        },
        InverterCommand::SetDriveEnable(r2d) => {
            let mut data = [0xFF; 8];
            data[0] = r2d as u8;
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
                                inverter.duty_cycle = ((data[5] as u16) | (data[4] as u16) << 8) as i16;
                                inverter.dc_voltage = (data[7] as u16) | (data[6] as u16) << 8;
                                
                            },
                            0x21 => {
                                inverter.ac_current = ((data[1] as u16) | (data[0] as u16) << 8) as i16;
                                inverter.dc_current = ((data[3] as u16) | (data[2] as u16) << 8) as i16;
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
                            0x21 => {
                                driver_interface.m1 = ( data[0] & 0x2 ) > 0;
                                driver_interface.m2 = ( data[0] & 0x1 ) > 0;
                                driver_interface.clear_faults = ( data[0] & 0x4 ) > 0;
                                driver_interface.regen = data[1];
                                //driver_interface.fan = ( data[0] & 0x8 ) > 0;

                            },
                            _ => {}
                        }
                        driver_interface.watchdog = Instant::now();
                    },
                    0x02 => {
                        let mut bms = MUT_BMS.lock().await;
                        bms.watchdog = Instant::now();
                        match packet_id {
                            0x02 => {
                                bms.pack_current = (data[1] as u16) | (data[0] as u16) << 8;
                                bms.pack_current *= 10;
                                bms.pack_voltage = (data[3] as u16) | (data[2] as u16) << 8;
                                bms.pack_soc = data[4] / 2;
                                
                            },
                            0x03 => {
                                bms.pack_dcl = ((data[1] as u32) << 0) | ((data[0] as u32) << 8);
                                bms.pack_dcl *= 10;
                            },
                            0x04 => {

                            },
                            _ => {}
                        }
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
                            _ => {}
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
    let mut _extra = Output::new(p.PA5, Level::High, Speed::Low);

    
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

    // All time tracking variables
    let mut command_timestamp = Instant::now();
    let mut broadcast_timestamp = Instant::now();
    let mut buzzer_timestamp = Instant::now();

    // All semi-global states
    let mut regen_enabled = false;
    let mut regen_active = false;
    let mut ready_to_drive = false;
    let mut bspd_lite = false;
    let mut buzzer_state = false;
    let mut updated_limits = true;
    // Global error state
    let mut vcu_fault_code = VCUFaultCode::None;

    loop {
        // APPS Section
        let brake_pressure: u8; let mut throttle: u8; let plausability: bool; let apps_timeout ;
        { // Control brakelight
            let apps = MUT_APPS.lock().await; 
            plausability = apps.plausability;
            apps_timeout = apps.watchdog.elapsed().as_millis() > 250;
            if apps_timeout {
                brake_pressure = 0;
                throttle = 0;
            }else{
                brake_pressure = max(apps.brake_pressure1, apps.brake_pressure2); // We only care about the highest pressure
                throttle = apps.throttle;
            }
        }

        // Inverter section
        let inverter_fault_code: u8; let inverter_timeout: bool;
        {
            let inverter = MUT_INVERTER.lock().await;
            inverter_fault_code = inverter.fault_code;
            inverter_timeout = inverter.watchdog.elapsed().as_millis() > 500;

        }

        let pack_soc: u8; let pack_current: u16;
        {
            let bms = MUT_BMS.lock().await;
            pack_soc = bms.pack_soc;
            pack_current = bms.pack_current;
        }

        // Driver Interface section
        let r2d: bool; let r2d_toggled: bool; let m1: bool; let m2: bool; let regen: u8; let clear_faults: bool;
        {
            let interface = MUT_DRIVER_INTERFACE.lock().await;
            r2d = interface.r2d;
            r2d_toggled = interface.r2d_toggled;
            m1 = interface.m1;
            m2 = interface.m2;
            regen = interface.regen;
            clear_faults = interface.clear_faults;
            // timeout = timeout || interface.watchdog.elapsed().as_millis() > 500; // Not as critical

        }

        let sdc: bool; let pre: bool; let airm: bool; let airp: bool; let acu_timeout: bool;
        {
            let acu = MUT_ACU_INTERFACE.lock().await;
            acu_timeout = acu.watchdog.elapsed().as_millis() > 1000;
            sdc = acu.sdc;
            pre = acu.pre;
            airm = acu.airm;
            airp = acu.airp;
        }
        

        /* Error Checking */
        if inverter_fault_code > 0 {
            vcu_fault_code = VCUFaultCode::InverterFault;
        } else if apps_timeout {
            vcu_fault_code = VCUFaultCode::APPSTimeout;
        } else if acu_timeout {
            vcu_fault_code = VCUFaultCode::ACUTimeout;
        } else if inverter_timeout {
            vcu_fault_code = VCUFaultCode::InverterTimeout;
        } else { 
            vcu_fault_code = VCUFaultCode::None;
        }
        /* Error checking done */
        // All other state checks go here

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

        if pack_soc < 80 {
            regen_enabled = true;
        }else if pack_soc > 85 {
            regen_enabled = false;
        }

        if throttle <= 10 {
            regen_active = true;
        }else {
            regen_active = false;
        }

        if brake_pressure > 15 {
            bspd_lite = true;
        }else if brake_pressure < 4 {
            bspd_lite = false;
        }

        if sdc && (brake_pressure > 15 || clear_faults) && r2d_toggled && r2d && vcu_fault_code == VCUFaultCode::None {
            ready_to_drive = true;
            buzzer_state = true;
            buzzer_timestamp = Instant::now();
        }
        if !r2d || vcu_fault_code != VCUFaultCode::None || !sdc {
            ready_to_drive = false;
        }
        if buzzer_state && buzzer_timestamp.elapsed().as_millis() >= 3000 {
            buzzer_state = false;
        }

        if command_timestamp.elapsed().as_millis() >= 10 {
            //It is very important to not use a Mutex Lock and a canbus await at the same place, as this can cause mutex deadlocks
            if updated_limits {
                drive_command(InverterCommand::SetMaxDCCurrent(pack_current as u32), &mut can2_tx).await;
                drive_command(InverterCommand::SetMaxACBrakeCurrent(MAX_AC_BRAKE_CURRENT), &mut can2_tx).await;
                drive_command(InverterCommand::SetMaxDCBrakeCurrent(MAX_DC_BRAKE_CURRENT), &mut can2_tx).await;
                drive_command(InverterCommand::SetMaxACCurrent(MAX_AC_CURRENT), &mut can2_tx).await;
                // updated_limits = false; not enabled until verified outside of competition
            }
            drive_command(InverterCommand::SetDriveEnable(ready_to_drive), &mut can2_tx).await;
            
            if ready_to_drive {

                if bspd_lite {
                    throttle = 0;
                }                
                
                if regen_active && regen_enabled {
                    // Note: Not legal in UK.
                    // let mut braking_current: u16 = 0;
                    // if regen_active && !bspd_lite && regen_enabled {
                    //     braking_current = MAX_AC_BRAKE_CURRENT as u16;
                    // }
                    // drive_command(InverterCommand::SetBrakeCurrent(braking_current as u16), &mut can2_tx).await;
                }else{
                    let mut current: u32 = MAX_THROTTLE_CURRENT * throttle as u32 / 255;
                    if current > MAX_THROTTLE_CURRENT {
                        current = MAX_THROTTLE_CURRENT;
                    }
                    drive_command(InverterCommand::SetCurrent(current), &mut can2_tx).await;
                }                
                
            }
            command_timestamp = Instant::now();
        }

        if broadcast_timestamp.elapsed().as_millis() >= 100 {

            let mut data = [0u8; 8];
            data[0] = throttle;
            data[1] = brake_pressure;
            data[2] = (sdc as u8) << 4; 
            data[2] |= (plausability as u8) << 3;
            data[2] |= (regen_enabled as u8) << 2; // Moved to fault code
            data[2] |= (bspd_lite as u8) << 1;
            data[2] |= (ready_to_drive as u8);
            data[3] = vcu_fault_code as u8;
            data[4] = pack_soc as u8;
            //data[5] = regen as u8;

            
            let frame = frame::Frame::new_extended(BRAODCAST_ID, &data).unwrap();
            can2_tx.write(&frame).await;
            
            broadcast_timestamp = Instant::now();

        }
        Timer::after_micros(10).await;
        
    }
}
