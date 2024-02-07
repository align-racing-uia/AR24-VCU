#![no_main]
#![no_std]

use core::num::{NonZeroU16, NonZeroU8};

use cortex_m::delay::Delay;
use defmt::{info, println};
use fdcan::{NormalOperationMode, FdCan, Instance};
use fdcan::config::NominalBitTiming;
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use fdcan::frame::{TxFrameHeader, FrameFormat, RxFrameInfo};
use fdcan::id::{StandardId, Id};
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::delay::SYSTDelayExt;
use stm32g4xx_hal::gpio::{self, Speed, AF9};
use stm32g4xx_hal::hal::digital::v2::OutputPin;
use stm32g4xx_hal::rcc::{PLLSrc, PllConfig, PllMDiv, PllNMul, PllPDiv, PllQDiv, PllRDiv, Prescaler, SysClockSrc};
use stm32g4xx_hal::stm32::{FDCAN1, FDCAN2};
use stm32g4xx_hal::{prelude, block};
use stm32g4xx_hal::{stm32::Peripherals, rcc::{RccExt, Config}, gpio::GpioExt, hal::{digital::v2::ToggleableOutputPin}};
use stm32g4xx_hal::time::U32Ext;

// Organizing
use vcu_v2 as _; // global logger + panicking-behavior + memory layout
use vcu_v2::helpers;
use vcu_v2::statemachine::StateMachine;

const DTI_NODE_ID: u16 = 0; // Must be filled out!

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    println!("Setting system clock");
    // Setting the system to run from a high speed external 12MHz crystal.
    let mut config = Config::new(SysClockSrc::PLL);
    config = config.ahb_psc(Prescaler::NotDivided);
    let pll_config = PllConfig { // 12 / 3 * 85 / 2 = 170MHz
        mux: PLLSrc::HSE(12.mhz()), 
        m: PllMDiv::DIV_3, 
        n: PllNMul::MUL_85, 
        r: Some(PllRDiv::DIV_2), 
        q: Some(PllQDiv::DIV_2), 
        p: Some(PllPDiv::DIV_2)
    };
    config = config.pll_cfg(pll_config);

    println!("Freezing RCC");
    // Locking the RCC
    let mut rcc = dp.RCC.freeze(config);
    println!("RCC Frozen");

    // Initializing the GPIO rows   
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    println!("Initializing delay");


    // Making a syst delay running on the system clock, on 170MHz
    let mut delay = Delay::new(cp.SYST, 170000000);
    let mut brk = gpioc.pc13.into_push_pull_output();
    
    println!("Initializing debug lights");
    // Debugging lights
    let mut blue_led = gpiob.pb9.into_push_pull_output();
    let mut red_led = gpiob.pb7.into_push_pull_output();
    blue_led.toggle().unwrap();


    // Initializing canbus1
    let mut can1 = {
        println!("Initializing CAN1");
        let tx = gpioa.pa12.into_alternate().set_speed(Speed::VeryHigh);
        let rx = gpioa.pa11.into_alternate().set_speed(Speed::VeryHigh);

        let mut can = dp.FDCAN1.fdcan(tx, rx, &rcc);
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(NominalBitTiming { // Nominal bit timing for 1000 kbps
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(14).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap()
        });

        can.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

        can.into_normal()
    };

    // Initializing canbus2
    let mut can2 = {
        println!("Initializing CAN2");
        let tx = gpiob.pb6.into_alternate().set_speed(Speed::VeryHigh);
        let rx = gpiob.pb5.into_alternate().set_speed(Speed::VeryHigh);

        let mut can: fdcan::FdCan<stm32g4xx_hal::can::Can<FDCAN2>, fdcan::ConfigMode> = dp.FDCAN2.fdcan(tx, rx, &rcc);
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(NominalBitTiming { // Nominal bit timing for 1000 kbps
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(14).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap()
        });

        can.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

        can.into_normal()
    };

    let mut sm = StateMachine::new();

    loop {
        sm.process_canbus_data(&mut can1, &mut can2);
        
        if sm.brakelight {
            brk.set_high().unwrap();
        }else{
            brk.set_low().unwrap();
        }

        red_led.toggle().unwrap(); // To know the loop is running - It should stay constantly on

    }

}



// Helper functions to talk to DTI Inverter
fn send_canbus_dti<I1>(
    dti_can: &mut FdCan<I1, NormalOperationMode>,
    frame_data: [u8; 8],
    packet_id: u16
) -> () 
    where
        I1: Instance
{
    let id: u16 = (packet_id << 5) | DTI_NODE_ID; // format of DTI canbus messages
    let frame_header = TxFrameHeader {
        len: 8,
        frame_format: FrameFormat::Standard,
        id: Id::Standard(StandardId::new(id).unwrap()), // Insert DTI id here
        bit_rate_switching: false,
        marker: None,
    };
    dti_can.transmit(frame_header, &frame_data).unwrap();
}

// A cyclic message that should be sent every so often to keep the car in ready to drive mode
fn drive_enable<I1>(dti_can: &mut FdCan<I1, NormalOperationMode>) 
    where
        I1: Instance
{
    let packet_id: u16 = 0x0C;
    send_canbus_dti(dti_can, [1,0,0,0,0,0,0,0], packet_id);
}

// Sets the forward current draw to a certain percentage
fn set_forward_current<I1>(dti_can: &mut FdCan<I1, NormalOperationMode>, current_percentage: u8)
    where
        I1: Instance
{
    let packet_id: u16 = 0x05;
    send_canbus_dti(dti_can, [current_percentage,0,0,0,0,0,0,0], packet_id);
}

// Sets the braking force as a percentage
fn set_regen<I1>(dti_can: &mut FdCan<I1, NormalOperationMode>, regen_percentage: u8)
where
    I1: Instance
{
    let packet_id: u16 = 0x06;
    send_canbus_dti(dti_can, [regen_percentage,0,0,0,0,0,0,0], packet_id);
}

