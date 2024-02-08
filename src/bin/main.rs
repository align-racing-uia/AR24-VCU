#![no_main]
#![no_std]

use core::num::{NonZeroU16, NonZeroU8};

use cortex_m::delay::Delay;
use defmt::println;
use fdcan::frame::{FrameFormat, TxFrameHeader};
use fdcan::id::{Id, StandardId};
use fdcan::{NormalOperationMode, FdCan, Instance};
use fdcan::config::NominalBitTiming;
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::gpio::Speed;
use stm32g4xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32g4xx_hal::rcc::{PLLSrc, PllConfig, PllMDiv, PllNMul, PllPDiv, PllQDiv, PllRDiv, Prescaler, SysClockSrc};
use stm32g4xx_hal::stm32::FDCAN2;
use stm32g4xx_hal::timer::MonoTimer;
use stm32g4xx_hal::{stm32::Peripherals, rcc::{RccExt, Config}, gpio::GpioExt};
use stm32g4xx_hal::time::U32Ext;

use vcu_v2::inverter::{self, Inverter};
// Organizing
use vcu_v2 as _; // global logger + panicking-behavior + memory layout
use vcu_v2::statemachine::StateMachine;


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

    println!("Sys clock: {}", rcc.clocks.sys_clk.0);

    // Initializing the GPIO rows   
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    println!("Initializing delay");


    // Making a syst delay running on the system clock, on 170MHz
    let _delay = Delay::new(cp.SYST, 170000000);


    // Breaking out the direct I/O
    let mut brk = gpioc.pc13.into_push_pull_output();
    let mut buz = gpioc.pc14.into_push_pull_output();
    let mut _ext = gpioc.pc15.into_push_pull_output(); // This is currently unused
    
    println!("Initializing debug lights");
    // Debugging lights
    let mut blue_led = gpiob.pb9.into_push_pull_output();
    let mut red_led = gpiob.pb7.into_push_pull_output();

    // Debug timer
    let mono_timer = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);
    let mut debug_timestamp = mono_timer.now();


    // Initializing canbus1
    let mut dti_can = {
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
    let mut sensor_can = {
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
    let mut inverter = Inverter::new(0);
    let mut buzzer_timestamp = mono_timer.now();

    loop {
        sm.process_canbus_data(&mut sensor_can);
        sm.update();

        inverter.process_canbus_data(&mut dti_can);
        inverter.update(&mut dti_can, &sm);
        
        if sm.brakelight {
            brk.set_high().unwrap();
        }else{
            brk.set_low().unwrap();
        }

        if sm.buzzer { // One shot
            buz.set_high().unwrap();
            buzzer_timestamp = mono_timer.now();
            sm.buzzer = false;
        }

        // TODO: Make this more efficient. How expensive is buzzer_timestamp.elapsed?
        if buzzer_timestamp.elapsed() > mono_timer.frequency().0 * 3 { // Buzzer on for 3 seconds
            buz.set_low().unwrap();
        }

        if debug_timestamp.elapsed() > mono_timer.frequency().0 / 2 { // Sending message at roughly 1Hz
            broadcast_message(&mut sensor_can, &sm, &inverter);
            blue_led.toggle().unwrap(); // To know the loop is running
            red_led.toggle().unwrap();
            debug_timestamp = mono_timer.now();
        }

    }

}

fn broadcast_message<I1>(sensor_can: &mut FdCan<I1, NormalOperationMode>, sm: &StateMachine, inverter: &Inverter)
    where
        I1: Instance
{
    let frame_data = [sm.brake_prs_front, sm.brake_prs_rear, sm.throttle_pos, sm.r2d as u8,
                                    inverter.power, inverter.regen, 0, 0];
    let frame_header = TxFrameHeader {
        len: 8,
        frame_format: FrameFormat::Standard,
        id: Id::Standard(StandardId::new(0x234).unwrap()), // Insert VCUs CAN ID here
        bit_rate_switching: false,
        marker: None,
    };
    sensor_can.transmit(frame_header, &frame_data).unwrap();  
}