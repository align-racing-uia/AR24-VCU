#![no_main]
#![no_std]

use core::fmt::Debug;
use core::num::{NonZeroU16, NonZeroU8};

use cortex_m::delay::Delay;
use cortex_m_semihosting::debug;
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
use stm32g4xx_hal::timer::{MonoTimer, Timer};
use stm32g4xx_hal::{prelude, block};
use stm32g4xx_hal::{stm32::Peripherals, rcc::{RccExt, Config}, gpio::GpioExt, hal::{digital::v2::ToggleableOutputPin}};
use stm32g4xx_hal::time::U32Ext;

// Organizing
use vcu_v2 as _; // global logger + panicking-behavior + memory layout
use vcu_v2::helpers;
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
    let mut delay = Delay::new(cp.SYST, 170000000);
    let mut brk = gpioc.pc13.into_push_pull_output();
    
    println!("Initializing debug lights");
    // Debugging lights
    let mut blue_led = gpiob.pb9.into_push_pull_output();
    let mut red_led = gpiob.pb7.into_push_pull_output();

    // Debug timer
    let mut debug_timer = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);
    let mut debug_timestamp = debug_timer.now();


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

    let mut sm = StateMachine::new(0);

    loop {
        sm.process_canbus_data(&mut dti_can, &mut sensor_can);
        
        if sm.brakelight {
            brk.set_high().unwrap();
        }else{
            brk.set_low().unwrap();
        }

        if sm.buzzer {
            brk.set_high();
        }else{
            brk.set_low();
        }

        // if debug_timestamp.elapsed() > debug_timer.frequency().0 { // Light blinking every second
        //     red_led.toggle().unwrap(); // To know the loop is running - It should be constant yellow
        //     debug_timestamp = debug_timer.now();
        // }

    }

}





