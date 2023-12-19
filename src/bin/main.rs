#![no_main]
#![no_std]

use core::num::{NonZeroU16, NonZeroU8};

use defmt::{info, println};
use fdcan::config::NominalBitTiming;
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use fdcan::frame::{TxFrameHeader, FrameFormat};
use fdcan::id::StandardId;
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::gpio::{Speed, self};
use stm32g4xx_hal::rcc::SysClockSrc;
use stm32g4xx_hal::{prelude, block};
use stm32g4xx_hal::{stm32::Peripherals, rcc::{RccExt, Config}, gpio::GpioExt, timer::Timer, delay::DelayFromCountDownTimer, hal::{digital::v2::ToggleableOutputPin, blocking::delay::DelayMs}};
use vcu_v2 as _; // global logger + panicking-behavior + memory layout
use stm32g4xx_hal::time::U32Ext;



#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");
    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut config = Config::new(SysClockSrc::HSE(12.mhz()));
    let mut rcc = dp.RCC.freeze(config);

    info!("Initialize led pins");
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(10_u32.ms()));


    let mut can1 = {
        println!("Initializing CAN1");
        let tx = gpioa.pa12.into_alternate().set_speed(Speed::VeryHigh);
        let rx = gpioa.pa11.into_alternate().set_speed(Speed::VeryHigh);

        let mut can = dp.FDCAN1.fdcan(tx, rx, &rcc);
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(NominalBitTiming {
            prescaler: NonZeroU16::new(1).unwrap(),
            seg1: NonZeroU8::new(10).unwrap(),
            seg2: NonZeroU8::new(1).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap()
        });

        can.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

        can.into_normal()
    };

    let mut can2 = {
        println!("Initializing CAN2");
        let tx = gpiob.pb6.into_alternate().set_speed(Speed::VeryHigh);
        let rx = gpiob.pb5.into_alternate().set_speed(Speed::VeryHigh);

        let mut can = dp.FDCAN2.fdcan(tx, rx, &rcc);
        can.set_protocol_exception_handling(false);

        can.set_nominal_bit_timing(NominalBitTiming {
            prescaler: NonZeroU16::new(1).unwrap(),
            seg1: NonZeroU8::new(10).unwrap(),
            seg2: NonZeroU8::new(1).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap()
        });

        can.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

        can.into_normal()
    };

    let mut data = [0u8; 16];

    loop {

        match can2.receive0(&mut data) {
            Ok(d) => {
                let rxFrame = d.unwrap();
                match rxFrame.id {
                    fdcan::id::Id::Standard(id) => {
                        println!("{}: {}", id.as_raw(), data[0]);
                    },
                    fdcan::id::Id::Extended(_) => {},
                }
                    
                
                
            },
            Err(e) => {

            }
        }
        

        let header = TxFrameHeader {
            len: 1,
            id: StandardId::new(0x23).unwrap().into(),
            frame_format: FrameFormat::Standard,
            bit_rate_switching: false,
            marker: None

        };

        block!(can1.transmit(header, &[0x00])).unwrap();
        delay.delay_ms(1u32);
    }


    vcu_v2::exit()
}
