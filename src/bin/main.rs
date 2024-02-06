#![no_main]
#![no_std]

use core::num::{NonZeroU16, NonZeroU8};

use defmt::{info, println};
use fdcan::{NormalOperationMode, FdCan, Instance};
use fdcan::config::NominalBitTiming;
use fdcan::filter::{StandardFilter, StandardFilterSlot};
use fdcan::frame::{TxFrameHeader, FrameFormat, RxFrameInfo};
use fdcan::id::{StandardId, Id};
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::gpio::{self, Speed, AF9};
use stm32g4xx_hal::rcc::SysClockSrc;
use stm32g4xx_hal::stm32::{FDCAN1, FDCAN2};
use stm32g4xx_hal::{prelude, block};
use stm32g4xx_hal::{stm32::Peripherals, rcc::{RccExt, Config}, gpio::GpioExt, timer::Timer, delay::DelayFromCountDownTimer, hal::{digital::v2::ToggleableOutputPin, blocking::delay::DelayMs}};
use vcu_v2 as _; // global logger + panicking-behavior + memory layout
use stm32g4xx_hal::time::U32Ext;

const DTI_NODE_ID: u16 = 0; // Must be filled out!

struct StateMachine       
{
    throttle_pos: u8,
    r2d: bool,
    brakelight: bool,
    buzzer: bool,
    r2d_button: bool,
    error_code: u8,
}

impl StateMachine
{
    fn new() -> StateMachine
    {
        StateMachine {
            throttle_pos: 0,
            r2d: false,
            brakelight: false,
            buzzer: false,
            r2d_button: false,
            error_code: 0,
        }
    }

    fn process_data_drivetrain(&mut self, id: u32, data: &[u8; 16])
    {
        match id {
            0x123 => { // <-- APPS values
                self.throttle_pos = data[0];
                if self.throttle_pos > 105 {
                    self.throttle_pos = 0;
                }
            },
            0x124 => { // <-- DTI Values

            },
            0x125 => { // <-- Cockpit
                self.r2d = (0x1 & data[0]) != 0; // R2D button
            }
            _ => {} // <-- No function
        }
    }

    fn process_data_sensors(&mut self, id: u32, data: &[u8; 16])
    {
        match id {
            0x123 => {}
            _ => {} // <-- Id have no purpose
        }
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Setting the system to run from a high speed external 12MHz crystal.
    let mut config = Config::new(SysClockSrc::HSE(12.mhz()));
    // Locking the RCC
    let mut rcc = dp.RCC.freeze(config);


    // Initializing the GPIO rows
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);


    // Making a delay timer
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(10_u32.ms()));

    let mut brk = gpioc.pc13.into_push_pull_output();


    // Initializing canbus1
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

    // Initializing canbus2
    let mut can2 = {
        println!("Initializing CAN2");
        let tx = gpiob.pb6.into_alternate().set_speed(Speed::VeryHigh);
        let rx = gpiob.pb5.into_alternate().set_speed(Speed::VeryHigh);

        let mut can: fdcan::FdCan<stm32g4xx_hal::can::Can<FDCAN2>, fdcan::ConfigMode> = dp.FDCAN2.fdcan(tx, rx, &rcc);
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

    let mut sm = StateMachine::new();

    loop {
        
        delay.delay_ms(1000u32);
        brk.toggle().unwrap();
    }

}

fn process_canbus_data<I1, I2>(
    can1: &mut FdCan<I1, NormalOperationMode>,
    can2: &mut FdCan<I2, NormalOperationMode>,
    sm: &mut StateMachine,
) -> () 
    where
        I1: Instance,
        I2: Instance
{
    let mut data = [0u8; 16];
    match can1.receive0(&mut data) {
        Ok(d) => {
            let rx_frame = d.unwrap();       
            sm.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
        },
        Err(_) => {}
    }

    match can1.receive1(&mut data) {
        Ok(d) => {
            let rx_frame = d.unwrap();
            sm.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
        },
        Err(_) => {}
    }

    match can2.receive0(&mut data) {
        Ok(d) => {
            let rx_frame = d.unwrap(); 
            sm.process_data_sensors(id_to_u32(rx_frame.id), &data);            
        },
        Err(_) => {}
    }

    match can2.receive1(&mut data) {
        Ok(d) => {
            let rx_frame = d.unwrap();
            sm.process_data_sensors(id_to_u32(rx_frame.id), &data); 
        },
        Err(_) => {}
    }
}

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
fn set_brake_current<I1>(dti_can: &mut FdCan<I1, NormalOperationMode>, brake_percentage: u8)
where
    I1: Instance
{
    let packet_id: u16 = 0x06;
    send_canbus_dti(dti_can, [brake_percentage,0,0,0,0,0,0,0], packet_id);
}

fn id_to_u32(id: Id) -> u32 {
    match id {
        Id::Standard(sid) => return sid.as_raw() as u32,
        Id::Extended(eid) => return eid.as_raw(),
    }
}