
use fdcan::{FdCan, Instance, NormalOperationMode};
use crate::{helpers::id_to_u32, inverter::Inverter};
use defmt::println;

pub struct StateMachine
{
    pub throttle_pos: u8,
    pub brake_prs: u8,
    pub r2d: bool,
    pub brakelight: bool,
    pub buzzer: bool,
    pub regen_stage: u8,
    pub wheel_speed: u8,
    r2d_button: bool,
    error_code: u8
}

// The cars state machine
impl StateMachine
{
    pub fn new() -> StateMachine
    {
        StateMachine {
            throttle_pos: 0,
            brake_prs: 0,
            r2d: false,
            brakelight: false,
            buzzer: false,
            r2d_button: false,
            wheel_speed: 0,
            regen_stage: 0,
            error_code: 0,
        }
    }

    pub fn update(&mut self){
        // A function to handle the logic of the car
    }

    // Put all the canbus data here
    fn process_data(&mut self, id: u32, data: &[u8; 16])
    {
        match id {
            0x123 => { // <-- APPS values
                self.throttle_pos = data[0];
                if self.throttle_pos > 101 {
                    self.throttle_pos = 0;
                }
            },
            0x124 => { // <-- DTI Values

            },
            0x125 => { // <-- Cockpit
                self.r2d_button = (0x1 & data[0]) != 0; // R2D button
                if !self.r2d && self.brake_prs > 10 && self.wheel_speed < 5 && self.error_code == 0 {
                    self.r2d = true;
                    self.buzzer = true;
                }
            },
            0x127 => { // <-- Ping
            }
            _ => {} // <-- No function
        }
    }

    // A catch all function
    pub fn process_canbus_data<I1>(&mut self, sensor_can: &mut FdCan<I1, NormalOperationMode>)
    where
        I1: Instance
    {
        
        let mut data = [0u8; 16];
        match sensor_can.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap(); 
                self.process_data(id_to_u32(rx_frame.id), &data);            
            },
            Err(_) => {}
        }
    
        match sensor_can.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data(id_to_u32(rx_frame.id), &data); 
            },
            Err(_) => {}
        }
    }
    
    
    

}
