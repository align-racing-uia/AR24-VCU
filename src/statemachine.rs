
use fdcan::{FdCan, Instance, NormalOperationMode};
use crate::helpers::id_to_u32;

pub struct StateMachine
{
    throttle_pos: u8,
    brake_percentage: u8,
    pub r2d: bool,
    pub brakelight: bool,
    buzzer: bool,
    r2d_button: bool,
    error_code: u8,
}

// The cars state machine
impl StateMachine
{
    pub fn new() -> StateMachine
    {
        StateMachine {
            throttle_pos: 0,
            brake_percentage: 0,
            r2d: false,
            brakelight: false,
            buzzer: false,
            r2d_button: false,
            error_code: 0,
        }
    }

    pub fn process_canbus_data<I1, I2>(&mut self, can1: &mut FdCan<I1, NormalOperationMode>, can2: &mut FdCan<I2, NormalOperationMode>)
    where
        I1: Instance,
        I2: Instance
    {
        
        let mut data = [0u8; 16];
        match can1.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();       
                self.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    
        match can1.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    
        match can2.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap(); 
                self.process_data_sensors(id_to_u32(rx_frame.id), &data);            
            },
            Err(_) => {}
        }
    
        match can2.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data_sensors(id_to_u32(rx_frame.id), &data); 
            },
            Err(_) => {}
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
