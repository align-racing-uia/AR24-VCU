
use fdcan::{FdCan, Instance, NormalOperationMode};
use crate::{helpers::id_to_u32, inverter::Inverter};

pub struct StateMachine
{
    throttle_pos: u8,
    brake_percentage: u8,
    pub r2d: bool,
    pub brakelight: bool,
    buzzer: bool,
    r2d_button: bool,
    error_code: u8,
    inverter: Inverter
}

// The cars state machine
impl StateMachine
{
    pub fn new(dti_node_id: u16) -> StateMachine
    {
        StateMachine {
            throttle_pos: 0,
            brake_percentage: 0,
            r2d: false,
            brakelight: false,
            buzzer: false,
            r2d_button: false,
            error_code: 0,
            inverter: Inverter::new(dti_node_id)
        }
    }

    // A catch all function
    pub fn process_canbus_data<I1, I2>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>, sensor_can: &mut FdCan<I2, NormalOperationMode>)
    where
        I1: Instance,
        I2: Instance
    {
        
        let mut data = [0u8; 16];
        match dti_can.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();       
                self.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    
        match dti_can.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data_drivetrain(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    
        match sensor_can.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap(); 
                self.process_data_sensors(id_to_u32(rx_frame.id), &data);            
            },
            Err(_) => {}
        }
    
        match sensor_can.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data_sensors(id_to_u32(rx_frame.id), &data); 
            },
            Err(_) => {}
        }
    }
    
    
    // Inverter related stuff here
    fn process_data_drivetrain(&mut self, id: u32, data: &[u8; 16])
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
                self.r2d = (0x1 & data[0]) != 0; // R2D button
            }
            _ => {} // <-- No function
        }
    }

    // Sensor related stuff here
    fn process_data_sensors(&mut self, id: u32, data: &[u8; 16])
    {
        match id {
            0x123 => {}
            _ => {} // <-- No function
        }
    }

}
