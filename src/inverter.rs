use defmt::println;
use fdcan::{frame::{FrameFormat, TxFrameHeader}, id::{Id, StandardId}, FdCan, Instance, NormalOperationMode};

use crate::{helpers::id_to_u32, statemachine::StateMachine};

pub struct Inverter {
    dti_node_id: u16,
    regen: u8,
    power: u8
    
}

impl Inverter {

    pub fn new(dti_node_id: u16) -> Inverter {
        Inverter {
            dti_node_id,
            regen: 0,
            power: 0
        }
    }

    pub fn handle_logic(&mut self, sm: &StateMachine){

    }

    pub fn process_canbus_data<I1>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>)
    where
        I1: Instance
    {
        
        let mut data = [0u8; 16];
        match dti_can.receive0(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();  
                println!("Can Frame: {}, {}", id_to_u32(rx_frame.id), &data);
                self.process_data(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    
        match dti_can.receive1(&mut data) {
            Ok(d) => {
                let rx_frame = d.unwrap();
                self.process_data(id_to_u32(rx_frame.id), &data);
            },
            Err(_) => {}
        }
    }

        // Inverter related stuff here
        fn process_data(&mut self, id: u32, data: &[u8; 16])
        {
            match id {
                _ => {} // Nothing here yet
            }
        }

    // Helper functions to talk to DTI Inverter
    fn send_canbus_dti<I1>(
        &mut self,
        dti_can: &mut FdCan<I1, NormalOperationMode>,
        frame_data: [u8; 8],
        packet_id: u16
    ) -> () 
        where
            I1: Instance
    {
        let id: u16 = (packet_id << 5) | self.dti_node_id; // format of DTI canbus messages
        let frame_header = TxFrameHeader {
            len: 8,
            frame_format: FrameFormat::Standard,
            id: Id::Standard(StandardId::new(id).unwrap()), // Insert DTI id here
            bit_rate_switching: false,
            marker: None,
        };
        dti_can.transmit(frame_header, &frame_data).unwrap();
    }

    pub fn ping<I1>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>)
        where
            I1: Instance
    {
        self.send_canbus_dti(dti_can, [1,2,3,4,5,6,7,8], 0);
    }

    // A cyclic message that should be sent every so often to keep the car in ready to drive mode
    pub fn drive_enable<I1>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>) 
        where
            I1: Instance
    {
        let packet_id: u16 = 0x0C;
        self.send_canbus_dti(dti_can, [1,0,0,0,0,0,0,0], packet_id);
    }

    // Sets the forward current draw to a certain percentage
    pub fn set_power<I1>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>, power_percentage: u8)
        where
            I1: Instance
    {
        let packet_id: u16 = 0x05;
        self.send_canbus_dti(dti_can, [power_percentage,0,0,0,0,0,0,0], packet_id);
    }

    // Sets the braking force as a percentage
    pub fn set_regen<I1>(&mut self, dti_can: &mut FdCan<I1, NormalOperationMode>, regen_percentage: u8)
        where
            I1: Instance
    {
        let packet_id: u16 = 0x06;
        self.send_canbus_dti(dti_can, [regen_percentage,0,0,0,0,0,0,0], packet_id);
    }
}