use fdcan::id::Id;


pub fn id_to_u32(id: Id) -> u32 {
    match id {
        Id::Standard(sid) => return sid.as_raw() as u32,
        Id::Extended(eid) => return eid.as_raw(),
    }
}

