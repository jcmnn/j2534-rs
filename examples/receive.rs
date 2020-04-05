use j2534::PassThruMsg;

fn main() -> j2534::Result<()> {
    // Get a list of interfaces
    let device = match j2534::list()?.into_iter().next() {
        Some(device) => device,
        None => {
            println!("No J2534 interfaces found");
            return Ok(());
        }
    };

    println!("Opening interface '{}'", device.name);
    let i = j2534::Interface::new(&device.path)?;
    // Open any connected device
    let d = i.open_any()?;
    // Get version information
    let version_info = d.read_version().unwrap();
    println!("Version info: {:?}", version_info);

    // Open a CAN channel with a baudrate of 500k.
    let channel = d
        .connect(j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000)
        .unwrap();

    // Create a filter allowing all messages to be received
    let filter = PassThruMsg::new(j2534::Protocol::CAN, 0, 0, 0, 5, &[0_u8; 5]);
    let _ = channel.start_msg_filter(j2534::FilterType::Pass, Some(&filter), Some(&filter), None);

    // Read one message
    let mut msgs: [j2534::PassThruMsg; 1] = [j2534::PassThruMsg::default(); 1];
    let read = channel.read_msgs(&mut msgs, 1000)?;
    for msg in read.iter() {
        if msg.data_size > 4 {
            let id = ((msg.data[0] as u32) << 24)
                | ((msg.data[1] as u32) << 16)
                | ((msg.data[2] as u32) << 8)
                | (msg.data[3] as u32);
            println!("{:X}: {:X?}", id, &msg.data[4..msg.data_size as usize]);
        }
    }
    Ok(())
}
