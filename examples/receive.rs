fn main() -> j2534::Result<()> {
    let devices = j2534::list().unwrap();
    if devices.is_empty() {
        println!("No J2534 interfaces found");
        return Ok(())
    }

    let device = j2534::list().unwrap().remove(0);
    println!("Opening interface '{}'", device.name);
    let i = j2534::Interface::new(&device.path)?;
    let d = i.open_any()?;
    let version_info = d.read_version().unwrap();
    println!("Version info: {:?}", version_info);

    let channel = d
        .connect(j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000)
        .unwrap();

    let mut msgs: [j2534::PassThruMsg; 1] = [j2534::PassThruMsg::default(); 1];
    let read = channel.read_msgs(&mut msgs, 1000).unwrap();
    for msg in read.iter() {
        println!("Message Size: {}", msg.data_size);
    }
    Ok(())
}
