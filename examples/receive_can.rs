use j2534::{PassThruMsg, Protocol, RxStatus, TxFlags};

fn main() -> j2534::Result<()> {
    // Get a list of interfaces
    let device = match j2534::drivers()?.into_iter().next() {
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
    println!("{:#?}", version_info);

    // Open a CAN channel with a baudrate of 500k.
    let channel = d
        .connect(j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000)
        .unwrap();

    // Create a filter allowing all messages to be received
    let filter = PassThruMsg::new_can(0, &[0]);
    let _ =
        channel.start_message_filter(j2534::FilterType::Pass, Some(&filter), Some(&filter), None);

    let mut messages = [PassThruMsg::new(Protocol::CAN); 32];
    // Read up to 32 messages
    let count = channel.read(&mut messages, 1000)?;
    for msg in &messages[..count] {
        if let Some((id, data)) = msg.can_message() {
            println!("{:X}: {:X?}", id, data);
        }
    }
    Ok(())
}
