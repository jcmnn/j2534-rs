use j2534::{PassThruMsg, RxStatus, TxFlags};

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
    println!("Version info: {:?}", version_info);

    // Open a CAN channel with a baudrate of 500k.
    let channel = d
        .connect(j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000)
        .unwrap();

    let message = PassThruMsg::new(
        j2534::Protocol::CAN,
        RxStatus::NONE, // rx_status
        TxFlags::NONE,  // tx_flags
        0, // timestamp
        0, // extra_data_index
        &[
            0_u8, 0, 0, 8, // Arbitration ID of 0x8
            0, 1, 2, 3,
        ], // Message
    );

    while channel.write_msgs(&mut [message], 100)? == 0 {}

    Ok(())
}
