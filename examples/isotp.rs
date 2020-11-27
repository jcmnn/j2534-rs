//! This example queries a vehicle's VIN using ISO-TP.

use j2534::{ConfigId, PassThruMsg, Protocol, RxStatus, TxFlags};

fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    // Open a ISO-TP channel with a baudrate of 500k.
    let channel = d
        .connect(j2534::Protocol::ISO15765, j2534::ConnectFlags::NONE, 500000)
        .unwrap();

    // Create a filter allowing messages with id `0x7E8` to be received.
    let mask = PassThruMsg::new_isotp(0xFFFFFFFF, &[]).tx_flags(TxFlags::ISO15765_FRAME_PAD);
    let pattern = PassThruMsg::new_isotp(0x7E8, &[]).tx_flags(TxFlags::ISO15765_FRAME_PAD);
    // Set flow control packet to use ID `0x7E0`.
    let fc_pattern = PassThruMsg::new_isotp(0x7E0, &[]).tx_flags(TxFlags::ISO15765_FRAME_PAD);
    let _ = channel.start_message_filter(
        j2534::FilterType::FlowControl,
        Some(&mask),
        Some(&pattern),
        Some(&fc_pattern),
    );

    // Send VIN request
    channel.write(
        &mut [PassThruMsg::new_isotp(0x7E0, &[0x09, 0x02]).tx_flags(TxFlags::ISO15765_FRAME_PAD)],
        1000,
    )?;

    // Receive response
    loop {
        let message = channel.read_once(1000)?;
        if message.transmitted() || message.first_frame() {
            // Message has not been fully processed yet..
            continue;
        }
        if let Some((_id, data)) = message.isotp_message() {
            println!("VIN: {}", std::str::from_utf8(&data[3..]).unwrap());
            break;
        }
    }
    Ok(())
}
