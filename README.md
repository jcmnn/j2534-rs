J2534
=====

Low-level J2534 PassThru support for rust.

## J2534 Introduction
SAE J2534 PassThru defines a standard library interface for communicating with vehicles.
All automakers in the US are required to provide J2534-compatible service software.
J2534 provides access to the communication layer required for accessing vehicle diagnostics services as
well as downloading and reflashing control modules.

## Usage
The J2534 specification requires PassThru libraries to be compiled for 32 bit, so any program using this crate must be compiled as 32 bit.

### Example
```rust
use j2534::{Interface, PassThruMsg, Protocol, ConnectFlags, RxStatus, TxFlags};

fn main() -> j2534::Result<()> {
    // Open the library and connect to a device
    let interface = Interface::new("C:\\device.dll")?;
    let device = interface.open_any()?;
    
    // Create a CAN channel
    let channel = device
        .connect(Protocol::CAN, ConnectFlags::NONE, 500000)
        .unwrap();

    let message = PassThruMsg::new(
        Protocol::CAN,
        RxStatus::NONE, // rx_status
        TxFlags::NONE, // tx_flags
        0, // timestamp
        0, // extra_data_index
        &[
            0_u8, 0, 0, 8, // Arbitration ID of 0x8
            0, 1, 2, 3,
        ], // Message
    );

    channel.write_msgs(&[message], 1000)
}
```