J2534
=====
Low-level SAE J2534 (PassThru) support for rust.

## J2534 Introduction
SAE J2534 PassThru defines a standard library interface for communicating with vehicle control modules.
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
    // Create a new message with an arbitration id of `8` and payload of `[0, 1, 2, 3]`.
    let message = PassThruMsg::new_can(8, &[0, 1, 2, 3]);
    channel.write(&mut [message], 1000)?;
    Ok(())
}
```