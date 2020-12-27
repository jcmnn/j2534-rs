//! SAE J2534 PassThru defines a standard library interface for communicating with vehicle control modules.
//! All automakers in the US are required to provide J2534-compatible service software.
//! J2534 provides access to the communication layers required for accessing vehicle diagnostics services as
//! well as downloading and reflashing control modules.
//!
//! ### Example
//! ```rust
//! use j2534::{Interface, PassThruMsg, Protocol, ConnectFlags, RxStatus, TxFlags};
//!
//! fn main() -> j2534::Result<()> {
//!     // Open the library and connect to a device
//!     let interface = Interface::new("C:\\device.dll")?;
//!     let device = interface.open_any()?;
//!
//!     // Create a CAN channel
//!     let channel = device
//!         .connect(Protocol::CAN, ConnectFlags::NONE, 500000)
//!         .unwrap();
//!
//!     // Create a new message with an arbitration id of `8` and payload of `[0, 1, 2, 3]`.
//!     let message = PassThruMsg::new_can(8, &[0, 1, 2, 3]);
//!     channel.write(&mut [message], 1000)?;
//!     Ok(())
//! }
//! ```

#[macro_use]
extern crate bitflags;

use std::ffi;
use std::ffi::OsStr;
use std::fmt;
use std::fmt::Debug;
use std::io;
use std::marker::PhantomData;
use std::path::Path;
use std::str::Utf8Error;

use bitflags::_core::fmt::Formatter;
use libloading::{Library, Symbol};

#[cfg(windows)]
use winreg::{enums::*, RegKey};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error(transparent)]
    Utf8Error(#[from] Utf8Error),
    #[error(transparent)]
    Io(#[from] io::Error),

    #[error("success")]
    NoError,
    #[error("option not supported')")]
    NotSupported,
    #[error("invalid channel id")]
    InvalidChannelId,
    #[error("invalid protocol id")]
    InvalidProtocolId,
    #[error("NULL was incorrectly passed as a parameter")]
    NullParameter,
    #[error("invalid ioctl parameter")]
    InvalidIoctlValue,
    #[error("invalid flags")]
    InvalidFlags,
    #[error("unspecified error")]
    Failed,
    #[error("a PassThru device is not connected")]
    DeviceNotConnected,
    #[error("timed out")]
    Timeout,
    #[error("invalid message")]
    InvalidMessage,
    #[error("invalid time interval")]
    InvalidTimeInterval,
    #[error("exceeded message limit")]
    ExceededLimit,
    #[error("invalid message id")]
    InvalidMessageId,
    #[error("device in use")]
    DeviceInUse,
    #[error("invalid ioctl id")]
    InvalidIoctlId,
    /// Could not read any messages from the vehicle network
    #[error("could not read messages")]
    BufferEmpty,
    #[error("transmit queue full")]
    BufferFull,
    /// Receive buffer overflowed and messages were lost
    #[error("receive buffer overflowed")]
    BufferOverflow,
    #[error("unknown pin number or resource in use")]
    PinInvalid,
    #[error("channel already in use")]
    ChannelInUse,
    #[error("message protocol does not match channel protocol")]
    MessageProtocolId,
    #[error("invalid filter id")]
    InvalidFilterId,
    #[error("no flow control filter matches outgoing message")]
    NoFlowControl,
    #[error("filter already exists")]
    NotUnique,
    #[error("unsupported baudrate")]
    InvalidBaudrate,
    #[error("invalid device id")]
    InvalidDeviceId,
    #[error("unknown j2534 error code {0}")]
    Unknown(i32),
}

impl Error {
    pub fn from_code(code: i32) -> Error {
        match code {
            0x00 => Error::NoError,
            0x01 => Error::NotSupported,
            0x02 => Error::InvalidChannelId,
            0x03 => Error::InvalidProtocolId,
            0x04 => Error::NullParameter,
            0x05 => Error::InvalidIoctlValue,
            0x06 => Error::InvalidFlags,
            0x07 => Error::Failed,
            0x08 => Error::DeviceNotConnected,
            0x09 => Error::Timeout,
            0x0A => Error::InvalidMessage,
            0x0B => Error::InvalidTimeInterval,
            0x0C => Error::ExceededLimit,
            0x0D => Error::InvalidMessageId,
            0x0E => Error::DeviceInUse,
            0x0F => Error::InvalidIoctlId,
            0x10 => Error::BufferEmpty,
            0x11 => Error::BufferFull,
            0x12 => Error::BufferOverflow,
            0x13 => Error::PinInvalid,
            0x14 => Error::ChannelInUse,
            0x15 => Error::MessageProtocolId,
            0x16 => Error::InvalidFilterId,
            0x17 => Error::NoFlowControl,
            0x18 => Error::NotUnique,
            0x19 => Error::InvalidBaudrate,
            0x1A => Error::InvalidDeviceId,
            other => Error::Unknown(other),
        }
    }
}

type PassThruOpenFn =
    unsafe extern "stdcall" fn(name: *const libc::c_void, device_id: *mut u32) -> i32;
type PassThruCloseFn = unsafe extern "stdcall" fn(device_id: u32) -> i32;
type PassThruConnectFn = unsafe extern "stdcall" fn(
    device_id: u32,
    protocol_id: u32,
    flags: u32,
    baudrate: u32,
    channel_id: *mut u32,
) -> i32;
type PassThruDisconnectFn = unsafe extern "stdcall" fn(channel_id: u32) -> i32;
type PassThruReadMsgsFn = unsafe extern "stdcall" fn(
    channel_id: u32,
    msgs: *mut PassThruMsg,
    num_msgs: *mut u32,
    timeout: u32,
) -> i32;
type PassThruWriteMsgsFn = unsafe extern "stdcall" fn(
    channel_id: u32,
    msgs: *mut PassThruMsg,
    num_msgs: *mut u32,
    timeout: u32,
) -> i32;
type PassThruStartPeriodicMsgFn = unsafe extern "stdcall" fn(
    channel_id: u32,
    msg: *const PassThruMsg,
    msg_id: *mut u32,
    time_interval: u32,
) -> i32;
type PassThruStopPeriodicMsgFn = unsafe extern "stdcall" fn(channel_id: u32, msg_id: u32) -> i32;
type PassThruStartMsgFilterFn = unsafe extern "stdcall" fn(
    channel_id: u32,
    filter_type: u32,
    msg_mask: *const PassThruMsg,
    pattern_msg: *const PassThruMsg,
    flow_control_msg: *const PassThruMsg,
    filter_id: *mut u32,
) -> i32;
type PassThruStopMsgFilterFn = unsafe extern "stdcall" fn(channel_id: u32, filter_id: u32) -> i32;
type PassThruSetProgrammingVoltageFn =
    unsafe extern "stdcall" fn(device_id: u32, pin_number: u32, voltage: u32) -> i32;
type PassThruReadVersionFn = unsafe extern "stdcall" fn(
    device_id: u32,
    firmware_version: *mut libc::c_char,
    dll_version: *mut libc::c_char,
    api_version: *mut libc::c_char,
) -> i32;
type PassThruGetLastErrorFn =
    unsafe extern "stdcall" fn(error_description: *mut libc::c_char) -> i32;
type PassThruIoctlFn = unsafe extern "stdcall" fn(
    handle_id: u32,
    ioctl_id: u32,
    input: *mut libc::c_void,
    output: *mut libc::c_void,
) -> i32;

// Much of the descriptions and APIs used here were taken from http://www.drewtech.com/support/passthru.html

#[derive(Copy, Clone)]
#[repr(C, packed(1))]
/// A message sent a received from the device
pub struct PassThruMsg {
    pub protocol_id: u32,
    pub rx_status: u32,
    pub tx_flags: u32,
    pub timestamp: u32,
    pub data_size: u32,
    pub extra_data_index: u32,
    pub data: [u8; 4128],
}

#[repr(C, packed(1))]
struct SConfig {
    parameter: u32,
    value: u32,
}

#[repr(C, packed(1))]
struct SConfigList {
    size: u32,
    config_ptr: *const SConfig,
}

#[repr(C, packed(1))]
struct SByteArray {
    size: u32,
    byte_ptr: *const u8,
}

impl PassThruMsg {
    pub fn new_raw(
        protocol: Protocol,
        rx_status: RxStatus,
        tx_flags: TxFlags,
        timestamp: u32,
        data_size: u32,
        extra_data_index: u32,
        data: [u8; 4128],
    ) -> PassThruMsg {
        PassThruMsg {
            protocol_id: protocol as u32,
            rx_status: rx_status.bits,
            tx_flags: tx_flags.bits,
            timestamp,
            data_size,
            extra_data_index,
            data,
        }
    }

    pub fn new(protocol: Protocol) -> PassThruMsg {
        PassThruMsg {
            protocol_id: protocol as u32,
            rx_status: 0,
            tx_flags: 0,
            timestamp: 0,
            data_size: 0,
            extra_data_index: 0,
            data: [0; 4128],
        }
    }

    /// Creates a new CAN message.
    /// The data size must be less than or equal to 8 bytes.
    pub fn new_can(id: u32, data: &[u8]) -> PassThruMsg {
        let mut msg_data = [0_u8; 4128];
        // Copy arbitration ID
        msg_data[0] = ((id >> 24) & 0xFF) as u8;
        msg_data[1] = ((id >> 16) & 0xFF) as u8;
        msg_data[2] = ((id >> 8) & 0xFF) as u8;
        msg_data[3] = (id & 0xFF) as u8;

        // Copy the message
        &mut msg_data[4..data.len() + 4].copy_from_slice(data);

        PassThruMsg {
            data: msg_data,
            data_size: (data.len() + 4) as u32,
            ..Self::new(Protocol::CAN)
        }
    }

    /// Creates a new ISO 15765-2 (ISO-TP) message.
    /// ISO-TP is a transport-layer protocol that uses multiple CAN messages to transmit
    /// up to 4095 bytes per packet.
    /// The data size must be less than or equal to 4095.
    pub fn new_isotp(id: u32, data: &[u8]) -> PassThruMsg {
        let mut msg_data = [0_u8; 4128];
        // Copy arbitration ID
        msg_data[0] = ((id >> 24) & 0xFF) as u8;
        msg_data[1] = ((id >> 16) & 0xFF) as u8;
        msg_data[2] = ((id >> 8) & 0xFF) as u8;
        msg_data[3] = (id & 0xFF) as u8;

        // Copy the message
        &mut msg_data[4..data.len() + 4].copy_from_slice(data);

        PassThruMsg {
            data: msg_data,
            data_size: (data.len() + 4) as u32,
            ..Self::new(Protocol::ISO15765)
        }
    }

    /// Returns the CAN ID and payload. Also use this method for reading ISO-TP messages.
    /// Returns `None` if the protocol is not CAN or ISO15765 or if the message is too short.
    pub fn can_message(&self) -> Option<(u32, &[u8])> {
        if (self.protocol_id == (Protocol::CAN as u32)
            || self.protocol_id == (Protocol::ISO15765 as u32))
            && self.data_size >= 4
        {
            let id = ((self.data[0] as u32) << 24)
                | ((self.data[1] as u32) << 16)
                | ((self.data[2] as u32) << 8)
                | (self.data[3] as u32);
            Some((id, &self.data[4..self.data_size as usize]))
        } else {
            None
        }
    }

    /// Alias to [`PassThruMsg::can_message`]
    #[inline]
    pub fn isotp_message(&self) -> Option<(u32, &[u8])> {
        self.can_message()
    }

    /// Sets the rx status of the message
    pub fn rx_status(mut self, rx_status: RxStatus) -> Self {
        self.rx_status = rx_status.bits;
        self
    }

    /// Sets the transmit flags of the message
    pub fn tx_flags(mut self, tx_flags: TxFlags) -> Self {
        self.tx_flags = tx_flags.bits;
        self
    }

    /// Returns true if this is an echo of a message transmitted by
    /// the PassThru device.
    pub fn transmitted(&self) -> bool {
        self.rx_status & (RxStatus::TX_MSG_TYPE).bits() != 0
    }

    /// Returns true if this message indicates the first frame of a ISO15765 packet
    /// has been received.
    pub fn first_frame(&self) -> bool {
        self.rx_status & (RxStatus::ISO15765_FIRST_FRAME).bits() != 0
    }
}

impl Default for PassThruMsg {
    fn default() -> PassThruMsg {
        PassThruMsg {
            protocol_id: 0,
            rx_status: 0,
            tx_flags: 0,
            timestamp: 0,
            data_size: 0,
            extra_data_index: 0,
            data: [0; 4128],
        }
    }
}

impl Debug for PassThruMsg {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        unsafe {
            f.debug_struct("PassThruMsg")
                .field("protocol_id", &self.protocol_id)
                .field("rx_status", &self.rx_status)
                .field("tx_flags", &self.tx_flags)
                .field("timestamp", &self.timestamp)
                .field("extra_data_index", &self.extra_data_index)
                .field("data", &&self.data[..self.data_size as usize])
                .finish()
        }
    }
}

/// Vehicle communication channel ID
#[derive(Copy, Clone, Debug)]
struct ChannelId(u32);

/// Device ID
#[derive(Copy, Clone, Debug)]
struct DeviceId(u32);

/// Periodic message ID
#[derive(Copy, Clone, Debug)]
pub struct MessageId(u32);

/// Message filter ID
#[derive(Copy, Clone, Debug)]
pub struct FilterId(u32);

/// A J2534 library
pub struct Interface {
    library: Library,

    c_pass_thru_open: PassThruOpenFn,
    c_pass_thru_close: PassThruCloseFn,
    c_pass_thru_connect: PassThruConnectFn,
    c_pass_thru_disconnect: PassThruDisconnectFn,
    c_pass_thru_read_version: PassThruReadVersionFn,
    c_pass_thru_get_last_error: PassThruGetLastErrorFn,
    c_pass_thru_read_msgs: PassThruReadMsgsFn,
    c_pass_thru_start_msg_filter: PassThruStartMsgFilterFn,
    c_pass_thru_stop_msg_filter: PassThruStopMsgFilterFn,
    c_pass_thru_write_msgs: PassThruWriteMsgsFn,
    c_pass_thru_start_periodic_msg: PassThruStartPeriodicMsgFn,
    c_pass_thru_stop_periodic_msg: PassThruStopPeriodicMsgFn,
    c_pass_thru_set_programming_voltage: PassThruSetProgrammingVoltageFn,
    c_pass_thru_ioctl: PassThruIoctlFn,

    _marker: PhantomData<*mut ()>,
}

/// A device created with [`Interface::open`]
pub struct Device<'a> {
    interface: &'a Interface,
    id: DeviceId,
}

/// A communication channel
pub struct Channel<'a> {
    device: &'a Device<'a>,
    id: ChannelId,
    protocol: Protocol,
}

impl Interface {
    /// Returns a J2534 library given the path
    ///
    /// # Arguments
    ///
    /// * `path` - The absolute path to the J2534 shared library
    ///
    /// # Example
    /// ```no_run
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// ```
    pub fn new<S: AsRef<OsStr>>(path: S) -> Result<Interface, libloading::Error> {
        let library = Library::new(path)?;

        let interface = unsafe {
            let c_pass_thru_open: Symbol<PassThruOpenFn> = library.get(b"PassThruOpen\0")?;
            let c_pass_thru_close: Symbol<PassThruCloseFn> = library.get(b"PassThruClose\0")?;
            let c_pass_thru_connect: Symbol<PassThruConnectFn> =
                library.get(b"PassThruConnect\0")?;
            let c_pass_thru_disconnect: Symbol<PassThruDisconnectFn> =
                library.get(b"PassThruDisconnect\0")?;
            let c_pass_thru_read_version: Symbol<PassThruReadVersionFn> =
                library.get(b"PassThruReadVersion\0")?;
            let c_pass_thru_get_last_error: Symbol<PassThruGetLastErrorFn> =
                library.get(b"PassThruGetLastError\0")?;
            let c_pass_thru_read_msgs: Symbol<PassThruReadMsgsFn> =
                library.get(b"PassThruReadMsgs\0")?;
            let c_pass_thru_start_msg_filter: Symbol<PassThruStartMsgFilterFn> =
                library.get(b"PassThruStartMsgFilter\0")?;
            let c_pass_thru_stop_msg_filter: Symbol<PassThruStopMsgFilterFn> =
                library.get(b"PassThruStopMsgFilter\0")?;
            let c_pass_thru_write_msgs: Symbol<PassThruWriteMsgsFn> =
                library.get(b"PassThruWriteMsgs\0")?;
            let c_pass_thru_start_periodic_msg: Symbol<PassThruStartPeriodicMsgFn> =
                library.get(b"PassThruStartPeriodicMsg\0")?;
            let c_pass_thru_stop_periodic_msg: Symbol<PassThruStopPeriodicMsgFn> =
                library.get(b"PassThruStopPeriodicMsg\0")?;
            let c_pass_thru_set_programming_voltage: Symbol<PassThruSetProgrammingVoltageFn> =
                library.get(b"PassThruSetProgrammingVoltage\0")?;
            let c_pass_thru_ioctl: Symbol<PassThruIoctlFn> = library.get(b"PassThruIoctl\0")?;
            Interface {
                c_pass_thru_open: *c_pass_thru_open.into_raw(),
                c_pass_thru_close: *c_pass_thru_close.into_raw(),
                c_pass_thru_connect: *c_pass_thru_connect.into_raw(),
                c_pass_thru_disconnect: *c_pass_thru_disconnect.into_raw(),
                c_pass_thru_read_version: *c_pass_thru_read_version.into_raw(),
                c_pass_thru_get_last_error: *c_pass_thru_get_last_error.into_raw(),
                c_pass_thru_read_msgs: *c_pass_thru_read_msgs.into_raw(),
                c_pass_thru_start_msg_filter: *c_pass_thru_start_msg_filter.into_raw(),
                c_pass_thru_stop_msg_filter: *c_pass_thru_stop_msg_filter.into_raw(),
                c_pass_thru_write_msgs: *c_pass_thru_write_msgs.into_raw(),
                c_pass_thru_start_periodic_msg: *c_pass_thru_start_periodic_msg.into_raw(),
                c_pass_thru_stop_periodic_msg: *c_pass_thru_stop_periodic_msg.into_raw(),
                c_pass_thru_set_programming_voltage: *c_pass_thru_set_programming_voltage
                    .into_raw(),
                c_pass_thru_ioctl: *c_pass_thru_ioctl.into_raw(),
                library,

                _marker: PhantomData,
            }
        };

        // OpenPort 2.0 debug mode: unsafe { (&interface.c_pass_thru_ioctl)(0, 0x70001, 0x1 as *mut libc::c_void, std::ptr::null_mut() as *mut libc::c_void); }

        Ok(interface)
    }

    /// Returns a text description of the most recent error
    pub fn get_last_error(&self) -> Result<String, Error> {
        let mut error: [u8; 80] = [0; 80];
        let res =
            unsafe { (&self.c_pass_thru_get_last_error)(error.as_mut_ptr() as *mut libc::c_char) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        unsafe {
            Ok(String::from(
                ffi::CStr::from_ptr(error.as_mut_ptr() as *mut libc::c_char).to_str()?,
            ))
        }
    }

    /// Creates a `Device` from the PassThru interface connected to the given port
    ///
    /// # Arguments
    ///
    /// * `port` - The port to search for a J2534 device
    ///
    /// # Example
    /// ```no_run
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// let device = interface.open("COM2").unwrap();
    /// ```
    pub fn open<S: Into<Vec<u8>>>(&self, port: S) -> Result<Device, Error> {
        let s = ffi::CString::new(port).unwrap();
        let raw = s.as_ptr() as *const libc::c_void;
        let mut id = 0;

        let res = unsafe { (&self.c_pass_thru_open)(raw, &mut id as *mut u32) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {
            interface: self,
            id: DeviceId(id),
        })
    }

    /// Creates a `Device` from any connected PassThru devices
    ///
    /// # Example
    /// ```
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// let device = interface.open_any().unwrap();
    /// ```
    pub fn open_any(&self) -> Result<Device, Error> {
        let raw = std::ptr::null() as *const libc::c_void;
        let mut id = 0;
        let res = unsafe { (&self.c_pass_thru_open)(raw, &mut id as *mut u32) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {
            interface: self,
            id: DeviceId(id),
        })
    }

    /// General purpose I/O control for modifying device or channel characteristics.
    pub unsafe fn ioctl(
        &self,
        handle: u32,
        id: IoctlId,
        input: *mut libc::c_void,
        output: *mut libc::c_void,
    ) -> Result<i32, Error> {
        let res = (&self.c_pass_thru_ioctl)(handle, id as u32, input, output);
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(res)
    }
}

#[derive(Copy, Clone)]
pub enum Protocol {
    J1850VPW = 1,
    J1850PWM = 2,
    ISO9141 = 3,
    ISO14230 = 4,
    CAN = 5,
    ISO15765 = 6,
    SCI_A_ENGINE = 7,
    SCI_A_TRANS = 8,
    SCI_B_ENGINE = 9,
    SCI_B_TRANS = 10,
}

bitflags! {
    /// Flags used when creating a communication channel
    pub struct ConnectFlags: u32 {
        const NONE = 0;
        const CAN_29_BIT_ID = 0x100;
        const ISO9141_NO_CHECKSUM = 0x200;
        const CAN_ID_BOTH = 0x800;
        const ISO9141_K_LINE_ONLY = 0x1000;
    }
}

bitflags! {
    /// Transmit status flags
    pub struct TxFlags: u32 {
        const NONE = 0;
        // 0 = no padding
        // 1 = pad all flow controlled messages to a full CAN frame using zeroes
        const ISO15765_FRAME_PAD = 0x00000040;

        const ISO15765_ADDR_TYPE = 0x00000080;
        const CAN_29BIT_ID =  0x00000100;

        // 0 = Interface message timing as specified in ISO 14230
        // 1 = After a response is received for a physical request, the wait time shall be reduced to P3_MIN
        // Does not affect timing on responses to functional requests
        const WAIT_P3_MIN_ONLY = 0x00000200;

        const SW_CAN_HV_TX = 0x00000400;

        // 0 = Transmit using SCI Full duplex mode
        // 1 = Transmit using SCI Half duplex mode
        const SCI_MODE = 0x00400000;

        // 0 = no voltage after message transmit
        // 1 = apply 20V after message transmit
        const SCI_TX_VOLTAGE = 0x00800000;

        const DT_PERIODIC_UPDATE = 0x10000000;
    }
}

bitflags! {
    /// Receive status flags
    pub struct RxStatus: u32 {
        const NONE = 0;
        // 0 = received
        // 1 = transmitted
        const TX_MSG_TYPE  = 0x00000001;

        // 0 = Not a start of message indication
        // 1 = First byte or frame received
        const START_OF_MESSAGE = 0x00000002;
        const ISO15765_FIRST_FRAME = 0x00000002  ;

        const ISO15765_EXT_ADDR = 0x00000080;

        // 0 = No break received
        // 1 = Break received
        const RX_BREAK = 0x00000004;

        // 0 = No TxDone
        // 1 = TxDone
        const TX_INDICATION   = 0x00000008;
        const TX_DONE = 0x00000008;

        // 0 = No Error
        // 1 = Padding Error
        const ISO15765_PADDING_ERROR = 0x00000010;

        // 0 = no extended address,
        // 1 = extended address is first byte after the CAN ID
        const ISO15765_ADDR_TYPE = 0x00000080;

        const CAN_29BIT_ID = 0x00000100;

        const SW_CAN_NS_RX = 0x00040000;
        const SW_CAN_HS_RX = 0x00020000;
        const SW_CAN_HV_RX = 0x00010000;
    }
}

#[derive(Copy, Clone)]
pub enum IoctlId {
    GET_CONFIG = 0x01,
    SET_CONFIG = 0x02,
    READ_VBATT = 0x03,
    FIVE_BAUD_INIT = 0x04,
    FAST_INIT = 0x05,
    // unused 0x06
    CLEAR_TX_BUFFER = 0x07,
    CLEAR_RX_BUFFER = 0x08,
    CLEAR_PERIODIC_MSGS = 0x09,
    CLEAR_MSG_FILTERS = 0x0A,
    CLEAR_FUNCT_MSG_LOOKUP_TABLE = 0x0B,
    ADD_TO_FUNCT_MSG_LOOKUP_TABLE = 0x0C,
    DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE = 0x0D,
    READ_PROG_VOLTAGE = 0x0E,

    SW_CAN_HS = 0x8000,
    SW_CAN_NS = 0x8001,
    SET_POLL_RESPONSE = 0x8002,
    BECOME_MASTER = 0x8003,
}

#[derive(Copy, Clone)]
/// Channel configuration parameters. Use with [`Channel::get_config`] and [`Channel::set_config`]
pub enum ConfigId {
    DATA_RATE = 0x01,
    LOOPBACK = 0x03,
    NODE_ADDRESS = 0x04,
    NETWORK_LINE = 0x05,
    P1_MIN = 0x06,
    P1_MAX = 0x07,
    P2_MIN = 0x08,
    P2_MAX = 0x09,
    P3_MIN = 0x0A,
    P3_MAX = 0x0B,
    P4_MIN = 0x0C,
    P4_MAX = 0x0D,

    W1 = 0x0E,
    W2 = 0x0F,
    W3 = 0x10,
    W4 = 0x11,
    W5 = 0x12,
    TIDLE = 0x13,
    TINIL = 0x14,
    TWUP = 0x15,
    PARITY = 0x16,
    BIT_SAMPLE_POINT = 0x17,
    SYNC_JUMP_WIDTH = 0x18,
    W0 = 0x19,
    T1_MAX = 0x1A,
    T2_MAX = 0x1B,

    T4_MAX = 0x1C,
    T5_MAX = 0x1D,
    ISO15765_BS = 0x1E,
    ISO15765_STMIN = 0x1F,
    DATA_BITS = 0x20,
    FIVE_BAUD_MOD = 0x21,
    BS_TX = 0x22,
    STMIN_TX = 0x23,
    T3_MAX = 0x24,
    ISO15765_WFT_MAX = 0x25,

    CAN_MIXED_FORMAT = 0x8000,

    J1962_PINS = 0x8001,

    SW_CAN_HS_DATA_RATE = 0x8010,
    SW_CAN_SPEEDCHANGE_ENABLE = 0x8011,
    SW_CAN_RES_SWITCH = 0x8012,
    ACTIVE_CHANNELS = 0x8020,
    SAMPLE_RATE = 0x8021,
    SAMPLES_PER_READING = 0x8022,
    READINGS_PER_MSG = 0x8023,
    AVERAGING_METHOD = 0x8024,
    SAMPLE_RESOLUTION = 0x8025,
    INPUT_RANGE_LOW = 0x8026,
    INPUT_RANGE_HIGH = 0x8027,
}

#[derive(Copy, Clone)]
pub enum FilterType {
    /// Allows matching messages into the receive queue. This filter type is only valid on non-ISO 15765 channels
    Pass = 1,
    /// Keeps matching messages out of the receive queue. This filter type is only valid on non-ISO 15765 channels
    Block = 2,
    /// Allows matching messages into the receive queue and defines an outgoing flow control message to support
    /// the ISO 15765-2 flow control mechanism. This filter type is only valid on ISO 15765 channels.
    FlowControl = 3,
}

#[derive(Debug)]
/// Information about a device's version.
pub struct VersionInfo {
    pub firmware_version: String,
    pub dll_version: String,
    pub api_version: String,
}

pub const SHORT_TO_GROUND: u32 = 0xFFFFFFFE;
pub const VOLTAGE_OFF: u32 = 0xFFFFFFFF;

impl<'a> Device<'a> {
    /// Reads the version info of the device
    pub fn read_version(&self) -> Result<VersionInfo, Error> {
        let mut firmware_version: [u8; 80] = [0; 80];
        let mut dll_version: [u8; 80] = [0; 80];
        let mut api_version: [u8; 80] = [0; 80];
        let res = unsafe {
            (&self.interface.c_pass_thru_read_version)(
                self.id.0,
                firmware_version.as_mut_ptr() as *mut libc::c_char,
                dll_version.as_mut_ptr() as *mut libc::c_char,
                api_version.as_mut_ptr() as *mut libc::c_char,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        unsafe {
            Ok(VersionInfo {
                firmware_version: String::from(
                    ffi::CStr::from_ptr(firmware_version.as_mut_ptr() as *mut libc::c_char)
                        .to_str()?,
                ),
                api_version: String::from(
                    ffi::CStr::from_ptr(api_version.as_mut_ptr() as *mut libc::c_char).to_str()?,
                ),
                dll_version: String::from(
                    ffi::CStr::from_ptr(dll_version.as_mut_ptr() as *mut libc::c_char).to_str()?,
                ),
            })
        }
    }

    /// Outputs a programmable voltage on the specified J1962 connector pin.
    /// Only one pin can have a specified voltage applied at a time. The only exception: it is permissible to program pin 15 for SHORT_TO_GROUND, and another pin to a voltage level.
    /// When switching pins, the user application must disable the first voltage (VOLTAGE_OFF option) before enabling the second.
    ///
    /// # Arguments
    ///
    /// * `pin_number` - The J1962 connector pin to which the PassThru device will apply the specified voltage
    /// * `voltage` - The voltage value (in millivolts) that will be applied to the specified pin
    pub fn set_programming_voltage(&self, pin_number: u32, voltage: u32) -> Result<(), Error> {
        let res = unsafe {
            (&self.interface.c_pass_thru_set_programming_voltage)(self.id.0, pin_number, voltage)
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }

    /// Creates a channel
    ///
    /// # Arguments
    ///
    /// * protocol - Protocol to use with the channel
    /// * flags - Protocol-specific flags. This is usually set to zero
    /// * baudrate - Initial baud rate for the channel
    pub fn connect(
        &self,
        protocol: Protocol,
        flags: ConnectFlags,
        baudrate: u32,
    ) -> Result<Channel, Error> {
        let mut id: u32 = 0;
        let res = unsafe {
            (&self.interface.c_pass_thru_connect)(
                self.id.0,
                protocol as u32,
                flags.bits,
                baudrate,
                &mut id as *mut u32,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(Channel {
            device: self,
            id: ChannelId(id),
            protocol: protocol as Protocol,
        })
    }

    /// Returns the battery voltage in millivolts read from Pin 16 on the J1962 connector.
    pub fn read_battery_voltage(&self) -> Result<u32, Error> {
        let mut voltage: u32 = 0;
        unsafe {
            self.interface.ioctl(
                self.id.0,
                IoctlId::READ_VBATT,
                std::ptr::null_mut::<libc::c_void>(),
                (&mut voltage) as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(voltage)
    }

    /// Returns the current output voltage for ECU reprogramming in millivolts.
    pub fn read_programming_voltage(&self) -> Result<u32, Error> {
        let mut voltage: u32 = 0;
        unsafe {
            self.interface.ioctl(
                self.id.0,
                IoctlId::READ_PROG_VOLTAGE,
                std::ptr::null_mut::<libc::c_void>(),
                (&mut voltage) as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(voltage)
    }
}

impl<'a> Drop for Device<'a> {
    fn drop(&mut self) {
        unsafe { (&self.interface.c_pass_thru_close)(self.id.0) };
    }
}

impl<'a> Channel<'a> {
    /// Fills `msgs` with messages until timing out or until `msgs` is filled. Returns the slice of messages read.
    ///
    /// # Arguments
    ///
    /// * `msgs` - The array of messages to fill.
    /// * `timeout` - The amount of time in milliseconds to wait. If set to zero, reads buffered messages and returns immediately
    ///
    /// Returns the amount of messages read.
    pub fn read(&self, buf: &mut [PassThruMsg], timeout: u32) -> Result<usize, Error> {
        for msg in buf.iter_mut() {
            msg.protocol_id = self.protocol as u32;
        }
        let mut num_msgs: u32 = buf.len() as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_read_msgs)(
                self.id.0,
                buf.as_mut_ptr(),
                &mut num_msgs as *mut u32,
                timeout,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(num_msgs as usize)
    }

    /// Reads a single message
    pub fn read_once(&self, timeout: u32) -> Result<PassThruMsg, Error> {
        let mut msg = PassThruMsg::new(self.protocol);

        let mut num_msgs = 1 as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_read_msgs)(
                self.id.0,
                &mut msg as *mut PassThruMsg,
                &mut num_msgs as *mut u32,
                timeout,
            )
        };
        if num_msgs == 0 {
            // This should never happen, but just in case...
            Err(Error::BufferEmpty)
        } else if res != 0 {
            Err(Error::from_code(res))
        } else {
            Ok(msg)
        }
    }

    /// Writes `msgs` to the device until all messages have been written or until the timeout has been reached. Returns the amount of message written.
    ///
    /// # Arguments
    ///
    /// * msgs - The array of messages to send.
    /// * timeout - The amount of time in milliseconds to wait. If set to zero, queues as many messages as possible and returns immediately.
    pub fn write(&self, msgs: &mut [PassThruMsg], timeout: u32) -> Result<usize, Error> {
        let mut num_msgs: u32 = msgs.len() as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_write_msgs)(
                self.id.0,
                msgs.as_mut_ptr(),
                &mut num_msgs as *mut u32,
                timeout,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(num_msgs as usize)
    }

    /// Sets up a network protocol filter to filter messages received by the PassThru device. There is a limit of ten filters per network layer protocol.
    /// The device blocks all receive frames by default when no filters are defined.
    ///
    /// Returns filter ID
    /// http://www.drewtech.com/support/passthru/startmsgfilter.html
    pub fn start_message_filter(
        &self,
        filter_type: FilterType,
        mask_msg: Option<&PassThruMsg>,
        pattern_msg: Option<&PassThruMsg>,
        flow_control_msg: Option<&PassThruMsg>,
    ) -> Result<FilterId, Error> {
        let mut msg_id: u32 = 0;

        let mask_ptr = match mask_msg {
            Some(msg) => msg as *const PassThruMsg,
            None => std::ptr::null() as *const PassThruMsg,
        };

        let pattern_ptr = match pattern_msg {
            Some(msg) => msg as *const PassThruMsg,
            None => std::ptr::null() as *const PassThruMsg,
        };

        let flow_control_ptr = match flow_control_msg {
            Some(msg) => msg as *const PassThruMsg,
            None => std::ptr::null() as *const PassThruMsg,
        };

        let res = unsafe {
            (&self.device.interface.c_pass_thru_start_msg_filter)(
                self.id.0,
                filter_type as u32,
                mask_ptr,
                pattern_ptr,
                flow_control_ptr,
                &mut msg_id as *mut u32,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(FilterId(msg_id))
    }

    /// Removes a message filter started with `Channel::start_msg_filter`
    ///
    /// # Arguments
    ///
    /// * `msg_id` - The id of the message returned from `Channel::start_msg_filter`
    pub fn stop_message_filter(&self, filter_id: FilterId) -> Result<(), Error> {
        let res =
            unsafe { (&self.device.interface.c_pass_thru_stop_msg_filter)(self.id.0, filter_id.0) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }

    /// Repetitively transmit network protocol messages at the specified time interval over an existing logical communication channel. There is a limit of ten periodic messages per network layer protocol.
    /// Returns a handle for the periodic message used in `Channel::stop_periodic_msg`
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send
    /// * `time_interval` - The time in milliseconds to wait between sending messages. The acceptable range is between 5 and 65,535 milliseconds.
    pub fn start_periodic_message(
        &self,
        msg: &PassThruMsg,
        time_interval: u32,
    ) -> Result<MessageId, Error> {
        let mut msg_id = 0;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_start_periodic_msg)(
                self.id.0,
                msg as *const PassThruMsg,
                &mut msg_id as *mut u32,
                time_interval,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(MessageId(msg_id))
    }

    /// Stops a periodic mesage started with `Channel::start_periodic_msg`
    ///
    /// # Arguments
    ///
    /// * msg_id = the id of the periodic message returned from `Channel::start_periodiC_msg`
    pub fn stop_periodic_message(&self, msg_id: MessageId) -> Result<(), Error> {
        let res =
            unsafe { (&self.device.interface.c_pass_thru_stop_periodic_msg)(self.id.0, msg_id.0) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }

    /// Clear transmit message queue
    pub fn clear_transmit_buffer(&self) -> Result<(), Error> {
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::CLEAR_TX_BUFFER,
                std::ptr::null_mut::<libc::c_void>(),
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(())
    }

    /// Halt continuous messages
    pub fn clear_periodic_messages(&self) -> Result<(), Error> {
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::CLEAR_PERIODIC_MSGS,
                std::ptr::null_mut::<libc::c_void>(),
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(())
    }

    /// Clear receive message queue
    pub fn clear_receive_buffer(&self) -> Result<(), Error> {
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::CLEAR_RX_BUFFER,
                std::ptr::null_mut::<libc::c_void>(),
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(())
    }

    /// Removes all message filters
    pub fn clear_message_filters(&self) -> Result<(), Error> {
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::CLEAR_MSG_FILTERS,
                std::ptr::null_mut::<libc::c_void>(),
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(())
    }

    /// Gets a single configuration parameter.
    pub fn get_config(&self, id: ConfigId) -> Result<u32, Error> {
        let mut item = SConfig {
            parameter: id as u32,
            value: 0,
        };
        let mut input = SConfigList {
            size: 1,
            config_ptr: &mut item as *mut SConfig,
        };
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::GET_CONFIG,
                &mut input as *mut _ as *mut libc::c_void,
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(item.value)
    }

    /// Sets a single configuration parameter.
    pub fn set_config(&self, id: ConfigId, value: u32) -> Result<(), Error> {
        let mut item = SConfig {
            parameter: id as u32,
            value,
        };
        let mut input = SConfigList {
            size: 1,
            config_ptr: &mut item as *mut SConfig,
        };
        unsafe {
            self.device.interface.ioctl(
                self.id.0,
                IoctlId::SET_CONFIG,
                &mut input as *mut _ as *mut libc::c_void,
                std::ptr::null_mut::<libc::c_void>(),
            )
        }?;
        Ok(())
    }
}

impl<'a> Drop for Channel<'a> {
    fn drop(&mut self) {
        unsafe { (&self.device.interface.c_pass_thru_disconnect)(self.id.0) };
    }
}

/// Information about an installed PassThru driver
#[derive(Debug)]
pub struct Driver {
    pub name: String,
    pub vendor: String,
    pub path: String,
}

#[cfg(windows)]
/// Returns a list of all installed PassThru drivers
pub fn drivers() -> io::Result<Vec<Driver>> {
    let passthru = match RegKey::predef(HKEY_LOCAL_MACHINE)
        .open_subkey(Path::new("SOFTWARE").join("PassThruSupport.04.04"))
    {
        Err(err) if err.kind() == io::ErrorKind::NotFound => {
            return Ok(Vec::new());
        }
        other => other,
    }?;
    let mut listings = Vec::new();

    for name in passthru.enum_keys() {
        let name = name?;
        let key = passthru.open_subkey(name)?;

        let device_name: String = key.get_value("Name")?;
        let vendor: String = key.get_value("Vendor")?;
        let path: String = key.get_value("FunctionLibrary")?;

        listings.push(Driver {
            name: device_name,
            vendor,
            path,
        });
    }

    Ok(listings)
}

#[cfg(not(windows))]
/// Returns a list of all installed PassThru drivers
pub fn drivers() -> io::Result<Vec<Driver>> {
    Ok(Vec::new())
}
