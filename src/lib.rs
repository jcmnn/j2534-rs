//! J2534 PassThru defines a standard library interface for communicating with vehicles.
//! All automakers in the US are required to provide J2534-compatible service software.
//! J2534 provides access to the communication layer required for accessing vehicle diagnostics services as
//! well as downloading and reflashing control modules.

#![cfg(windows)]

#[macro_use]
extern crate bitflags;

use std::error;
use std::ffi;
use std::ffi::OsStr;
use std::fmt;
use std::fmt::{Debug, Display};
use std::io;
use std::path::Path;
use std::str::Utf8Error;

use bitflags::_core::fmt::Formatter;
use libloading::{Library, Symbol};
use winreg::enums::*;
use winreg::RegKey;

pub type Result<T> = std::result::Result<T, Error>;

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
    // Could not read any messages from the vehicle network
    #[error("could not read messages")]
    BufferEmpty,
    #[error("transmit queue full")]
    BufferFull,
    // Receive buffer overflowed and messages were lost
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

type PassThruOpenFn = unsafe extern "system" fn(
    name: *const libc::c_void,
    device_id: *mut libc::uint32_t,
) -> libc::int32_t;
type PassThruCloseFn = unsafe extern "system" fn(device_id: libc::uint32_t) -> libc::int32_t;
type PassThruConnectFn = unsafe extern "system" fn(
    device_id: libc::uint32_t,
    protocol_id: libc::uint32_t,
    flags: libc::uint32_t,
    baudrate: libc::uint32_t,
    channel_id: *mut libc::uint32_t,
) -> libc::int32_t;
type PassThruDisconnectFn = unsafe extern "system" fn(channel_id: libc::uint32_t) -> libc::int32_t;
type PassThruReadMsgsFn = unsafe extern "system" fn(
    channel_id: libc::uint32_t,
    msgs: *mut PassThruMsg,
    num_msgs: *mut libc::uint32_t,
    timeout: libc::uint32_t,
) -> libc::int32_t;
type PassThruWriteMsgsFn = unsafe extern "system" fn(
    channel_id: libc::uint32_t,
    msgs: *mut PassThruMsg,
    num_msgs: *mut libc::uint32_t,
    timeout: libc::uint32_t,
) -> libc::int32_t;
type PassThruStartPeriodicMsgFn = unsafe extern "system" fn(
    channel_id: libc::uint32_t,
    msg: *const PassThruMsg,
    msg_id: *mut libc::uint32_t,
    time_interval: libc::uint32_t,
) -> libc::int32_t;
type PassThruStopPeriodicMsgFn =
unsafe extern "system" fn(channel_id: libc::uint32_t, msg_id: libc::uint32_t) -> libc::int32_t;
type PassThruStartMsgFilterFn = unsafe extern "system" fn(
    channel_id: libc::uint32_t,
    filter_type: libc::uint32_t,
    msg_mask: *const PassThruMsg,
    pattern_msg: *const PassThruMsg,
    flow_control_msg: *const PassThruMsg,
    filter_id: *mut libc::uint32_t,
) -> libc::int32_t;
type PassThruStopMsgFilterFn = unsafe extern "system" fn(
    channel_id: libc::uint32_t,
    filter_id: libc::uint32_t,
) -> libc::int32_t;
type PassThruSetProgrammingVoltageFn = unsafe extern "system" fn(
    device_id: libc::uint32_t,
    pin_number: libc::uint32_t,
    voltage: libc::uint32_t,
) -> libc::int32_t;
type PassThruReadVersionFn = unsafe extern "system" fn(
    device_id: libc::uint32_t,
    firmware_version: *mut libc::c_char,
    dll_version: *mut libc::c_char,
    api_version: *mut libc::c_char,
) -> libc::int32_t;
type PassThruGetLastErrorFn =
unsafe extern "system" fn(error_description: *mut libc::c_char) -> libc::int32_t;
type PassThruIoctlFn = unsafe extern "system" fn(
    handle_id: libc::uint32_t,
    ioctl_id: libc::uint32_t,
    input: *mut libc::c_void,
    output: *mut libc::c_void,
) -> libc::int32_t;

// Much of the descriptions and APIs used here were taken from http://www.drewtech.com/support/passthru.html

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PassThruMsg {
    pub protocol_id: u32,
    pub rx_status: u32,
    pub tx_flags: u32,
    pub timestamp: u32,
    pub data_size: u32,
    pub extra_data_index: u32,
    pub data: [u8; 4128],
}

impl PassThruMsg {
    pub fn new_raw(
        protocol: Protocol,
        rx_status: u32,
        tx_flags: u32,
        timestamp: u32,
        data_size: u32,
        extra_data_index: u32,
        data: [u8; 4128],
    ) -> PassThruMsg {
        PassThruMsg {
            protocol_id: protocol as u32,
            rx_status,
            tx_flags,
            timestamp,
            data_size,
            extra_data_index,
            data,
        }
    }

    pub fn new(
        protocol: Protocol,
        rx_status: u32,
        tx_flags: u32,
        timestamp: u32,
        extra_data_index: u32,
        data: &[u8],
    ) -> PassThruMsg {
        PassThruMsg::new_raw(
            protocol,
            rx_status,
            tx_flags,
            timestamp,
            data.len() as u32,
            extra_data_index,
            {
                let mut d: [u8; 4128] = [0; 4128];
                d[..data.len()].copy_from_slice(&data);
                d
            },
        )
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

/// Vehicle communication channel ID
#[derive(Copy, Clone, Debug)]
pub struct ChannelId(u32);

/// Period message ID
#[derive(Copy, Clone, Debug)]
pub struct MessageId(u32);

/// Message filter ID
#[derive(Copy, Clone, Debug)]
pub struct FilterId(u32);

/// Represents a J2534 library
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
}

/// Represents a J2534 device created with [`Interface::open`]
pub struct Device<'a> {
    interface: &'a Interface,
    id: u32,
}

/// Represents a J2534 channel
pub struct Channel<'a> {
    device: &'a Device<'a>,
    id: u32,
    protocol_id: u32,
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
    pub fn new<S: AsRef<OsStr>>(path: S) -> libloading::Result<Interface> {
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
            }
        };

        // OpenPort 2.0 debug mode: unsafe { (&interface.c_pass_thru_ioctl)(0, 0x70001, 0x1 as *mut libc::c_void, std::ptr::null_mut() as *mut libc::c_void); }

        Ok(interface)
    }

    /// Returns a text description of the most recent error
    pub fn get_last_error(&self) -> Result<String> {
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
    pub fn open<S: Into<Vec<u8>>>(&self, port: S) -> Result<Device> {
        let s = ffi::CString::new(port).unwrap();
        let raw = s.as_ptr() as *const libc::c_void;
        let mut id = 0;

        let res = unsafe { (&self.c_pass_thru_open)(raw, &mut id as *mut libc::uint32_t) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {
            interface: self,
            id,
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
    pub fn open_any(&self) -> Result<Device> {
        let raw = std::ptr::null() as *const libc::c_void;
        let mut id = 0;
        let res = unsafe { (&self.c_pass_thru_open)(raw, &mut id as *mut libc::uint32_t) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {
            interface: self,
            id,
        })
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
    pub struct ConnectFlags: u32 {
        const NONE = 0;
        const CAN_29_BIT_ID = 0x100;
        const ISO9141_NO_CHECKSUM = 0x200;
        const CAN_ID_BOTH = 0x800;
        const ISO9141_K_LINE_ONLY = 0x1000;
    }
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
pub struct VersionInfo {
    pub firmware_version: String,
    pub dll_version: String,
    pub api_version: String,
}

pub const SHORT_TO_GROUND: u32 = 0xFFFFFFFE;
pub const VOLTAGE_OFF: u32 = 0xFFFFFFFF;

impl<'a> Device<'a> {
    /// Reads the version info of the device
    pub fn read_version(&self) -> Result<VersionInfo> {
        let mut firmware_version: [u8; 80] = [0; 80];
        let mut dll_version: [u8; 80] = [0; 80];
        let mut api_version: [u8; 80] = [0; 80];
        let res = unsafe {
            (&self.interface.c_pass_thru_read_version)(
                self.id,
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
    pub fn set_programming_voltage(&self, pin_number: u32, voltage: u32) -> Result<()> {
        let res = unsafe {
            (&self.interface.c_pass_thru_set_programming_voltage)(self.id, pin_number, voltage)
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }

    /// See `Channel::connect`
    pub fn connect_raw(&self, protocol: u32, flags: u32, baudrate: u32) -> Result<Channel> {
        let mut id: u32 = 0;
        let res = unsafe {
            (&self.interface.c_pass_thru_connect)(
                self.id,
                protocol,
                flags,
                baudrate,
                &mut id as *mut libc::uint32_t,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(Channel {
            device: self,
            id,
            protocol_id: protocol,
        })
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
    ) -> Result<Channel> {
        self.connect_raw(protocol as u32, flags.bits(), baudrate)
    }
}

impl<'a> Drop for Device<'a> {
    fn drop(&mut self) {
        unsafe { (&self.interface.c_pass_thru_disconnect)(self.id) };
    }
}

impl<'a> Channel<'a> {
    /// Fills `msgs` with messages until timing out or until `msgs` is filled. Returns the slice of messages read.
    ///
    /// # Arguments
    ///
    /// * `msgs` - The array of messages to fill.
    /// * `timeout` - The amount of time in milliseconds to wait. If set to zero, reads buffered messages and returns immediately
    pub fn read_msgs<'b>(
        &self,
        msgs: &'b mut [PassThruMsg],
        timeout: u32,
    ) -> Result<&'b [PassThruMsg]> {
        for msg in msgs.iter_mut() {
            msg.protocol_id = self.protocol_id;
        }
        let mut num_msgs: u32 = msgs.len() as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_read_msgs)(
                self.id,
                msgs.as_mut_ptr(),
                &mut num_msgs as *mut libc::uint32_t,
                timeout,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(&msgs[..(num_msgs as usize)])
    }

    /// Reads a single message
    pub fn read_msg(&self, timeout: u32) -> Result<PassThruMsg> {
        let mut msg = PassThruMsg {
            protocol_id: self.protocol_id,
            ..Default::default()
        };

        let mut num_msgs = 1 as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_read_msgs)(
                self.id,
                &mut msg as *mut PassThruMsg,
                &mut num_msgs as *mut libc::uint32_t,
                timeout,
            )
        };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(msg)
    }

    /// Writes `msgs` to the device until all messages have been written or until the timeout has been reached. Returns the amount of message written.
    ///
    /// # Arguments
    ///
    /// * msgs - The array of messages to send.
    /// * timeout - The amount of time in milliseconds to wait. If set to zero, queues as many messages as possible and returns immediately.
    pub fn write_msgs(&self, msgs: &mut [PassThruMsg], timeout: u32) -> Result<usize> {
        for msg in msgs.iter_mut() {
            msg.protocol_id = self.protocol_id;
        }
        let mut num_msgs: u32 = msgs.len() as u32;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_write_msgs)(
                self.id,
                msgs.as_mut_ptr(),
                &mut num_msgs as *mut libc::uint32_t,
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
    pub fn start_msg_filter(
        &self,
        filter_type: FilterType,
        mask_msg: Option<&PassThruMsg>,
        pattern_msg: Option<&PassThruMsg>,
        flow_control_msg: Option<&PassThruMsg>,
    ) -> Result<FilterId> {
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
                self.id,
                filter_type as u32,
                mask_ptr,
                pattern_ptr,
                flow_control_ptr,
                &mut msg_id as *mut libc::uint32_t,
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
    pub fn stop_msg_filter(&self, filter_id: FilterId) -> Result<()> {
        let res =
            unsafe { (&self.device.interface.c_pass_thru_stop_msg_filter)(self.id, filter_id.0) };
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
    pub fn start_periodic_msg(&self, msg: &PassThruMsg, time_interval: u32) -> Result<MessageId> {
        let mut msg_id = 0;
        let res = unsafe {
            (&self.device.interface.c_pass_thru_start_periodic_msg)(
                self.id,
                msg as *const PassThruMsg,
                &mut msg_id as *mut libc::uint32_t,
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
    pub fn stop_periodic_msg(&self, msg_id: MessageId) -> Result<()> {
        let res =
            unsafe { (&self.device.interface.c_pass_thru_stop_periodic_msg)(self.id, msg_id.0) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }
}

impl<'a> Drop for Channel<'a> {
    fn drop(&mut self) {
        unsafe { (&self.device.interface.c_pass_thru_disconnect)(self.id) };
    }
}

#[derive(Debug)]
pub struct Listing {
    pub name: String,
    pub vendor: String,
    pub path: String,
}

/// Returns a list of all installed PassThru drivers
pub fn list() -> io::Result<Vec<Listing>> {
    let hklm = RegKey::predef(HKEY_LOCAL_MACHINE);

    let passthru = match hklm.open_subkey(Path::new("SOFTWARE").join("PassThruSupport.04.04")) {
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

        listings.push(Listing {
            name: device_name,
            vendor,
            path,
        });
    }

    Ok(listings)
}
