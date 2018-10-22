extern crate winreg;
extern crate libc;
#[macro_use]
extern crate bitflags;

use std::ffi;
use std::io;
use std::fmt;
use std::error;
use std::rc;
use std::str::Utf8Error;
use winreg::RegKey;
use winreg::enums::*;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, Clone)]
pub struct Error {
    kind: ErrorKind,
}

#[derive(Copy, Clone, Debug)]
pub enum ErrorKind {
    NotFound,
    Code(i32),
    Utf8,
}

impl Error {
    pub fn kind(&self) -> ErrorKind {
        self.kind
    }

    pub fn from_code(code: i32) -> Error {
        Error { kind: ErrorKind::Code(code) }
    }

    fn as_str(&self) -> &str {
        match self.kind {
            ErrorKind::NotFound => "not found",
            ErrorKind::Code(code) => match code {
                _ => "unknown error",
            },
            ErrorKind::Utf8 => "utf8 error",
        }
    }
}

impl From<Utf8Error> for Error {
    fn from(_err: Utf8Error) -> Self {
        Error {
            kind: ErrorKind::Utf8,
        }
    }
}

impl error::Error for Error {
    fn description(&self) -> &str {
        self.as_str()
    }

    fn cause(&self) -> Option<&error::Error> {
        None
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

extern {
    fn j2534_load(path: *const libc::c_char) -> *mut libc::c_void;
    fn j2534_close(handle: *const libc::c_void);
    fn j2534_PassThruClose(handle: *const libc::c_void, device_id: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruOpen(handle: *const libc::c_void, port: *const libc::c_char, device_id: *mut libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruConnect(handle: *const libc::c_void, device_id: libc::uint32_t, protocol_id: libc::uint32_t, flags: libc::uint32_t, baudrate: libc::uint32_t, channel_id: *mut libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruDisconnect(handle: *const libc::c_void, channel_id: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruReadMsgs(handle: *const libc::c_void, channel_id: libc::uint32_t, msgs: *mut PassThruMsg, num_msgs: *mut libc::uint32_t, timeout: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruWriteMsgs(handle: *const libc::c_void, channel_id: libc::uint32_t, msgs: *mut PassThruMsg, num_msgs: *mut libc::uint32_t, timeout: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruStartPeriodicMsg(handle: *const libc::c_void, channel_id: libc::uint32_t, msg: *const PassThruMsg, msg_id: *mut libc::uint32_t, time_interval: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruStopPeriodicMsg(handle: *const libc::c_void, channel_id: libc::uint32_t, msg_id: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruStartMsgFilter(handle: *const libc::c_void, channel_id: libc::uint32_t, filter_type: libc::uint32_t, msg_mask: *const PassThruMsg, pattern_msg: *const PassThruMsg, flow_control_msg: *const PassThruMsg, filter_id: *mut libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruStopMsgFilter(handle: *const libc::c_void, channel_id: libc::uint32_t, filter_id: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruSetProgrammingVoltage(handle: *const libc::c_void, device_id: libc::uint32_t, pin_number: libc::uint32_t, voltage: libc::uint32_t) -> libc::int32_t;
    fn j2534_PassThruReadVersion(handle: *const libc::c_void, device_id: libc::uint32_t, firmware_version: *mut libc::c_char, dll_version: *mut libc::c_char, api_version: *mut libc::c_char) -> libc::int32_t;
    fn j2534_PassThruGetLastError(handle: *const libc::c_void, error_description: *mut libc::c_char) -> libc::int32_t;
    fn j2534_PassThruIoctl(handle: *const libc::c_void, handle_id: libc::uint32_t, ioctl_id: libc::uint32_t, input: *mut libc::c_void, output: *mut libc::c_void) -> libc::int32_t;
}

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
    pub fn new_raw(protocol: Protocol, rx_status: u32, tx_flags: u32, timestamp: u32, data_size: u32, extra_data_index: u32, data: [u8; 4128]) -> PassThruMsg {
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

    pub fn new(protocol: Protocol, rx_status: u32, tx_flags: u32, timestamp: u32, data_size: u32, extra_data_index: u32, data: &[u8]) -> PassThruMsg {
        PassThruMsg::new_raw(protocol, rx_status, tx_flags, timestamp, data.len() as u32, extra_data_index, {
                let mut d: [u8; 4128] = [0; 4128];
                d[..data.len()].copy_from_slice(&data);
                d })
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

/// Represents a J2534 library
pub struct Interface {
    handle: *const libc::c_void,
}

/// Represents a J2534 device created with `Interface::open`
pub struct Device {
    interface: Rc<Interface>,
    id: u32,
}

/// Represents a J2534 channel
pub struct Channel {
    device: Rc<Device>,
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
    /// ```
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// ```
    pub fn new(path: &str) -> Result<Interface> {
        let cstring  = ffi::CString::new(path).unwrap();
        let handle = unsafe { j2534_load(cstring.as_ptr()) };
        if handle.is_null() {
            return Err(Error{kind: ErrorKind::NotFound});
        }
        Ok(Interface{handle})
    }

    /// Creates a `Device` from the J2534 connected to the given port
    /// 
    /// # Arguments
    /// 
    /// * `port` - The port to search for a J2534 device
    /// 
    /// # Example
    /// ```
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// let device = interface.open("COM2").unwrap();
    /// ```
    pub fn open(&self, port: &str) -> Result<Device> {
        let s = ffi::CString::new(port).unwrap();
        let raw = s.as_ptr();
        let mut id = 0;
        
        let res = unsafe { j2534_PassThruOpen(self.handle, raw, &mut id as *mut libc::uint32_t) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {interface: self, id})
    }

    /// Creates a `Device` from any connected J2534 devices
    /// 
    /// # Example
    /// ```
    /// use j2534::Interface;
    /// let interface = Interface::new("C:\\j2534_driver.dll").unwrap();
    /// let device = interface.open_any().unwrap();
    /// ```
    pub fn open_any(&self) -> Result<Device> {
        let raw = 0 as *const libc::c_void;
        let mut id = 0;
        let res = unsafe { j2534_PassThruOpen(self.handle, raw as *const libc::c_char, &mut id as *mut libc::uint32_t) };
        if res != 0 {
            return Err(Error::from_code(res));
        }

        Ok(Device {interface: self, id})
    }

    /// Returns a text description of the most recent error
    pub fn get_last_error(&self) -> Result<String> {
        let mut error: [u8; 80] = [0; 80];
        let res = unsafe { j2534_PassThruGetLastError(self.handle, error.as_mut_ptr() as *mut libc::c_char) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        
        unsafe { Ok(String::from(ffi::CStr::from_ptr(error.as_mut_ptr() as *mut libc::c_char).to_str()?)) }
    }
}

impl Drop for Interface {
    fn drop(&mut self) {
        unsafe { j2534_close(self.handle) };
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

pub const SHORT_TO_GROUND: u32= 0xFFFFFFFE;
pub const VOLTAGE_OFF: u32 = 0xFFFFFFFF;

impl<'a> Device<'a> {
    /// See `Device::connect`
    pub fn connect_raw(&self, protocol: u32, flags: u32, baudrate: u32) -> Result<Channel> {
        let mut id: u32 = 0;
        let res = unsafe { j2534_PassThruConnect(self.interface.handle, self.id, protocol, flags, baudrate, &mut id as *mut libc::uint32_t) };
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
    pub fn connect(&self, protocol: Protocol, flags: ConnectFlags, baudrate: u32) -> Result<Channel> {
        self.connect_raw(protocol as u32, flags.bits(), baudrate)
    }

    /// Reads the version info of the device
    pub fn read_version(&self) -> Result<VersionInfo> {
        let mut firmware_version: [u8; 80] = [0; 80];
        let mut dll_version: [u8; 80] = [0; 80];
        let mut api_version: [u8; 80] = [0; 80];
        let res = unsafe { j2534_PassThruReadVersion(self.interface.handle, self.id, firmware_version.as_mut_ptr() as *mut libc::c_char, dll_version.as_mut_ptr() as *mut libc::c_char, api_version.as_mut_ptr() as *mut libc::c_char) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        unsafe {
            Ok(VersionInfo {
                firmware_version: String::from(ffi::CStr::from_ptr(firmware_version.as_mut_ptr() as *mut libc::c_char).to_str()?),
                api_version: String::from(ffi::CStr::from_ptr(api_version.as_mut_ptr() as *mut libc::c_char).to_str()?),
                dll_version: String::from(ffi::CStr::from_ptr(dll_version.as_mut_ptr() as *mut libc::c_char).to_str()?),
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
        let res = unsafe { j2534_PassThruSetProgrammingVoltage(self.interface.handle, self.id, pin_number, voltage) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }
}

impl<'a> Drop for Device<'a> {
    fn drop(&mut self) {
        unsafe { j2534_PassThruClose(self.interface.handle, self.id) };
    }
}

impl<'a> Channel<'a> {
    /// Fills `msgs` with messages until timing out or until `msgs` is filled. Returns the slice of messages read.
    /// 
    /// # Arguments
    /// 
    /// * msgs - The array of messages to fill.
    /// * timeout - The amount of time in milliseconds to wait. If set to zero, reads buffered messages and returns immediately
    pub fn read_msgs<'b>(&self, msgs: &'b mut [PassThruMsg], timeout: u32) -> Result<&'b [PassThruMsg]> {
        for msg in msgs.iter_mut() {
            msg.protocol_id = self.protocol_id;
        }
        let mut num_msgs: u32 = msgs.len() as u32;
        let res = unsafe { j2534_PassThruReadMsgs(self.device.interface.handle, self.id, msgs.as_mut_ptr(), &mut num_msgs as *mut libc::uint32_t, timeout) };
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
        let res = unsafe { j2534_PassThruReadMsgs(self.device.interface.handle, self.id, &mut msg as *mut PassThruMsg, &mut num_msgs as *mut libc::uint32_t, timeout) };
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
        let res = unsafe { j2534_PassThruWriteMsgs(self.device.interface.handle, self.id, msgs.as_mut_ptr(), &mut num_msgs as *mut libc::uint32_t, timeout) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(num_msgs as usize)
    }

    /// Sets up a network protocol filter to filter messages received by the PassThru device. There is a limit of ten filters per network layer protocol.
    /// The device blocks all receive frames by default when no filters are defined.
    /// http://www.drewtech.com/support/passthru/startmsgfilter.html
    pub fn start_msg_filter(&self, filter_type: FilterType, mask_msg: Option<&PassThruMsg>, pattern_msg: Option<&PassThruMsg>, flow_control_msg: Option<&PassThruMsg>) -> Result<u32> {
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

        let res = unsafe { j2534_PassThruStartMsgFilter(self.device.interface.handle, self.id, filter_type as u32, mask_ptr, pattern_ptr, flow_control_ptr, &mut msg_id as *mut libc::uint32_t) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(msg_id)
    }

    /// Removes a message filter started with `Channel::start_msg_filter`
    /// 
    /// # Arguments
    /// 
    /// * `msg_id` - The id of the message returned from `Channel::start_msg_filter`
    pub fn stop_msg_filter(&self, msg_id: u32) -> Result<()> {
        let res = unsafe { j2534_PassThruStopMsgFilter(self.device.interface.handle, self.id, msg_id) };
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
    pub fn start_periodic_msg(&self, msg: &PassThruMsg, time_interval: u32) -> Result<u32> {
        let mut msg_id = 0;
        let res = unsafe { j2534_PassThruStartPeriodicMsg(self.device.interface.handle, self.id, msg as *const PassThruMsg, &mut msg_id as *mut libc::uint32_t, time_interval) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(msg_id)
    }

    /// Stops a periodic mesage started with `Channel::start_periodic_msg`
    /// 
    /// # Arguments
    /// 
    /// * msg_id = the id of the periodic message returned from `Channel::start_periodiC_msg`
    pub fn stop_periodic_msg(&self, msg_id: u32) -> Result<()> {
        let res = unsafe { j2534_PassThruStopPeriodicMsg(self.device.interface.handle, self.id, msg_id) };
        if res != 0 {
            return Err(Error::from_code(res));
        }
        Ok(())
    }
}

impl<'a> Drop for Channel<'a> {
    fn drop(&mut self) {
        unsafe { j2534_PassThruDisconnect(self.device.interface.handle, self.id) };
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

    let software = hklm.open_subkey("SOFTWARE")?;
    let passthru = software.open_subkey("PassThruSupport.04.04");
    if let Err(err) = passthru {
        if err.kind() == io::ErrorKind::NotFound {
            return Ok(Vec::new());
        }
        return Err(err);
    }
    let passthru = passthru.unwrap();
    let mut listings = Vec::new();

    for name in passthru.enum_keys() {
        let name = name?;
        let key = passthru.open_subkey(name)?;

        let device_name: String = key.get_value("Name")?;
        let vendor: String = key.get_value("Vendor")?;
        let path: String = key.get_value("FunctionLibrary")?;

        listings.push(Listing {name: device_name, vendor: vendor, path: path});
    }

    Ok(listings)
}