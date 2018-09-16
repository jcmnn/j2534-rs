extern crate winreg;
extern crate libc;
extern crate libloading as libl;

use std::io;
use std::ffi;
use winreg::RegKey;
use winreg::enums::*;

#[repr(C)]
pub struct PASSTHRU_MSG {
    pub ProtocolID: u32,
    pub RxStatus: u32,
    pub TxFlags: u32,
    pub Timestamp: u32,
    pub DataSize: u32,
    pub ExtraDataIndex: u32,
    pub Data: [u8; 4128],
}


pub struct Manager {
    loaded_libraries: Vec<libl::Library>,
}

struct Api<'a> {
    PassThruOpen: libl::Symbol<'a, unsafe extern fn(*mut libc::c_void, *mut libc::uint32_t) -> libc::int32_t>,
    PassThruClose: libl::Symbol<'a, unsafe extern fn(u32) -> i32>,
    PassThruConnect: libl::Symbol<'a, unsafe extern fn(u32, u32, u32, u32, *mut u32) -> i32>,
    PassThruDisconnect: libl::Symbol<'a, unsafe extern fn(u32) -> i32>,
    PassThruReadMsgs: libl::Symbol<'a, unsafe extern fn(u32, *mut PASSTHRU_MSG, *mut u32, u32) -> i32>,
    PassThruWriteMsgs: libl::Symbol<'a, unsafe extern fn(u32, *mut PASSTHRU_MSG, *mut u32, u32) -> i32>,
    PassThruStartPeriodicMsg: libl::Symbol<'a, unsafe extern fn(u32, *const PASSTHRU_MSG, *mut u32, u32) -> i32>,
    PassThruStopPeriodicMsg: libl::Symbol<'a, unsafe extern fn(u32, u32) -> i32>,
    PassThruStartMsgFilter: libl::Symbol<'a, unsafe extern fn(u32, u32, *const PASSTHRU_MSG, *const PASSTHRU_MSG, *const PASSTHRU_MSG, *mut u32) -> i32>,
    PassThruStopMsgFilter: libl::Symbol<'a, unsafe extern fn(u32, u32) -> i32>,
    PassThruSetProgrammingVoltage: libl::Symbol<'a, unsafe extern fn(u32, u32) -> i32>,
    PassThruReadVersion: libl::Symbol<'a, unsafe extern fn(*mut libc::c_char, *mut libc::c_char, *mut libc::c_char) -> i32>,
    PassThruGetLastError: libl::Symbol<'a, unsafe extern fn(*mut libc::c_char) -> i32>,
    PassThruIoctl: libl::Symbol<'a, unsafe extern fn(u32, u32, *mut libc::c_void, *mut libc::c_void) -> i32>,
}

pub struct Interface<'a> {
    api: Api<'a>,
}

pub struct Device {
    id: u32,
}

impl<'a> Interface<'a> {
    pub fn open(&self, port: &str) -> Device {
        let s = ffi::CString::new(port).unwrap();
        let raw = s.into_raw();
        let mut id = 0;
        unsafe {
            let res = (self.api.PassThruOpen)(raw as *mut libc::c_void, id as *mut libc::uint32_t);
            println!("res: {}", res);
        }
        Device {id}
    }

    pub fn open_any(&self) -> Device {
        let mut id :u32 = 0;
        unsafe {
            println!("Running open");
            let v: *const libc::c_char = std::ptr::null();
            let res = (self.api.PassThruOpen)(v as *mut libc::c_void, id as *mut libc::uint32_t);
            println!("res: {}", res);
        }
        println!("id: {}", id);
        Device {id}
    }
}


impl Manager {
    pub fn new() -> Manager {
        Manager{ loaded_libraries: Vec::new(), }
    }

    pub fn load(&mut self, path: &str) -> io::Result<Interface> {
        let library = libl::Library::new(path)?;
        self.loaded_libraries.push(library);
        let library = self.loaded_libraries.last().unwrap();
        unsafe {
            let mut iface = Interface {
                api: Api {
                    PassThruOpen: library.get(b"PassThruOpen")?,
                    PassThruClose: library.get(b"PassThruClose")?,
                    PassThruConnect: library.get(b"PassThruConnect")?,
                    PassThruDisconnect: library.get(b"PassThruDisconnect")?,
                    PassThruReadMsgs: library.get(b"PassThruReadMsgs")?,
                    PassThruWriteMsgs: library.get(b"PassThruWriteMsgs")?,
                    PassThruStartPeriodicMsg: library.get(b"PassThruStartPeriodicMsg")?,
                    PassThruStopPeriodicMsg: library.get(b"PassThruStopPeriodicMsg")?,
                    PassThruStartMsgFilter: library.get(b"PassThruStartMsgFilter")?,
                    PassThruStopMsgFilter: library.get(b"PassThruStopMsgFilter")?,
                    PassThruSetProgrammingVoltage: library.get(b"PassThruSetProgrammingVoltage")?,
                    PassThruReadVersion: library.get(b"PassThruReadVersion")?,
                    PassThruGetLastError: library.get(b"PassThruGetLastError")?,
                    PassThruIoctl: library.get(b"PassThruIoctl")?,
                }
            };
            Ok(iface)
        }
    }
}


#[derive(Debug)]
pub struct Listing {
    pub name: String,
    pub vendor: String,
    pub path: String,
}

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