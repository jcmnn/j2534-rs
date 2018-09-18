extern crate j2534;


fn main() {
    for device in j2534::list().unwrap().iter() {
        let i = j2534::Interface::new(&device.path).unwrap();
        let d = i.open_any().unwrap();
        let version_info = d.read_version().unwrap();
        println!("Version info: {:?}", version_info);
    }
}