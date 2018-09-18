extern crate j2534;


fn main() {
    for device in j2534::list().unwrap().iter() {
        println!("Opening interface {}", device.path);
        let i = j2534::Interface::new(&device.path).unwrap();
        println!("Opening device");
        let d = i.open_any().unwrap();
        println!("Reading version");
        let version_info = d.read_version().unwrap();
        println!("Version info: {:?}", version_info);
    }
}