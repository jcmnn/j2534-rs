extern crate j2534;

fn main() {
    let mut manager = j2534::Manager::new();

    for device in j2534::list().unwrap().iter() {
        let i = manager.load(&device.path).unwrap();
        println!("opening");
        let d = i.open_any();
        println!("Opened device");
    }
}