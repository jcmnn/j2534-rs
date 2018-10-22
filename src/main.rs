extern crate j2534;

use std::rc::Rc;

fn main() {
    for device in j2534::list().unwrap().iter() {
        println!("Opening interface {}", device.path);
        let i = Rc::new(j2534::Interface::new(&device.path).unwrap());
        println!("Opening device");
        let d = Rc::new(j2534::Device::open_any(i).unwrap());
        println!("Reading version");
        let version_info = d.read_version().unwrap();
        println!("Version info: {:?}", version_info);

        let channel = j2534::Channel::connect(d, j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000).unwrap();

        let mut msgs: [j2534::PassThruMsg; 1] = [j2534::PassThruMsg::default(); 1];
        let read = channel.read_msgs(&mut msgs, 1000);
        for msg in read.iter() {
            println!("Got message");
        }
    }
}