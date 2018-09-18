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

        let channel = d.connect(j2534::Protocol::CAN, j2534::ConnectFlags::NONE, 500000).unwrap();

        let mut msgs: [j2534::PassThruMsg; 1] = [j2534::PassThruMsg::default(); 1];
        let read = channel.read_msgs(&mut msgs, 1000);
        for msg in read.iter() {
            println!("Got message");
        }
    }
}