extern crate cc;

fn main() {
    cc::Build::new()
        .file("src/cpp/j2534.cpp")
        .cpp(true)
        .compile("test");
}