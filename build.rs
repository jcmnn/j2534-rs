extern crate cc;
#[cfg(windows)]
fn main() {
    cc::Build::new()
        .file("src/cpp/j2534.cpp")
        .cpp(true)
        .compile("test");
}

#[cfg(not(windows))]
fn main () {

}