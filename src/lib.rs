pub mod jtag;
pub mod rawio;
pub mod swd;

pub mod  error{
    pub enum Error {
        PinState,
        Other
    }
}