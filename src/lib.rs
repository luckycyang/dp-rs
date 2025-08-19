pub mod jtag;
pub mod rawio;
pub mod swd;

pub mod error {
    #[derive(Debug)]
    pub enum Error {
        PinState,
        Other,
    }

    impl core::fmt::Display for Error {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "")
        }
    }

    impl core::error::Error for Error {}
}

