// 我推荐 TMS 使用两个管脚, 接法就是如下接法，然后用结构体包装一下实现
// embedded_hal::digital::{OutputPin, InputPin} 就行了
//                  ┌─────────┐
// Output  ─────────┼         ┼──────┐
//                  └─────────┘      │
//                                   │
//                                   │
//                                   │
// Input   ──────────────────────────┴────────────────────   TMS/SDIO

use bitvec::vec::BitVec;
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

use crate::{
    error,
    jtag::{JtagDriverState, RawJtagIo},
};

#[derive(Clone, Copy)]
pub struct Clock {
    tms: bool,
    tdi: bool,
    capture: bool,
}

// 裸 IO 结构体, 如果你只需要 swd, TDI 和 TDO 可以自己包装一个结构体欺骗
pub struct Adapter<TMS, TCK, TDI, TDO, D> {
    tms: TMS,
    tck: TCK,
    tdi: TDI,
    tdo: TDO,
    delay: D,
    delay_ns: u32,
    // 下面就是一些适配器相关的储存了
    clocks: Vec<Clock>,
    state: JtagDriverState,
    bits: BitVec,
}

impl<TMS: OutputPin, TCK: OutputPin, TDI: OutputPin, TDO: InputPin, D: DelayNs>
    Adapter<TMS, TCK, TDI, TDO, D>
{
    pub fn new(tms: TMS, tck: TCK, tdi: TDI, tdo: TDO, delay: D) -> Self {
        // 默认 15Mhz 左右
        Self {
            tms,
            tck,
            tdi,
            tdo,
            delay,
            delay_ns: 33,
            clocks: Vec::new(),
            state: Default::default(),
            bits: BitVec::new(),
        }
    }

    // 将时钟缓存全部推出
    pub fn flush(&mut self) -> Result<(), error::Error> {
        for clock in &self.clocks {
            println!(
                "shift bits tms: {}, tdi: {}, captures: {}",
                clock.tms, clock.tdi, clock.capture
            );
            self.tck.set_low().unwrap();
            self.tms.set_state(clock.tms.into()).unwrap();
            self.tdi.set_state(clock.tdi.into()).unwrap();

            self.delay.delay_ns(self.delay_ns);

            self.tck.set_high().unwrap();
            self.delay.delay_ns(self.delay_ns);

            if clock.capture {
                self.bits.push(self.tdo.is_high().unwrap());
            }
        }
        self.clocks.clear();
        Ok(())
    }
}

impl<TMS: OutputPin, TCK: OutputPin, TDI: OutputPin, TDO: InputPin, D: DelayNs> RawJtagIo
    for Adapter<TMS, TCK, TDI, TDO, D>
{
    fn read_captured_bits(&mut self) -> Result<BitVec, crate::error::Error> {
        self.flush()?;
        Ok(std::mem::take(&mut self.bits))
    }

    fn shift_bit(
        &mut self,
        tms: bool,
        tdi: bool,
        capture: bool,
    ) -> Result<(), crate::error::Error> {
        self.clocks.push(Clock { tms, tdi, capture });
        Ok(())
    }

    fn state(&self) -> &JtagDriverState {
        &self.state
    }

    fn state_mut(&mut self) -> &mut JtagDriverState {
        &mut self.state
    }
}
