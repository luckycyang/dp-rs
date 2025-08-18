use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}}


pub struct Adapter<TMS, TCK, TDO, TDI, D> {
    tms: TMS,
    tck: TCK,
    tdo: TDO,
    tdi: TDI,
    delay: D,
    delay_ns: u32
}

trait RawIo {
    // 为 Jtag 服务, 也同时为 SWD 服务
    fn shift_bit(&mut  self, tms: bool, tdi: bool,capture: Option<&mut bool> );

    // 设置半个周期的时钟时间, 例如 33ns 表示 15MHz
    fn set_delay_ns(&mut self, ns: u32);

    // 获取 TMS 电平， 纯纯为 SWD 服务
    fn check_tms_bit(&mut self) -> bool;
}


// 我推荐 TMS 使用两个管脚, 接法就是如下接法，然后用结构体包装一下实现
// embedded_hal::digital::{OutputPin, InputPin} 就行了
//                  ┌─────────┐                                      
// Output  ─────────┼         ┼──────┐                               
//                  └─────────┘      │                               
//                                   │                               
//                                   │                               
//                                   │                               
// Input   ──────────────────────────┴────────────────────   TMS/SDIO
impl <TMS: OutputPin + InputPin, TCK: OutputPin, TDO: InputPin, TDI: OutputPin, D: DelayNs> Adapter<TMS,TCK,TDO,TDI, D> {
    fn new(tms: TMS, tck: TCK, tdo: TDO, tdi: TDI, delay:D) -> Self {
        Self { tms,tck,tdo,tdi, delay,delay_ns: 33 }
    }
}


