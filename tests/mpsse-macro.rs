#![recursion_limit = "1024"]
//#![feature(trace_macros)]
//trace_macros!(true);

use libftd2xx::{
    ClockBits, ClockBitsIn, ClockBitsOut, ClockData, ClockDataIn, ClockDataOut, MpsseCmd,
    TimeoutError,
};
use libftd2xx_cc1101::regs::*;
use libftd2xx_cc1101::*;
use std::convert::TryInto;

struct MockFtdi {
    pub write_data: Vec<u8>,
    pub read_len: usize,
}

impl MockFtdi {
    fn new() -> Self {
        Self {
            write_data: Vec::new(),
            read_len: 0,
        }
    }

    fn write_all(&mut self, buf: &[u8]) -> Result<(), TimeoutError> {
        self.write_data.extend_from_slice(&buf);
        Ok(())
    }

    fn read_all(&mut self, buf: &mut [u8]) -> Result<(), TimeoutError> {
        self.read_len += buf.len();
        Ok(())
    }
}

#[test]
fn commands() {
    let mut ftdi = MockFtdi::new();

    mpsse! {
        ftdi <=> {
            sres();
            sidle();
            write_tx_fifo([11, 22, 33, 44]);
            write_pa_table([55, 66, 77, 88]);
            stx();
            set_iocfg2(IOCFG2::new());
            set_freq(123456);
            set_sync(512);
            write_all_regs(Regs::new());
        };
    }

    assert_eq!(
        ftdi.write_data,
        [
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            Command::SRES as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            Command::SIDLE as u8,
            ClockDataOut::MsbNeg as u8,
            0x4 as u8,
            0x0 as u8,
            0x7f as u8,
            11 as u8,
            22 as u8,
            33 as u8,
            44 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockDataOut::MsbNeg as u8,
            0x4 as u8,
            0x0 as u8,
            0x7e as u8,
            55 as u8,
            66 as u8,
            77 as u8,
            88 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            Command::STX as u8,
            ClockDataOut::MsbNeg as u8,
            0x1 as u8,
            0x0 as u8,
            RegAddrs::IOCFG2 as u8,
            0x0 as u8,
            ClockDataOut::MsbNeg as u8,
            0x5 as u8,
            0x0 as u8,
            RegAddrs::FREQ as u8,
            1 as u8,
            RegAddrs::FREQ as u8 + 1,
            226 as u8,
            RegAddrs::FREQ as u8 + 2,
            64 as u8,
            ClockDataOut::MsbNeg as u8,
            0x3 as u8,
            0x0 as u8,
            RegAddrs::SYNC as u8,
            2 as u8,
            RegAddrs::SYNC as u8 + 1,
            0 as u8,
            ClockDataOut::MsbNeg as u8,
            47 as u8,
            0x0 as u8,
            0x40 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
        ]
    );
    assert_eq!(ftdi.read_len, 0);
}

#[test]
fn command_reads() {
    let mut ftdi = MockFtdi::new();

    mpsse! {
        ftdi <=> {
            let _sres_status = sres();
            let _sidle_status = sidle();
            let _write_tx_status = write_tx_fifo([11, 22, 33, 44]);
            let _stx_status = stx();
            let _read_data_only = read_rx_fifo(4);
            let (_read_data_status, _read_data) = read_rx_fifo(4);
            let _read_pa_only = read_pa_table(4);
            let (_read_pa_status, _read_pa) = read_pa_table(4);
            let _lqi = lqi();
            let (_lqi_status, _lqi2) = lqi();
            let _partnum = partnum();
            let _wortime = wortime();
            let (_wortime_status, _wortime2) = wortime();
            let _iocfg2 = iocfg2();
            let _iocfg2_status = set_iocfg2(IOCFG2::new());
            let _freq = freq();
            let _freq_status = set_freq(123456);
            let _sync = sync();
            let _sync_status = set_sync(512);
            let _regs = read_all_regs();
            let (_regs_status, _regs2) = read_all_regs();
            let _regs_status2 = write_all_regs(Regs::new());
        };
    }

    assert_eq!(
        ftdi.write_data,
        [
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            Command::SRES as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            Command::SIDLE as u8,
            ClockData::MsbPosIn as u8,
            0x4 as u8,
            0x0 as u8,
            0x7f as u8,
            11 as u8,
            22 as u8,
            33 as u8,
            44 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            Command::STX as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            0xff as u8,
            ClockDataIn::MsbNeg as u8,
            0x3 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            0xff as u8,
            ClockDataIn::MsbNeg as u8,
            0x3 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            0xfe as u8,
            ClockDataIn::MsbNeg as u8,
            0x3 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            0xfe as u8,
            ClockDataIn::MsbNeg as u8,
            0x3 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            ReadRegAddrs::LQI as u8,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            ReadRegAddrs::LQI as u8,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            ReadRegAddrs::PARTNUM as u8,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            ReadRegAddrs::WORTIME as u8,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            ReadRegAddrs::WORTIME as u8 + 1,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            ReadRegAddrs::WORTIME as u8,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            ReadRegAddrs::WORTIME as u8 + 1,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::IOCFG2 as u8 + 0x80,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockData::MsbPosIn as u8,
            0x1 as u8,
            0x0 as u8,
            RegAddrs::IOCFG2 as u8,
            0x0 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::FREQ as u8 + 0x80,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::FREQ as u8 + 0x80 + 1,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::FREQ as u8 + 0x80 + 2,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockData::MsbPosIn as u8,
            0x5 as u8,
            0x0 as u8,
            RegAddrs::FREQ as u8,
            1 as u8,
            RegAddrs::FREQ as u8 + 1,
            226 as u8,
            RegAddrs::FREQ as u8 + 2,
            64 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::SYNC as u8 + 0x80,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::SYNC as u8 + 0x80 + 1,
            ClockBitsIn::MsbNeg as u8,
            0x7 as u8,
            ClockData::MsbPosIn as u8,
            0x3 as u8,
            0x0 as u8,
            RegAddrs::SYNC as u8,
            2 as u8,
            RegAddrs::SYNC as u8 + 1,
            0 as u8,
            ClockBitsOut::MsbNeg as u8,
            0x7 as u8,
            RegAddrs::IOCFG2 as u8 + 0xC0,
            ClockDataIn::MsbNeg as u8,
            46 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockBits::MsbPosIn as u8,
            0x7 as u8,
            RegAddrs::IOCFG2 as u8 + 0xC0,
            ClockDataIn::MsbNeg as u8,
            46 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x0 as u8,
            0xb as u8,
            ClockData::MsbPosIn as u8,
            47 as u8,
            0x0 as u8,
            0x40 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            0x0 as u8,
            MpsseCmd::SetDataBitsLowbyte as u8,
            0x8 as u8,
            0xb as u8,
            MpsseCmd::SendImmediate as u8,
        ]
    );
    assert_eq!(ftdi.read_len, 197);
}
