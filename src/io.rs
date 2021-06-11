//! This module contains TX/RX FIFO implementations conforming to [`std::io`] traits.

#![deny(missing_docs, unsafe_code, warnings)]

use crate::regs::{FifoThreshold, GDOCfg, RXBYTES, TXBYTES};
use crate::{Command, MpsseCmdBuilder, ReadRegAddrs, RegAddrs, Status, CC1101};
use libftd2xx::{
    mpsse, ClockBits, ClockBitsIn, ClockBitsOut, ClockDataIn, FtdiCommon, TimeoutError,
};
use log::trace;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer, RingBufferRead, RingBufferWrite};
use std::io;

const fn size_to_rx_threshold(size: usize) -> FifoThreshold {
    match size {
        1..=4 => FifoThreshold::Tx61Rx4,
        5..=8 => FifoThreshold::Tx57Rx8,
        9..=12 => FifoThreshold::Tx53Rx12,
        13..=16 => FifoThreshold::Tx49Rx16,
        17..=20 => FifoThreshold::Tx45Rx20,
        21..=24 => FifoThreshold::Tx41Rx24,
        25..=28 => FifoThreshold::Tx37Rx28,
        29..=32 => FifoThreshold::Tx33Rx32,
        33..=36 => FifoThreshold::Tx29Rx36,
        37..=40 => FifoThreshold::Tx25Rx40,
        41..=44 => FifoThreshold::Tx21Rx44,
        45..=48 => FifoThreshold::Tx17Rx48,
        49..=52 => FifoThreshold::Tx13Rx52,
        53..=56 => FifoThreshold::Tx9Rx56,
        57..=60 => FifoThreshold::Tx5Rx60,
        61..=64 => FifoThreshold::Tx1Rx64,
        _ => FifoThreshold::Tx61Rx4,
    }
}

const fn size_to_tx_threshold(size: usize) -> FifoThreshold {
    match size {
        57..=60 => FifoThreshold::Tx61Rx4,
        53..=56 => FifoThreshold::Tx57Rx8,
        49..=52 => FifoThreshold::Tx53Rx12,
        45..=48 => FifoThreshold::Tx49Rx16,
        41..=44 => FifoThreshold::Tx45Rx20,
        37..=40 => FifoThreshold::Tx41Rx24,
        33..=36 => FifoThreshold::Tx37Rx28,
        29..=32 => FifoThreshold::Tx33Rx32,
        25..=28 => FifoThreshold::Tx29Rx36,
        21..=24 => FifoThreshold::Tx25Rx40,
        17..=20 => FifoThreshold::Tx21Rx44,
        13..=16 => FifoThreshold::Tx17Rx48,
        9..=12 => FifoThreshold::Tx13Rx52,
        5..=8 => FifoThreshold::Tx9Rx56,
        1..=4 => FifoThreshold::Tx5Rx60,
        0 => FifoThreshold::Tx1Rx64,
        _ => FifoThreshold::Tx61Rx4,
    }
}

fn timeout_error(_: TimeoutError) -> io::Error {
    io::Error::from(io::ErrorKind::TimedOut)
}

/// Buffered RX-FIFO reader for CC1101 that implements [`io::Read`] trait.
///
/// Instances are created via [`CC1101::reader`].
///
/// Maintains a circular buffer of user-configurable size via the `BUF_CAP` parameter.
/// The buffer size should be adjusted for the application to balance between low access latency
/// and low frequency of underlying read operations.
pub struct FifoReader<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> {
    cc1101: &'c mut CC1101<'f, Ft>,
    rb: ConstGenericRingBuffer<u8, BUF_CAP>,
}

impl<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> FifoReader<'f, 'c, Ft, BUF_CAP> {
    pub(crate) fn new(cc1101: &'c mut CC1101<'f, Ft>) -> Self {
        assert!(BUF_CAP > 0 && BUF_CAP <= 64);
        Self {
            cc1101,
            rb: ConstGenericRingBuffer::new(),
        }
    }

    fn srx(&mut self) -> io::Result<usize> {
        let cur_state = self.cc1101.get_state().map_err(timeout_error)?;
        self.cc1101.flush_fifos(cur_state).map_err(timeout_error)?;

        mpsse! {
            const (SRX_RXBYTES, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, Command::SRX as u8, 8);
                clock_bits_out(ClockBitsOut::MsbNeg, ReadRegAddrs::RXBYTES as u8, 8);
                clock_bits_in(ClockBitsIn::MsbNeg, 8);
                set_gpio_lower(0x8, 0xb);
                send_immediate();
            };
        }

        trace!(">CC1101 SRX/RXBYTES");
        self.cc1101
            .ftdi
            .write_all(&SRX_RXBYTES)
            .map_err(timeout_error)?;
        let mut buf = [0_u8; READ_LEN];
        self.cc1101.ftdi.read_all(&mut buf).map_err(timeout_error)?;
        let rxbytes = RXBYTES::from_bytes(buf).num_rxbytes() as usize;
        trace!("<CC1101 SRX/RXBYTES {}", rxbytes);

        Ok(rxbytes)
    }

    fn wait_for_rx_fifo_len(&mut self, len: usize) -> io::Result<()> {
        let new_iocfg2 = self
            .cc1101
            .regs
            .iocfg2()
            .with_gdo2_cfg(GDOCfg::RxFifoThreshold);
        let new_fifothr = self
            .cc1101
            .regs
            .fifothr()
            .with_fifo_thr(size_to_rx_threshold(len));

        mpsse! {
            let (wait, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, RegAddrs::IOCFG2 as u8, 8);
                clock_bits_out(ClockBitsOut::MsbNeg, new_iocfg2.into_bytes()[0], 8);
                clock_bits_out(ClockBitsOut::MsbNeg, RegAddrs::FIFOTHR as u8, 8);
                clock_bits_out(ClockBitsOut::MsbNeg, new_fifothr.into_bytes()[0], 8);
                set_gpio_lower(0x8, 0xb);
                wait_on_io_high();
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, ReadRegAddrs::RXBYTES as u8, 8);
                clock_bits_in(ClockBitsIn::MsbNeg, 8);
                set_gpio_lower(0x8, 0xb);
            };
        }

        trace!(">CC1101 WAITING_FOR_RXBYTES >= {}", len);
        self.cc1101.ftdi.write_all(&wait).map_err(timeout_error)?;
        let mut buf = [0_u8; READ_LEN];
        while self.cc1101.ftdi.read_all(&mut buf).is_err() {}
        let rxbytes = RXBYTES::from_bytes(buf).num_rxbytes() as usize;
        trace!("<CC1101 WAITING_FOR_RXBYTES DONE {}", rxbytes);

        if rxbytes >= len {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::Other,
                "WAITING_FOR_RXBYTES failed",
            ))
        }
    }

    fn read_into_buffer(&mut self, len: usize) -> io::Result<()> {
        mpsse! {
            let read_rx_fifo = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, 0xff, 8);
                clock_data_in(ClockDataIn::MsbNeg, len);
                set_gpio_lower(0x8, 0xb);
                send_immediate();
            };
        }

        trace!(">CC1101 RXFIFO {}", len);
        self.cc1101
            .ftdi
            .write_all(&read_rx_fifo)
            .map_err(timeout_error)?;
        let mut buf = [0_u8; BUF_CAP];
        self.cc1101
            .ftdi
            .read_all(&mut buf[0..len])
            .map_err(timeout_error)?;
        trace!("<CC1101 RXFIFO {:x?}", &buf[0..len]);

        for b in buf[0..len].iter() {
            self.rb.push(*b);
        }

        Ok(())
    }

    fn fill_buffer(&mut self) -> io::Result<()> {
        let rxbytes = self.srx()?;

        let read_len = self.rb.capacity() - self.rb.len();
        if rxbytes < read_len {
            self.wait_for_rx_fifo_len(read_len)?;
        }

        self.read_into_buffer(read_len)
    }
}

impl<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> io::Read for FifoReader<'f, 'c, Ft, BUF_CAP> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let read_len = std::cmp::min(BUF_CAP, buf.len());

        if read_len > self.rb.len() {
            self.fill_buffer()?;
        }

        for b in buf[0..read_len].iter_mut() {
            *b = self.rb.dequeue().unwrap();
        }

        Ok(read_len)
    }
}

/// Buffered TX-FIFO writer for CC1101 that implements [`io::Write`] trait.
///
/// Instances are created via [`CC1101::writer`].
///
/// Maintains a circular buffer of user-configurable size via the `BUF_CAP` parameter.
/// The buffer size should be adjusted for the application to balance between low access latency
/// and low frequency of underlying write operations.
pub struct FifoWriter<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> {
    cc1101: &'c mut CC1101<'f, Ft>,
    rb: ConstGenericRingBuffer<u8, BUF_CAP>,
}

impl<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> FifoWriter<'f, 'c, Ft, BUF_CAP> {
    pub(crate) fn new(cc1101: &'c mut CC1101<'f, Ft>) -> Self {
        assert!(BUF_CAP > 0 && BUF_CAP <= 64);
        Self {
            cc1101,
            rb: ConstGenericRingBuffer::new(),
        }
    }

    fn txbytes(&mut self) -> io::Result<usize> {
        mpsse! {
            const (TXBYTES, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits(ClockBits::MsbPosIn, ReadRegAddrs::TXBYTES as u8, 8);
                clock_bits_in(ClockBitsIn::MsbNeg, 8);
                set_gpio_lower(0x8, 0xb);
                send_immediate();
            };
        }

        trace!(">CC1101 TXBYTES");
        self.cc1101
            .ftdi
            .write_all(&TXBYTES)
            .map_err(timeout_error)?;
        let mut buf = [0_u8; READ_LEN];
        self.cc1101.ftdi.read_all(&mut buf).map_err(timeout_error)?;
        let status = Status::from_bytes([buf[0]]);
        let txbytes = TXBYTES::from_bytes([buf[1]]).num_txbytes() as usize;
        trace!("<CC1101 TXBYTES {:?} {}", status, txbytes);

        self.cc1101
            .flush_fifos(status.state())
            .map_err(timeout_error)?;

        Ok(txbytes)
    }

    fn wait_for_tx_fifo_len(&mut self, len: usize) -> io::Result<()> {
        let new_iocfg2 = self
            .cc1101
            .regs
            .iocfg2()
            .with_gdo2_cfg(GDOCfg::TxFifoThreshold);
        let new_fifothr = self
            .cc1101
            .regs
            .fifothr()
            .with_fifo_thr(size_to_tx_threshold(len));

        mpsse! {
            let (wait, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, RegAddrs::IOCFG2 as u8, 8);
                clock_bits_out(ClockBitsOut::MsbNeg, new_iocfg2.into_bytes()[0], 8);
                clock_bits_out(ClockBitsOut::MsbNeg, RegAddrs::FIFOTHR as u8, 8);
                clock_bits_out(ClockBitsOut::MsbNeg, new_fifothr.into_bytes()[0], 8);
                clock_bits_out(ClockBitsOut::MsbNeg, Command::STX as u8, 8);
                set_gpio_lower(0x8, 0xb);
                wait_on_io_low();
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, ReadRegAddrs::TXBYTES as u8, 8);
                clock_bits_in(ClockBitsIn::MsbNeg, 8);
                set_gpio_lower(0x8, 0xb);
            };
        }

        trace!(">CC1101 WAITING_FOR_TXBYTES <= {}", len);
        self.cc1101.ftdi.write_all(&wait).map_err(timeout_error)?;
        let mut buf = [0_u8; READ_LEN];
        while self.cc1101.ftdi.read_all(&mut buf).is_err() {}
        let txbytes = TXBYTES::from_bytes(buf).num_txbytes() as usize;
        trace!("<CC1101 WAITING_FOR_TXBYTES DONE {}", txbytes);

        if txbytes <= len {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::Other,
                "WAITING_FOR_TXBYTES failed",
            ))
        }
    }

    fn write_from_buffer(&mut self) -> io::Result<()> {
        let write_len = self.rb.len();
        let mut buf = [0_u8; 65];
        buf[0] = 0x7F;
        for b in buf[1..=write_len].iter_mut() {
            *b = self.rb.dequeue().unwrap();
        }

        trace!(">CC1101 TXFIFO {:x?}", &buf[1..=write_len]);
        self.cc1101
            .ftdi
            .write_all(
                MpsseCmdBuilder::new()
                    .cs_low()
                    .clock_data_out(&buf[0..=write_len])
                    .cs_high()
                    .cs_low()
                    .clock_bits_out(Command::STX as u8, 8)
                    .cs_high()
                    .as_slice(),
            )
            .map_err(timeout_error)
    }

    fn write_cycle(&mut self) -> io::Result<()> {
        let txbytes = self.txbytes()?;

        if 64 - txbytes < self.rb.len() {
            self.wait_for_tx_fifo_len(64 - self.rb.len())?;
        }

        self.write_from_buffer()
    }

    /// Block until all pending bytes have been transmitted.
    pub fn finish(&mut self) -> io::Result<()> {
        if !self.rb.is_empty() {
            self.write_cycle()?;
        }

        let txbytes = self.txbytes()?;

        if txbytes > 0 {
            self.wait_for_tx_fifo_len(0)?;
        }

        Ok(())
    }
}

impl<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize> io::Write for FifoWriter<'f, 'c, Ft, BUF_CAP> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.rb.is_full() {
            self.flush()?;
        }

        let write_len = std::cmp::min(self.rb.capacity() - self.rb.len(), buf.len());
        for b in buf[0..write_len].iter() {
            self.rb.push(*b);
        }

        Ok(write_len)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.write_cycle()
    }
}
