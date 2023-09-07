//! Executes an Arm Cortex-M0+ program in a virtual CPU.
//!
//! A binary linked to execute at 0x0000_0000 must be given on the command line and the binary must start
//! with a valid interrupt vector table for the Cortex-M0+.
//!
//! 1024 KiB of ROM at 0x0000_0000 and 1024 KiB of SRAM at 0x2000_0000 are simulated.

use std::io::prelude::*;
use tracing::{error, info};

use cmsim::Memory;

struct Region {
    contents: Box<[u32]>,
}

impl cmsim::Memory for Region {
    fn load_u32(&self, addr: u32) -> Result<u32, cmsim::Error> {
        self.contents
            .get(addr as usize >> 2)
            .copied()
            .ok_or(cmsim::Error::InvalidAddress(addr))
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), cmsim::Error> {
        *self
            .contents
            .get_mut(addr as usize >> 2)
            .ok_or(cmsim::Error::InvalidAddress(addr))? = value;
        Ok(())
    }
}

/// Represents a basic UART, compatible with the one in QEMU
///
/// Writes to standard out and reads from stdin.
#[derive(Debug)]
struct Uart {
    baud: u32,
    control: u32,
}

impl cmsim::Memory for Uart {
    fn load_u32(&self, addr: u32) -> Result<u32, cmsim::Error> {
        match addr {
            0 => {
                // Data in
                if let crossterm::event::Event::Key(k) = crossterm::event::read().unwrap() {
                    match k.code {
                        crossterm::event::KeyCode::Backspace => todo!(),
                        crossterm::event::KeyCode::Enter => todo!(),
                        crossterm::event::KeyCode::Left => todo!(),
                        crossterm::event::KeyCode::Right => todo!(),
                        crossterm::event::KeyCode::Up => todo!(),
                        crossterm::event::KeyCode::Down => todo!(),
                        crossterm::event::KeyCode::Home => todo!(),
                        crossterm::event::KeyCode::End => todo!(),
                        crossterm::event::KeyCode::PageUp => todo!(),
                        crossterm::event::KeyCode::PageDown => todo!(),
                        crossterm::event::KeyCode::Tab => todo!(),
                        crossterm::event::KeyCode::BackTab => todo!(),
                        crossterm::event::KeyCode::Delete => todo!(),
                        crossterm::event::KeyCode::Insert => todo!(),
                        crossterm::event::KeyCode::F(_) => todo!(),
                        crossterm::event::KeyCode::Char(x) => Ok(x as u32),
                        crossterm::event::KeyCode::Null => todo!(),
                        crossterm::event::KeyCode::Esc => todo!(),
                        crossterm::event::KeyCode::CapsLock => todo!(),
                        crossterm::event::KeyCode::ScrollLock => todo!(),
                        crossterm::event::KeyCode::NumLock => todo!(),
                        crossterm::event::KeyCode::PrintScreen => todo!(),
                        crossterm::event::KeyCode::Pause => todo!(),
                        crossterm::event::KeyCode::Menu => todo!(),
                        crossterm::event::KeyCode::KeypadBegin => todo!(),
                        crossterm::event::KeyCode::Media(_) => todo!(),
                        crossterm::event::KeyCode::Modifier(_) => todo!(),
                    }
                } else {
                    Ok(0)
                }
            }
            4 => {
                // Status
                if crossterm::event::poll(std::time::Duration::from_micros(10)).unwrap() {
                    Ok(2)
                } else {
                    Ok(0)
                }
            }
            8 => Ok(self.control),
            16 => Ok(self.baud),
            _ => Err(cmsim::Error::InvalidAddress(addr)),
        }
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), cmsim::Error> {
        match addr {
            0 => {
                let mut out = std::io::stdout();
                out.write_all(&[value as u8]).unwrap();
                out.flush().unwrap();
            }
            8 => self.control = value,
            16 => self.baud = value,
            _ => {
                return Err(cmsim::Error::InvalidAddress(addr));
            }
        }
        Ok(())
    }
}

struct System {
    /// Flash at 0x0000_0000
    flash: Region,
    /// SRAM at 0x2000_0000
    ram: Region,
    /// UART at 0x5930_3000
    uart: Uart,
}

impl cmsim::Memory for System {
    fn load_u32(&self, addr: u32) -> Result<u32, cmsim::Error> {
        if addr < 0x2000_0000 {
            self.flash.load_u32(addr)
        } else if (0x5930_3000..0x5930_3100).contains(&addr) {
            self.uart.load_u32(addr - 0x5930_3000)
        } else {
            self.ram.load_u32(addr - 0x2000_0000)
        }
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), cmsim::Error> {
        if addr < 0x2000_0000 {
            Err(cmsim::Error::InvalidAddress(addr))
        } else if (0x5930_3000..0x5930_3100).contains(&addr) {
            self.uart.store_u32(addr - 0x5930_3000, value)
        } else {
            self.ram.store_u32(addr - 0x2000_0000, value)
        }
    }
}

fn main() {
    // We need raw more for the UART emulation
    crossterm::terminal::enable_raw_mode().unwrap();

    tracing_subscriber::fmt::fmt()
        .with_writer(std::io::stderr)
        .with_max_level(tracing::Level::DEBUG)
        .init();

    let binary_name = std::env::args().nth(1).unwrap();
    let contents = std::fs::read(binary_name).unwrap();

    let mut system = System {
        flash: Region {
            contents: vec![0u32; 256 * 1024].into(),
        },
        ram: Region {
            contents: vec![0u32; 256 * 1024].into(),
        },
        uart: Uart {
            baud: 0,
            control: 0,
        },
    };

    for (idx, b) in contents.iter().enumerate() {
        system.flash.store_u8(idx as u32, *b).unwrap();
    }

    let sp = system.flash.load_u32(0).unwrap();
    let reset = system.flash.load_u32(4).unwrap();
    info!("SP is 0x{:08x}, Reset is 0x{:08x}", sp, reset);
    let mut cpu = cmsim::Armv6M::new(sp, reset);

    let mut steps = 0;
    loop {
        if let Err(e) = cpu.step(&mut system) {
            match e {
                cmsim::Error::InvalidAddress(a) => error!("Invalid address {a:#08x}"),
                cmsim::Error::InvalidInstruction(i) => error!(
                    "Invalid opcode {:#04x} @ {:#08x}",
                    i,
                    cpu.register(cmsim::Register::Pc)
                ),
                cmsim::Error::InvalidInstruction32(i1, i2) => error!(
                    "Invalid opcode {:#04x}{:04x} @ {:#08x}",
                    i1,
                    i2,
                    cpu.register(cmsim::Register::Pc)
                ),
                cmsim::Error::UnalignedAccess => error!("Unaligned access"),
                cmsim::Error::WideInstruction => unreachable!(),
                cmsim::Error::UnknownSpecialRegister(r) => error!("Unknown special register {}", r),
            }
            break;
        }
        steps += 1;
        info!("Steps {}, CPU:\n{:08x?}", steps, cpu);
        if let Some(arg) = cpu.breakpoint() {
            info!("Got breakpoint 0x{:02X}", arg);
            if arg == 0xAB {
                // Semihosting
                let op_num = cpu.register(cmsim::Register::R0);
                let _param = cpu.register(cmsim::Register::R1);
                match op_num {
                    0x18 => {
                        // SYS_EXIT
                        info!("SYS_EXIT...");
                        std::process::exit(0);
                    }
                    _ => {
                        panic!("Unsupported semihosting operation 0x{:02x}", op_num);
                    }
                }
            } else {
                panic!("Unsupported breakpoint 0x{:02x}", arg);
            }
        };
    }
}
