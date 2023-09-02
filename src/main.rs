//! Executes an Arm Cortex-M0+ program in a virtual CPU.
//!
//! A binary linked to execute at 0x0000_0000 must be given on the command line and the binary must start
//! with a valid interrupt vector table for the Cortex-M0+.
//!
//! 256 KiB of ROM at 0x0000_0000 and 256 KiB of SRAM at 0x2000_0000 are simulated.

use cmsim::Memory;

struct Region {
    contents: Box<[u32]>,
}

struct System {
    /// At 0x0000_0000
    flash: Region,
    /// At 0x2000_0000
    ram: Region,
}

impl cmsim::Memory for Region {
    fn load_u32(&self, addr: u32) -> Result<u32, cmsim::Error> {
        self.contents
            .get(addr as usize >> 2)
            .copied()
            .ok_or(cmsim::Error::InvalidAddress)
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), cmsim::Error> {
        *self
            .contents
            .get_mut(addr as usize >> 2)
            .ok_or(cmsim::Error::InvalidAddress)? = value;
        Ok(())
    }
}

impl cmsim::Memory for System {
    fn load_u32(&self, addr: u32) -> Result<u32, cmsim::Error> {
        if addr < 0x2000_0000 {
            self.flash.load_u32(addr)
        } else {
            self.ram.load_u32(addr)
        }
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), cmsim::Error> {
        if addr >= 0x2000_0000 {
            self.ram.store_u32(addr, value)
        } else {
            // Can't write to flash
            Err(cmsim::Error::InvalidAddress)
        }
    }
}

fn main() {
    let binary_name = std::env::args().nth(1).unwrap();
    let contents = std::fs::read(binary_name).unwrap();

    let mut system = System {
        flash: Region {
            contents: vec![0u32; 256 * 1024].into(),
        },
        ram: Region {
            contents: vec![0u32; 64 * 1024].into(),
        },
    };

    for (idx, b) in contents.iter().enumerate() {
                system.flash.store_u8(idx as u32, *b).unwrap();
    }

    let sp = system.flash.load_u32(0).unwrap();
    let reset = system.flash.load_u32(4).unwrap();
    println!("SP is 0x{:08x}, Reset is 0x{:08x}", sp, reset);
    let mut cpu = cmsim::Armv6M::new(sp, reset);

    loop {
        cpu.step(&mut system).unwrap();
        println!("CPU:\n{:08x?}", cpu);
        if let Some(arg) = cpu.breakpoint() {
            println!("Got breakpoint 0x{:02X}", arg);
            if arg == 0xAB {
                // Semihosting
                let op_num = cpu.register(cmsim::Register::R0);
                let _param = cpu.register(cmsim::Register::R1);
                match op_num {
                    0x18 => {
                        // SYS_EXIT
                        println!("SYS_EXIT...");
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
