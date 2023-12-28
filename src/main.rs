struct AddressSpace {
    fixed_rom: [u8; 0x4000],
    swappable_rom: [u8; 0x4000],
    vram: [u8; 0x2000],
    cart_ram: [u8; 0x2000],
    wram1: [u8; 0x1000],
    wram2: [u8; 0x1000],
    oam: [u8; 0xa0],
    unused: [u8; 0x60],
    ioregs: [u8; 0x80],
    hram: [u8; 0x7f],
    ie: [u8; 0x1],
}

struct CPU {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    flag_z: bool,
    flag_n: bool,
    flag_h: bool,
    flag_cy: bool,
    pc: u16,
    sp: u16,
    ime: bool,
    halted: bool,
    stopped: bool,
}

impl CPU {
    fn hl(&self) -> u16 {
        return ((self.h as u16) << 8) | self.l as u16;
    }
    fn de(&self) -> u16 {
        return ((self.d as u16) << 8) | self.e as u16;
    }
    fn bc(&self) -> u16 {
        return ((self.b as u16) << 8) | self.c as u16;
    }
    fn delay(&self, _cycles: u16) {
        // println!("Delayed {} M-cycles", cycles)
    }
    fn getreg(&self, id: u8) -> u8 {
        match id {
            0b111 => self.a,
            0b000 => self.b,
            0b001 => self.c,
            0b010 => self.d,
            0b011 => self.e,
            0b100 => self.h,
            0b101 => self.l,
            _ => panic!("Invalid register!"),
        }
    }
    fn setreg(&mut self, id: u8, val: u8) {
        match id {
            0b111 => self.a = val,
            0b000 => self.b = val,
            0b001 => self.c = val,
            0b010 => self.d = val,
            0b011 => self.e = val,
            0b100 => self.h = val,
            0b101 => self.l = val,
            _ => panic!("Invalid register!"),
        }
    }
    fn dumpregs(&self) {
        println!(
            "A: {:02x} B: {:02x} C: {:02x} D: {:02x} E: {:02x} H: {:02x} L: {:02x}",
            self.a, self.b, self.c, self.d, self.e, self.h, self.l
        );
        println!("SP: {:04x} PC: {:04x}", self.sp, self.pc);
        println!(
            "Z: {:01x} H: {:01x} CY: {:01x} N: {:01x}",
            u8::from(self.flag_z),
            u8::from(self.flag_h),
            u8::from(self.flag_cy),
            u8::from(self.flag_n)
        );
    }
}

impl AddressSpace {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3fff => self.fixed_rom[addr as usize],
            0x4000..=0x7fff => self.swappable_rom[(addr - 0x4000) as usize],
            0x8000..=0x9fff => self.vram[(addr - 0x8000) as usize],
            0xa000..=0xbfff => self.cart_ram[(addr - 0xa000) as usize],
            0xc000..=0xcfff => self.wram1[(addr - 0xc000) as usize],
            0xd000..=0xdfff => self.wram2[(addr - 0xd000) as usize],
            0xe000..=0xefff => self.wram1[(addr - 0xe000) as usize],
            0xf000..=0xfdff => self.wram2[(addr - 0xf000) as usize],
            0xfe00..=0xfe9f => self.oam[(addr - 0xfe00) as usize],
            0xfea0..=0xfeff => self.unused[(addr - 0xfea0) as usize],
            0xff00..=0xff7f => self.ioregs[(addr - 0xff00) as usize],
            0xff80..=0xfffe => self.hram[(addr - 0xff80) as usize],
            0xffff => self.ie[0],
        }
    }
    fn write(&mut self, addr: u16, val: u8) {
        if addr == 0xff01 {
            println!("SERIAL: {}", val);
        }
        match addr {
            0x0000..=0x3fff => self.fixed_rom[addr as usize] = val,
            0x4000..=0x7fff => self.swappable_rom[(addr - 0x4000) as usize] = val,
            0x8000..=0x9fff => self.vram[(addr - 0x8000) as usize] = val,
            0xa000..=0xbfff => self.cart_ram[(addr - 0xa000) as usize] = val,
            0xc000..=0xcfff => self.wram1[(addr - 0xc000) as usize] = val,
            0xd000..=0xdfff => self.wram2[(addr - 0xd000) as usize] = val,
            0xe000..=0xefff => self.wram1[(addr - 0xe000) as usize] = val,
            0xf000..=0xfdff => self.wram2[(addr - 0xf000) as usize] = val,
            0xfe00..=0xfe9f => self.oam[(addr - 0xfe00) as usize] = val,
            0xfea0..=0xfeff => self.unused[(addr - 0xfea0) as usize] = val,
            0xff00..=0xff7f => self.ioregs[(addr - 0xff00) as usize] = val,
            0xff80..=0xfffe => self.hram[(addr - 0xff80) as usize] = val,
            0xffff => self.ie[0] = val,
        }
    }
}

fn step(cpu: &mut CPU, address_space: &mut AddressSpace) -> (u16, u16) {
    let ins = address_space.read(cpu.pc);
    let n = address_space.read(cpu.pc.wrapping_add(1));
    let nn = (address_space.read(cpu.pc.wrapping_add(2)) as u16) << 8
        | (address_space.read(cpu.pc.wrapping_add(1)) as u16);
    let e = n as i8 as i16 as u16;
    let r1 = (ins >> 3) & 0b111;
    let r2 = ins & 0b111;
    let r3 = n & 0b111;
    let dd = (ins >> 4) & 0b11;
    let qq = (ins >> 4) & 0b11;
    let cc = (ins >> 3) & 0b11;
    let ss = (ins >> 4) & 0b11;
    let t = (ins >> 3) & 0b111;
    let b = (n >> 3) & 0b111;

    match [7, 6, 5, 4, 3, 2, 1, 0].map(|x| (ins >> x) & 1) {
        // LD r, r'
        [0, 1, _, _, _, _, _, _] if r1 != 0b110 && r2 != 0b110 => {
            let r = r1;
            let r_prime = r2;
            let val = cpu.getreg(r_prime);
            cpu.setreg(r, val);
            (1, 1)
        }
        // LD r, n
        [0, 0, _, _, _, 1, 1, 0] if r1 != 0b110 => {
            let r = r1;
            cpu.setreg(r, n);
            (2, 2)
        }
        // LD r, (HL)
        [0, 1, _, _, _, 1, 1, 0] if r1 != 0b110 => {
            let val = address_space.read(cpu.hl());
            let r = r1;
            cpu.setreg(r, val);
            (2, 1)
        }
        // LD (HL), r
        [0, 1, 1, 1, 0, _, _, _] if r2 != 0b110 => {
            let r = r2;
            address_space.write(cpu.hl(), cpu.getreg(r));
            (2, 1)
        }
        // LD (HL), n
        [0, 0, 1, 1, 0, 1, 1, 0] => {
            address_space.write(cpu.hl(), n);
            (3, 2)
        }
        // LD A, (BC)
        [0, 0, 0, 0, 1, 0, 1, 0] => {
            cpu.a = address_space.read(cpu.bc());
            (2, 1)
        }
        // LD A, (DE)
        [0, 0, 0, 1, 1, 0, 1, 0] => {
            cpu.a = address_space.read(cpu.de());
            (2, 1)
        }
        // LD A, (C)
        [1, 1, 1, 1, 0, 0, 1, 0] => {
            cpu.a = address_space.read(0xff00 | (cpu.c as u16));
            (2, 1)
        }
        // LD (C), A
        [1, 1, 1, 0, 0, 0, 1, 0] => {
            address_space.write(0xff00 | (cpu.c as u16), cpu.a);
            (2, 1)
        }
        // LD A, (n)
        [1, 1, 1, 1, 0, 0, 0, 0] => {
            cpu.a = address_space.read(0xff00 | (n as u16));
            (3, 2)
        }
        // LD (n), A
        [1, 1, 1, 0, 0, 0, 0, 0] => {
            address_space.write(0xff00 | (n as u16), cpu.a);
            (3, 2)
        }
        // LD A, (nn)
        [1, 1, 1, 1, 1, 0, 1, 0] => {
            cpu.a = address_space.read(nn);
            (4, 3)
        }
        // LD (nn), A
        [1, 1, 1, 0, 1, 0, 1, 0] => {
            address_space.write(nn, cpu.a);
            (4, 3)
        }
        // LD A, (HLI)
        [0, 0, 1, 0, 1, 0, 1, 0] => {
            cpu.a = address_space.read(cpu.hl());
            let carry: bool;
            (cpu.l, carry) = cpu.l.overflowing_add(1);
            cpu.h = cpu.h.wrapping_add(u8::from(carry));
            (2, 1)
        }
        // LD A, (HLD)
        [0, 0, 1, 1, 1, 0, 1, 0] => {
            cpu.a = address_space.read(cpu.hl());
            let carry: bool;
            (cpu.l, carry) = cpu.l.overflowing_sub(1);
            cpu.h = cpu.h.wrapping_sub(u8::from(carry));
            (2, 1)
        }
        // LD (BC), A
        [0, 0, 0, 0, 0, 0, 1, 0] => {
            address_space.write(cpu.bc(), cpu.a);
            (2, 1)
        }
        // LD (DE), A
        [0, 0, 0, 1, 0, 0, 1, 0] => {
            address_space.write(cpu.de(), cpu.a);
            (2, 1)
        }
        // LD (HLI), A
        [0, 0, 1, 0, 0, 0, 1, 0] => {
            address_space.write(cpu.hl(), cpu.a);
            let carry: bool;
            (cpu.l, carry) = cpu.l.overflowing_add(1);
            cpu.h = cpu.h.wrapping_add(u8::from(carry));
            (2, 1)
        }
        // LD (HLD), A
        [0, 0, 1, 1, 0, 0, 1, 0] => {
            address_space.write(cpu.hl(), cpu.a);
            let carry: bool;
            (cpu.l, carry) = cpu.l.overflowing_sub(1);
            cpu.h = cpu.h.wrapping_sub(u8::from(carry));
            (2, 1)
        }
        // LD dd, nn
        [0, 0, _, _, 0, 0, 0, 1] => {
            let hi = (nn >> 8) as u8;
            let lo = nn as u8;
            match dd {
                0b00 => {
                    cpu.b = hi;
                    cpu.c = lo;
                }
                0b01 => {
                    cpu.d = hi;
                    cpu.e = lo;
                }
                0b10 => {
                    cpu.h = hi;
                    cpu.l = lo;
                }
                0b11 => {
                    cpu.sp = nn;
                }
                _ => panic!("Invalid register!"),
            }
            (3, 3)
        }
        // LD SP, HL
        [1, 1, 1, 1, 1, 0, 0, 1] => {
            cpu.sp = cpu.hl();
            (2, 1)
        }
        // PUSH qq
        [1, 1, _, _, 0, 1, 0, 1] => {
            let (hi, lo) = match qq {
                0b00 => (cpu.b, cpu.c),
                0b01 => (cpu.d, cpu.e),
                0b10 => (cpu.h, cpu.l),
                0b11 => (
                    cpu.a,
                    u8::from(cpu.flag_z) << 7
                        | u8::from(cpu.flag_n) << 6
                        | u8::from(cpu.flag_h) << 5
                        | u8::from(cpu.flag_z) << 4,
                ),
                _ => panic!("Invalid register!"),
            };
            address_space.write(cpu.sp.wrapping_sub(1), hi);
            address_space.write(cpu.sp.wrapping_sub(2), lo);
            cpu.sp = cpu.sp.wrapping_sub(2);
            (4, 1)
        }
        // POP qq
        [1, 1, _, _, 0, 0, 0, 1] => {
            let lo = address_space.read(cpu.sp);
            let hi = address_space.read(cpu.sp.wrapping_add(1));
            match qq {
                0b00 => {
                    cpu.c = lo;
                    cpu.b = hi;
                }
                0b01 => {
                    cpu.e = lo;
                    cpu.d = hi;
                }
                0b10 => {
                    cpu.l = lo;
                    cpu.h = hi;
                }
                0b11 => {
                    cpu.flag_z = lo >> 7 != 0;
                    cpu.flag_n = ((lo >> 6) & 0b10) != 0;
                    cpu.flag_h = ((lo >> 5) & 0b100) != 0;
                    cpu.flag_cy = ((lo >> 4) & 0b1000) != 0;
                    cpu.a = hi;
                }
                _ => panic!("Invalid register!"),
            }
            cpu.sp = cpu.sp.wrapping_add(2);
            (1, 3)
        }
        // LDHL SP, e
        [1, 1, 1, 1, 1, 0, 0, 0] => {
            let sum: u16;
            (sum, cpu.flag_cy) = cpu.sp.overflowing_add(e);
            cpu.flag_n = false;
            cpu.flag_z = false;
            cpu.h = (sum >> 8) as u8;
            cpu.l = sum as u8;
            // TODO: handle H flag
            (3, 2)
        }
        // LD (nn), SP
        [0, 0, 0, 0, 1, 0, 0, 0] => {
            address_space.write(nn, cpu.sp as u8);
            address_space.write(nn.wrapping_add(1), (cpu.sp >> 8) as u8);
            (5, 3)
        }
        // ADD A, r
        [1, 0, 0, 0, 0, _, _, _] if r2 != 0b110 => {
            let r = r2;
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_add(cpu.getreg(r));
            // TODO: handle H flag
            cpu.flag_n = false;
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // ADD A, n
        [1, 1, 0, 0, 0, 1, 1, 0] => {
            // TODO: handle H flag
            cpu.flag_n = false;
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_add(n);
            cpu.flag_z = cpu.a == 0;
            (2, 2)
        }
        // ADD A, (HL)
        [1, 0, 0, 0, 0, 1, 1, 0] => {
            // TODO: handle H flag
            cpu.flag_n = false;
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_add(address_space.read(cpu.hl()));
            cpu.flag_z = cpu.a == 0;
            (2, 1)
        }
        // ADC A, r
        [1, 0, 0, 0, 1, _, _, _] if r2 != 0b110 => {
            let r = r2;
            cpu.a += cpu.getreg(r) + u8::from(cpu.flag_cy);
            cpu.flag_n = false;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_add
            (1, 1)
        }
        // ADC A, n
        [1, 1, 0, 0, 1, 1, 1, 0] => {
            cpu.a += n + u8::from(cpu.flag_cy);
            cpu.flag_n = false;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_add
            (2, 2)
        }
        // ADC A, (HL)
        [1, 0, 0, 0, 1, 1, 1, 0] => {
            cpu.a += address_space.read(cpu.hl()) + u8::from(cpu.flag_cy);
            cpu.flag_n = false;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_add
            (2, 1)
        }
        // SUB r
        [1, 0, 0, 1, 0, _, _, _] if r2 != 0b110 => {
            let r = r2;
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_sub(cpu.getreg(r));
            // TODO: handle H flag
            cpu.flag_n = true;
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // SUB n
        [1, 1, 0, 1, 0, 1, 1, 0] => {
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_sub(n);
            cpu.flag_z = cpu.a == 0;
            cpu.flag_n = true;
            // TODO: handle H flag
            (2, 2)
        }
        // SUB (HL)
        [1, 0, 0, 1, 0, 1, 1, 0] => {
            (cpu.a, cpu.flag_cy) = cpu.a.overflowing_sub(address_space.read(cpu.hl()));
            cpu.flag_n = true;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            (2, 1)
        }
        // SBC A, r
        [1, 0, 0, 1, 1, _, _, _] if r2 != 0b110 => {
            let r = r2;
            cpu.a -= cpu.getreg(r) - u8::from(cpu.flag_cy);
            cpu.flag_n = true;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_sub
            (1, 1)
        }
        // SBC A, n
        [1, 1, 0, 1, 1, 1, 1, 0] => {
            cpu.a -= n - u8::from(cpu.flag_cy);
            cpu.flag_n = true;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_sub
            (2, 2)
        }
        // SBC A, (HL)
        [1, 0, 0, 1, 1, 1, 1, 0] => {
            cpu.a -= address_space.read(cpu.hl()) - u8::from(cpu.flag_cy);
            cpu.flag_n = true;
            cpu.flag_z = cpu.a == 0;
            // TODO: handle H flag
            // TODO: handle CY flag
            // TODO: use overflowing_sub
            (2, 1)
        }
        // AND r
        [1, 0, 1, 0, 0, _, _, _] if r2 != 0b110 => {
            let r = r2;
            cpu.a &= cpu.getreg(r);
            cpu.flag_h = true;
            cpu.flag_n = false;
            cpu.flag_cy = false;
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // AND n
        [1, 1, 1, 0, 0, 1, 1, 0] => {
            cpu.flag_cy = false;
            cpu.flag_h = true;
            cpu.flag_n = false;
            cpu.a ^= n;
            cpu.flag_z = cpu.a == 0;
            (2, 2)
        }
        // AND (HL)
        [1, 0, 1, 0, 0, 1, 1, 0] => {
            cpu.a &= address_space.read(cpu.hl());
            cpu.flag_cy = false;
            cpu.flag_n = false;
            cpu.flag_h = true;
            cpu.flag_z = cpu.a == 0;
            (2, 1)
        }
        // OR r
        [1, 0, 1, 1, 0, _, _, _] if r2 != 0b110 => {
            let r = r2;
            cpu.a |= cpu.getreg(r);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_cy = false;
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // OR n
        [1, 1, 1, 1, 0, 1, 1, 0] => {
            cpu.flag_cy = false;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.a |= n;
            cpu.flag_z = cpu.a == 0;
            (2, 2)
        }
        // OR (HL)
        [1, 0, 1, 1, 0, 1, 1, 0] => {
            cpu.a |= address_space.read(cpu.hl());
            cpu.flag_cy = false;
            cpu.flag_n = false;
            cpu.flag_h = false;
            cpu.flag_z = cpu.a == 0;
            (2, 1)
        }
        // XOR r
        [1, 0, 1, 0, 1, _, _, _] if r2 != 0b110 => {
            let r = r2;
            cpu.a ^= cpu.getreg(r);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_cy = false;
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // XOR n
        [1, 1, 1, 0, 1, 1, 1, 0] => {
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_cy = false;
            cpu.a ^= n;
            cpu.flag_z = cpu.a == 0;
            (2, 2)
        }
        // XOR (HL)
        [1, 0, 1, 0, 1, 1, 1, 0] => {
            cpu.a ^= address_space.read(cpu.hl());
            cpu.flag_cy = false;
            cpu.flag_n = false;
            cpu.flag_h = false;
            cpu.flag_z = cpu.a == 0;
            (2, 1)
        }
        // CP r
        [1, 0, 1, 1, 1, _, _, _] if r2 != 0b110 => {
            let r = r2;
            // TODO: handle H flag
            let diff: u8;
            (diff, cpu.flag_cy) = cpu.a.overflowing_sub(cpu.getreg(r));
            cpu.flag_n = true;
            cpu.flag_z = diff == 0;
            (1, 1)
        }
        // CP n
        [1, 1, 1, 1, 1, 1, 1, 0] => {
            // TODO: handle H flag
            let diff: u8;
            (diff, cpu.flag_cy) = cpu.a.overflowing_sub(n);
            cpu.flag_z = diff == 0;
            cpu.flag_n = true;
            (2, 2)
        }
        // CP (HL)
        [1, 0, 1, 1, 1, 1, 1, 0] => {
            // TODO: handle H flag
            let diff: u8;
            (diff, cpu.flag_cy) = cpu.a.overflowing_sub(address_space.read(cpu.hl()));
            cpu.flag_z = diff == 0;
            cpu.flag_n = true;
            (2, 2)
        }
        // INC r
        [0, 0, _, _, _, 1, 0, 0] if r1 != 0b110 => {
            let r = r1;
            // TODO: handle H flag
            cpu.flag_n = false;
            let new_val = cpu.getreg(r).wrapping_add(1);
            cpu.setreg(r, new_val);
            cpu.flag_z = new_val == 0;
            (1, 1)
        }
        // INC (HL)
        [0, 0, 1, 1, 0, 1, 0, 0] => {
            address_space.write(cpu.hl(), address_space.read(cpu.hl()).wrapping_add(1));
            cpu.flag_n = true;
            cpu.flag_z = address_space.read(cpu.hl()) == 0;
            // TODO: handle H flag
            (3, 1)
        }
        // DEC r
        [0, 0, _, _, _, 1, 0, 1] if r1 != 0b110 => {
            let r = r1;
            let val = cpu.getreg(r).wrapping_sub(1);
            cpu.setreg(r, val);
            // TODO: handle H flag
            cpu.flag_n = true;
            cpu.flag_z = val == 0;
            (1, 1)
        }
        // DEC (HL)
        [0, 0, 1, 1, 0, 1, 0, 1] => {
            address_space.write(cpu.hl(), address_space.read(cpu.hl()).wrapping_sub(1));
            cpu.flag_n = true;
            cpu.flag_z = address_space.read(cpu.hl()) == 0;
            // TODO: handle H flag
            (3, 1)
        }
        // ADD HL, ss
        [0, 0, _, _, 1, 0, 0, 1] => {
            // TODO: handle H flag
            let sum: u16;
            (sum, cpu.flag_cy) = cpu.hl().overflowing_add(match ss {
                0b00 => cpu.bc(),
                0b01 => cpu.de(),
                0b10 => cpu.hl(),
                0b11 => cpu.sp,
                _ => panic!("Invalid register!"),
            });
            cpu.h = (sum >> 8) as u8;
            cpu.l = sum as u8;
            cpu.flag_n = true;
            (2, 1)
        }
        // ADD SP, e
        [1, 1, 1, 0, 1, 0, 0, 0] => {
            (cpu.sp, cpu.flag_cy) = cpu.sp.overflowing_add(e);
            cpu.flag_n = false;
            cpu.flag_z = false;
            // TODO: handle H flag
            (4, 2)
        }
        // INC ss
        [0, 0, _, _, 0, 0, 1, 1] => {
            let carry: bool;
            match ss {
                0b00 => {
                    (cpu.c, carry) = cpu.c.overflowing_add(1);
                    cpu.b = cpu.b.wrapping_add(u8::from(carry));
                }
                0b01 => {
                    (cpu.e, carry) = cpu.e.overflowing_add(1);
                    cpu.d = cpu.d.wrapping_add(u8::from(carry));
                }
                0b10 => {
                    (cpu.l, carry) = cpu.l.overflowing_add(1);
                    cpu.h = cpu.h.wrapping_add(u8::from(carry));
                }
                0b11 => cpu.sp += 1,
                _ => panic!("Invalid register!"),
            }
            (1, 2)
        }
        // DEC ss
        [0, 0, _, _, 1, 0, 1, 1] => {
            let carry: bool;
            match ss {
                0b00 => {
                    (cpu.c, carry) = cpu.c.overflowing_sub(1);
                    cpu.b = cpu.b.wrapping_sub(u8::from(carry));
                }
                0b01 => {
                    (cpu.e, carry) = cpu.e.overflowing_sub(1);
                    cpu.d = cpu.d.wrapping_sub(u8::from(carry));
                }
                0b10 => {
                    (cpu.l, carry) = cpu.l.overflowing_sub(1);
                    cpu.h = cpu.h.wrapping_sub(u8::from(carry));
                }
                0b11 => cpu.sp += 1,
                _ => panic!("Invalid register!"),
            }
            (1, 2)
        }
        // RLCA
        [0, 0, 0, 0, 0, 1, 1, 1] => {
            cpu.flag_cy = (cpu.a >> 7) != 0;
            cpu.a <<= 1;
            cpu.a |= u8::from(cpu.flag_cy);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (1, 1)
        }
        // RLA
        [0, 0, 0, 1, 0, 1, 1, 1] => {
            let cy: u8 = u8::from(cpu.flag_cy);
            cpu.flag_cy = (cpu.a >> 7) != 0;
            cpu.a <<= 1;
            cpu.a |= cy;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (1, 1)
        }
        // RRCA
        [0, 0, 0, 0, 1, 1, 1, 1] => {
            cpu.flag_cy = (cpu.a & 0b1) != 0;
            cpu.a >>= 1;
            cpu.a |= (u8::from(cpu.flag_cy)) << 7;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (1, 1)
        }
        // RRA
        [0, 0, 0, 1, 1, 1, 1, 1] => {
            let cy: u8 = u8::from(cpu.flag_cy) << 7;
            cpu.flag_cy = (cpu.a & 0b1) != 0;
            cpu.a >>= 1;
            cpu.a |= cy;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (1, 1)
        }
        // RLC r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00000 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = (val >> 7) != 0;
            let new_val = (val << 1) | u8::from(cpu.flag_cy);
            cpu.setreg(r, new_val);
            address_space.write(cpu.hl(), (val << 1) | u8::from(cpu.flag_cy));
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (2, 2)
        }
        // RLC (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00000110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val >> 7) != 0;
            address_space.write(cpu.hl(), (val << 1) | u8::from(cpu.flag_cy));
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (4, 2)
        }
        // RL r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00010 && r3 != 0b110 => {
            let r = r3;
            let cy: u8 = u8::from(cpu.flag_cy);
            let val = cpu.getreg(r);
            cpu.flag_cy = (val >> 7) != 0;
            let new_val = (val << 1) | cy;
            cpu.setreg(r, new_val);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (2, 2)
        }
        // RL (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00010110 => {
            let cy: u8 = u8::from(cpu.flag_cy);
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val >> 7) != 0;
            address_space.write(cpu.hl(), (val << 1) | cy);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (4, 2)
        }
        // RRC r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00001 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = (val >> 1) | (u8::from(cpu.flag_cy) << 7);
            cpu.setreg(r, new_val);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (2, 2)
        }
        // RRC (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00001110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val & 0b1) != 0;
            address_space.write(cpu.hl(), (val >> 1) | (u8::from(cpu.flag_cy) << 7));
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = false;
            (4, 2)
        }
        // RR r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00011 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = (u8::from(cpu.flag_cy) << 7) | (val >> 1);
            cpu.setreg(r, new_val);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = new_val == 0;
            (2, 2)
        }
        // RR (HL)
        [1, 1, 0, 1, 1, 0, 1, 1] if n == 0b00011110 => {
            let cy: u8 = u8::from(cpu.flag_cy) << 7;
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = (cy << 7) | (val >> 1);
            address_space.write(cpu.hl(), new_val);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = new_val == 0;
            (4, 2)
        }
        // SLA r
        [1, 1, 0, 0, 1, 0, 1, 1] if n >> 3 == 0b00100 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = (val >> 7) != 0;
            cpu.flag_h = false;
            cpu.flag_n = false;
            let new_val = val << 1;
            cpu.flag_z = new_val == 0;
            cpu.setreg(r, new_val);
            (2, 2)
        }
        // SLA (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n >> 3 == 0b00100 && r3 != 0b110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val >> 7) != 0;
            cpu.flag_h = false;
            cpu.flag_n = false;
            let new_val = val << 1;
            cpu.flag_z = new_val == 0;
            address_space.write(cpu.hl(), new_val);
            (2, 2)
        }
        // SRA r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00101 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = (u8::from(cpu.flag_cy) << 7) | val >> 1;
            cpu.flag_z = new_val == 0;
            cpu.setreg(r, new_val);
            (2, 2)
        }
        // SRA (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00101110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = (u8::from(cpu.flag_cy) << 7) | val >> 1;
            cpu.flag_z = new_val == 0;
            address_space.write(cpu.hl(), new_val);
            (4, 2)
        }
        // SRL r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00111 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = (val & 0b1) != 0;
            let new_val = val >> 1;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.setreg(r, new_val);
            cpu.flag_z = new_val == 0;
            (2, 2)
        }
        // SRL (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00111110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = (val & 0b1) != 0;
            cpu.flag_h = false;
            cpu.flag_n = false;
            cpu.flag_z = (val >> 1) == 0;
            address_space.write(cpu.hl(), val >> 1);
            (4, 2)
        }
        // SWAP r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00110 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_cy = false;
            cpu.flag_n = false;
            cpu.flag_h = false;
            cpu.flag_z = val == 0;
            let new_val = (val << 4) | (val >> 4);
            cpu.setreg(r, new_val);
            (2, 2)
        }
        // SWAP (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00110110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_cy = false;
            cpu.flag_n = false;
            cpu.flag_h = false;
            cpu.flag_z = val == 0;
            address_space.write(cpu.hl(), (val << 4) | (val >> 4));
            (4, 2)
        }
        // BIT b, r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b01 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            cpu.flag_n = false;
            cpu.flag_h = true;
            cpu.flag_z = ((val >> b) & 0b1) != 0;
            (2, 2)
        }
        // BIT b, (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b01000110 => {
            let val = address_space.read(cpu.hl());
            cpu.flag_n = false;
            cpu.flag_h = true;
            cpu.flag_z = ((val >> b) & 0b1) != 0;
            (3, 2)
        }
        // SET b, r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b11 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            let new_val = val | (1 << b);
            cpu.setreg(r, new_val);
            (2, 2)
        }
        // SET b, (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b11000110 => {
            let val = address_space.read(cpu.hl());
            let new_val = val | (1 << b);
            address_space.write(cpu.hl(), new_val);
            (4, 2)
        }
        // RES b, r
        [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b10 && r3 != 0b110 => {
            let r = r3;
            let val = cpu.getreg(r);
            let new_val = val & !(1 << b);
            cpu.setreg(r, new_val);
            (2, 2)
        }
        // RES b, (HL)
        [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b10000110 => {
            let val = address_space.read(cpu.hl());
            let new_val = val & !(1 << b);
            address_space.write(cpu.hl(), new_val);
            (4, 2)
        }
        // JP nn
        [1, 1, 0, 0, 0, 0, 1, 1] => {
            cpu.pc = nn;
            (4, 0)
        }
        // JP cc, nn
        [1, 1, 0, _, _, 0, 1, 0] => {
            if match cc {
                0b00 => !cpu.flag_z,
                0b01 => cpu.flag_z,
                0b10 => !cpu.flag_cy,
                0b11 => cpu.flag_cy,
                _ => panic!("Invalid condition!"),
            } {
                cpu.pc = nn;
                (4, 0)
            } else {
                (3, 3)
            }
        }
        // JR e
        [0, 0, 0, 1, 1, 0, 0, 0] => {
            cpu.pc = cpu.pc.wrapping_add(e.wrapping_add(2));
            (3, 0)
        }
        // JR cc, e
        [0, 0, 1, _, _, 0, 0, 0] => {
            if match cc {
                0b00 => !cpu.flag_z,
                0b01 => cpu.flag_z,
                0b10 => !cpu.flag_cy,
                0b11 => cpu.flag_cy,
                _ => panic!("Invalid condition!"),
            } {
                cpu.pc = cpu.pc.wrapping_add(e.wrapping_add(2));
                (3, 0)
            } else {
                (2, 2)
            }
        }
        // JP (HL)
        [1, 1, 1, 0, 1, 0, 0, 1] => {
            cpu.pc = cpu.hl();
            (1, 0)
        }
        // CALL nn
        [1, 1, 0, 0, 1, 1, 0, 1] => {
            address_space.write(cpu.sp.wrapping_sub(1), (cpu.pc >> 8) as u8);
            address_space.write(cpu.sp.wrapping_sub(2), cpu.pc as u8);
            cpu.pc = nn;
            cpu.sp = cpu.sp.wrapping_sub(2);
            (6, 0)
        }
        // CALL cc, nn
        [1, 1, 0, _, _, 1, 0, 0] => {
            if match cc {
                0b00 => !cpu.flag_z,
                0b01 => cpu.flag_z,
                0b10 => !cpu.flag_cy,
                0b11 => cpu.flag_cy,
                _ => panic!("Invalid condition!"),
            } {
                address_space.write(cpu.sp.wrapping_sub(1), (cpu.pc >> 8) as u8);
                address_space.write(cpu.sp.wrapping_sub(2), cpu.pc as u8);
                cpu.pc = nn;
                cpu.sp = cpu.sp.wrapping_sub(2);
                (6, 0)
            } else {
                (3, 3)
            }
        }
        // RET
        [1, 1, 0, 0, 1, 0, 0, 1] => {
            cpu.pc = ((address_space.read(cpu.sp.wrapping_add(1)) as u16) << 8)
                | (address_space.read(cpu.sp) as u16);
            cpu.sp = cpu.sp.wrapping_add(2);
            (4, 0)
        }
        // RETI
        [1, 1, 0, 1, 1, 0, 0, 1] => {
            cpu.pc = ((address_space.read(cpu.sp.wrapping_add(1)) as u16) << 8)
                | (address_space.read(cpu.sp) as u16);
            cpu.sp = cpu.sp.wrapping_add(2);
            // TODO: Reset the IME to its pre-interrupt status
            (4, 0)
        }
        // RET cc
        [1, 1, 0, _, _, 0, 0, 0] => {
            if match cc {
                0b00 => !cpu.flag_z,
                0b01 => cpu.flag_z,
                0b10 => !cpu.flag_cy,
                0b11 => cpu.flag_cy,
                _ => panic!("Invalid condition!"),
            } {
                cpu.pc = ((address_space.read(cpu.sp.wrapping_add(1)) as u16) << 8)
                    | (address_space.read(cpu.sp) as u16);
                cpu.sp = cpu.sp.wrapping_add(2);
                (5, 0)
            } else {
                (2, 1)
            }
        }
        // RST t
        [1, 1, _, _, _, 1, 1, 1] => {
            address_space.write(cpu.sp.wrapping_sub(1), (cpu.pc >> 8) as u8);
            address_space.write(cpu.sp.wrapping_sub(2), cpu.pc as u8);
            cpu.sp = cpu.sp.wrapping_sub(2);
            cpu.pc = match t {
                0b000 => 0x00,
                0b001 => 0x08,
                0b010 => 0x10,
                0b011 => 0x18,
                0b100 => 0x20,
                0b101 => 0x28,
                0b110 => 0x30,
                0b111 => 0x38,
                _ => panic!("Invalid t!"),
            };
            (4, 0)
        }
        // DAA
        [0, 0, 1, 0, 0, 1, 1, 1] => {
            let mut addend: u8 = 0;
            let mut carry = false;
            if cpu.flag_n {
                if cpu.flag_cy {
                    addend -= 0x60;
                }
                if cpu.flag_h {
                    addend -= 6;
                }
            } else {
                if cpu.flag_cy || cpu.a > 0x99 {
                    addend += 0x60;
                    carry = true;
                }
                if cpu.flag_h || (cpu.a & 0b1111) > 9 {
                    addend += 6;
                }
            }
            cpu.flag_cy = carry;
            cpu.flag_h = false;
            cpu.a = cpu.a.wrapping_add(addend);
            cpu.flag_z = cpu.a == 0;
            (1, 1)
        }
        // CPL
        [0, 0, 1, 0, 1, 1, 1, 1] => {
            cpu.a = !cpu.a;
            cpu.flag_h = true;
            cpu.flag_n = true;
            (1, 1)
        }
        // NOP
        [0, 0, 0, 0, 0, 0, 0, 0] => (1, 1),
        // CCF
        [0, 0, 1, 1, 1, 1, 1, 1] => {
            cpu.flag_cy = !cpu.flag_cy;
            cpu.flag_h = false;
            cpu.flag_n = false;
            (1, 1)
        }
        // SCF
        [0, 0, 1, 1, 0, 1, 1, 1] => {
            cpu.flag_cy = true;
            cpu.flag_h = false;
            cpu.flag_n = false;
            (1, 1)
        }
        // DI
        [1, 1, 1, 1, 0, 0, 1, 1] => {
            cpu.ime = false;
            (1, 1)
        }
        // EI
        [1, 1, 1, 1, 1, 0, 1, 1] => {
            cpu.ime = true;
            (1, 1)
        }
        // HALT
        [0, 1, 1, 1, 0, 1, 1, 0] => {
            cpu.halted = true;
            (1, 1)
        }
        // STOP
        [0, 0, 0, 1, 0, 0, 0, 0] => {
            cpu.stopped = true;
            (1, 2)
        }
        // BAD
        [1, 1, 0, 1, 0, 0, 1, 1]
        | [1, 1, 1, 0, 0, 0, 1, 1]
        | [1, 1, 1, 0, 0, 1, 0, 0]
        | [1, 1, 1, 1, 0, 1, 0, 0]
        | [1, 1, 0, 1, 1, 0, 1, 1]
        | [1, 1, 1, 0, 1, 0, 1, 1]
        | [1, 1, 1, 0, 1, 1, 0, 0]
        | [1, 1, 1, 1, 1, 1, 0, 0]
        | [1, 1, 0, 1, 1, 1, 0, 1]
        | [1, 1, 1, 0, 1, 1, 0, 1]
        | [1, 1, 1, 1, 1, 1, 0, 1] => (1, 1),
        _ => {
            panic!("Instruction {}:{} is unimplemented", ins, n)
        }
    }
}

use std::env;
use std::fs::File;
use std::io::{stdin, Read};
use std::process;

fn main() {
    let args: Vec<_> = env::args_os().collect();
    if args.len() != 2 {
        println!("Usage: ./grambo <rom>");
        process::exit(1);
    }

    let mut rom_file = File::open(&args[1]).expect("Couldn't open rom file");
    let mut fixed_rom = [0; 0x4000];
    rom_file
        .read_exact(&mut fixed_rom)
        .expect("Couldn't read fixed rom");
    let mut swappable_rom = [0; 0x4000];
    rom_file
        .read_exact(&mut swappable_rom)
        .expect("Couldn't read swappable rom");

    let mut cpu = CPU {
        a: 0x01,
        b: 0xff,
        c: 0x13,
        d: 0x00,
        e: 0xc1,
        h: 0x84,
        l: 0x03,
        flag_z: false,
        flag_n: false,
        flag_h: false,
        flag_cy: false,
        pc: 0x0100,
        sp: 0xfffe,
        ime: false,
        halted: false,
        stopped: false,
    };

    let mut address_space = AddressSpace {
        fixed_rom: fixed_rom,
        swappable_rom: swappable_rom,
        vram: [0; 0x2000],
        cart_ram: [0; 0x2000],
        wram1: [0; 0x1000],
        wram2: [0; 0x1000],
        oam: [0; 0xa0],
        unused: [0; 0x60],
        ioregs: [0; 0x80],
        hram: [0; 0x7f],
        ie: [0; 1],
    };

    // println!("Started up! (press enter to step)");
    loop {
        // cpu.dumpregs();
        // let mut line: String = Default::default();
        // let _ = stdin().read_line(&mut line);
        let (cycles_to_wait, ins_size) = step(&mut cpu, &mut address_space);
        cpu.pc = cpu.pc.wrapping_add(ins_size);
        cpu.delay(cycles_to_wait);
    }
}
