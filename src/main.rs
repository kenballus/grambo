struct AddressSpace {
    fixed_rom: [u8; 0x4000],
    swappable_rom: [u8; 0x4000],
    vram: [u8; 0x2000],
    cart_ram: [u8; 0x2000],
    wram1: [u8; 0x1000],
    wram2: [u8; 0x1000],
    oam: [u8; 0xa0],
    unused: [u8; 0x60],
    p1: [u8; 0x1],
    sb: [u8; 0x1],
    sc: [u8; 0x1],
    ff03: [u8; 0x1],
    div: [u8; 0x1],
    tima: [u8; 0x1],
    tma: [u8; 0x1],
    tac: [u8; 0x1],
    ff08_ff0e: [u8; 0x7],
    r#if: [u8; 0x1],
    nr10: [u8; 0x1],
    nr11: [u8; 0x1],
    nr12: [u8; 0x1],
    nr13: [u8; 0x1],
    nr14: [u8; 0x1],
    ff15: [u8; 0x1],
    nr21: [u8; 0x1],
    nr22: [u8; 0x1],
    nr23: [u8; 0x1],
    nr24: [u8; 0x1],
    nr30: [u8; 0x1],
    nr31: [u8; 0x1],
    nr32: [u8; 0x1],
    nr33: [u8; 0x1],
    nr34: [u8; 0x1],
    ff1f: [u8; 0x1],
    nr41: [u8; 0x1],
    nr42: [u8; 0x1],
    nr43: [u8; 0x1],
    nr44: [u8; 0x1],
    nr50: [u8; 0x1],
    nr51: [u8; 0x1],
    nr52: [u8; 0x1],
    ff27_ff2f: [u8; 0x9],
    wave_ram: [u8; 0x10],
    lcdc: [u8; 0x1],
    stat: [u8; 0x1],
    scy: [u8; 0x1],
    scx: [u8; 0x1],
    ly: [u8; 0x1],
    lyc: [u8; 0x1],
    dma: [u8; 0x1],
    bgp: [u8; 0x1],
    obp0: [u8; 0x1],
    obp1: [u8; 0x1],
    wy: [u8; 0x1],
    wx: [u8; 0x1],
    ff4c: [u8; 0x1],
    key1: [u8; 0x1],
    ff4e: [u8; 0x1],
    vbk: [u8; 0x1],
    ff50: [u8; 0x1],
    hdma1: [u8; 0x1],
    hdma2: [u8; 0x1],
    hdma3: [u8; 0x1],
    hdma4: [u8; 0x1],
    hdma5: [u8; 0x1],
    rp: [u8; 0x1],
    ff57_ff67: [u8; 0x11],
    bcps: [u8; 0x1],
    bcpd: [u8; 0x1],
    ocps: [u8; 0x1],
    ocpd: [u8; 0x1],
    opri: [u8; 0x1],
    ff6d_ff6f: [u8; 0x3],
    svbk: [u8; 0x1],
    ff71_ff75: [u8; 0x5],
    pcm12: [u8; 0x1],
    pcm34: [u8; 0x1],
    ff78_ff7f: [u8; 0x8],
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

enum Button {
    Right,
    Left,
    Up,
    Down,
    A,
    B,
    Select,
    Start,
}

struct Joypad {
    up: bool,
    down: bool,
    left: bool,
    right: bool,
    a: bool,
    b: bool,
    start: bool,
    select: bool,
}

struct GameBoy {
    joypad: Joypad,
    address_space: AddressSpace,
    cpu: CPU,
}

impl Joypad {
    fn key_down(&mut self, button: Button) {
        match button {
            Button::Right => self.right = true,
            Button::Left => self.left = true,
            Button::Up => self.up = true,
            Button::Down => self.down = true,
            Button::A => self.a = true,
            Button::B => self.b = true,
            Button::Select => self.select = true,
            Button::Start => self.start = true,
        };
    }
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
        println!("IME: {}", u8::from(self.ime));
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
            0xff00 => self.p1[0],
            0xff01 => self.sb[0],
            0xff02 => self.sc[0],
            0xff03 => self.ff03[0],
            0xff04 => self.div[0],
            0xff05 => self.tima[0],
            0xff06 => self.tma[0],
            0xff07 => self.tac[0],
            0xff00..=0xff0e => self.ff08_ff0e[(addr - 0xff08) as usize],
            0xff0f => self.r#if[0],
            0xff10 => self.nr10[0],
            0xff11 => self.nr11[0],
            0xff12 => self.nr12[0],
            0xff13 => self.nr13[0],
            0xff14 => self.nr14[0],
            0xff15 => self.ff15[0],
            0xff16 => self.nr21[0],
            0xff17 => self.nr22[0],
            0xff18 => self.nr23[0],
            0xff19 => self.nr24[0],
            0xff1a => self.nr30[0],
            0xff1b => self.nr31[0],
            0xff1c => self.nr32[0],
            0xff1d => self.nr33[0],
            0xff1e => self.nr34[0],
            0xff1f => self.ff1f[0],
            0xff20 => self.nr41[0],
            0xff21 => self.nr42[0],
            0xff22 => self.nr43[0],
            0xff23 => self.nr44[0],
            0xff24 => self.nr50[0],
            0xff25 => self.nr51[0],
            0xff26 => self.nr52[0],
            0xff27..=0xff2f => self.ff27_ff2f[(addr - 0xff27) as usize],
            0xff30..=0xff3f => self.wave_ram[(addr - 0xff30) as usize],
            0xff40 => self.lcdc[0],
            0xff41 => self.stat[0],
            0xff42 => self.scy[0],
            0xff43 => self.scx[0],
            0xff44 => self.ly[0],
            0xff45 => self.lyc[0],
            0xff46 => self.dma[0],
            0xff47 => self.bgp[0],
            0xff48 => self.obp0[0],
            0xff49 => self.obp1[0],
            0xff4a => self.wy[0],
            0xff4b => self.wx[0],
            0xff4c => self.ff4c[0],
            0xff4d => self.key1[0],
            0xff4e => self.ff4e[0],
            0xff4f => self.vbk[0],
            0xff50 => self.ff50[0],
            0xff51 => self.hdma1[0],
            0xff52 => self.hdma2[0],
            0xff53 => self.hdma3[0],
            0xff54 => self.hdma4[0],
            0xff55 => self.hdma5[0],
            0xff56 => self.rp[0],
            0xff57..=0xff67 => self.ff57_ff67[(addr - 0xff57) as usize],
            0xff68 => self.bcps[0],
            0xff69 => self.bcpd[0],
            0xff6a => self.ocps[0],
            0xff6b => self.ocpd[0],
            0xff6c => self.opri[0],
            0xff6d..=0xff6f => self.ff6d_ff6f[(addr - 0xff6d) as usize],
            0xff70 => self.svbk[0],
            0xff71..=0xff75 => self.ff71_ff75[(addr - 0xff71) as usize],
            0xff76 => self.pcm12[0],
            0xff77 => self.pcm34[0],
            0xff78..=0xff7f => self.ff78_ff7f[(addr - 0xff78) as usize],
            0xff80..=0xfffe => self.hram[(addr - 0xff80) as usize],
            0xffff => self.ie[0],
        }
    }
    fn write(&mut self, addr: u16, val: u8) {
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
            0xff00 => self.p1[0] = (val & 0b11110000) | (self.p1[0] & 0b00001111),
            0xff01 => self.sb[0] = val,
            0xff02 => self.sc[0] = val,
            0xff03 => self.ff03[0] = val,
            0xff04 => self.div[0] = val,
            0xff05 => self.tima[0] = val,
            0xff06 => self.tma[0] = val,
            0xff07 => self.tac[0] = val,
            0xff00..=0xff0e => self.ff08_ff0e[(addr - 0xff08) as usize] = val,
            0xff0f => self.r#if[0] = val,
            0xff10 => self.nr10[0] = val,
            0xff11 => self.nr11[0] = val,
            0xff12 => self.nr12[0] = val,
            0xff13 => self.nr13[0] = val,
            0xff14 => self.nr14[0] = val,
            0xff15 => self.ff15[0] = val,
            0xff16 => self.nr21[0] = val,
            0xff17 => self.nr22[0] = val,
            0xff18 => self.nr23[0] = val,
            0xff19 => self.nr24[0] = val,
            0xff1a => self.nr30[0] = val,
            0xff1b => self.nr31[0] = val,
            0xff1c => self.nr32[0] = val,
            0xff1d => self.nr33[0] = val,
            0xff1e => self.nr34[0] = val,
            0xff1f => self.ff1f[0] = val,
            0xff20 => self.nr41[0] = val,
            0xff21 => self.nr42[0] = val,
            0xff22 => self.nr43[0] = val,
            0xff23 => self.nr44[0] = val,
            0xff24 => self.nr50[0] = val,
            0xff25 => self.nr51[0] = val,
            0xff26 => self.nr52[0] = val,
            0xff27..=0xff2f => self.ff27_ff2f[(addr - 0xff27) as usize] = val,
            0xff30..=0xff3f => self.wave_ram[(addr - 0xff30) as usize] = val,
            0xff40 => self.lcdc[0] = val,
            0xff41 => self.stat[0] = val,
            0xff42 => self.scy[0] = val,
            0xff43 => self.scx[0] = val,
            0xff44 => self.ly[0] = val,
            0xff45 => self.lyc[0] = val,
            0xff46 => self.dma[0] = val,
            0xff47 => self.bgp[0] = val,
            0xff48 => self.obp0[0] = val,
            0xff49 => self.obp1[0] = val,
            0xff4a => self.wy[0] = val,
            0xff4b => self.wx[0] = val,
            0xff4c => self.ff4c[0] = val,
            0xff4d => self.key1[0] = val,
            0xff4e => self.ff4e[0] = val,
            0xff4f => self.vbk[0] = val,
            0xff50 => self.ff50[0] = val,
            0xff51 => self.hdma1[0] = val,
            0xff52 => self.hdma2[0] = val,
            0xff53 => self.hdma3[0] = val,
            0xff54 => self.hdma4[0] = val,
            0xff55 => self.hdma5[0] = val,
            0xff56 => self.rp[0] = val,
            0xff57..=0xff67 => self.ff57_ff67[(addr - 0xff57) as usize] = val,
            0xff68 => self.bcps[0] = val,
            0xff69 => self.bcpd[0] = val,
            0xff6a => self.ocps[0] = val,
            0xff6b => self.ocpd[0] = val,
            0xff6c => self.opri[0] = val,
            0xff6d..=0xff6f => self.ff6d_ff6f[(addr - 0xff6d) as usize] = val,
            0xff70 => self.svbk[0] = val,
            0xff71..=0xff75 => self.ff71_ff75[(addr - 0xff71) as usize] = val,
            0xff76 => self.pcm12[0] = val,
            0xff77 => self.pcm34[0] = val,
            0xff78..=0xff7f => self.ff78_ff7f[(addr - 0xff78) as usize] = val,
            0xff80..=0xfffe => self.hram[(addr - 0xff80) as usize] = val,
            0xffff => self.ie[0] = val,
        }
    }
}

impl GameBoy {
    fn new(fixed_rom: &mut [u8; 0x4000], swappable_rom: &mut [u8; 0x4000]) -> GameBoy {
        GameBoy {
            cpu: CPU {
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
            },
            address_space: AddressSpace {
                fixed_rom: *fixed_rom,
                swappable_rom: *swappable_rom,
                vram: [0; 0x2000],
                cart_ram: [0; 0x2000],
                wram1: [0; 0x1000],
                wram2: [0; 0x1000],
                oam: [0; 0xa0],
                unused: [0; 0x60],
                p1: [0; 0x1],
                sb: [0; 0x1],
                sc: [0; 0x1],
                ff03: [0; 0x1],
                div: [0; 0x1],
                tima: [0; 0x1],
                tma: [0; 0x1],
                tac: [0; 0x1],
                ff08_ff0e: [0; 0x7],
                r#if: [0; 0x1],
                nr10: [0; 0x1],
                nr11: [0; 0x1],
                nr12: [0; 0x1],
                nr13: [0; 0x1],
                nr14: [0; 0x1],
                ff15: [0; 0x1],
                nr21: [0; 0x1],
                nr22: [0; 0x1],
                nr23: [0; 0x1],
                nr24: [0; 0x1],
                nr30: [0; 0x1],
                nr31: [0; 0x1],
                nr32: [0; 0x1],
                nr33: [0; 0x1],
                nr34: [0; 0x1],
                ff1f: [0; 0x1],
                nr41: [0; 0x1],
                nr42: [0; 0x1],
                nr43: [0; 0x1],
                nr44: [0; 0x1],
                nr50: [0; 0x1],
                nr51: [0; 0x1],
                nr52: [0; 0x1],
                ff27_ff2f: [0; 0x9],
                wave_ram: [0; 0x10],
                lcdc: [0; 0x1],
                stat: [0; 0x1],
                scy: [0; 0x1],
                scx: [0; 0x1],
                ly: [0; 0x1],
                lyc: [0; 0x1],
                dma: [0; 0x1],
                bgp: [0; 0x1],
                obp0: [0; 0x1],
                obp1: [0; 0x1],
                wy: [0; 0x1],
                wx: [0; 0x1],
                ff4c: [0; 0x1],
                key1: [0; 0x1],
                ff4e: [0; 0x1],
                vbk: [0; 0x1],
                ff50: [0; 0x1],
                hdma1: [0; 0x1],
                hdma2: [0; 0x1],
                hdma3: [0; 0x1],
                hdma4: [0; 0x1],
                hdma5: [0; 0x1],
                rp: [0; 0x1],
                ff57_ff67: [0; 0x11],
                bcps: [0; 0x1],
                bcpd: [0; 0x1],
                ocps: [0; 0x1],
                ocpd: [0; 0x1],
                opri: [0; 0x1],
                ff6d_ff6f: [0; 0x3],
                svbk: [0; 0x1],
                ff71_ff75: [0; 0x5],
                pcm12: [0; 0x1],
                pcm34: [0; 0x1],
                ff78_ff7f: [0; 0x8],
                hram: [0; 0x7f],
                ie: [0; 0x1],
            },
            joypad: Joypad {
                up: false,
                down: false,
                left: false,
                right: false,
                a: false,
                b: false,
                start: false,
                select: false,
            },
        }
    }

    fn step(&mut self) {
        let ins = self.address_space.read(self.cpu.pc);
        let n = self.address_space.read(self.cpu.pc.wrapping_add(1));
        let nn = (self.address_space.read(self.cpu.pc.wrapping_add(2)) as u16) << 8
            | (self.address_space.read(self.cpu.pc.wrapping_add(1)) as u16);
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

        let (_cycles_to_wait, pc_bump) = match [7, 6, 5, 4, 3, 2, 1, 0].map(|x| (ins >> x) & 1) {
            // LD r, r'
            [0, 1, _, _, _, _, _, _] if r1 != 0b110 && r2 != 0b110 => {
                let r = r1;
                let r_prime = r2;
                let val = self.cpu.getreg(r_prime);
                self.cpu.setreg(r, val);
                (1, 1)
            }
            // LD r, n
            [0, 0, _, _, _, 1, 1, 0] if r1 != 0b110 => {
                let r = r1;
                self.cpu.setreg(r, n);
                (2, 2)
            }
            // LD r, (HL)
            [0, 1, _, _, _, 1, 1, 0] if r1 != 0b110 => {
                let val = self.address_space.read(self.cpu.hl());
                let r = r1;
                self.cpu.setreg(r, val);
                (2, 1)
            }
            // LD (HL), r
            [0, 1, 1, 1, 0, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.address_space.write(self.cpu.hl(), self.cpu.getreg(r));
                (2, 1)
            }
            // LD (HL), n
            [0, 0, 1, 1, 0, 1, 1, 0] => {
                self.address_space.write(self.cpu.hl(), n);
                (3, 2)
            }
            // LD A, (BC)
            [0, 0, 0, 0, 1, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(self.cpu.bc());
                (2, 1)
            }
            // LD A, (DE)
            [0, 0, 0, 1, 1, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(self.cpu.de());
                (2, 1)
            }
            // LD A, (C)
            [1, 1, 1, 1, 0, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(0xff00 | (self.cpu.c as u16));
                (2, 1)
            }
            // LD (C), A
            [1, 1, 1, 0, 0, 0, 1, 0] => {
                self.address_space
                    .write(0xff00 | (self.cpu.c as u16), self.cpu.a);
                (2, 1)
            }
            // LD A, (n)
            [1, 1, 1, 1, 0, 0, 0, 0] => {
                self.cpu.a = self.address_space.read(0xff00 | (n as u16));
                (3, 2)
            }
            // LD (n), A
            [1, 1, 1, 0, 0, 0, 0, 0] => {
                self.address_space.write(0xff00 | (n as u16), self.cpu.a);
                (3, 2)
            }
            // LD A, (nn)
            [1, 1, 1, 1, 1, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(nn);
                (4, 3)
            }
            // LD (nn), A
            [1, 1, 1, 0, 1, 0, 1, 0] => {
                self.address_space.write(nn, self.cpu.a);
                (4, 3)
            }
            // LD A, (HLI)
            [0, 0, 1, 0, 1, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(self.cpu.hl());
                let carry: bool;
                (self.cpu.l, carry) = self.cpu.l.overflowing_add(1);
                self.cpu.h = self.cpu.h.wrapping_add(u8::from(carry));
                (2, 1)
            }
            // LD A, (HLD)
            [0, 0, 1, 1, 1, 0, 1, 0] => {
                self.cpu.a = self.address_space.read(self.cpu.hl());
                let carry: bool;
                (self.cpu.l, carry) = self.cpu.l.overflowing_sub(1);
                self.cpu.h = self.cpu.h.wrapping_sub(u8::from(carry));
                (2, 1)
            }
            // LD (BC), A
            [0, 0, 0, 0, 0, 0, 1, 0] => {
                self.address_space.write(self.cpu.bc(), self.cpu.a);
                (2, 1)
            }
            // LD (DE), A
            [0, 0, 0, 1, 0, 0, 1, 0] => {
                self.address_space.write(self.cpu.de(), self.cpu.a);
                (2, 1)
            }
            // LD (HLI), A
            [0, 0, 1, 0, 0, 0, 1, 0] => {
                self.address_space.write(self.cpu.hl(), self.cpu.a);
                let carry: bool;
                (self.cpu.l, carry) = self.cpu.l.overflowing_add(1);
                self.cpu.h = self.cpu.h.wrapping_add(u8::from(carry));
                (2, 1)
            }
            // LD (HLD), A
            [0, 0, 1, 1, 0, 0, 1, 0] => {
                self.address_space.write(self.cpu.hl(), self.cpu.a);
                let carry: bool;
                (self.cpu.l, carry) = self.cpu.l.overflowing_sub(1);
                self.cpu.h = self.cpu.h.wrapping_sub(u8::from(carry));
                (2, 1)
            }
            // LD dd, nn
            [0, 0, _, _, 0, 0, 0, 1] => {
                let hi = (nn >> 8) as u8;
                let lo = nn as u8;
                match dd {
                    0b00 => {
                        self.cpu.b = hi;
                        self.cpu.c = lo;
                    }
                    0b01 => {
                        self.cpu.d = hi;
                        self.cpu.e = lo;
                    }
                    0b10 => {
                        self.cpu.h = hi;
                        self.cpu.l = lo;
                    }
                    0b11 => {
                        self.cpu.sp = nn;
                    }
                    _ => panic!("Invalid register!"),
                }
                (3, 3)
            }
            // LD SP, HL
            [1, 1, 1, 1, 1, 0, 0, 1] => {
                self.cpu.sp = self.cpu.hl();
                (2, 1)
            }
            // PUSH qq
            [1, 1, _, _, 0, 1, 0, 1] => {
                let (hi, lo) = match qq {
                    0b00 => (self.cpu.b, self.cpu.c),
                    0b01 => (self.cpu.d, self.cpu.e),
                    0b10 => (self.cpu.h, self.cpu.l),
                    0b11 => (
                        self.cpu.a,
                        u8::from(self.cpu.flag_z) << 7
                            | u8::from(self.cpu.flag_n) << 6
                            | u8::from(self.cpu.flag_h) << 5
                            | u8::from(self.cpu.flag_z) << 4,
                    ),
                    _ => panic!("Invalid register!"),
                };
                self.address_space.write(self.cpu.sp.wrapping_sub(1), hi);
                self.address_space.write(self.cpu.sp.wrapping_sub(2), lo);
                self.cpu.sp = self.cpu.sp.wrapping_sub(2);
                (4, 1)
            }
            // POP qq
            [1, 1, _, _, 0, 0, 0, 1] => {
                let lo = self.address_space.read(self.cpu.sp);
                let hi = self.address_space.read(self.cpu.sp.wrapping_add(1));
                match qq {
                    0b00 => {
                        self.cpu.c = lo;
                        self.cpu.b = hi;
                    }
                    0b01 => {
                        self.cpu.e = lo;
                        self.cpu.d = hi;
                    }
                    0b10 => {
                        self.cpu.l = lo;
                        self.cpu.h = hi;
                    }
                    0b11 => {
                        self.cpu.flag_z = lo >> 7 != 0;
                        self.cpu.flag_n = ((lo >> 6) & 0b10) != 0;
                        self.cpu.flag_h = ((lo >> 5) & 0b100) != 0;
                        self.cpu.flag_cy = ((lo >> 4) & 0b1000) != 0;
                        self.cpu.a = hi;
                    }
                    _ => panic!("Invalid register!"),
                }
                self.cpu.sp = self.cpu.sp.wrapping_add(2);
                (1, 3)
            }
            // LDHL SP, e
            [1, 1, 1, 1, 1, 0, 0, 0] => {
                let sum: u16;
                (sum, self.cpu.flag_cy) = self.cpu.sp.overflowing_add(e);
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                self.cpu.h = (sum >> 8) as u8;
                self.cpu.l = sum as u8;
                // TODO: handle H flag
                (3, 2)
            }
            // LD (nn), SP
            [0, 0, 0, 0, 1, 0, 0, 0] => {
                self.address_space.write(nn, self.cpu.sp as u8);
                self.address_space
                    .write(nn.wrapping_add(1), (self.cpu.sp >> 8) as u8);
                (5, 3)
            }
            // ADD A, r
            [1, 0, 0, 0, 0, _, _, _] if r2 != 0b110 => {
                let r = r2;
                (self.cpu.a, self.cpu.flag_cy) = self.cpu.a.overflowing_add(self.cpu.getreg(r));
                // TODO: handle H flag
                self.cpu.flag_n = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // ADD A, n
            [1, 1, 0, 0, 0, 1, 1, 0] => {
                // TODO: handle H flag
                self.cpu.flag_n = false;
                (self.cpu.a, self.cpu.flag_cy) = self.cpu.a.overflowing_add(n);
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 2)
            }
            // ADD A, (HL)
            [1, 0, 0, 0, 0, 1, 1, 0] => {
                // TODO: handle H flag
                self.cpu.flag_n = false;
                (self.cpu.a, self.cpu.flag_cy) = self
                    .cpu
                    .a
                    .overflowing_add(self.address_space.read(self.cpu.hl()));
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 1)
            }
            // ADC A, r
            [1, 0, 0, 0, 1, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.cpu.a += self.cpu.getreg(r) + u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = false;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_add
                (1, 1)
            }
            // ADC A, n
            [1, 1, 0, 0, 1, 1, 1, 0] => {
                self.cpu.a += n + u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = false;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_add
                (2, 2)
            }
            // ADC A, (HL)
            [1, 0, 0, 0, 1, 1, 1, 0] => {
                self.cpu.a += self.address_space.read(self.cpu.hl()) + u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = false;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_add
                (2, 1)
            }
            // SUB r
            [1, 0, 0, 1, 0, _, _, _] if r2 != 0b110 => {
                let r = r2;
                (self.cpu.a, self.cpu.flag_cy) = self.cpu.a.overflowing_sub(self.cpu.getreg(r));
                // TODO: handle H flag
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // SUB n
            [1, 1, 0, 1, 0, 1, 1, 0] => {
                (self.cpu.a, self.cpu.flag_cy) = self.cpu.a.overflowing_sub(n);
                self.cpu.flag_z = self.cpu.a == 0;
                self.cpu.flag_n = true;
                // TODO: handle H flag
                (2, 2)
            }
            // SUB (HL)
            [1, 0, 0, 1, 0, 1, 1, 0] => {
                (self.cpu.a, self.cpu.flag_cy) = self
                    .cpu
                    .a
                    .overflowing_sub(self.address_space.read(self.cpu.hl()));
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                (2, 1)
            }
            // SBC A, r
            [1, 0, 0, 1, 1, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.cpu.a -= self.cpu.getreg(r) - u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_sub
                (1, 1)
            }
            // SBC A, n
            [1, 1, 0, 1, 1, 1, 1, 0] => {
                self.cpu.a -= n - u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_sub
                (2, 2)
            }
            // SBC A, (HL)
            [1, 0, 0, 1, 1, 1, 1, 0] => {
                self.cpu.a -= self.address_space.read(self.cpu.hl()) - u8::from(self.cpu.flag_cy);
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.cpu.a == 0;
                // TODO: handle H flag
                // TODO: handle CY flag
                // TODO: use overflowing_sub
                (2, 1)
            }
            // AND r
            [1, 0, 1, 0, 0, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.cpu.a &= self.cpu.getreg(r);
                self.cpu.flag_h = true;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // AND n
            [1, 1, 1, 0, 0, 1, 1, 0] => {
                self.cpu.flag_cy = false;
                self.cpu.flag_h = true;
                self.cpu.flag_n = false;
                self.cpu.a ^= n;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 2)
            }
            // AND (HL)
            [1, 0, 1, 0, 0, 1, 1, 0] => {
                self.cpu.a &= self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = false;
                self.cpu.flag_n = false;
                self.cpu.flag_h = true;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 1)
            }
            // OR r
            [1, 0, 1, 1, 0, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.cpu.a |= self.cpu.getreg(r);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // OR n
            [1, 1, 1, 1, 0, 1, 1, 0] => {
                self.cpu.flag_cy = false;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.a |= n;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 2)
            }
            // OR (HL)
            [1, 0, 1, 1, 0, 1, 1, 0] => {
                self.cpu.a |= self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = false;
                self.cpu.flag_n = false;
                self.cpu.flag_h = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 1)
            }
            // XOR r
            [1, 0, 1, 0, 1, _, _, _] if r2 != 0b110 => {
                let r = r2;
                self.cpu.a ^= self.cpu.getreg(r);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // XOR n
            [1, 1, 1, 0, 1, 1, 1, 0] => {
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = false;
                self.cpu.a ^= n;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 2)
            }
            // XOR (HL)
            [1, 0, 1, 0, 1, 1, 1, 0] => {
                self.cpu.a ^= self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = false;
                self.cpu.flag_n = false;
                self.cpu.flag_h = false;
                self.cpu.flag_z = self.cpu.a == 0;
                (2, 1)
            }
            // CP r
            [1, 0, 1, 1, 1, _, _, _] if r2 != 0b110 => {
                let r = r2;
                // TODO: handle H flag
                let diff: u8;
                (diff, self.cpu.flag_cy) = self.cpu.a.overflowing_sub(self.cpu.getreg(r));
                self.cpu.flag_n = true;
                self.cpu.flag_z = diff == 0;
                (1, 1)
            }
            // CP n
            [1, 1, 1, 1, 1, 1, 1, 0] => {
                // TODO: handle H flag
                let diff: u8;
                (diff, self.cpu.flag_cy) = self.cpu.a.overflowing_sub(n);
                self.cpu.flag_z = diff == 0;
                self.cpu.flag_n = true;
                (2, 2)
            }
            // CP (HL)
            [1, 0, 1, 1, 1, 1, 1, 0] => {
                // TODO: handle H flag
                let diff: u8;
                (diff, self.cpu.flag_cy) = self
                    .cpu
                    .a
                    .overflowing_sub(self.address_space.read(self.cpu.hl()));
                self.cpu.flag_z = diff == 0;
                self.cpu.flag_n = true;
                (2, 2)
            }
            // INC r
            [0, 0, _, _, _, 1, 0, 0] if r1 != 0b110 => {
                let r = r1;
                // TODO: handle H flag
                self.cpu.flag_n = false;
                let new_val = self.cpu.getreg(r).wrapping_add(1);
                self.cpu.setreg(r, new_val);
                self.cpu.flag_z = new_val == 0;
                (1, 1)
            }
            // INC (HL)
            [0, 0, 1, 1, 0, 1, 0, 0] => {
                self.address_space.write(
                    self.cpu.hl(),
                    self.address_space.read(self.cpu.hl()).wrapping_add(1),
                );
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.address_space.read(self.cpu.hl()) == 0;
                // TODO: handle H flag
                (3, 1)
            }
            // DEC r
            [0, 0, _, _, _, 1, 0, 1] if r1 != 0b110 => {
                let r = r1;
                let val = self.cpu.getreg(r).wrapping_sub(1);
                self.cpu.setreg(r, val);
                // TODO: handle H flag
                self.cpu.flag_n = true;
                self.cpu.flag_z = val == 0;
                (1, 1)
            }
            // DEC (HL)
            [0, 0, 1, 1, 0, 1, 0, 1] => {
                self.address_space.write(
                    self.cpu.hl(),
                    self.address_space.read(self.cpu.hl()).wrapping_sub(1),
                );
                self.cpu.flag_n = true;
                self.cpu.flag_z = self.address_space.read(self.cpu.hl()) == 0;
                // TODO: handle H flag
                (3, 1)
            }
            // ADD HL, ss
            [0, 0, _, _, 1, 0, 0, 1] => {
                // TODO: handle H flag
                let sum: u16;
                (sum, self.cpu.flag_cy) = self.cpu.hl().overflowing_add(match ss {
                    0b00 => self.cpu.bc(),
                    0b01 => self.cpu.de(),
                    0b10 => self.cpu.hl(),
                    0b11 => self.cpu.sp,
                    _ => panic!("Invalid register!"),
                });
                self.cpu.h = (sum >> 8) as u8;
                self.cpu.l = sum as u8;
                self.cpu.flag_n = true;
                (2, 1)
            }
            // ADD SP, e
            [1, 1, 1, 0, 1, 0, 0, 0] => {
                (self.cpu.sp, self.cpu.flag_cy) = self.cpu.sp.overflowing_add(e);
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                // TODO: handle H flag
                (4, 2)
            }
            // INC ss
            [0, 0, _, _, 0, 0, 1, 1] => {
                let carry: bool;
                match ss {
                    0b00 => {
                        (self.cpu.c, carry) = self.cpu.c.overflowing_add(1);
                        self.cpu.b = self.cpu.b.wrapping_add(u8::from(carry));
                    }
                    0b01 => {
                        (self.cpu.e, carry) = self.cpu.e.overflowing_add(1);
                        self.cpu.d = self.cpu.d.wrapping_add(u8::from(carry));
                    }
                    0b10 => {
                        (self.cpu.l, carry) = self.cpu.l.overflowing_add(1);
                        self.cpu.h = self.cpu.h.wrapping_add(u8::from(carry));
                    }
                    0b11 => self.cpu.sp += 1,
                    _ => panic!("Invalid register!"),
                }
                (1, 2)
            }
            // DEC ss
            [0, 0, _, _, 1, 0, 1, 1] => {
                let carry: bool;
                match ss {
                    0b00 => {
                        (self.cpu.c, carry) = self.cpu.c.overflowing_sub(1);
                        self.cpu.b = self.cpu.b.wrapping_sub(u8::from(carry));
                    }
                    0b01 => {
                        (self.cpu.e, carry) = self.cpu.e.overflowing_sub(1);
                        self.cpu.d = self.cpu.d.wrapping_sub(u8::from(carry));
                    }
                    0b10 => {
                        (self.cpu.l, carry) = self.cpu.l.overflowing_sub(1);
                        self.cpu.h = self.cpu.h.wrapping_sub(u8::from(carry));
                    }
                    0b11 => self.cpu.sp += 1,
                    _ => panic!("Invalid register!"),
                }
                (1, 2)
            }
            // RLCA
            [0, 0, 0, 0, 0, 1, 1, 1] => {
                self.cpu.flag_cy = (self.cpu.a >> 7) != 0;
                self.cpu.a <<= 1;
                self.cpu.a |= u8::from(self.cpu.flag_cy);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (1, 1)
            }
            // RLA
            [0, 0, 0, 1, 0, 1, 1, 1] => {
                let cy: u8 = u8::from(self.cpu.flag_cy);
                self.cpu.flag_cy = (self.cpu.a >> 7) != 0;
                self.cpu.a <<= 1;
                self.cpu.a |= cy;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (1, 1)
            }
            // RRCA
            [0, 0, 0, 0, 1, 1, 1, 1] => {
                self.cpu.flag_cy = (self.cpu.a & 0b1) != 0;
                self.cpu.a >>= 1;
                self.cpu.a |= (u8::from(self.cpu.flag_cy)) << 7;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (1, 1)
            }
            // RRA
            [0, 0, 0, 1, 1, 1, 1, 1] => {
                let cy: u8 = u8::from(self.cpu.flag_cy) << 7;
                self.cpu.flag_cy = (self.cpu.a & 0b1) != 0;
                self.cpu.a >>= 1;
                self.cpu.a |= cy;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (1, 1)
            }
            // RLC r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00000 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val >> 7) != 0;
                let new_val = (val << 1) | u8::from(self.cpu.flag_cy);
                self.cpu.setreg(r, new_val);
                self.address_space
                    .write(self.cpu.hl(), (val << 1) | u8::from(self.cpu.flag_cy));
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (2, 2)
            }
            // RLC (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00000110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val >> 7) != 0;
                self.address_space
                    .write(self.cpu.hl(), (val << 1) | u8::from(self.cpu.flag_cy));
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (4, 2)
            }
            // RL r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00010 && r3 != 0b110 => {
                let r = r3;
                let cy: u8 = u8::from(self.cpu.flag_cy);
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val >> 7) != 0;
                let new_val = (val << 1) | cy;
                self.cpu.setreg(r, new_val);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (2, 2)
            }
            // RL (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00010110 => {
                let cy: u8 = u8::from(self.cpu.flag_cy);
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val >> 7) != 0;
                self.address_space.write(self.cpu.hl(), (val << 1) | cy);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (4, 2)
            }
            // RRC r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00001 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = (val >> 1) | (u8::from(self.cpu.flag_cy) << 7);
                self.cpu.setreg(r, new_val);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (2, 2)
            }
            // RRC (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00001110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val & 0b1) != 0;
                self.address_space.write(
                    self.cpu.hl(),
                    (val >> 1) | (u8::from(self.cpu.flag_cy) << 7),
                );
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = false;
                (4, 2)
            }
            // RR r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00011 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = (u8::from(self.cpu.flag_cy) << 7) | (val >> 1);
                self.cpu.setreg(r, new_val);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = new_val == 0;
                (2, 2)
            }
            // RR (HL)
            [1, 1, 0, 1, 1, 0, 1, 1] if n == 0b00011110 => {
                let cy: u8 = u8::from(self.cpu.flag_cy) << 7;
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = (cy << 7) | (val >> 1);
                self.address_space.write(self.cpu.hl(), new_val);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = new_val == 0;
                (4, 2)
            }
            // SLA r
            [1, 1, 0, 0, 1, 0, 1, 1] if n >> 3 == 0b00100 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val >> 7) != 0;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                let new_val = val << 1;
                self.cpu.flag_z = new_val == 0;
                self.cpu.setreg(r, new_val);
                (2, 2)
            }
            // SLA (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n >> 3 == 0b00100 && r3 != 0b110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val >> 7) != 0;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                let new_val = val << 1;
                self.cpu.flag_z = new_val == 0;
                self.address_space.write(self.cpu.hl(), new_val);
                (2, 2)
            }
            // SRA r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00101 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = (u8::from(self.cpu.flag_cy) << 7) | val >> 1;
                self.cpu.flag_z = new_val == 0;
                self.cpu.setreg(r, new_val);
                (2, 2)
            }
            // SRA (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00101110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = (u8::from(self.cpu.flag_cy) << 7) | val >> 1;
                self.cpu.flag_z = new_val == 0;
                self.address_space.write(self.cpu.hl(), new_val);
                (4, 2)
            }
            // SRL r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00111 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = (val & 0b1) != 0;
                let new_val = val >> 1;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.setreg(r, new_val);
                self.cpu.flag_z = new_val == 0;
                (2, 2)
            }
            // SRL (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00111110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = (val & 0b1) != 0;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                self.cpu.flag_z = (val >> 1) == 0;
                self.address_space.write(self.cpu.hl(), val >> 1);
                (4, 2)
            }
            // SWAP r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 3) == 0b00110 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_cy = false;
                self.cpu.flag_n = false;
                self.cpu.flag_h = false;
                self.cpu.flag_z = val == 0;
                let new_val = (val << 4) | (val >> 4);
                self.cpu.setreg(r, new_val);
                (2, 2)
            }
            // SWAP (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if n == 0b00110110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_cy = false;
                self.cpu.flag_n = false;
                self.cpu.flag_h = false;
                self.cpu.flag_z = val == 0;
                self.address_space
                    .write(self.cpu.hl(), (val << 4) | (val >> 4));
                (4, 2)
            }
            // BIT b, r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b01 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                self.cpu.flag_n = false;
                self.cpu.flag_h = true;
                self.cpu.flag_z = ((val >> b) & 0b1) != 0;
                (2, 2)
            }
            // BIT b, (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b01000110 => {
                let val = self.address_space.read(self.cpu.hl());
                self.cpu.flag_n = false;
                self.cpu.flag_h = true;
                self.cpu.flag_z = ((val >> b) & 0b1) != 0;
                (3, 2)
            }
            // SET b, r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b11 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                let new_val = val | (1 << b);
                self.cpu.setreg(r, new_val);
                (2, 2)
            }
            // SET b, (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b11000110 => {
                let val = self.address_space.read(self.cpu.hl());
                let new_val = val | (1 << b);
                self.address_space.write(self.cpu.hl(), new_val);
                (4, 2)
            }
            // RES b, r
            [1, 1, 0, 0, 1, 0, 1, 1] if (n >> 6) == 0b10 && r3 != 0b110 => {
                let r = r3;
                let val = self.cpu.getreg(r);
                let new_val = val & !(1 << b);
                self.cpu.setreg(r, new_val);
                (2, 2)
            }
            // RES b, (HL)
            [1, 1, 0, 0, 1, 0, 1, 1] if (n & 0b11000111) == 0b10000110 => {
                let val = self.address_space.read(self.cpu.hl());
                let new_val = val & !(1 << b);
                self.address_space.write(self.cpu.hl(), new_val);
                (4, 2)
            }
            // JP nn
            [1, 1, 0, 0, 0, 0, 1, 1] => {
                self.cpu.pc = nn;
                (4, 0)
            }
            // JP cc, nn
            [1, 1, 0, _, _, 0, 1, 0] => {
                if match cc {
                    0b00 => !self.cpu.flag_z,
                    0b01 => self.cpu.flag_z,
                    0b10 => !self.cpu.flag_cy,
                    0b11 => self.cpu.flag_cy,
                    _ => panic!("Invalid condition!"),
                } {
                    self.cpu.pc = nn;
                    (4, 0)
                } else {
                    (3, 3)
                }
            }
            // JR e
            [0, 0, 0, 1, 1, 0, 0, 0] => {
                self.cpu.pc = self.cpu.pc.wrapping_add(e.wrapping_add(2));
                (3, 0)
            }
            // JR cc, e
            [0, 0, 1, _, _, 0, 0, 0] => {
                if match cc {
                    0b00 => !self.cpu.flag_z,
                    0b01 => self.cpu.flag_z,
                    0b10 => !self.cpu.flag_cy,
                    0b11 => self.cpu.flag_cy,
                    _ => panic!("Invalid condition!"),
                } {
                    self.cpu.pc = self.cpu.pc.wrapping_add(e.wrapping_add(2));
                    (3, 0)
                } else {
                    (2, 2)
                }
            }
            // JP (HL)
            [1, 1, 1, 0, 1, 0, 0, 1] => {
                self.cpu.pc = self.cpu.hl();
                (1, 0)
            }
            // CALL nn
            [1, 1, 0, 0, 1, 1, 0, 1] => {
                self.address_space
                    .write(self.cpu.sp.wrapping_sub(1), (self.cpu.pc >> 8) as u8);
                self.address_space
                    .write(self.cpu.sp.wrapping_sub(2), self.cpu.pc as u8);
                self.cpu.pc = nn;
                self.cpu.sp = self.cpu.sp.wrapping_sub(2);
                (6, 0)
            }
            // CALL cc, nn
            [1, 1, 0, _, _, 1, 0, 0] => {
                if match cc {
                    0b00 => !self.cpu.flag_z,
                    0b01 => self.cpu.flag_z,
                    0b10 => !self.cpu.flag_cy,
                    0b11 => self.cpu.flag_cy,
                    _ => panic!("Invalid condition!"),
                } {
                    self.address_space
                        .write(self.cpu.sp.wrapping_sub(1), (self.cpu.pc >> 8) as u8);
                    self.address_space
                        .write(self.cpu.sp.wrapping_sub(2), self.cpu.pc as u8);
                    self.cpu.pc = nn;
                    self.cpu.sp = self.cpu.sp.wrapping_sub(2);
                    (6, 0)
                } else {
                    (3, 3)
                }
            }
            // RET
            [1, 1, 0, 0, 1, 0, 0, 1] => {
                self.cpu.pc = ((self.address_space.read(self.cpu.sp.wrapping_add(1)) as u16) << 8)
                    | (self.address_space.read(self.cpu.sp) as u16);
                self.cpu.sp = self.cpu.sp.wrapping_add(2);
                (4, 0)
            }
            // RETI
            [1, 1, 0, 1, 1, 0, 0, 1] => {
                self.cpu.pc = ((self.address_space.read(self.cpu.sp.wrapping_add(1)) as u16) << 8)
                    | (self.address_space.read(self.cpu.sp) as u16);
                self.cpu.sp = self.cpu.sp.wrapping_add(2);
                self.cpu.ime = true;
                (4, 0)
            }
            // RET cc
            [1, 1, 0, _, _, 0, 0, 0] => {
                if match cc {
                    0b00 => !self.cpu.flag_z,
                    0b01 => self.cpu.flag_z,
                    0b10 => !self.cpu.flag_cy,
                    0b11 => self.cpu.flag_cy,
                    _ => panic!("Invalid condition!"),
                } {
                    self.cpu.pc = ((self.address_space.read(self.cpu.sp.wrapping_add(1)) as u16)
                        << 8)
                        | (self.address_space.read(self.cpu.sp) as u16);
                    self.cpu.sp = self.cpu.sp.wrapping_add(2);
                    (5, 0)
                } else {
                    (2, 1)
                }
            }
            // RST t
            [1, 1, _, _, _, 1, 1, 1] => {
                self.address_space
                    .write(self.cpu.sp.wrapping_sub(1), (self.cpu.pc >> 8) as u8);
                self.address_space
                    .write(self.cpu.sp.wrapping_sub(2), self.cpu.pc as u8);
                self.cpu.sp = self.cpu.sp.wrapping_sub(2);
                self.cpu.pc = match t {
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
                if self.cpu.flag_n {
                    if self.cpu.flag_cy {
                        addend -= 0x60;
                    }
                    if self.cpu.flag_h {
                        addend -= 6;
                    }
                } else {
                    if self.cpu.flag_cy || self.cpu.a > 0x99 {
                        addend += 0x60;
                        carry = true;
                    }
                    if self.cpu.flag_h || (self.cpu.a & 0b1111) > 9 {
                        addend += 6;
                    }
                }
                self.cpu.flag_cy = carry;
                self.cpu.flag_h = false;
                self.cpu.a = self.cpu.a.wrapping_add(addend);
                self.cpu.flag_z = self.cpu.a == 0;
                (1, 1)
            }
            // CPL
            [0, 0, 1, 0, 1, 1, 1, 1] => {
                self.cpu.a = !self.cpu.a;
                self.cpu.flag_h = true;
                self.cpu.flag_n = true;
                (1, 1)
            }
            // NOP
            [0, 0, 0, 0, 0, 0, 0, 0] => (1, 1),
            // CCF
            [0, 0, 1, 1, 1, 1, 1, 1] => {
                self.cpu.flag_cy = !self.cpu.flag_cy;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                (1, 1)
            }
            // SCF
            [0, 0, 1, 1, 0, 1, 1, 1] => {
                self.cpu.flag_cy = true;
                self.cpu.flag_h = false;
                self.cpu.flag_n = false;
                (1, 1)
            }
            // DI
            [1, 1, 1, 1, 0, 0, 1, 1] => {
                self.cpu.ime = false;
                (1, 1)
            }
            // EI
            [1, 1, 1, 1, 1, 0, 1, 1] => {
                self.cpu.ime = true;
                (1, 1)
            }
            // HALT
            [0, 1, 1, 1, 0, 1, 1, 0] => {
                self.cpu.halted = true;
                (1, 1)
            }
            // STOP
            [0, 0, 0, 1, 0, 0, 0, 0] => {
                self.cpu.stopped = true;
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
        };
        self.cpu.pc = self.cpu.pc.wrapping_add(pc_bump);
    }
}

use std::env;
use std::fs::File;
use std::io::Read;
use std::process;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;

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

    let sdl_context = sdl2::init().expect("Couldn't initialize SDL2");
    let video_subsystem = sdl_context
        .video()
        .expect("Couldn't initialize video subsystem");

    let window = video_subsystem
        .window("grambo", 160, 144)
        .position_centered()
        .build()
        .expect("Couldn't build window");

    let mut canvas = window.into_canvas().build().expect("Couldn't build canvas");
    canvas.present();
    let mut event_pump = sdl_context.event_pump().expect("Couldn't make event pump");

    let mut gb = GameBoy::new(&mut fixed_rom, &mut swappable_rom);

    'gameloop: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'gameloop,
                Event::KeyDown {
                    keycode: Some(Keycode::W),
                    ..
                } => gb.joypad.key_down(Button::Up),
                Event::KeyDown {
                    keycode: Some(Keycode::S),
                    ..
                } => gb.joypad.key_down(Button::Down),
                Event::KeyDown {
                    keycode: Some(Keycode::A),
                    ..
                } => gb.joypad.key_down(Button::Left),
                Event::KeyDown {
                    keycode: Some(Keycode::D),
                    ..
                } => gb.joypad.key_down(Button::Right),
                Event::KeyDown {
                    keycode: Some(Keycode::E),
                    ..
                } => gb.joypad.key_down(Button::A),
                Event::KeyDown {
                    keycode: Some(Keycode::Q),
                    ..
                } => gb.joypad.key_down(Button::B),
                Event::KeyDown {
                    keycode: Some(Keycode::LShift),
                    ..
                } => gb.joypad.key_down(Button::Start),
                Event::KeyDown {
                    keycode: Some(Keycode::RShift),
                    ..
                } => gb.joypad.key_down(Button::Select),
                _ => println!("Got an event!"),
            }
        }
        gb.step();
        canvas.present();
    }
}
