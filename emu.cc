#include <assert.h> // assert()
#include <string.h> // memcmp
#include <stdint.h> // [u]int[\d]_t
#include <iostream> // cout
#include <fstream>  // fstream
#include <vector>   // std::vector<>

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

using i8  = int8_t;
using i16 = int16_t;

using std::cout;
using std::cerr;
using std::endl;
using std::size_t;

static constexpr u32 CLOCK_HZ = 4194304;

enum struct CART_TYPE {
    UNKNOWN = -1,
    ROMONLY,
    MBC1,
    MBC2,
    MBC3,
    MBC4,
    MBC5,
    MBC6,
    MBC7
};

#define PICNIC(message) do { cerr << __LINE__ << " " << \
    __FUNCTION__ << " :" message << endl; abort(); } while(0)

struct Memory;

struct Cartridge
{
    std::vector<u8> rom;
    std::vector<u8> ram;
    u16 rompos;
    u16 rampos;
    CART_TYPE type;
    bool ram_enabled;

    void store(u16 address, u8 value);
    void store5(u16 address, u8 value);
    u8   load(u16 address);
    u8   load5(u16 address);
};

void
Cartridge::store(u16 address, u8 value)
{
    switch(type) {
    case CART_TYPE::MBC5:
        store5(address, value); break;
    default:
        abort(); // nope
    }
}

u8
Cartridge::load(u16 address)
{
    switch(type) {
    case CART_TYPE::MBC5:
        return load5(address);
    default:
        abort(); // nope
    }
}

void
Cartridge::store5(u16 address, u8 value)
{
    if (address >= 0x0000 && address < 0x2000) {
        ram_enabled = ((value & 0b00001111) == 0x0a); // RAM Enable
        if (!ram_enabled) {
            // TODO: STORE IN A FILE
        }
    } else if (address >= 0x2000 && address < 0x3000)
        rompos = ((rompos & 0x0100) | (value & 0x00ff)); // ROM Bank Number
    else if (address >= 0x3000 && address < 0x4000)
        // High bit of ROM Bank Number
        rompos = ((rompos & 0x00ff) | (((u16)value << 8) & 0x0100));
    else if (address >= 0x4000 && address < 0x6000)
        rampos = (value & 0x0f); // RAM Bank Number
    else if (address >= 0xa000 && address < 0xc000)
        ram[0x2000 * rampos + address - 0xa000] = value;
    else
        PICNIC("Tried to write to an invalid address! " << value);
}

u8
Cartridge::load5(u16 address)
{
    if (address >= 0x0000 && address < 0x4000)
        return rom[address];
    else if (address >= 0x4000 && address < 0x8000)
        return rom[0x4000 * rompos + address - 0x4000];
    else if (address >= 0xa000 && address < 0xc000)
        return ram[0x2000 * rampos + address - 0xa000];
    else
        PICNIC("Tried to load from an invalid address! " << address);
}

struct MemoryElementProxy
{
    MemoryElementProxy(const u16 address, Memory& memory)
        : address(address), memory(memory){}

    // This is the implementation of setting
    void operator=(u8 value);

    // This is the implmentation of getting
    operator u8() const;

private:
    u16 address;
    Memory& memory;
};

struct Memory
{
    friend MemoryElementProxy;

    Memory(Cartridge& cartridge)
        : memory(0x10000), cartridge(cartridge){}

    MemoryElementProxy operator[](u16 address)
    {
        return MemoryElementProxy(address, *this);
    }

private:
    std::vector<u8> memory;
    Cartridge& cartridge;
};

// This is the implementation of setting
void
MemoryElementProxy::operator=(u8 value)
{
    if ((address >= 0x0000 && address < 0x8000) || (address >= 0xa000 && address < 0xc000))
        memory.cartridge.store(address, value);
    else
        memory.memory[address] = value;
}

MemoryElementProxy::operator u8() const
{
    if ((address >= 0x0000 && address < 0x8000) || (address >= 0xa000 && address < 0xc000))
        return memory.cartridge.load(address);
    else
        return memory.memory[address];
}

// special struct in order to perform
// instructions easily on register pairs
struct Regpair
{
    // Emulate the regpair by combining reg1
    // and reg2 together
    Regpair(u8& reg1, u8& reg2) : reg1(reg1), reg2(reg2)
    {
        reg = ((u16)reg1 << 8) | reg2;
    }

    // Evil hack
    operator u16() const { return reg; };

    // overload to decrement the register
    u16 operator--(int)
    {
        const u16 ret = reg;
        reg--; storesplit();
        return ret;
    }

    // overload to increment the register
    u16 operator++(int)
    {
        const u16 ret = reg; // Cache the old value
        reg++; storesplit(); // Update reg and store the splits
        return ret;
    }

    // overload for addition assignment
    Regpair& operator+=(u16 newvalue)
    {
        reg += newvalue;
        storesplit();
        return *this;
    }

    // overload to assign the register
    Regpair& operator=(u16 newvalue)
    {
        reg = newvalue; // Update the reg
        storesplit(); // Store the splits
        return *this; // Return the struct
    }

private:
    u16 reg;  // pseudo register
    u8& reg1; // first  register
    u8& reg2; // second register

    void storesplit()
    {
        reg1 = (u8)reg; // Throw away the upper 8 bits
        reg2 = (u8)(reg >> 8); // shift right, then throw away the upper 8 bits
    }
};

struct Gameboy
{
    Gameboy() : ram(cartridge) {}

    u8 A=0, B=0, C=0, D=0, E=0, H=0, L=0; // General purpose CPU registers
    u16 SP=0xfffe, PC=0x0100; // Stack pointer and program counter

    // F register
    struct Freg
    {
        // Shift the flags to the first 4 bits, then put the
        // remaining unused bits from LF at the end.
        operator u8() const
        {
            return (Z << 7) | (N << 6) | (H << 5) | (CY << 4) | LF;
        };

        Freg& operator=(u8 newvalue)
        {
            // Bit mask the value and corospond it
            // to each bit of the register.
            // For examples sake, think of it as
            //  ----------------------------------
            //  | Z | N | H | CY | - | - | - | - |
            //  ----------------------------------
            //0b  1   1   1    1   0   0   0   0
            //
            // Also set the values of LF to the last
            // 4 bits by masking out the first 4 bits.
            CY = newvalue & 0b00010000;
            H  = newvalue & 0b00100000;
            N  = newvalue & 0b01000000;
            Z  = newvalue & 0b10000000;
            LF = newvalue & 0b00001111;

            return *this;
        }

        bool Z=0, N=0, H=0, CY=0; // CPU Flags
        u8 LF=0; // Emulating the lower 4 bits of the F register
    } F;

    Cartridge cartridge; // cartridge cartridge cartridge
    Memory ram; // Ram persistence

    void execIns(); // Execute a single instruction
    void incCycle(unsigned); // Add to the clock ticks
    void loadROM(const char *); // Load ROM in to memory
    CART_TYPE getCartridgeTable(const u8); // Print memory cartridge information
    u16 getRomTable(const u8); // Get ROM information
    u8  getRamTable(const u8); // Get RAM information
    void runGame(); // The most useless part
    template<typename FTYPE> auto getReg(u8, FTYPE); // Parse the register
private:
    bool system_halt = false; // Designates when we HALT opcode processing
    bool IME = false; // TODO ?
    void sendInterrupt(const u8 itype); // Set by hardware
    void handleInterrupt(); // Handle interrupts
};

void
Gameboy::loadROM(const char *const filename)
{
    auto& rom = this->cartridge.rom;

    // Nintendo logo
    static constexpr u8 logo[] = {
        0xce, 0xed, 0x66, 0x66, 0xcc, 0x0d, 0x00, 0x0b, 0x03,
        0x73, 0x00, 0x83, 0x00, 0x0c, 0x00, 0x0d, 0x00, 0x08,
        0x11, 0x1f, 0x88, 0x89, 0x00, 0x0e, 0xdc, 0xcc, 0x6e,
        0xe6, 0xdd, 0xdd, 0xd9, 0x99, 0xbb, 0xbb, 0x67, 0x63,
        0x6e, 0x0e, 0xec, 0xcc, 0xdd, 0xdc, 0x99, 0x9f, 0xbb,
        0xb9, 0x33, 0x3e
    };

    {
        std::ifstream romfile(filename);
        romfile.seekg(0, std::ios::end);
        const size_t rom_size = romfile.tellg();
        romfile.seekg(0, std::ios::beg);
        rom.resize(rom_size);
        romfile.read(reinterpret_cast<char *>(rom.data()), rom_size);
    }

    if (memcmp(&rom[0x0104], logo, 48) != 0) {
        cout << "Invalid ROM detected, terminating." << endl;
        std::exit(EXIT_FAILURE);
    }

    char buf[16];
    memcpy(buf, &rom[0x134], 16);

    std::cout << "\n@@@@@@@@@@@@@@@@@@@@\n";
    std::cout << "@  " << buf << "  @" << std::endl;
    std::cout << "@@@@@@@@@@@@@@@@@@@@\n\n";

    cout << "Platform: GameBoy";

    // If the MSB of 0x0143 is 1, then the
    // color flag is turned on
    if ((rom[0x134] >> 7)) {
        cout << " Color";
        if (rom[0x134] == 0xc0) // Designates Color only
            cout << " (ONLY)";
    }
    cout << " (";
    cout << "0x" << std::hex << (unsigned)rom[0x134] << ")" << endl;

    const CART_TYPE cartType  = getCartridgeTable(rom[0x0147]);

    if (cartType == CART_TYPE::UNKNOWN)
        PICNIC("ERROR: Unknown Cartridge Type.");

    this->cartridge.type = cartType;
    const u16 bankCount = getRomTable(rom[0x0148]);
    size_t mb = bankCount * 0x4000;
    if (mb != rom.size())
        PICNIC("ERROR: ROM Size does not match file size.");

    const u8 ramSize   = getRamTable(rom[0x0149]);
    this->cartridge.ram.resize(ramSize * 1024);

    cout << "Destination Code: ";
    if (rom[0x014a] == 0x00)
        cout << "Japanese";
    else
        cout << "Non-Japanese";
    cout << " (0x" << std::hex << (unsigned)rom[0x014a] << ")" << endl;

    cout << "ROM Version: 0x" << std::hex << (unsigned)rom[0x14c] << endl;

    cout << "Checksum: ";
    const u8 checksumbyte = rom[0x14d];

    u8 x = 0;
    for (size_t i = 0x0134; i < 0x014d; ++i)
        x -= (rom[i] + 1);

    if (x != checksumbyte) {
        cout << "Failed";
        exit(EXIT_FAILURE);
    } else {
        cout << "Passed";
    }
    cout << endl;
}

CART_TYPE
Gameboy::getCartridgeTable(const u8 value)
{
    cout << "Cartridge Type: ";
    #define t(type, size) cout << type <<      \
        " (0x" << std::hex << (unsigned)((u8)value) \
        << ")" << endl; return size;

    switch(value) {
    case 0x00: t("ROM ONLY", CART_TYPE::ROMONLY)
    case 0x01: t("MBC1"    , CART_TYPE::MBC1)
    case 0x02: t("MBC1+RAM", CART_TYPE::MBC1)
    case 0x03: t("MBC1+RAM+BATTERY", CART_TYPE::MBC1)
    case 0x05: t("MBC2"    , CART_TYPE::MBC2)
    case 0x06: t("MBC2+BATTERY", CART_TYPE::MBC2)
    case 0x08: t("ROM+RAM", CART_TYPE::ROMONLY)
    case 0x09: t("ROM+RAM+BATTERY", CART_TYPE::ROMONLY)
    case 0x0B: t("MMM01", CART_TYPE::ROMONLY)
    case 0x0C: t("MMM01+RAM", CART_TYPE::ROMONLY)
    case 0x0D: t("MMM01+RAM+BATTERY", CART_TYPE::ROMONLY)
    case 0x0F: t("MBC3+TIMER+BATTERY", CART_TYPE::MBC3)
    case 0x10: t("MBC3+TIMER+RAM+BATTERY", CART_TYPE::MBC3)
    case 0x11: t("MBC3", CART_TYPE::MBC3)
    case 0x12: t("MBC3+RAM", CART_TYPE::MBC3)
    case 0x13: t("MBC3+RAM+BATTERY", CART_TYPE::MBC3)
    case 0x15: t("MBC4", CART_TYPE::MBC4)
    case 0x16: t("MBC4+RAM", CART_TYPE::MBC4)
    case 0x17: t("MBC4+RAM+BATTERY", CART_TYPE::MBC4)
    case 0x19: t("MBC5", CART_TYPE::MBC5)
    case 0x1A: t("MBC5+RAM", CART_TYPE::MBC5)
    case 0x1B: t("MBC5+RAM+BATTERY", CART_TYPE::MBC5)
    case 0x1C: t("MBC5+RUMBLE", CART_TYPE::MBC5)
    case 0x1D: t("MBC5+RUMBLE+RAM", CART_TYPE::MBC5)
    case 0x1E: t("MBC5+RUMBLE+RAM+BATTERY", CART_TYPE::MBC5)
    case 0x20: t("MBC6", CART_TYPE::MBC6)
    case 0x22: t("MBC7+SENSOR+RUMBLE+RAM+BATTERY", CART_TYPE::MBC7)
    case 0xfc: t("POCKET CAMERA", CART_TYPE::ROMONLY)
    case 0xfd: t("BANDAI TAMA5", CART_TYPE::ROMONLY)
    case 0xfe: t("HuC3", CART_TYPE::ROMONLY)
    case 0xff: t("HuC1+RAM+BATTERY", CART_TYPE::ROMONLY)
    default: t("UNKNOWN", CART_TYPE::UNKNOWN)
    }
    cout << " (0x" << std::hex << (unsigned)value << ")" << endl;
    #undef t

    return CART_TYPE::UNKNOWN;
}

u16
Gameboy::getRomTable(const u8 value)
{
    cout << "ROM Size: ";
    #define t(type, size) cout << type <<      \
        " (0x" << std::hex << (unsigned)((u8)value) \
        << ")" << endl; return size;

    switch(value) {
    case 0x00: t("32KByte (no ROM banking)", 0)
    case 0x01: t("64KByte (4 banks)",    4)
    case 0x02: t("128KByte (8 banks)",   8)
    case 0x03: t("256KByte (16 banks)", 16)
    case 0x04: t("512KByte (32 banks)", 32)
    case 0x05: t("1MByte (64 banks) - only 63 banks used by MBC1", 64)
    case 0x06: t("2MByte (128 banks) - only 125 banks used by MBC1", 128)
    case 0x07: t("4MByte (256 banks)", 256)
    case 0x52: t("1.1MByte (72 banks)", 72)
    case 0x53: t("1.2MByte (80 banks)", 80)
    case 0x54: t("1.5MByte (96 banks)", 96)
    default: t("UNKNOWN", 0)
    }
    #undef t
    return 0;
}

u8
Gameboy::getRamTable(const u8 value)
{
    cout << "RAM Size: ";
    #define t(type, size) cout << type <<         \
        " (0x" << std::hex << (unsigned)((u8)value) \
        << ")" << endl; return size;

    switch(value) {
    case 0x00: t("None",     0)
    case 0x01: t("2 KBytes", 2)
    case 0x02: t("8 Kbytes", 8)
    case 0x03: t("32 KBytes (4 banks of 8KBytes each)", 32)
    case 0x04: t("128 KBytes (16 banks of 8KBytes each)", 128)
    case 0x05: t("64 KBytes (8 banks of 8KBytes each)", 64)
    default: t("UNKNOWN", 0)
    }
    #undef t
    return 0;
}

// Safetely turn the unsigned byte in to
// a signed byte without any compiler or
// OS magic
i8
toSigned8(u8 byte)
{
    if (byte < 0x80)
        return (i8)byte;

    return (i8)(byte - 0x80) - 0x80;
}

void
Gameboy::incCycle(unsigned cycles)
{
    static u32 s_cycles;
    s_cycles += cycles;
    if (s_cycles == CLOCK_HZ) {
        // cout << "one second has passed" << endl;
        s_cycles = 0;
    } else if (s_cycles > CLOCK_HZ) {
        // cout << "PAST ONE SECOND?? " << std::dec << (s_cycles - CLOCK_HZ) << endl;
        s_cycles = 0;
    }
}

template<typename FTYPE>
auto Gameboy::getReg(u8 r, FTYPE f)
{
    Regpair HL(H, L);

    // TODO Fix me so incCycle() happens
    // after
    switch(r) {
    case 0b111: incCycle(8);  return f(A);
    case 0b000: incCycle(8);  return f(B);
    case 0b001: incCycle(8);  return f(C);
    case 0b010: incCycle(8);  return f(D);
    case 0b011: incCycle(8);  return f(E);
    case 0b100: incCycle(8);  return f(H);
    case 0b101: incCycle(8);  return f(L);
    case 0b110: incCycle(16); return f(ram[HL]);
    default: PICNIC("Unknown register in RES " << r);
    }
}

void
Gameboy::handleInterrupt()
{
    if (IME) {
        const u8 IE = ram[0xffff];
        const u8 IF = ram[0xff0f];

        // Nothing to handle
        if (IF == 0x0) { return; }

        IME = false;
        ram[--SP] = (u8)(PC >> 8);
        ram[--SP] = (u8)PC;

        if ((IF & 0b1) && (IE & 0b1)) {
            PC = 0x0040;
            ram[0xff0f] = (~0b1 & IF);
        } else if ((IF & 0b10) && (IE & 0b10)) {
            PC = 0x0048;
            ram[0xff0f] = (~0b10 & IF);
        } else if ((IF & 0b100) && (IE & 0b100)) {
            PC = 0x0050;
            ram[0xff0f] = (~0b100 & IF);
        } else if ((IF & 0b1000) && (IE & 0b1000)) {
            PC = 0x0058;
            ram[0xff0f] = (~0b1000 & IF);
        } else if ((IF & 0b10000) && (IE & 0b10000)) {
            PC = 0x0060;
            ram[0xff0f] = (~0b10000 & IF);
        } else {
            PICNIC("Received invalid IF interrupt! " << IF);
        }

        system_halt = false; // TODO Reset system halt status
    }
}

void
Gameboy::sendInterrupt(const u8 itype)
{
    ram[0xff0f] = (ram[0xff0f] | itype); // Store to IF register
}

void
Gameboy::execIns()
{
    // fyl hates me for this :<
    #define t(opcode, action, sleep) \
        case (opcode): action; \
        incCycle(sleep); break

    // The Z80 processor allowed for combining
    // registers to increase the capacity of the
    // potential register storage. In this case,
    // pre-calculate the combined-register values
    // so we can use it later on.
    Regpair BC(B, C);
    Regpair DE(D, E);
    Regpair HL(H, L);

    // get the nex parameter in the instruction
    const auto get8 = [this]() -> u8 { return ram[PC++]; };

    // calculate the two byte immediate value
    const auto get16 = [this, get8]() -> u16
    {
        u16 lower = get8();
        u16 upper = get8();
        return upper << 8 | lower;
    };

    // Adds a signed 8 bit parameter to the
    // stack pointer while setting the carry flag
    // and half carry flag. This also resets the
    // zero and subtract flag.
    const auto Addi8toSP = [this, get8]() -> u16
    {
        const u8 ub8 = get8();
        const i8 b8 = toSigned8(ub8);
        const u16 op = SP + b8;
        F.Z = 0; F.N = 0;
        F.CY = (u16)((u8)SP) + ub8 > 0xff;
        F.H = (SP & 0x0f) + (ub8 & 0x0f) >= 0x10;

        return op;
    };

    /* == 8-bit ALU OPERATIONS */
    // Add another byte to register A
    // Flag in this case could potentially be
    // a carry flag
    const auto Add = [&](u8 other, u8 flag = 0)
    {
        F.Z = !(A + other + flag);
        F.N = 0;
        F.H = ((A & 0x0f) + (other & 0x0f) + (flag & 0x0f)) > 0x0f;
        F.CY = ((u16)A + other + flag) > 0xff;
        A += other + flag;
    };

    // Subtract another byte to register A
    // Flag in this case could potentially be
    // a carry flag
    const auto Sub = [&](u8 other, u8 flag = 0)
    {
        F.Z = !(A - (other + flag));
        F.N = 1;
        F.H = ((other & 0x0f) + (flag & 0x0f)) <= (A & 0x0f);
        F.CY = ((u16)other + flag) > A;
        A -= other + flag;
    };

    // Logical AND other byte with A.
    // Set the zero flag accordingly, subtract
    // and carry flag are reset, half carry
    // flag is set to 1.
    const auto And = [&](u8 other)
    {
        A &= other;
        F.Z = !A;
        F.N = 0; F.H = 1; F.CY = 0;
    };

    // Logical OR other byte with A.
    // Set the zero flag accordingly, subtract,
    // half carry flag and carry flag are reset
    const auto Or = [&](u8 other)
    {
        A |= other;
        F.Z = !A;
        F.N = 0; F.H = 0; F.CY = 0;
    };

    // Logical XOR other byte with A.
    // Set the zero flag accordingly, subtract,
    // half carry flag and carry flag are reset
    const auto Xor = [&](u8 other)
    {
        A ^= other;
        F.Z = !A;
        F.N = 0; F.H = 0; F.CY = 0;
    };

    // Compare other byte with A
    // Set the zero flag accordingly set the
    // subtract flag to 1, set the carry flag if
    // the other byte is greater than A, set the
    // half carry flag if the lower nibble is less
    // than or equal to the lower nibble of A.
    const auto CP = [&](u8 other)
    {
        F.Z = A == other;
        F.N = 1;
        F.H = (other & 0x0f) <= (A & 0x0f);
        F.CY = other > A;
    };

    // Increment register.
    // Set the zero flag accordingly, reset the half
    // carry flag, set the half carry flag if we get
    // a carry from the lower nibble of the register
    const auto INC = [&](auto&& reg)
    {
        F.Z = !(reg + 1);
        F.N = 0;
        F.H = (reg & 0x0f) >= 0xf;
    };

    // Decrement register.
    // Set the zero flag accordingly, set the subtract
    // flag to 1, set the half carry flag if we get a borrow
    // from the lower nibble of the register
    const auto DEC = [&](auto&& reg)
    {
        F.Z = !(reg - 1);
        F.N = 1;
        F.H = (reg & 0x0f) >= 1;
    };

    /* 16-bit ALU OPERATIONS */
    // Same as the 8-bit ADD operation, except
    // combining 2 16-bit register pairs, not
    // 2 8-bit registers.
    const auto ADD16 = [&](Regpair& reg1, u16 reg2)
    {
        F.N = 0;
        F.H = ((reg1 & 0xff) + (reg2 & 0xff)) > 0xff;
        F.CY = ((u32)reg1 + reg2) > 0xffff;
        reg1 += reg2;
    };

    /* MISC ALU OPERATIONS */
    // Swap the upper and lower nibble of the register
    // Set the zero flag accordingly, reset all other
    // flags
    const auto SWAP = [&](auto&& reg)
    {
        reg = (reg >> 4 | reg << 4);
        F.Z = !(reg);
        F.N = 0; F.H = 0; F.CY = 0;
    };

    // Rotate val left. Old bit 7 to carry flag. Reset
    // subtract flag and half carry flag, set zero flag to 0
    // if result is 0. Carry flag contains old bit 7 data.
    // If `carry` is specified, then we carry the bit through
    // the carry flag
    const auto RLEFT = [&](auto&& val, bool carry = false)
    {
        bool tmp = (val >> 7);
        F.N = 0; F.H = 0;

        if (carry) {
            val = ((val >> 7) | F.CY);
            F.CY = tmp;
        } else {
            val = ((val >> 7) | (val << 1));
            F.CY = (val & 0b1);
        }

        F.Z = !(val);
    };

    // Rotate val right. Old bit 0 to carry flag. Reset
    // subtract flag and half carry flag, set zero flag to 0
    // if result is 0. Carry flag contains old bit 0 data.
    // If `carry` is specified, then we carry the bit through
    // the carry flag
    const auto RRIGHT = [&](auto&& val, bool carry = false)
    {
        bool tmp = (val & 0b00000001); // Get the 0th bit
        F.N = 0; F.H = 0;

        if (carry) {
            // Shift val over 1 bit, move F.CY over to the 7th bit
            val = ((val >> 1) | (F.CY << 7));
            F.CY = tmp;
        } else {
            F.CY = (val & 0b1);
            val = ((val >> 1) | (val << 7));
        }

        F.Z = !(val);
    };

    // Shift val left into carry.
    // Set LSB (bit 0) to 0
    const auto SLEFT = [&](auto&& val)
    {
        F.N = 0; F.H = 0;

        F.CY = (val >> 7); // set carry flag to old 7th bit
        val = (val << 1); // shift left once

        F.Z = !(val);
    };

    // Shift val right into carry.
    // MSB does not change if msb is true,
    // otherwise msb is set to 0
    const auto SRIGHT = [&](auto&& val, bool msb = true)
    {
        F.N = 0; F.H = 0;

        F.CY = (val & 0b1);
        val = ((val >> 1) | ((msb ? (val & 0b10000000) : 0b0)));

        F.Z = !(val);
    };

    // Test bit in register
    // Set zero flag if bit of register is 0
    // Reset subtract flag, set half carry flag
    const auto BIT = [&](auto&& reg, const u8 bit)
    {
        assert(bit < 8);

        F.N = 0; F.H = 1;
        F.Z = !(reg & (1 << bit));
    };

    // Set bit in register
    const auto SETBIT = [&](auto&& reg, u8 bit, bool reset = false)
    {
        assert(bit < 8);

        if (reset)
            reg = (reg & ~(1 << bit));
        else
            reg = (reg | (1 << bit));
    };

    // CALL two byte immediate value
    // Push address of next instruction on to stack
    // and then jump to address nn
    const auto CALL = [&](bool exec = true)
    {
        const u16 addr = get16();

        if (!exec)
            return;

        ram[--SP] = (u8)(PC >> 8);
        ram[--SP] = (u8)PC;
        PC = addr;
    };

    // RET
    // Pop two bytes from the stack and jump
    // to that address
    const auto RET = [&]
    {
        const u8 lower = ram[SP++];
        const u8 upper = ram[SP++];

        PC = ((upper << 8) | lower);
    };

    // Get the next opcode
    const u8 ins = ram[PC++];
    // cout << "Executing 0x" << std::hex
    //     << (unsigned)ins << "at PC 0x" << std::hex <<
    //     (unsigned)PC << " with next byte 0x" << std::hex <<
    //     (unsigned)ram[PC]     << " 0x" << std::hex <<
    //     (unsigned)ram[PC + 1] << " 0x" << std::hex <<
    //     (unsigned)ram[PC + 2] << endl;
    switch (ins) {
    // LD register, value
    t(0x06, B = get8()   , 8); // store to B
    t(0x0e, C = get8()   , 8); // store to C
    t(0x16, D = get8()   , 8); // store to D
    t(0x1e, E = get8()   , 8); // store to E
    t(0x26, H = get8()   , 8); // store to H
    t(0x2e, L = get8()   , 8); // store to L
    // LD register, register
    t(0x7f, /*A = A*/    , 4); // shut up compiler
    t(0x78, A = B        , 4); // store B to A
    t(0x79, A = C        , 4); // store C to A
    t(0x7a, A = D        , 4); // store D to A
    t(0x7b, A = E        , 4); // store E to A
    t(0x7c, A = H        , 4); // store H to A
    t(0x7d, A = L        , 4); // store L to A
    t(0x7e, A = ram[HL]  , 8); // store the value at HL to A
    t(0x40, /*B = B*/    , 4); // shut up compiler
    t(0x41, B = C        , 4); // store C to B
    t(0x42, B = D        , 4); // store D to B
    t(0x43, B = E        , 4); // store E to B
    t(0x44, B = H        , 4); // store H to B
    t(0x45, B = L        , 4); // store L to B
    t(0x46, B = ram[HL]  , 8); // store the value at HL to B
    t(0x48, C = B        , 4); // store B to C
    t(0x49, /*C = C*/    , 4); // shut up compiler
    t(0x4a, C = D        , 4); // store D to C
    t(0x4b, C = E        , 4); // store E to C
    t(0x4c, C = H        , 4); // store H to C
    t(0x4d, C = L        , 4); // store L to C
    t(0x4e, C = ram[HL]  , 8); // store the value at HL to C
    t(0x50, D = B        , 4); // store B to D
    t(0x51, D = C        , 4); // store C to D
    t(0x52, /*D = D*/    , 4); // shut up compiler
    t(0x53, D = E        , 4); // store E to D
    t(0x54, D = H        , 4); // store H to D
    t(0x55, D = L        , 4); // store L to D
    t(0x56, D = ram[HL]  , 8); // store the value at HL to D
    t(0x58, E = B        , 4); // store B to E
    t(0x59, E = C        , 4); // store C to E
    t(0x5a, E = D        , 4); // store D to E
    t(0x5b, /*E = E*/    , 4); // shut up compiler
    t(0x5c, E = H        , 4); // store H to E
    t(0x5d, E = L        , 4); // store L to E
    t(0x5e, E = ram[HL]  , 8); // store the value at HL to E
    t(0x60, H = B        , 4); // store B to H
    t(0x61, H = C        , 4); // store C to H
    t(0x62, H = D        , 4); // store D to H
    t(0x63, H = E        , 4); // store E to H
    t(0x64, /*H = H*/    , 4); // shut up compiler
    t(0x65, H = L        , 4); // store L to H
    t(0x66, H = ram[HL]  , 8); // store the value at HL to H
    t(0x68, L = B        , 4); // store B to L
    t(0x69, L = C        , 4); // store C to L
    t(0x6a, L = D        , 4); // store D to L
    t(0x6b, L = E        , 4); // store E to L
    t(0x6c, L = H        , 4); // store H to L
    t(0x6d, /*L = L*/    , 4); // shut up compiler
    t(0x6e, L = ram[HL]  , 8); // store the value at HL to L
    t(0x70, ram[HL] = B  , 8); // store B at the value of HL
    t(0x71, ram[HL] = C  , 8); // store C at the value of HL
    t(0x72, ram[HL] = D  , 8); // store D at the value of HL
    t(0x73, ram[HL] = E  , 8); // store E at the value of HL
    t(0x74, ram[HL] = H  , 8); // store H at the value of HL
    t(0x75, ram[HL] = L  , 8); // store L at the value of HL
    t(0x36, ram[HL] = get8(), 12); // store to the value of HL
    // LD A, register|#
    t(0x0a, A = ram[BC]  , 8); // store the value at BC to A
    t(0x1a, A = ram[DE]  , 8); // store the value at DE to A
    t(0xfa, A = ram[get16()], 16); // store the value at the
                                   // parameter to A
    t(0x3e, A = get8()   , 8); // store to A
    // LD register|#, A
    t(0x47, B = A        , 4); // store A to B
    t(0x4f, C = A        , 4); // store A to C
    t(0x57, D = A        , 4); // store A to D
    t(0x5f, E = A        , 4); // store A to E
    t(0x67, H = A        , 4); // store A to H
    t(0x6f, L = A        , 4); // store A to L
    t(0x02, ram[BC] = A  , 8); // store A at the value of BC
    t(0x12, ram[DE] = A  , 8); // store A at the value of DE
    t(0x77, ram[HL] = A  , 8); // store A at the value of HL
    t(0xea, ram[get16()] = A, 16); // store A to the value at the parameter
    // LD A, ($FF00 + C) ; I/O
    t(0xf2, A = ram[C+0xff00], 8); // store the value at address 0xff00 + register C into A
    // LD ($FF00 + C), A ; I/O
    t(0xe2, ram[C+0xff00] = A, 8); // store A into the address at 0xff00 + register C
    // LD  A, (HLD): LD  A, (HL-): LDD A, (HL)
    // LD  A, (HL) : DEC HL ; pseudo reference
    t(0x3a, A = ram[HL--], 8); // store decremented address HL into A
    // LD (HLD), A: LD (HL-), A: LDD (HL), A
    // LD (HL) , A: DEC HL ; pseudo reference
    t(0x32, ram[HL--] = A, 8); // store A into decremented address HL
    // LD  A, (HLI): LD A, (HL+): LDI A, (HL)
    // LD  A, (HL) : INC HL ; pseudo reference
    t(0x2a, A = ram[HL++], 8); // store incremented address HL into A
    // LD (HLI), A: LD (HL+), A: LDI (HL), A
    // LD (HL) , A: INC HL ; pseudo reference
    t(0x22, ram[HL++] = A, 8); // store A into incremented address HL
    // LDH (#), A ; I/O
    t(0xe0, ram[0xff00+get8()] = A, 12); // store A at address 0xff00 + one byte
                                         // immediate value
    // LDH A, (#) ; I/O
    t(0xf0, A = ram[0xff00+get8()], 12); // store address of 0xff00 + one byte
                                         // immediate value into A
    // LD register, ##
    t(0x01, BC = get16(), 12); // store 2 byte immediate value to BC
    t(0x11, DE = get16(), 12); // store 2 byte immediate value to DE
    t(0x21, HL = get16(), 12); // store 2 byte immediate value to HL
    t(0x31, SP = get16(), 12); // store 2 byte immediate value to SP
    // LD SP, HL
    t(0xf9, SP = HL     ,  8); // store HL into the stack pointer
    // LD HL, SP + #: LDHL SP, #
    // store stack pointer + one byte signed
    // immediate parameter into register HL, reset the zero flag and subtract
    // flag, update half carry flag and carry flag according to the addition.
    // TODO: Verify this works
    t(0xf8, {const u16 op = Addi8toSP(); L = ram[op]; H = ram[op + 0x01];}, 12);
    // LD (##), SP
    t(0x08, ram[get16()] = SP, 20); // Store stack pointer at address of ##
    // PUSH registers
    t(0xf5, ram[--SP] = A; ram[--SP] = F, 16); // store A and F on the stack, decrementing the stack pointer
    t(0xc5, ram[--SP] = B; ram[--SP] = C, 16); // store B and C on the stack, decrementing the stack pointer
    t(0xd5, ram[--SP] = D; ram[--SP] = E, 16); // store D and E on the stack, decrementing the stack pointer
    t(0xe5, ram[--SP] = H; ram[--SP] = L, 16); // store H and L on the stack, decrementing the stack pointer
    // POP registers
    t(0xf1, F = ram[SP++]; A = ram[SP++], 12); // store two bytes off the stack in to register pair AF
    t(0xc1, C = ram[SP++]; B = ram[SP++], 12); // store two bytes off the stack in to register pair BC
    t(0xd1, E = ram[SP++]; D = ram[SP++], 12); // store two bytes off the stack in to register pair DE
    t(0xe1, L = ram[SP++]; H = ram[SP++], 12); // store two bytes off the stack in to register pair HL
    /* == BEGIN 8-bit ALU OPERATIONS == */
    // ADD A, #
    // for these next instructions we set the zero flag if the result is zero,
    // reset the subtract flag to 0, update the half carry flag if there is a carry
    // from bit 3, and update the carry flag if there is a carry from bit 7.
    t(0x87, Add(A), 4); // Add A to A
    t(0x80, Add(B), 4); // Add B to A
    t(0x81, Add(C), 4); // Add C to A
    t(0x82, Add(D), 4); // Add D to A
    t(0x83, Add(E), 4); // Add E to A
    t(0x84, Add(H), 4); // Add H to A
    t(0x85, Add(L), 4); // Add L to A
    t(0x86, Add(ram[HL]), 8); // Add *HL to A
    t(0xc6, Add(get8()), 4); // Add the one byte immediate parameter to A
    // ADC A, #
    // now we add n + the carry flag to A. Same as above.
    t(0x8f, Add(A, F.CY), 4); // Add A + carry flag to A
    t(0x88, Add(B, F.CY), 4); // Add B + carry flag to A
    t(0x89, Add(C, F.CY), 4); // Add C + carry flag to A
    t(0x8a, Add(D, F.CY), 4); // Add D + carry flag to A
    t(0x8b, Add(E, F.CY), 4); // Add E + carry flag to A
    t(0x8c, Add(H, F.CY), 4); // Add H + carry flag to A
    t(0x8d, Add(L, F.CY), 4); // Add L + carry flag to A
    t(0x8e, Add(ram[HL], F.CY), 8); // Add *HL + carry flag to A
    t(0xce, Add(get8(), F.CY), 8); // Add one byte immediate value + carry flag to A
    // SUB n
    // Subtract n from A, update the zero flag if the result is zero, update the subtract flag on 1,
    // update the half carry flag if there is no borrow from bit 4, update the borrow flag if there
    // is no borrow.
    t(0x97, Sub(A), 4); // Subtract A from A
    t(0x90, Sub(B), 4); // Subtract B from A
    t(0x91, Sub(C), 4); // Subtract C from A
    t(0x92, Sub(D), 4); // Subtract D from A
    t(0x93, Sub(E), 4); // Subtract E from A
    t(0x94, Sub(H), 4); // Subtract H from A
    t(0x95, Sub(L), 4); // Subtract L from A
    t(0x96, Sub(ram[HL]), 8); // Subtract *HL from A
    t(0xd6, Sub(get8()), 8); // Subtract one byte immediate value from A
    // SUB n + #
    // now we subtract n + the carry flag from A. Same as above
    t(0x9f, Sub(A, F.CY), 4); // Subtract A + carry flag from A
    t(0x98, Sub(B, F.CY), 4); // Subtract B + carry flag from A
    t(0x99, Sub(C, F.CY), 4); // Subtract C + carry flag from A
    t(0x9a, Sub(D, F.CY), 4); // Subtract D + carry flag from A
    t(0x9b, Sub(E, F.CY), 4); // Subtract E + carry flag from A
    t(0x9c, Sub(H, F.CY), 4); // Subtract H + carry flag from A
    t(0x9d, Sub(L, F.CY), 4); // Subtract L + carry flag from A
    t(0x9e, Sub(ram[HL], F.CY), 8); // Subtract *HL + carry flagHL from A
    t(0xde, Sub(get8(), F.CY), 8); // Subtract one byte immediate value + carry flag from A
    // AND n
    // Update zero flag to 1 if zero, update subtract flag to 0, update half carry flag to 1,
    // update half carry flag to 0
    t(0xa7, And(A), 4); // Logical AND A with A
    t(0xa0, And(B), 4); // Logical AND A with B
    t(0xa1, And(C), 4); // Logical AND A with C
    t(0xa2, And(D), 4); // Logical AND A with D
    t(0xa3, And(E), 4); // Logical AND A with E
    t(0xa4, And(H), 4); // Logical AND A with H
    t(0xa5, And(L), 4); // Logical AND A with L
    t(0xa6, And(ram[HL]), 8); // Logical AND A with *HL
    t(0xe6, And(get8()), 8); // Logical AND A with one byte immediate value
    // OR n
    // Update zero flag to 1 if result is 0, reset all other flags
    t(0xb7, Or(A), 4); // Logical OR A with A
    t(0xb0, Or(B), 4); // Logical OR A with B
    t(0xb1, Or(C), 4); // Logical OR A with C
    t(0xb2, Or(D), 4); // Logical OR A with D
    t(0xb3, Or(E), 4); // Logical OR A with E
    t(0xb4, Or(H), 4); // Logical OR A with H
    t(0xb5, Or(L), 4); // Logical OR A with L
    t(0xb6, Or(ram[HL]), 8); // Logical OR A with *HL
    t(0xf6, Or(get8()), 8); // Logical OR A with one byte immediate value
    // XOR n
    // Update zero flag to 1 if result is 0, reset all other flags
    t(0xaf, Xor(A), 4); // Logical XOR A with A
    t(0xa8, Xor(B), 4); // Logical XOR A with B
    t(0xa9, Xor(C), 4); // Logical XOR A with C
    t(0xaa, Xor(D), 4); // Logical XOR A with D
    t(0xab, Xor(E), 4); // Logical XOR A with E
    t(0xac, Xor(H), 4); // Logical XOR A with H
    t(0xad, Xor(L), 4); // Logical XOR A with L
    t(0xae, Xor(ram[HL]), 8); // Logical XOR A with *HL
    t(0xee, Xor(get8()), 8); // Logical XOR A with one byte immediate value
    // CP n
    // Set zero flag to 1 if A = n, update subtract flag to 1, update half carry
    // flag to 1 if no borrow from bit 4, update carry flag to 1 if n <= A
    t(0xbf, CP(A), 4); // Compare A to A
    t(0xb8, CP(B), 4); // Compare A to B
    t(0xb9, CP(C), 4); // Compare A to C
    t(0xba, CP(D), 4); // Compare A to D
    t(0xbb, CP(E), 4); // Compare A to E
    t(0xbc, CP(H), 4); // Compare A to H
    t(0xbd, CP(L), 4); // Compare A to L
    t(0xbe, CP(ram[HL]), 8); // Compare A to *HL
    t(0xfe, CP(get8()), 8); // Compare A to one immediate byte value
    // INC n
    // Set zero flag if result is zero, reset the subtract flag, update the
    // half carry flag if carry from bit 3, don't touch the carry flag.
    t(0x3c, INC(A), 4); // Increment register A
    t(0x04, INC(B), 4); // Increment register B
    t(0x0c, INC(C), 4); // Increment register C
    t(0x14, INC(D), 4); // Increment register D
    t(0x1c, INC(E), 4); // Increment register E
    t(0x24, INC(H), 4); // Increment register H
    t(0x2c, INC(L), 4); // Increment register L
    t(0x34, INC(ram[HL]), 12); // Increment *HL
    // DEC n
    // Set zero flag if result is zero, set the subtract flag to 1, set the half
    // carry flag if borrow from bit 4, don't touch the carry flag.
    t(0x3d, DEC(A), 4); // Decrement register A
    t(0x05, DEC(B), 4); // Decrement register B
    t(0x0d, DEC(C), 4); // Decrement register C
    t(0x15, DEC(D), 4); // Decrement register D
    t(0x1d, DEC(E), 4); // Decrement register E
    t(0x25, DEC(H), 4); // Decrement register H
    t(0x2d, DEC(L), 4); // Decrement register L
    t(0x35, DEC(ram[HL]), 12); // Decrement *HL
    /* == BEGIN 16-bit ALU OPERATIONS == */
    // ADD HL, n
    t(0x09, ADD16(HL, BC), 8); // Add BC to HL
    t(0x19, ADD16(HL, DE), 8); // Add DE to HL
    t(0x29, ADD16(HL, HL), 8); // Add HL to HL
    t(0x39, ADD16(HL, SP), 8); // Add SP to HL
    // ADD SP, #
    t(0xe8, SP = Addi8toSP(), 16); // Add immediate signed byte to the stack pointer
    // INC regpair
    t(0x03, BC++, 8); // Increment regpair BC
    t(0x13, DE++, 8); // Increment regpair DE
    t(0x23, HL++, 8); // Increment regpair HL
    t(0x33, SP++, 8); // Increment stack pointer
    // DEC regpair
    t(0x0b, BC--, 8); // Decrement regpair BC
    t(0x1b, DE--, 8); // Decrement regpair DE
    t(0x2b, HL--, 8); // Decrement regpair HL
    t(0x3b, SP--, 8); // Decrement stack pointer
    // DAA : TODO :
    //
    // CPL
    // Flips all bits in register A. Set the subtract and
    // half carry flags.
    t(0x2f, A = ~A; F.N = 1; F.H = 1, 4);
    // CCF
    // Complement carry flag
    t(0x3f, F.CY = !F.CY, 4); // Invert the carry flag.
    // SCF
    // Set the carry flag. Reset the subtract and half carry
    // flags
    t(0x37, F.N = 0; F.H = 0; F.CY = 1, 4);
    // NOP
    // No operation
    t(0x00, /* contemplate life here */ , 4);
    // HALT
    // Power down the CPU until an interrupt occurs
    t(0x76, system_halt = true, 4);
    // STOP
    // Halt CPU & CLD display until button is pressed
    // Skip over the next opcode
    t(0x10, PC++, 4);
    // DI
    // Disable interrups but not immediately. Interrups are
    // disabled after instruction after DI is executed.
    t(0xf3, IME = false, 4);
    // EI
    // Enable interrups but not immediatly. Interrupts are
    // enabled after instruction after EI is executed.
    t(0xfb, IME = true, 4);
    // RLCA
    t(0x07, RLEFT(A), 4); // Rotate register A left
    // RLA
    t(0x17, RLEFT(A, true), 4); // Rotate register A left through carry flag.
    // RRCA
    // Rotate A right, old bit 0 to carry flag. Reset
    // subtract and half carry flag, set zero flag to 1
    // if result is 0, carry flag contains old bit 0 data.
    t(0x0f, RRIGHT(A), 4); // Rotate register A right
    // RRA
    t(0x1f, RRIGHT(A, true), 4); // Rotate A right through carry flag, same as above
    // JP nn
    // Jump to address nn
    // Use nn as two byte immediate value, LS byte first
    t(0xc3, PC = get16(), 12); // Jump to address nn
    // JP condition, address
    // Jump to the address (2 byte immediate value) if
    // the condition is met. Note: we must consume the
    // argument reguardless. We do it this way so we
    // don't need to write curely braces.
    t(0xc2, !(F.Z) ? PC = get16() : get16(), 12); // Jump if zero flag is reset
    t(0xca,  (F.Z) ? PC = get16() : get16(), 12); // Jump if zero flag is set
    t(0xd2, !(F.CY)? PC = get16() : get16(), 12); // Jump if carry flag is reset
    t(0xda,  (F.CY)? PC = get16() : get16(), 12); // Jump if carry flag is set
    // JP *HL
    // Jump to the address contained in HL
    t(0xe9, PC = ram[HL], 4); // Jump to *HL
    // JR n
    // Add one byte signed immediate value to
    // our current address and jump to it
    t(0x18, PC += toSigned8(get8()), 8);
    // JR condition, one byte signed immediate value
    // If the condition is true, then add one byte
    // signed immediate value to the current address
    // and jump to it
    t(0x20, !(F.Z) ? PC += toSigned8(get8()) : get8(), 8); // Jump if zero flag is reset
    t(0x28,  (F.Z) ? PC += toSigned8(get8()) : get8(), 8); // Jump if zero flag is set
    t(0x30, !(F.CY)? PC += toSigned8(get8()) : get8(), 8); // Jump if carry flag is reset
    t(0x38,  (F.CY)? PC += toSigned8(get8()) : get8(), 8); // Jump if carry flag is set
    // CALL two byte immediate value
    t(0xcd, CALL(), 12);
    // CALL condition, two byte immediate value
    // Only CALL if the condition is met
    t(0xc4, CALL(!(F.Z) ), 12); // Call if zero flag is reset
    t(0xcc, CALL( (F.Z) ), 12); // Call if zero flag is set
    t(0xd4, CALL(!(F.CY)), 12); // Call if carry flag is reset
    t(0xdc, CALL( (F.CY)), 12); // Call if carry flag is set
    // RST n
    // Push present address onto stack.
    // Jump to address specified
    t(0xc7, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x00, 32);
    t(0xcf, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x08, 32);
    t(0xd7, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x10, 32);
    t(0xdf, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x18, 32);
    t(0xe7, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x20, 32);
    t(0xef, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x28, 32);
    t(0xf7, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x30, 32);
    t(0xff, ram[--SP] = (u8)(PC >> 8); ram[--SP] = (u8)PC; PC = 0x38, 32);
    // RET
    // Pop two bytes from stack and jump to that address
    t(0xc9, RET(), 8);
    // RET cc
    // RET if the condition is true
    t(0xc0, if (!(F.Z)) RET(), 8); // Return if zero flag is reset
    t(0xc8, if ( (F.Z)) RET(), 8); // Return if zero flag is set
    t(0xd0, if (!(F.CY))RET(), 8); // Return if carry flag is reset
    t(0xd8, if ( (F.CY))RET(), 8); // Return if carry flag is set
    // RETI
    // Pop two bytes from stack and jump to that address
    // then enable interrupts
    t(0xd9, RET(); /* enable interrupt */, 8);
    }

    // Now parse the CB table, the Gameboy CPU has
    // two separate tables for instructions.
    if (ins == 0xcb) {
        const u8 ins2 = ram[PC++];

        // RES 0-7, register
        // Reset bit in register
        // Set the second parameter of SETBIT to true to
        // reset the bit
        if ((ins2 >> 6) == 0b10) {
            getReg((ins2 & 0x7), [&](auto&& reg){SETBIT(reg, ((ins2 >> 3) & 0x7), 1);});
            return;
        }

        // SET 0-7, register
        // Set bit in register
        if ((ins2 >> 6) == 0b11) {
            getReg((ins2 & 0x7), [&](auto&& reg){SETBIT(reg, ((ins2 >> 3) & 0x7));});
            return;
        }

        // BIT 0-7, register
        // Test bit in register
        if ((ins2 >> 6) == 0b01) {
            getReg((ins2 & 0x7), [&](auto&& reg){BIT(reg, ((ins2 >> 3) & 0x7));});
            return;
        }

        switch (ins2) {
        // SWAP n
        // sets the zero flag to 1 if the result is 0, reset all other flags
        t(0x37, SWAP(A), 8); // Swap nibbles of A
        t(0x30, SWAP(B), 8); // Swap nibbles of B
        t(0x31, SWAP(C), 8); // Swap nibbles of C
        t(0x32, SWAP(D), 8); // Swap nibbles of D
        t(0x33, SWAP(E), 8); // Swap nibbles of E
        t(0x34, SWAP(H), 8); // Swap nibbles of H
        t(0x35, SWAP(L), 8); // Swap nibbles of L
        t(0x36, SWAP(ram[HL]), 16); // Swap nibbles of *HL
        // RLC n
        // Rotate n left
        t(0x07, RLEFT(A), 8); // Rotate register A left
        t(0x00, RLEFT(B), 8); // Rotate register B left
        t(0x01, RLEFT(C), 8); // Rotate register C left
        t(0x02, RLEFT(D), 8); // Rotate register D left
        t(0x03, RLEFT(E), 8); // Rotate register E left
        t(0x04, RLEFT(H), 8); // Rotate register H left
        t(0x05, RLEFT(L), 8); // Rotate register L left
        t(0x06, RLEFT(ram[HL]), 16); // Rotate *HL left
        // RL n
        // Rotate n left through carry flag.
        t(0x17, RLEFT(A, true), 8); // Rotate register A left through carry flag
        t(0x10, RLEFT(B, true), 8); // Rotate register B left through carry flag
        t(0x11, RLEFT(C, true), 8); // Rotate register C left through carry flag
        t(0x12, RLEFT(D, true), 8); // Rotate register D left through carry flag
        t(0x13, RLEFT(E, true), 8); // Rotate register E left through carry flag
        t(0x14, RLEFT(H, true), 8); // Rotate register H left through carry flag
        t(0x15, RLEFT(L, true), 8); // Rotate register L left through carry flag
        t(0x16, RLEFT(ram[HL], true), 16); // Rotate *HL left through carry flag
        // RRC n
        // Rotate n right
        t(0x0f, RRIGHT(A), 8); // Rotate register A right
        t(0x08, RRIGHT(B), 8); // Rotate register B right
        t(0x09, RRIGHT(C), 8); // Rotate register C right
        t(0x0a, RRIGHT(D), 8); // Rotate register D right
        t(0x0b, RRIGHT(E), 8); // Rotate register E right
        t(0x0c, RRIGHT(H), 8); // Rotate register H right
        t(0x0d, RRIGHT(L), 8); // Rotate register L right
        t(0x0e, RRIGHT(ram[HL]), 16); // Rotate *HL right
        // RR n
        // Rotate n right through carry flag
        t(0x1f, RRIGHT(A, true), 8); // Rotate register A right through carry flag
        t(0x18, RRIGHT(B, true), 8); // Rotate register B right through carry flag
        t(0x19, RRIGHT(C, true), 8); // Rotate register C right through carry flag
        t(0x1a, RRIGHT(D, true), 8); // Rotate register D right through carry flag
        t(0x1b, RRIGHT(E, true), 8); // Rotate register E right through carry flag
        t(0x1c, RRIGHT(H, true), 8); // Rotate register H right through carry flag
        t(0x1d, RRIGHT(L, true), 8); // Rotate register L right through carry flag
        t(0x1e, RRIGHT(ram[HL], true), 16); // Rotate *HL right through carry flag
        // SLA n
        // Shift n left into carry, set bit 0 to 0
        t(0x27, SLEFT(A), 8); // Shift register A left
        t(0x20, SLEFT(B), 8); // Shift register B left
        t(0x21, SLEFT(C), 8); // Shift register C left
        t(0x22, SLEFT(D), 8); // Shift register D left
        t(0x23, SLEFT(E), 8); // Shift register E left
        t(0x24, SLEFT(H), 8); // Shift register H left
        t(0x25, SLEFT(L), 8); // Shift register L left
        t(0x26, SLEFT(ram[HL]), 16); // Shift *HL left
        // SRA n
        // Shift n right into carry. MSB doesn't change
        t(0x2f, SRIGHT(A), 8); // Shift register A right
        t(0x28, SRIGHT(B), 8); // Shift register B right
        t(0x29, SRIGHT(C), 8); // Shift register C right
        t(0x2a, SRIGHT(D), 8); // Shift register D right
        t(0x2b, SRIGHT(E), 8); // Shift register E right
        t(0x2c, SRIGHT(H), 8); // Shift register H right
        t(0x2d, SRIGHT(L), 8); // Shift register L right
        t(0x2e, SRIGHT(ram[HL]), 16); // Shift *HL right
        // SRL n
        // Shift n right into carry. MSB set to 0
        // Set the second parameter of SRIGHT to false to
        // set the most significant bit (MSB) to 0
        t(0x3f, SRIGHT(A, false), 8); // Shift register A right, set MSB to 0
        t(0x38, SRIGHT(B, false), 8); // Shift register B right, set MSB to 0
        t(0x39, SRIGHT(C, false), 8); // Shift register C right, set MSB to 0
        t(0x3a, SRIGHT(D, false), 8); // Shift register D right, set MSB to 0
        t(0x3b, SRIGHT(E, false), 8); // Shift register E right, set MSB to 0
        t(0x3c, SRIGHT(H, false), 8); // Shift register H right, set MSB to 0
        t(0x3d, SRIGHT(L, false), 8); // Shift register L right, set MSB to 0
        t(0x3e, SRIGHT(ram[HL], false), 16); // Shift *HL right, set MSB to 0
        default:
            cerr << "WARNING: Unknown opcode in CB table: 0x"
                << std::hex << (unsigned)ins2 << endl;
        }
    }

    #undef t
}

void
Gameboy::runGame()
{
    for(;;) {
        this->handleInterrupt();
        if (!system_halt) this->execIns();
    }
}

int
main(int argc, char ** argv)
{
    if (argc < 2) {
        cout << "USAGE: " << argv[0] << " game.gbc\n";
        return EXIT_FAILURE;
    }

    Gameboy gameboy;
    gameboy.loadROM(argv[1]);
    gameboy.runGame();

    return EXIT_SUCCESS;
}
