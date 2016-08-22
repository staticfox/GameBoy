#include <stdint.h>
#include <vector>

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

using i8  = int8_t;
using i16 = int16_t;

// special struct in order to perform
// instructions easily on register pairs
struct Regpair
{
    // Emulate the regpair by shifting reg1
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
    Gameboy()
        : A(), B(), C(), D(), E(), H(), L(),
          SP(), PC(), F(), ram(0x10000) {}

    u8 A, B, C, D, E, H, L; // General purpose CPU registers
    u16 SP, PC; // Stack pointer and program counter

    // F register
    struct Freg
    {
        Freg() : Z(), N(), H(), CY(), LF() {}

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

        bool Z, N, H, CY; // CPU Flags
        u8 LF; // Emulating the lower 4 bits of the F register
    } F;

    std::vector<u8> ram; // Ram persistence

    void execIns(); // Execute a single instruction
    void incCycle(unsigned); // Add to the clock ticks
};

Gameboy gameboy;

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
    (void) cycles;
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
    const auto Add = [&](u8 other, u8 flag = 0) -> void
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
    const auto Sub = [&](u8 other, u8 flag = 0) -> void
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
    const auto And = [&](u8 other) -> void
    {
        A &= other;
        F.Z = !A;
        F.N = 0; F.H = 1; F.CY = 0;
    };

    // Logical OR other byte with A.
    // Set the zero flag accordingly, subtract,
    // half carry flag and carry flag are reset
    const auto Or = [&](u8 other) -> void
    {
        A |= other;
        F.Z = !A;
        F.N = 0; F.H = 0; F.CY = 0;
    };

    // Logical XOR other byte with A.
    // Set the zero flag accordingly, subtract,
    // half carry flag and carry flag are reset
    const auto Xor = [&](u8 other) -> void
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
    const auto CP = [&](u8 other) -> void
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
    const auto INC = [&](u8& reg) -> void
    {
        F.Z = !(reg + 1);
        F.N = 0;
        F.H = (reg & 0x0f) >= 0xf;
    };

    // Decrement register.
    // Set the zero flag accordingly, set the subtract
    // flag to 1, set the half carry flag if we get a borrow
    // from the lower nibble of the register
    const auto DEC = [&](u8& reg) -> void
    {
        F.Z = !(reg - 1);
        F.N = 1;
        F.H = (reg & 0x0f) >= 1;
    };

    /* 16-bit ALU OPERATIONS */
    // Same as the 8-bit ADD operation, except
    // combining 2 16-bit register pairs, not
    // 2 8-bit registers.
    const auto ADD16 = [&](Regpair& reg1, u16 reg2) -> void
    {
        F.N = 0;
        F.H = ((reg1 & 0xff) + (reg2 & 0xff)) > 0xff;
        F.CY = ((u32)reg1 + reg2) > 0xffff;
        reg1 += reg2;
    };

    // Swap the upper and lower nibble of the register
    // Set the zero flag accordingly, reset all other
    // flags
    const auto SWAP = [&](u8& reg) -> void
    {
        reg = (reg >> 4 | reg << 4);
        F.Z = !(reg);
        F.N = 0; F.H = 0; F.CY = 0;
    };

    // Get the next opcode
    const u8 ins = ram[PC++];
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
    t(0x76, /* contemplate life here */, 4);
    // STOP
    // Halt CPU & CLD display until button is pressed
    // Skip over the next opcode
    t(0x10, PC++, 4);
    // DI
    // Disable interrups but not immediately. Interrups are
    // disabled after instruction after DI is executed.
    t(0xf3, , 4);
    // EI
    // Enable interrups but not immediatly. Interrupts are
    // enabled after instruction after EI is executed.
    t(0xfb, , 4);
    // RLCA
    // Rotate register A left. Old bit 7 to carry flag. Reset
    // subtract flag and half carry flag, set zero flag to 0
    // if result is 0. Carry flag contains old bit 7 data.
    t(0x07, F.N = 0; F.H = 0; A = ((A >> 7) | (A << 1)); F.Z = !(A); F.CY = (A & 0b1), 4);
    // RLA
    // Rotate A left through carry flag.
    t(0x17, F.N = 0; F.H = 0; {bool tmp = (A >> 7);
        A = ((A >> 7) | F.CY); F.Z = !(A); F.CY = tmp; }, 4);
    }

    // Now parse the CB table, the Gameboy CPU has
    // two separate tables for instructions.
    if (ins == 0xcb) {
        switch (ram[PC++]) {
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
        }
    }

    #undef t
}

int
main(int /*argc*/, char ** /*argv*/)
{
    return 0;
}
