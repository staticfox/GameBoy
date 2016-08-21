#include <iostream>
#include <vector>

typedef uint8_t u8;
typedef uint16_t u16;

struct Gameboy {
    Gameboy()
        : A(),  F(), B(), C(),
          D(),  E(), H(), L(),
          SP(), PC(),
          ZF(), NF(), HF(), CF(),
          ram(0x10000) {}

    // 8 General purpose CPU
    // registers
    u8  A, F;
    u8  B, C;
    u8  D, E;
    u8  H, L;

    // Stack pointer and program
    // counter
    u16 SP, PC;

    // CPU Flags
    bool ZF, NF, HF, CF;

    // Ram persistence
    std::vector<u8> ram;

    void execIns();
    void incCycle(unsigned);
};

Gameboy gameboy;

void
Gameboy::incCycle(unsigned cycles)
{
}

void
Gameboy::execIns()
{
    const u8 ins = ram[PC++];

    #define t(opcode, action, sleep) \
        case (opcode): action; \
        incCycle(sleep); break

    // The Z80 processor allowed for combining
    // registers to increase the capacity of the
    // potential register storage. In this case,
    // pre-calculate the combined-register values
    // so we can use it later on.
    const u16 BC = ((u16)B << 8) | C;
    const u16 DE = ((u16)D << 8) | E;
    const u16 HL = ((u16)H << 8) | L;

    // calculate the two byte intermediate value.
    const auto get16 = [this]{
        u16 lower = ram[PC++];
        u16 upper = ram[PC++];
        return upper << 8 | lower;
    };

    // get the nex parameter in the instruction
    const auto get8 = [this]{ return ram[PC++]; };

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
    }

    #undef t
}

int
main(int argc, char **argv)
{
    return 0;
}
