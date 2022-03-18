#include "arch/riscv/insts/macroop.hh"

#include <sstream>
#include <string>

#include "arch/generic/memhelpers.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/utility.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

inline int
elem_mask(const uint8_t* v0, const int index)
{
    int idx = index / 8;
    int pos = index % 8;
    return (v0[idx] >> pos) & 1;
}

std::string VectorArithMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    return ss.str();
}

std::string VectorArithMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    if (vm == 0) {
        ss << ", v0.t";
    }
    return ss.str();
}

std::string VleMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        offset << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        offset << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VleMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

Fault
VldMvMicroInst::execute(ExecContext *xc,
        Trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    RiscvISA::vreg_t tmp_d0 = xc->getWritableVecRegOperand(this, 0);
    auto Vd = tmp_d0.as<uint8_t>();

    constexpr auto offset = cache_line_size / 8;
    for (int i = 0; i < NumVecMemInternalRegs; i++) {
        RiscvISA::vreg_t tmp_s = xc->readVecRegOperand(this, i);
        auto s = tmp_s.as<uint8_t>();
        memcpy(Vd + i * offset, s, offset);
    }
    xc->setVecRegOperand(this, 0, tmp_d0);
    return fault;
}

std::string VldMvMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    for (int i = 0; i < NumVecMemInternalRegs; i++) {
        ss << ", " << registerName(srcRegIdx(i));
    }
    return ss.str();
}

Fault
VstMvMicroInst::execute(ExecContext *xc,
        Trace::InstRecord *traceData) const
{
    Fault fault = NoFault;
    RiscvISA::vreg_t tmp_s0 = xc->readVecRegOperand(this, 0);
    auto Vs = tmp_s0.as<uint8_t>();

    constexpr auto offset = cache_line_size / 8;
    for (int i = 0; i < NumVecMemInternalRegs; i++) {
        RiscvISA::vreg_t tmp_d = xc->getWritableVecRegOperand(this, i);
        auto d = tmp_d.as<uint8_t>();
        memcpy(d, Vs + i * offset, offset);
        xc->setVecRegOperand(this, i, tmp_d);
    }
    return fault;
}

std::string VstMvMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ';
    for (int i = 0; i < NumVecMemInternalRegs; i++) {
        ss << registerName(destRegIdx(i)) << ", ";
    }
    ss << registerName(srcRegIdx(0));
    return ss.str();
}

} // namespace RiscvISA
} // namespace gem5