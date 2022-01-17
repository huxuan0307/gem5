#include "arch/riscv/insts/vector.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

std::string
VIntOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(0)) << ", " <<
        registerName(srcRegIdx(1));
    if (vm == 0) {
        ss << ", v0.t";
    }
    return ss.str();
}

std::string
VConfOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (bit31 && bit30 == 0) {
        ss << registerName(srcRegIdx(0)) << ", " << registerName(srcRegIdx(1));
    } else if (bit31 && bit30) {
        ss << uimm << ", " << zimm;
    } else {
        ss << registerName(srcRegIdx(0)) << ", " << zimm;
    }
    return ss.str();
}

std::string
VMemLoadOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", (" <<
        registerName(srcRegIdx(0)) << ")";
    return ss.str();
}

std::string
VMemStoreOp::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", (" <<
        registerName(srcRegIdx(0)) << ")";
    return ss.str();
}

} // namespace RiscvISA
} // namespace gem5
