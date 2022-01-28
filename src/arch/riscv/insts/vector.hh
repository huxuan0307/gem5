#ifndef __ARCH_RISCV_INSTS_VECTOR_HH__
#define __ARCH_RISCV_INSTS_VECTOR_HH__

#include <string>

#include "arch/riscv/insts/bitfields.hh"
#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/regs/misc.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * Base class for arith operations.
 */
class VArithOp : public RiscvStaticInst
{
  protected:
    uint64_t vm;

    VArithOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass),
          vm(VM)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

/**
 * Base class for Vector Config operations
 */
class VConfOp : public RiscvStaticInst
{
  protected:
    uint64_t bit30;
    uint64_t bit31;
    uint64_t zimm;
    uint64_t uimm;
    VConfOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass),
          bit30(BIT30), bit31(BIT31), zimm(ZIMM), uimm(UIMM)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VMemLoadStoreOp : public RiscvStaticInst
{
  protected:
    uint64_t sew;
    bool vm;
    Request::Flags memAccessFlags;
    VMemLoadStoreOp(const char *mnem, MachInst _machInst, OpClass __opClass,
        uint64_t _width, bool _vm)
        : RiscvStaticInst(mnem, _machInst, __opClass),
        vm(_vm)
    {
        switch (_width)
        {
        case 0x0        : sew = 8; break;
        case 0x5 ... 0x7: sew = 8 << (_width - 4); break;
        default:
            panic("not supported vector load width %d", _width);
            break;
        }
    }
};

/**
 * Base class for Vector Load operations
 */
class VMemLoadOp : public VMemLoadStoreOp
{
  protected:
    VMemLoadOp(const char *mnem, MachInst _machInst, OpClass __opClass,
        uint64_t _width, bool _vm)
        : VMemLoadStoreOp(mnem, _machInst, __opClass, _width, _vm)
        {}
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

/**
 * Base class for Vector Store operations
 */
class VMemStoreOp : public VMemLoadStoreOp
{
  protected:
    VMemStoreOp(const char *mnem, MachInst _machInst, OpClass __opClass,
        uint64_t _width, bool _vm)
        : VMemLoadStoreOp(mnem, _machInst, __opClass, _width, _vm)
        {}
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};
} // namespace RiscvISA
} // namespace gem5


#endif // __ARCH_RISCV_INSTS_VECTOR_HH__