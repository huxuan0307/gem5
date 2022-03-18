#ifndef __ARCH_RISCV_MACROOP_HH__
#define __ARCH_RISCV_MACROOP_HH__

#include <bitset>

#include "arch/generic/memhelpers.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/regs/vector.hh"
#include "arch/riscv/utility.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

namespace RiscvISA
{

static constexpr uint32_t cache_line_size = 64;

static uint32_t width2sew(uint64_t width) {
    switch (bits(width, 2, 0)) {
        case 0b000: return 8;
        case 0b101: return 16;
        case 0b110: return 32;
        case 0b111: return 64;
        default: panic("width: %x not supported", bits(width, 2, 0));
    }
}

static uint64_t get_sew(const uint8_t vtype) {
    const uint8_t vsew = bits(vtype, 5, 3);
    switch (bits(vsew, 2, 0)) {
        case 0b000 ... 0b011: return 8<<vsew;
        default: panic("vsew: %x not supported", vsew);
    }
}

static int8_t get_ilmul(const uint8_t vtype) {
    const uint8_t vlmul = bits(vtype, 2, 0);
    int8_t res = (int8_t)sext<3>(vlmul);
    if (GEM5_LIKELY(res != -4))
        return res;
    else
        panic("vlmul: %x not supported", vlmul);
}

class VectorArithMicroInst : public RiscvMicroInst
{
protected:
    VectorArithMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass)
        : RiscvMicroInst(mnem, extMachInst, __opClass)
    {
        this->flags[IsVector] = true;
    }
    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorArithMacroInst : public RiscvMacroInst
{
protected:
    uint32_t vl;
    uint32_t sew;
    int8_t ilmul;
    bool vm;
    VectorArithMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : RiscvMacroInst(mnem, _extMachInst, __opClass),
        vl(_extMachInst.vl), sew(get_sew(_extMachInst.vtype)),
        ilmul(get_ilmul(_extMachInst.vtype)),
        vm(_extMachInst.vm)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t numMicroOp() {
        switch (this->ilmul) {
            case -3 ... -1:
                return 1;
            case 0 ... 3:
                return 1 << this->ilmul;
            default:
                panic("vlmul: %x not supported", this->machInst.vtype.vlmul);
        }
        return 0;
    }
};

class VectorMemMacroInst : public RiscvMacroInst
{
protected:
    uint32_t vl;
    uint32_t sew;
    int8_t ilmul; // -3->1/8, ..., 0->1, ..., 3->8
    VectorMemMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : RiscvMacroInst(mnem, _extMachInst, __opClass),
        vl(_extMachInst.vl),
        sew(width2sew(_extMachInst.width)), // sew set by mem inst not vconfig
        ilmul(get_ilmul(_extMachInst.vtype))
    {
        this->flags[IsVector] = true;
    }

    uint32_t numElemPerMemAcc() {
        return cache_line_size / this->sew;
    }

    uint32_t numMemAcc() {
        const uint32_t elemNumPerMemAcc = this->numElemPerMemAcc();
        return (vl + elemNumPerMemAcc - 1) / elemNumPerMemAcc;
    }

    constexpr uint32_t numMemAccPerVReg() {
        return RiscvISA::VLEN / cache_line_size;
    }
};

class VleMacroInst : public VectorMemMacroInst
{
protected:
    VleMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : VectorMemMacroInst(mnem, _extMachInst, __opClass)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VseMacroInst : public VectorMemMacroInst
{
protected:
    VseMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : VectorMemMacroInst(mnem, _extMachInst, __opClass)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VleMicroInst : public RiscvMicroInst
{
protected:
    uint32_t sew;
    uint32_t offset; // base addr in rs1
    uint8_t dst_reg;
    uint8_t micro_vl;
    bool vm;
    Request::Flags memAccessFlags;

    VleMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint32_t _offset, uint8_t _dst_reg,
            uint8_t _micro_vl)
        : RiscvMicroInst(mnem, extMachInst, __opClass),
        sew(width2sew(extMachInst.width)), offset(_offset), dst_reg(_dst_reg),
        micro_vl(_micro_vl), vm(extMachInst.vm), memAccessFlags(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VseMicroInst : public RiscvMicroInst
{
protected:
    uint32_t sew;
    uint32_t offset; // base addr in rs1
    uint8_t src_reg;
    uint8_t micro_vl;
    bool vm;
    Request::Flags memAccessFlags;

    VseMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint32_t _offset, uint8_t _src_reg,
            uint8_t _micro_vl)
        : RiscvMicroInst(mnem, extMachInst, __opClass),
        sew(width2sew(extMachInst.width)), offset(_offset), src_reg(_src_reg),
        micro_vl(_micro_vl), vm(extMachInst.vm), memAccessFlags(0)
    {}
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VldMvMicroInst : public VectorArithMicroInst
{
private:
    RegId srcRegIdxArr[NumVecMemInternalRegs];
    RegId destRegIdxArr[1];

public:
    VldMvMicroInst(ExtMachInst extMachInst, uint8_t _dst_reg)
        : VectorArithMicroInst("vl_mv_micro", extMachInst, VectorMemLoadOp)
    {
        setRegIdxArrays(
            reinterpret_cast<RegIdArrayPtr>(
                &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
            reinterpret_cast<RegIdArrayPtr>(
                &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

        _numSrcRegs = 0;
        _numDestRegs = 0;
        _numFPDestRegs = 0;
        _numVecDestRegs = 0;
        _numVecElemDestRegs = 0;
        _numVecPredDestRegs = 0;
        _numIntDestRegs = 0;
        _numCCDestRegs = 0;

        setDestRegIdx(_numDestRegs++, RegId(VecRegClass, _dst_reg));
        _numVecDestRegs++;
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, VecMemInternalReg0));
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, VecMemInternalReg0+1));
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, VecMemInternalReg0+2));
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, VecMemInternalReg0+3));
        this->flags[IsVector] = true;
    }
    Fault execute(ExecContext *, Trace::InstRecord *) const override;
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VstMvMicroInst : public VectorArithMicroInst
{
private:
    RegId srcRegIdxArr[1];
    RegId destRegIdxArr[NumVecMemInternalRegs];

public:
    VstMvMicroInst(ExtMachInst extMachInst, uint8_t _src_reg)
        : VectorArithMicroInst("vs_mv_micro", extMachInst, VectorMemStoreOp)
    {
        setRegIdxArrays(
            reinterpret_cast<RegIdArrayPtr>(
                &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
            reinterpret_cast<RegIdArrayPtr>(
                &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

        _numSrcRegs = 0;
        _numDestRegs = 0;
        _numFPDestRegs = 0;
        _numVecDestRegs = 0;
        _numVecElemDestRegs = 0;
        _numVecPredDestRegs = 0;
        _numIntDestRegs = 0;
        _numCCDestRegs = 0;

        setDestRegIdx(_numDestRegs++,
                        RegId(VecRegClass, VecMemInternalReg0));
        _numVecDestRegs++;
        setDestRegIdx(_numDestRegs++,
                        RegId(VecRegClass, VecMemInternalReg0 + 1));
        _numVecDestRegs++;
        setDestRegIdx(_numDestRegs++,
                        RegId(VecRegClass, VecMemInternalReg0 + 2));
        _numVecDestRegs++;
        setDestRegIdx(_numDestRegs++,
                        RegId(VecRegClass, VecMemInternalReg0 + 3));
        _numVecDestRegs++;
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, _src_reg));
        this->flags[IsVector] = true;
    }
    Fault execute(ExecContext *, Trace::InstRecord *) const override;
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorIntMacroOp : public VectorArithMacroInst
{
public:
    VectorIntMacroOp(const char* mnem, ExtMachInst _extMachInst,
            OpClass __opClass)
        : VectorArithMacroInst(mnem, _extMachInst, __opClass)
    {}
    using VectorArithMacroInst::generateDisassembly;
};

class VectorIntMicroOp : public VectorArithMicroInst
{
public:
    uint32_t sew;
    uint8_t micro_vl;
    uint8_t micro_idx;
    VectorIntMicroOp(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl, uint8_t _micro_idx)
        : VectorArithMicroInst(mnem, extMachInst, __opClass),
        sew(get_sew(extMachInst.vtype)), micro_vl(_micro_vl),
        micro_idx(_micro_idx)
    {}
    using VectorArithMicroInst::generateDisassembly;
};

}
}
#endif
