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

static inline uint32_t width2sew(uint64_t width) {
    switch (bits(width, 2, 0)) {
        case 0b000: return 8;
        case 0b101: return 16;
        case 0b110: return 32;
        case 0b111: return 64;
        default: panic("width: %x not supported", bits(width, 2, 0));
    }
}

static inline uint8_t checked_vtype(bool vill, uint8_t vtype) {
    panic_if(vill, "vill has been set");
    const uint8_t vsew = bits(vtype, 5, 3);
    panic_if(vsew >= 0b100, "vsew: %#x not supported", vsew);
    const uint8_t vlmul = bits(vtype, 2, 0);
    panic_if(vlmul == 0b100, "vlmul: %#x not supported", vlmul);
    return vtype;
}

class VectorMacroInst : public RiscvMacroInst
{
protected:
    uint32_t vl;
    uint8_t vtype;
    bool vm;
    VectorMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : RiscvMacroInst(mnem, _extMachInst, __opClass),
        vl(_extMachInst.vl),
        vtype(checked_vtype(_extMachInst.vill, _extMachInst.vtype)),
        vm(_extMachInst.vm)
    {
        this->flags[IsVector] = true;
    }

    uint8_t vsew() const { return bits(this->vtype, 5, 3); }

    uint8_t vlmul() const { return bits(this->vtype, 2, 0); }

    virtual uint32_t sew() const = 0;

    int8_t ilmul() const { return (int8_t)sext<3>(this->vlmul()); }

    uint32_t vlmax() const { return RiscvISA::VLEN >> (vsew() + 3 - ilmul()); }
};

class VectorMicroInst : public RiscvMicroInst
{
protected:
    bool vm;
    uint8_t micro_vl;
    uint8_t vtype;
    VectorMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl)
        : RiscvMicroInst(mnem, extMachInst, __opClass),
        vm(extMachInst.vm), micro_vl(_micro_vl),
        vtype(extMachInst.vtype) // has been checked by vector macro inst
    {
        this->flags[IsVector] = true;
    }

    uint8_t vsew() const { return bits(this->vtype, 5, 3); }

    uint8_t vlmul() const { return bits(this->vtype, 2, 0); }

    virtual uint32_t sew() const = 0;

    int8_t ilmul() const { return (int8_t)sext<3>(this->vlmul()); }

    uint32_t vlmax() const { return RiscvISA::VLEN >> (vsew() + 3 - ilmul()); }

    uint64_t sew_mask() const { return (1 << (3 + vsew())) - 1; }
};

class VectorArithMicroInst : public VectorMicroInst
{
protected:
    VectorArithMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl)
        : VectorMicroInst(mnem, extMachInst, __opClass, _micro_vl)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t sew() const override { return 8 << this->vsew(); }

    uint32_t numMicroOp() const {
        return 1 << std::max<int8_t>(0, this->ilmul());
    }
};

class VectorArithMacroInst : public VectorMacroInst
{
protected:
    VectorArithMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : VectorMacroInst(mnem, _extMachInst, __opClass)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    uint32_t sew() const override { return 8 << this->vsew(); }

    uint32_t numMicroOp() const {
        return 1 << std::max<int8_t>(0, this->ilmul());
    }
};

class VectorMemMicroInst : public VectorMicroInst
{
    uint32_t _sew;
protected:
    VectorMemMicroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass, uint8_t _micro_vl)
        : VectorMicroInst(mnem, _extMachInst, __opClass, _micro_vl),
        _sew(width2sew(_extMachInst.width))
    {}

    uint32_t sew() const override { return _sew; }
};

class VectorMemMacroInst : public VectorMacroInst
{
    uint32_t _sew;
protected:
    VectorMemMacroInst(const char* mnem, ExtMachInst _extMachInst,
                   OpClass __opClass)
        : VectorMacroInst(mnem, _extMachInst, __opClass),
        _sew(width2sew(_extMachInst.width)) // sew set by mem inst not vconfig
    {}

    virtual uint32_t numElemPerMemAcc() {
        return cache_line_size / this->_sew;
    }

    uint32_t numMemAcc() {
        const uint32_t elemNumPerMemAcc = this->numElemPerMemAcc();
        return (vl + elemNumPerMemAcc - 1) / elemNumPerMemAcc;
    }

    constexpr uint32_t numMemAccPerVReg() {
        return RiscvISA::VLEN / cache_line_size;
    }

    uint32_t sew() const override { return _sew; }
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

class VleMicroInst : public VectorMemMicroInst
{
protected:
    uint32_t offset; // base addr in rs1
    uint8_t dst_reg;
    Request::Flags memAccessFlags;

    VleMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint32_t _offset, uint8_t _dst_reg,
            uint8_t _micro_vl)
        : VectorMemMicroInst(mnem, extMachInst, __opClass, _micro_vl),
        offset(_offset), dst_reg(_dst_reg),
        memAccessFlags(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VseMicroInst : public VectorMemMicroInst
{
protected:
    uint32_t offset; // base addr in rs1
    uint8_t src_reg;
    Request::Flags memAccessFlags;

    VseMicroInst(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint32_t _offset, uint8_t _src_reg,
            uint8_t _micro_vl)
        : VectorMemMicroInst(mnem, extMachInst, __opClass, _micro_vl),
        offset(_offset), src_reg(_src_reg),
        memAccessFlags(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VldMvMicroInst : public VectorArithMicroInst
{
private:
    RegId srcRegIdxArr[NumVecMemInternalRegs];
    RegId destRegIdxArr[1];
    uint8_t src_num;
public:
    VldMvMicroInst(ExtMachInst extMachInst, uint8_t _dst_reg, uint8_t _src_num)
        : VectorArithMicroInst("vl_mv_micro", extMachInst, VectorMemLoadOp, 0)
    {
        src_num = _src_num;
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
        for (uint8_t i=0; i<_src_num; i++) {
            setSrcRegIdx(_numSrcRegs++,
                        RegId(VecRegClass, VecMemInternalReg0 + i));
        }
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
    uint8_t dst_num;
public:
    VstMvMicroInst(ExtMachInst extMachInst, uint8_t _src_reg, uint8_t _dst_num)
        : VectorArithMicroInst("vs_mv_micro", extMachInst, VectorMemStoreOp, 0)
    {
        dst_num = _dst_num;
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

        for (uint8_t i=0; i<_dst_num; i++) {
            setDestRegIdx(_numDestRegs++,
                        RegId(VecRegClass, VecMemInternalReg0 + i));
            _numVecDestRegs++;
        }
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
    uint8_t micro_idx;
    VectorIntMicroOp(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl, uint8_t _micro_idx)
        : VectorArithMicroInst(mnem, extMachInst, __opClass, _micro_vl),
        micro_idx(_micro_idx)
    {}
    using VectorArithMicroInst::generateDisassembly;
};

class VectorFloatMacroOp : public VectorArithMacroInst
{
public:
    VectorFloatMacroOp(const char* mnem, ExtMachInst _extMachInst,
            OpClass __opClass)
        : VectorArithMacroInst(mnem, _extMachInst, __opClass)
    {}
    using VectorArithMacroInst::generateDisassembly;
};

class VectorFloatMicroOp : public VectorArithMicroInst
{
public:
    uint32_t sew;
    uint8_t micro_vl;
    uint8_t micro_idx;
    VectorFloatMicroOp(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl, uint8_t _micro_idx)
        : VectorArithMicroInst(mnem, extMachInst, __opClass),
        sew(get_sew(extMachInst.vtype.vsew)), micro_vl(_micro_vl),
        micro_idx(_micro_idx)
    {}
    using VectorArithMicroInst::generateDisassembly;
};

}
}
#endif
