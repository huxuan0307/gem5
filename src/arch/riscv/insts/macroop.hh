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

/*
  *  Spec Section 4.5
  *  Ref:
  *  https://github.com/qemu/qemu/blob/c7d773ae/target/riscv/vector_helper.c
*/
inline int
elem_mask(const uint8_t* v0, const int index)
{
  int idx = index / 8;
  int pos = index % 8;
  return (v0[idx] >> pos) & 1;
}

constexpr uint32_t cache_line_size = 64;

inline uint32_t width2sew(uint64_t width) {
    switch (bits(width, 2, 0)) {
        case 0b000: return 8;
        case 0b101: return 16;
        case 0b110: return 32;
        case 0b111: return 64;
        default: panic("width: %x not supported", bits(width, 2, 0));
    }
}

inline uint8_t checked_vtype(bool vill, uint8_t vtype) {
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

    uint32_t micro_vlmax() const { return RiscvISA::VLEN >> (vsew() + 3);}

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
    uint8_t micro_idx;
    VectorFloatMicroOp(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl, uint8_t _micro_idx)
        : VectorArithMicroInst(mnem, extMachInst, __opClass, _micro_vl),
        micro_idx(_micro_idx)
    {}
    using VectorArithMicroInst::generateDisassembly;
};

class VectorIntMaskMacroOp : public VectorArithMacroInst
{
public:
    VectorIntMaskMacroOp(const char* mnem, ExtMachInst _extMachInst,
            OpClass __opClass)
        : VectorArithMacroInst(mnem, _extMachInst, __opClass)
    {}
    using VectorArithMacroInst::generateDisassembly;
};

class VectorIntMaskMicroOp : public VectorArithMicroInst
{
public:
    uint8_t micro_idx;
    VectorIntMaskMicroOp(const char *mnem, ExtMachInst extMachInst,
            OpClass __opClass, uint8_t _micro_vl, uint8_t _micro_idx)
        : VectorArithMicroInst(mnem, extMachInst, __opClass, _micro_vl),
        micro_idx(_micro_idx)
    {}
    using VectorArithMicroInst::generateDisassembly;
};

// Todo: move somewhere
template<typename Type>
bool inline
carry_out(Type a, Type b, bool carry_in = false) {
    using TypeU = std::make_unsigned_t<Type>;
    TypeU s = *reinterpret_cast<TypeU*>(&a)
            + *reinterpret_cast<TypeU*>(&b) + carry_in;
    return carry_in
        ? (s <= *reinterpret_cast<TypeU*>(&a))
        : (s <  *reinterpret_cast<TypeU*>(&a));
}

template<typename ElemType>
class VmadcVV_micro;

template<typename ElemType>
class VmadcVV : public VectorIntMaskMacroOp
{
private:
    RegId srcRegIdxArr[4];
    RegId destRegIdxArr[1];
public:
    VmadcVV(ExtMachInst extMachInst);
    using VectorIntMaskMacroOp::generateDisassembly;
};

template<typename ElemType>
VmadcVV<ElemType>::VmadcVV(ExtMachInst extMachInst)
    : VectorIntMaskMacroOp("vmadc_vv", extMachInst, VectorIntOp)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));
            ;
    _numSrcRegs = 0;
    _numDestRegs = 0;
    _numFPDestRegs = 0;
    _numVecDestRegs = 0;
    _numVecElemDestRegs = 0;
    _numVecPredDestRegs = 0;
    _numIntDestRegs = 0;
    _numCCDestRegs = 0;
    setDestRegIdx(_numDestRegs++, RegId(VecRegClass, extMachInst.vd));
    _numVecDestRegs++;
    setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, extMachInst.vs1));
    setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, extMachInst.vs2));
    const uint32_t numMicroOps = numMicroOp();
    int32_t tmp_vl = this->vl;
    const int32_t micro_vlmax = TheISA::VLEN >> (extMachInst.vtype.vsew + 3);
    int32_t micro_vl = std::min(tmp_vl, micro_vlmax);
    StaticInstPtr micro_op;

    // allow one empty micro op to hold IsLastMicroop flag
    for (int i=0; i<numMicroOps && micro_vl>=0; ++i) {
        micro_op = new VmadcVV_micro<ElemType>(extMachInst, micro_vl, i);
        micro_op->setDelayedCommit();
        this->microops.push_back(micro_op);
        micro_vl = std::min(tmp_vl -= micro_vlmax, micro_vlmax);
    }

    this->microops.front()->setFirstMicroop();
    this->microops.back()->setLastMicroop();
}

template<typename ElemType>
class VmadcVV_micro : public VectorIntMaskMicroOp
{
private:
    RegId srcRegIdxArr[4];
    RegId destRegIdxArr[1];
    bool vm;
public:
    VmadcVV_micro(ExtMachInst extMachInst,
            uint8_t _micro_vl, uint8_t _micro_idx);
    Fault execute(ExecContext* xc, Trace::InstRecord* traceData)const override;
    using VectorIntMaskMicroOp::generateDisassembly;
};

template<typename ElemType>
VmadcVV_micro<ElemType>::VmadcVV_micro(ExtMachInst extMachInst,
        uint8_t _micro_vl, uint8_t _micro_idx)
    : VectorIntMaskMicroOp("vmadc_vv_micro", extMachInst,
        VectorIntOp, _micro_vl, _micro_idx)
{
    this->vm = extMachInst.vm;
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
                    RegId(VecRegClass, extMachInst.vd));
    _numVecDestRegs++;
    setSrcRegIdx(_numSrcRegs++,
                    RegId(VecRegClass, extMachInst.vs1 + _micro_idx));
    setSrcRegIdx(_numSrcRegs++,
                    RegId(VecRegClass, extMachInst.vs2 + _micro_idx));

    if (!this->vm)
        setSrcRegIdx(_numSrcRegs++, RegId(VecRegClass, 0));
}

template<typename ElemType>
Fault VmadcVV_micro<ElemType>::execute(ExecContext* xc,
                                        Trace::InstRecord* traceData) const
{
    using vu [[maybe_unused]] = std::make_unsigned_t<ElemType>;
    using vi [[maybe_unused]] = std::make_signed_t<ElemType>;
    if (machInst.vill)
      return std::make_shared<IllegalInstFault>("VILL is set", machInst);

    const auto micro_vlmax = TheISA::VLEN >> (machInst.vtype.vsew + 3);
    [[maybe_unused]] size_t greg_idx = 0;

    // %(op_decl)s;
    TheISA::VecRegContainer tmp_d0 = xc->getWritableVecRegOperand(this, 0);
    auto Vd = tmp_d0.as<uint8_t>();
    TheISA::VecRegContainer tmp_s0 = xc->readVecRegOperand(this, 0, greg_idx);
    auto Vs1 = tmp_s0.as<vu>();
    TheISA::VecRegContainer tmp_s1 = xc->readVecRegOperand(this, 1, greg_idx);
    auto Vs2 = tmp_s1.as<vu>();
    // %(op_rd)s;

    [[maybe_unused]] RiscvISA::vreg_t tmp_v0;
    [[maybe_unused]] uint8_t* v0;
    if (!this->vm) {
        tmp_v0 = xc->readVecRegOperand(this, _numSrcRegs-1); // v0 at the last
        v0 = tmp_v0.as<uint8_t>();
        for (uint32_t i = 0; i < this->micro_vl; i++) {
            uint32_t ei = i + micro_vlmax * this->micro_idx;
            Vd[ei/8] = (Vd[ei/8] & ~(1 << ei%8)) |
                    (carry_out(Vs2[i], Vs1[i], elem_mask(v0, ei)) << ei%8);
        }
    } else {
        for (uint32_t i = 0; i < this->micro_vl; i++) {
            uint32_t ei = i + micro_vlmax * this->micro_idx;
            Vd[ei/8] = (Vd[ei/8] & ~(1 << ei%8)) |
                    (carry_out(Vs2[i], Vs1[i]) << ei%8);
        }
    }

    xc->setVecRegOperand(this, 0, tmp_d0);
    if (traceData) {
        traceData->setData(tmp_d0);
    }
    return NoFault;
}


}
}
#endif
