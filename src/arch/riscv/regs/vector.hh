#ifndef __ARCH_RISCV_REGS_VECTOR_HH__
#define __ARCH_RISCV_REGS_VECTOR_HH__

#include <cstdint>
#include <string>
#include <vector>

#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"

namespace gem5
{

namespace RiscvISA
{

constexpr unsigned NumVecElemPerVecReg = 4;
using VecElem = uint64_t;
constexpr size_t VLEN = NumVecElemPerVecReg * sizeof(VecElem);
using VecRegContainer =
    gem5::VecRegContainer<VLEN>;
using vreg_t = VecRegContainer;

using VecPredReg =
    gem5::VecPredRegT<VecElem, NumVecElemPerVecReg, false, false>;
using ConstVecPredReg =
    gem5::VecPredRegT<VecElem, NumVecElemPerVecReg, false, true>;
using VecPredRegContainer = VecPredReg::Container;


const int NumVecRegs = 32;

const std::vector<std::string> VecRegNames = {
    "v0",   "v1",   "v2",   "v3",   "v4",   "v5",   "v6",   "v7",
    "v8",   "v9",   "v10",  "v11",  "v12",  "v13",  "v14",  "v15",
    "v16",  "v17",  "v18",  "v19",  "v20",  "v21",  "v22",  "v23",
    "v24",  "v25",  "v26",  "v27",  "v28",  "v29",  "v30",  "v31"
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_VECTOR_HH__
