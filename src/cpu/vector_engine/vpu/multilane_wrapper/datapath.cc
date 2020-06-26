/*
 * Copyright (c) 2020 Barcelona Supercomputing Center
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Cristóbal Ramírez
 */

#include "cpu/vector_engine/vpu/multilane_wrapper/datapath.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <queue>

#include "cpu/vector_engine/vpu/multilane_wrapper/func_unit.hh"
#include "cpu/vector_engine/vpu/multilane_wrapper/inst_info.hh"
#include "debug/Datapath.hh"
#include "debug/VectorEngine.hh"

Datapath::Datapath(
    DatapathParams *p) :
    TickedObject(p), VectorLanes(p->VectorLanes)
{
}

Datapath::~Datapath()
{
}


void
Datapath::startTicking(
    VectorLane& data_op_unit, RiscvISA::VectorStaticInst& insn,
    uint64_t src_count, uint64_t dst_count, uint64_t vsew,
    uint64_t slide_count, uint64_t src1,
    std::function<void(uint8_t*,uint8_t,bool)> data_callback)
{
    assert(!running);

    //copy over the configuration inputs
    this->vector_lane = &data_op_unit;
    this->insn = &insn;
    this->srcCount = src_count;
    this->dstCount = dst_count;
    this->vsew = vsew;
    this->slide_count = slide_count;
    this->src1 = src1;
    this->dataCallback = data_callback;
    //reset all state
    vecFuncs.clear();
    curSrcCount = 0;
    curDstCount = 0;
    first_elem = 0;
    accumInt = 0;
    accumDp = 0.0;
    accumSp = 0.0;
    red_SrcCount = 0;
    slide_SrcCount = 0;
    srcB_data_slide_count = 0;
    DATA_SIZE = (vsew == 3) ? sizeof(double) : (vsew == 2) ? sizeof(float) : 0;
    DST_SIZE = (vsew == 3) ? sizeof(double) : (vsew == 2) ? sizeof(float) : 0;
    assert(DATA_SIZE != 0);
    assert(DST_SIZE != 0);
    //reset config
    is_slide        =0;
    is_FP           =0;
    is_INT          =0;
    is_FP_Comp      =0;
    is_INT_to_FP    =0;
    is_FP_to_INT    =0;
    vfredsum_done   =0;
    slide_infligh   =0;

    std::string operation = this->insn->getName();
    vm = this->insn->vm();
    arith_src2              = this->insn->arith_src2();
    arith_src1_src2         = this->insn->arith_src1_src2();
    arith_src1_src2_src3    = this->insn->arith_src1_src2_src3();

    op_imm = (operation =="vfadd_vi") | (operation =="vadd_vi");

    vfredsum = (operation == "vfredsum_vs");

    is_slide =  this->insn->is_slide();

    vslideup =  this->insn->is_slideup();
    vslide1up = (operation == "vslide1up_vx");
    vslidedown = this->insn->is_slidedown();
    vslide1down = (operation == "vslide1down_vx");

    vmpopc = (operation == "vmpopc_m");
    vmfirst = (operation == "vmfirst_m");

    //Accumulator for reductions, vmpopc and vmfirst
    accumInt =-1;

    get_instruction_info();

    uint64_t pc = this->insn->getPC();
    DPRINTF(Datapath,"Executing inst %s, pc 0x%lx, Oplatency = %d,"
        " DataType = %d-bit\n" , this->insn->getName(),*(uint64_t*)&pc,
        Oplatency , DATA_SIZE*8);
    start();
}

void
Datapath::stopTicking()
{
    assert(running);
    stop();
}

void
Datapath::evaluate()
{
    assert(running);
    for (auto vecFunc : vecFuncs) {
        --vecFunc->cyclesLeft;
    }

    if (vecFuncs.size()) {
      TimedFunc * vecFunc = vecFuncs.front();
      if (vecFunc->cyclesLeft == 0) {
          vecFuncs.pop_front();
          vecFunc->execute();
          delete vecFunc;
      }
    }

    uint64_t max_simd_items = VectorLanes*(sizeof(double)/DST_SIZE);
    uint64_t simd_size = std::min(max_simd_items, srcCount-curSrcCount);

    if (simd_size == 0)
    {
        return;
    }

    if ((vm==0)) // Masked Operations uses
    {
        if (vfredsum)
        {
            if ((vector_lane->MdataQ.size() < simd_size))
            {
                return;
            }
        }
        else if ((vector_lane->MdataQ.size() < simd_size)
            | (vector_lane->DstdataQ.size() < simd_size))
        {
            return;
        }
    }

    if (arith_src1_src2_src3) // 3 sources operation
    {
        if ( (vector_lane->AdataQ.size() < simd_size) |
            (vector_lane->BdataQ.size() < simd_size) |
            (vector_lane->DstdataQ.size() < simd_size))
        {
           return;
        }
    }
    else if (vfredsum)   // Reduction Operation
    {
        if ( (vector_lane->BdataQ.size() < simd_size) |
            ((vector_lane->AdataQ.size() < 1) && !vfredsum_done))
        {
            return;
        }
    }
    else if (arith_src1_src2 | op_imm)  // 2 sources operation
    {
        //DPRINTF(VectorEngine," arith_src1_src2 \n" );
        if ( (vector_lane->AdataQ.size() < simd_size) |
            (vector_lane->BdataQ.size() < simd_size) )
        {
            return;
        }
    }
    else if (is_slide)
    {
        DPRINTF(Datapath," Is slide \n");
        if (( vector_lane->BdataQ.size() < simd_size ) |
            slide_infligh | (vector_lane->DstdataQ.size() < simd_size))
        {
            return;
        }
    }
    else if (arith_src2)  // 1 source operation (src2)
    {
        if ( vector_lane->BdataQ.size() < simd_size )
        {
            return;
        }
    }

    uint8_t * Adata = (uint8_t *)malloc(simd_size*DATA_SIZE);
    uint8_t * Bdata = (uint8_t *)malloc(simd_size*DATA_SIZE);
    uint8_t * Mdata = (uint8_t *)malloc(simd_size*DATA_SIZE);
    uint8_t * Dstdata = (uint8_t *)malloc(simd_size*DATA_SIZE);

    for (uint64_t i=0; i<simd_size; ++i) {

        if (vfredsum)
        {
            if (vector_lane->AdataQ.size() > 0)
            {
                vfredsum_done=1;
                uint8_t *Aitem = vector_lane->AdataQ.front();
                memcpy(Adata+(i*DATA_SIZE), Aitem, DATA_SIZE);
                vector_lane->AdataQ.pop_front();
                delete[] Aitem;

                if (vsew == 3)
                {
                    double Accitem = (double)((double*)Adata)[i] ;
                    accumDp = Accitem;
                }
                else
                {
                    float Accitem = (float)((float*)Adata)[i] ;
                    accumSp = Accitem;
                }
            }
        }
        else if (!arith_src2 | op_imm)
        {
            uint8_t *Aitem = vector_lane->AdataQ.front();
            memcpy(Adata+(i*DATA_SIZE), Aitem, DATA_SIZE);
            vector_lane->AdataQ.pop_front();
            delete[] Aitem;
        }

        uint8_t *Bitem = vector_lane->BdataQ.front();
        memcpy(Bdata+(i*DATA_SIZE), Bitem, DATA_SIZE);
        vector_lane->BdataQ.pop_front();
        delete[] Bitem;


        if (vm==0) // Mask register
        {
            uint8_t *Mitem = vector_lane->MdataQ.front();
            memcpy(Mdata+(i*DATA_SIZE), Mitem, DATA_SIZE);
            vector_lane->MdataQ.pop_front();
            delete[] Mitem;
        }

        if (((vm==0) & !vfredsum) | arith_src1_src2_src3 | is_slide)
        {
            uint8_t *Dstitem = vector_lane->DstdataQ.front();
            memcpy(Dstdata+(i*DATA_SIZE), Dstitem, DATA_SIZE);
            vector_lane->DstdataQ.pop_front();
            delete[] Dstitem;
        }
    }

    // need to track how many we've read in case
    // the block of elements is partially full
    curSrcCount += simd_size;

    //queue up the vecNode results for the given latency
    vecFuncs.push_back(new TimedFunc(Oplatency,
        [this,simd_size,Adata,Bdata,Mdata,Dstdata]() {

        //create data result buffer
        uint8_t * Ddata = (uint8_t *)malloc(simd_size*DST_SIZE);

        if (is_slide)
        {
            for (int i=0; i<simd_size; ++i)
            {
                if (vsew == 3)
                {
                    uint64_t Bitem = ((uint64_t*)Bdata)[i] ;
                    srcB_data_slide_64[srcB_data_slide_count] = Bitem;
                }
                else
                {
                    uint32_t Bitem = ((uint32_t*)Bdata)[i] ;
                    srcB_data_slide_32[srcB_data_slide_count] = Bitem;
                }
                srcB_data_slide_count++;
            }

            for (int i=0; i<simd_size; ++i)
            {
                if (vsew == 3)
                {
                    uint64_t slide_1element = src1;
                    long int Mitem = (0x0000000000000001) &&
                        (long int)((long int*)Mdata)[i];
                    uint64_t Dstitem = ((uint64_t*)Dstdata)[i] ;
                    uint64_t Ditem;
                    if (vslideup | vslide1up)
                    {
                        if (vslide1up && (slide_SrcCount == 0))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=slide_count)
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_64[slide_SrcCount-slide_count]:
                                Dstitem;
                        else
                            Ditem = Dstitem;

                        DPRINTF(Datapath," vslideup: Ditem 0x%x "
                            "Source 0x%x Old= 0x%x ,slide_1element = 0x%x \n",
                            Ditem,
                            srcB_data_slide_64[slide_SrcCount-slide_count],
                            Dstitem , slide_1element);
                        if (vm==0){
                            DPRINTF(Datapath," WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }
                   else if (vslidedown | vslide1down)
                    {
                        if (vslide1down && (slide_SrcCount+1 == srcCount))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=(srcCount-slide_count))
                            Ditem =  0;
                        else
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_64[slide_SrcCount] : Dstitem;

                        DPRINTF(Datapath," vslidedown: Ditem 0x%x "
                            "Source 0x%x Old= 0x%x ,slide_1element = 0x%x \n",
                            Ditem, srcB_data_slide_64[slide_SrcCount], Dstitem,
                            slide_1element);
                        if (vm==0) {
                            DPRINTF(Datapath," WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }

                    memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    slide_SrcCount++;
                }
                else
                {
                    uint32_t slide_1element = src1;
                    int Mitem = (0x00000001) && (int)((int*)Mdata)[i] ;
                    uint32_t Dstitem = ((uint32_t*)Dstdata)[i] ;
                    uint32_t Ditem;
                    if (vslideup | vslide1up)
                    {
                        if (vslide1up && (slide_SrcCount == 0))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=slide_count)
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_32[slide_SrcCount-slide_count]:
                                Dstitem;
                        else
                            Ditem = Dstitem;

                        DPRINTF(Datapath,"vslideup: Ditem 0x%x "
                            "Source 0x%x  Old= 0x%x  \n" ,Ditem,
                            srcB_data_slide_32[slide_SrcCount-slide_count],
                            Dstitem);
                        if (vm==0) {
                            DPRINTF(Datapath,"WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }
                    else if (vslidedown | vslide1down)
                    {
                        if (vslide1down && (slide_SrcCount+1 == srcCount))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=(srcCount-slide_count))
                            Ditem =  0;
                        else
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_32[slide_SrcCount] : Dstitem;

                        DPRINTF(Datapath,"vslidedown: Ditem 0x%x "
                            "Source 0x%x  Old= 0x%x  \n" ,Ditem,
                            srcB_data_slide_32[slide_SrcCount], Dstitem);
                        if (vm==0) {
                            DPRINTF(Datapath,"WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }

                    memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    slide_SrcCount++;
                }
            }
        }
        else
        {
            for (int i=0; i<simd_size; ++i)
            {
                if (is_FP | is_FP_Comp)  // FLOATING POINT OPERATION
                {
                    if (vsew == 3)
                    {
                        if (vfredsum)
                        {
                            double Bitem = (double)((double*)Bdata)[i] ;
                            long int Mitem = (0x0000000000000001) &&
                                (long int)((long int*)Mdata)[i] ;
                            accumDp = (vm==1) ? accumDp + Bitem : (Mitem) ?
                                accumDp + Bitem : accumDp;
                            red_SrcCount=red_SrcCount + 1;
                            DPRINTF(Datapath," Reduction: Source %lf"
                                " Acc= %lf\n" ,Bitem, accumDp);
                        } else {
                            double Aitem = (double)((double*)Adata)[i] ;
                            double Bitem = (double)((double*)Bdata)[i] ;
                            long int Mitem = (0x0000000000000001) &&
                                (long int)((long int*)Mdata)[i] ;
                            double Dstitem = (double)((double*)Dstdata)[i];
                            if (is_FP_Comp)
                            {
                                long int Ditem = compute_double_fp_comp_op(
                                    Aitem, Bitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            } else {
                                double Ditem = compute_double_fp_op(Aitem,
                                    Bitem, Mitem, Dstitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            }
                        }
                    }
                    else
                    {
                        if (vfredsum)
                        {
                            float Bitem = (float)((float*)Bdata)[i];
                            int Mitem = (0x00000001) && (int)((int*)Mdata)[i];
                            accumSp = (vm==1) ? accumSp + Bitem : (Mitem) ?
                                accumSp + Bitem : accumSp;
                            red_SrcCount=red_SrcCount + 1;
                            DPRINTF(Datapath," Reduction: Source %f "
                                "Acc= %f  \n" ,Bitem, accumSp);
                        } else {
                            float Aitem = (float)((float*)Adata)[i];
                            float Bitem = (float)((float*)Bdata)[i];
                            int Mitem = (0x00000001) && (int)((int*)Mdata)[i];
                            float Dstitem = (float)((float*)Dstdata)[i];
                            if (is_FP_Comp)
                            {
                                int Ditem = compute_float_fp_comp_op(Aitem,
                                    Bitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            } else {
                                float Ditem = compute_float_fp_op(Aitem, Bitem,
                                    Mitem, Dstitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            }
                        }
                    }

                    if (vfredsum && (red_SrcCount==srcCount))
                    {
                        uint8_t *ndata = new uint8_t[DST_SIZE];
                        if (vsew == 3)
                        {
                            memcpy(ndata, (uint8_t*)&accumDp, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        } else {
                            memcpy(ndata, (uint8_t*)&accumSp, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        }
                    }
                }
                else if (is_INT)
                {
                    if (vsew == 3)
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                long int Bitem = (0x0000000000000001) &&
                                    (long int)((long int*)Bdata)[i] ;
                                long int Mitem = (0x0000000000000001) &&
                                    (long int)((long int*)Mdata)[i] ;
                                accumInt = (vm==1) ? accumInt + Bitem :
                                    (Mitem) ? accumInt + Bitem : accumInt;
                                red_SrcCount=red_SrcCount + 1;
                                DPRINTF(Datapath," vmpopc: Source "
                                    "%ld  Acc= %ld  \n" ,Bitem, accumInt);
                            } else if (vmfirst) {
                                int Bitem = (0x0000000000000001) &&
                                    (long int)((long int*)Bdata)[i] ;
                                int Mitem = (0x0000000000000001) &&
                                    (long int)((long int*)Mdata)[i] ;

                                first_elem = (Bitem == 0x0000000000000001);
                                accumInt = ((vm==1) || ((vm==0) && (Mitem==1)))
                                        ? (first_elem) ? red_SrcCount :
                                        accumInt : accumInt;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath," vmfirst: "
                                        "first active element found at %ld"
                                        " position \n" ,accumInt);
                                    break;
                                }
                                red_SrcCount=red_SrcCount + 1;
                            }
                        }
                        else
                        {
                            long int Aitem = (long int)((long int*)Adata)[i];
                            long int Bitem = (long int)((long int*)Bdata)[i];
                            long int Mitem = (0x0000000000000001) &&
                                (long int)((long int*)Mdata)[i] ;
                            long int Dstitem =
                                (long int)((long int*)Dstdata)[i];
                            long int Ditem = compute_long_int_op(Aitem, Bitem,
                                Mitem, Dstitem, insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }
                    else
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                int Bitem = (0x00000001) &&
                                    (int)((int*)Bdata)[i];
                                int Mitem = (0x00000001) &&
                                    (int)((int*)Mdata)[i];
                                accumInt = (vm==1) ? accumInt + Bitem : (Mitem)
                                     ? accumInt + Bitem : accumInt;
                                red_SrcCount=red_SrcCount + 1;
                                DPRINTF(Datapath," vmpopc: Source %d"
                                    " Acc= %d  \n" ,Bitem, accumInt);

                            } else if (vmfirst) {
                                int Bitem = (0x00000001) &&
                                    (int)((int*)Bdata)[i];
                                int Mitem = (0x00000001) &&
                                    (int)((int*)Mdata)[i];

                                first_elem = (Bitem == 0x00000001);
                                accumInt = ((vm==1) || ((vm==0) && (Mitem==1)))
                                    ? (first_elem) ? red_SrcCount :
                                    accumInt : accumInt;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath," vmfirst: "
                                        "first active element found at %d "
                                        "position \n" ,accumInt);
                                    break;
                                }

                                red_SrcCount=red_SrcCount + 1;
                            }
                        } else {
                            int Aitem = (int)((int*)Adata)[i] ;
                            int Bitem = (int)((int*)Bdata)[i] ;
                            int Mitem = (0x00000001) && (int)((int*)Mdata)[i];
                            int Dstitem = (int)((int*)Dstdata)[i] ;
                            int Ditem = compute_int_op(Aitem, Bitem, Mitem,
                                Dstitem, insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }
                }
                else if (is_INT_to_FP)
                {
                    if (vsew == 3)
                    {
                        long int Bitem = (long int)((long int*)Bdata)[i];
                        long int Mitem = (0x0000000000000001) &&
                            (long int)((long int*)Mdata)[i] ;
                        long int Dstitem = (long int)((long int*)Dstdata)[i];
                        double Ditem = compute_cvt_f_x_64_op(Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    } else {
                        int Bitem = (int)((int*)Bdata)[i] ;
                        int Mitem = (0x00000001) && (int)((int*)Mdata)[i];
                        int Dstitem = (int)((int*)Dstdata)[i] ;
                        float Ditem = compute_cvt_f_x_32_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    }
                }
                else if (is_FP_to_INT)
                {
                    if (vsew == 3)
                    {
                        double Bitem = (double)((double*)Bdata)[i] ;
                        long int Mitem = (0x0000000000000001) &&
                            (long int)((long int*)Mdata)[i] ;
                        double Dstitem = (double)((double*)Dstdata)[i] ;
                        long int Ditem = compute_cvt_x_f_64_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                            DST_SIZE);
                    } else {
                        float Bitem = (float)((float*)Bdata)[i] ;
                        int Mitem = (0x00000001) && (int)((int*)Mdata)[i];
                        float Dstitem = (float)((float*)Dstdata)[i] ;
                        int Ditem = compute_cvt_x_f_32_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    }

                }
            }
        }
    //don't need these anymore
    delete[] Adata;
    delete[] Bdata;
    delete[] Mdata;
    delete[] Dstdata;


    if (!vfredsum)
    {
        if ( (vmpopc && (red_SrcCount==srcCount)) | (vmfirst & first_elem) |
            (vmfirst & (red_SrcCount==srcCount)) )
        {
            DPRINTF(Datapath," Elements= %d  \n" , accumInt);
            //WB Data to Int Register  File
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, (uint8_t*)&accumInt, DST_SIZE);
            dataCallback(ndata, DST_SIZE , 0);
        }
        else if (!vmpopc & !vmfirst)
        {
            //WB Data to Vect Register  File
            for (int i=0; i<simd_size; ++i) {
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, Ddata+(i*DST_SIZE), DST_SIZE);
                curDstCount++;
                bool _done = (  curDstCount == srcCount );
                dataCallback(ndata, DST_SIZE,_done);
            }
        }
        delete [] Ddata;
    }

    DPRINTF(Datapath," Leaving dataPath\n");

    }));
}

Datapath *
DatapathParams::create()
{
    return new Datapath(this);
}

