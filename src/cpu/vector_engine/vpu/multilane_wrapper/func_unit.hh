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

#include "debug/Datapath.hh"

float
Datapath::compute_float_fp_op(float Aitem, float Bitem, int Mitem,
    float Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    float Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfadd_vv") | (operation == "vfadd_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f + %f  = %f  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfsub_vv")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f - %f  = %f  \n",Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfmul_vv")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f  = %f  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfdiv_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f / %f  = %f\n" ,Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfsqrt_v")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? sqrt(Bitem) : Dstitem;
        DPRINTF(Datapath,"WB Instruction = sqrt (%f)   = %f  \n",
            Bitem,Ditem);
    }

    if ((operation == "vfmin_vv")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = min :%f\n",
            Aitem,Bitem, Ditem);
    }
    if ((operation == "vfmax_vv")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = max :%f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnj_vv")) {
        Ditem = (Bitem>=0.0) ? fabs(Aitem):-Aitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjn_vv")) {
        Ditem = (Bitem>=0.0) ?-Aitem: fabs(Aitem);
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjx_vv")) {
        Ditem = ((Bitem>=0.0) & (Aitem >= 0.0)) ? Aitem:
            ((Bitem<0.0) & (Aitem < 0.0)) ?  fabs(Aitem) :
            ((Bitem>=0.0) & (Aitem < 0.0)) ? Aitem : -Aitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmerge_vv")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        DPRINTF(Datapath,"WB Instruction = 0x%x : 0x%x  = 0x%x\n",
            *(uint32_t*)&Aitem,*(uint32_t*)&Bitem, *(uint32_t*)&Ditem);
    }

    if ((operation == "vfmacc_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Bitem) + Dstitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f + %f  = %f\n",
            Aitem,Bitem,Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Dstitem) + Bitem  : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f + %f  = %f\n",
            Aitem,Dstitem,Bitem, Ditem);
    }

    if (vm==0){
        DPRINTF(Datapath,"WB Instruction is masked vm(%d),"
            " old(%f)  \n",Mitem,Dstitem);
    }

    return Ditem;
}

double
Datapath::compute_double_fp_op(double Aitem, double Bitem,
     long int Mitem, double Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    double Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfadd_vv") | (operation == "vfadd_vi")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf + %lf  = %lf\n",
            Aitem,Bitem, Ditem);
    }
    if ((operation == "vfsub_vv")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf - %lf  = %lf\n",
            Bitem, Aitem, Ditem);
    }
    if ((operation == "vfmul_vv")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf  = %lf\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfdiv_vv")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf / %lf  = %lf\n",
            Bitem,Aitem, Ditem);
    }


    if ((operation == "vfsqrt_v")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? sqrt(Bitem) : Dstitem;
        DPRINTF(Datapath,"WB Instruction = sqrt (%lf)   = %lf\n",
            Bitem,Ditem);
    }

    if ((operation == "vfmin_vv"))  {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = min :%lf\n",
            Aitem,Bitem, Ditem);
    }
    if ((operation == "vfmax_vv")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = max :%lf\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnj_vv")) {
        Ditem = (Bitem>=0.0) ? fabs(Aitem):-Aitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjn_vv")) {
        Ditem = (Bitem>=0.0) ?-Aitem: fabs(Aitem);
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjx_vv")) {
        Ditem = ((Bitem>=0.0) & (Aitem >= 0.0)) ? Aitem:
            ((Bitem<0.0) & (Aitem < 0.0)) ?  fabs(Aitem) :
            ((Bitem>=0.0) & (Aitem < 0.0)) ? Aitem : -Aitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmerge_vv")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        DPRINTF(Datapath,"WB Instruction = 0x%lx : 0x%lx  = 0x%lx\n",
            *(uint64_t*)&Aitem,*(uint64_t*)&Bitem, *(uint64_t*)&Ditem);
    }

    if ((operation == "vfmacc_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Bitem) + Dstitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf + %lf  = %lf\n",
            Aitem,Bitem,Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Dstitem) + Bitem  : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf + %lf  = %lf\n"
            ,Aitem,Dstitem,Bitem, Ditem);
    }

    if (vm==0) {
        DPRINTF(Datapath,"WB Instruction is masked vm(%d), old(%lf)"
            "\n",Mitem,Dstitem);
    }

    return Ditem;
}

int
Datapath::compute_float_fp_comp_op(float Aitem, float Bitem,
    RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vflt_vv")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f < %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }
    if ((operation == "vfle_vv")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f <= %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_double_fp_comp_op(double Aitem, double Bitem,
    RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vflt_vv")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f < %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }
    if ((operation == "vfle_vv")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f <= %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    return Ditem;
}


long int
Datapath::compute_long_int_op(long int Aitem, long int Bitem,
    long int Mitem, long int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vadd_vv") | (operation == "vadd_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d + %d  = %d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vsub_vv") | (operation == "vsub_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d - %d  = %d  \n"
            ,Bitem,Aitem, Ditem);
    }

    if ((operation == "vmul_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d * %d  = %d  \n"
            ,Aitem,Bitem, Ditem);
    }

    if ((operation == "vdiv_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d / %d  = %d  \n"
            ,Bitem,Aitem, Ditem);
    }
    if ((operation == "vrem_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d mod %d  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsll_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem << Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %x << %x  = %x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsrl_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem >> Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d >> %d  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmseq_vv")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath,"WB Instruction = %d == %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmslt_vv")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vand_vv")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vor_vv")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vxor_vv")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmin_vv")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if (vm==0) {
        DPRINTF(Datapath,"WB Instruction is masked vm(%d), old(%d)"
            "\n",Mitem,Dstitem);
    }

    return Ditem;
}

int
Datapath::compute_int_op(int Aitem, int Bitem, int Mitem,
        int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();


    if ((operation == "vadd_vv") | (operation == "vadd_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d + %d  = %d\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vsub_vv") | (operation == "vsub_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d - %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmul_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d * %d  = %d\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vdiv_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d / %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vrem_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d mod %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsll_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem << Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lx << %lx  = %lx\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsrl_vv")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem >> Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d >> %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmseq_vv")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath,"WB Instruction = %d == %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmslt_vv")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vand_vv")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vor_vv")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vxor_vv")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmin_vv")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if (vm==0) {
        DPRINTF(Datapath,"WB Instruction is masked vm(%d), old(%d)"
            "\n",Mitem,Dstitem);
    }


    return Ditem;
}

double
Datapath::compute_cvt_f_x_64_op( long int Bitem, long int Mitem,
    long int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    double Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (double)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%d)  = %0.2lf\n",
            Bitem, Ditem);
    }

    return Ditem;
}

float
Datapath::compute_cvt_f_x_32_op( int Bitem, int Mitem, int Dstitem,
    RiscvISA::VectorStaticInst* insn)
{
    float Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (float)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%d)  = %0.2f\n",
            Bitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_cvt_x_f_64_op( double Bitem, long int Mitem,
    double Dstitem, RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (long int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%0.2lf) = %d\n",
            Bitem, Ditem);
    }

    return Ditem;
}

int
Datapath::compute_cvt_x_f_32_op( float Bitem, int Mitem,
    float Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%0.2f) = %d\n",
            Bitem, Ditem);
    }

    return Ditem;
}
