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

void
Datapath::get_instruction_info()
{
    std::string operation = insn->getName();

    /**************************************************************************
     * Floating point Operations
     *************************************************************************/
    if ((operation == "vfadd_vv") | (operation == "vfadd_vi")) {
        Oplatency           = 4;
        is_FP               = 1;
    }

    if (operation == "vfsub_vv") {
        Oplatency           = 4;
        is_FP               = 1;
    }

    if (operation == "vfmul_vv") {
        Oplatency           = 3;
        is_FP               = 1;
    }

    if (operation == "vfdiv_vv") {
        Oplatency           = 14;
        is_FP               = 1;
    }
    if (operation == "vfsqrt_v") {
        Oplatency           = 12;
        is_FP               = 1;
    }

    if (operation == "vfmin_vv") {
        Oplatency           = 4;
        is_FP               = 1;
    }

    if (operation == "vfmax_vv") {
        Oplatency           = 4;
        is_FP               = 1;
    }

    if (operation == "vfsgnj_vv") {
        Oplatency           = 1;
        is_FP               = 1;
    }

    if (operation == "vfsgnjn_vv") {
        Oplatency           = 1;
        is_FP               = 1;
    }

    if (operation == "vfsgnjx_vv") {
        Oplatency           = 1;
        is_FP               = 1;
    }

     if (operation == "vmerge_vv") {
        Oplatency           = 1;
        is_FP               = 1;
    }

    if (operation == "vfmacc_vv") {
        Oplatency           = 6;
        is_FP               =1;
    }

    if (operation == "vfmadd_vv")   {
        Oplatency           = 6;
        is_FP               = 1;
    }

    /**************************************************************************
     * Floating point reductions
     *************************************************************************/

    if (operation == "vfredsum_vs") {
        Oplatency           = 4;
        is_FP               = 1;
    }

    /**************************************************************************
     * Floating point comparisons
     *************************************************************************/

    if (operation == "vflt_vv") {
        Oplatency           = 4;
        is_FP_Comp          = 1;
    }

    if (operation == "vfle_vv") {
        Oplatency           = 4;
        is_FP_Comp          = 1;
    }

    /**************************************************************************
     * Floating point conversions
     *************************************************************************/

    if (operation == "vfcvt_x_f_v") {
        Oplatency           = 4;
        is_FP_to_INT        = 1;
    }

    if (operation == "vfcvt_f_x_v") {
        Oplatency           = 4;
        is_INT_to_FP        = 1;
    }

    /**************************************************************************
     * Intefer Operations
     *************************************************************************/

    if ((operation == "vadd_vv") | (operation == "vadd_vi")) {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if ((operation == "vsub_vv") | (operation == "vsub_vi")) {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vmul_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vdiv_vv") {
        Oplatency           = 41;
        is_INT              = 1;
    }

    if (operation == "vrem_vv") {
        Oplatency           = 41;
        is_INT              = 1;
    }

    if (operation == "vsll_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vsrl_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vmseq_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vmslt_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vand_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vor_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vxor_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vmin_vv") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    /**************************************************************************
     * Slide Operations
     *************************************************************************/

    if ((operation == "vslideup_vi") | (operation == "vslideup_vx")) {
        Oplatency =((slide_count%VectorLanes)== 0) ? 1:slide_count%VectorLanes;
    }

    if (operation == "vslide1up_vx") {
        Oplatency           = 1;
    }

    if ((operation == "vslidedown_vi") | (operation == "vslidedown_vx")) {
        Oplatency =((slide_count%VectorLanes)== 0) ? 1:slide_count%VectorLanes;
    }

    if (operation == "vslide1down_vx") {
        Oplatency           = 1;
        is_slide            = 1;
    }

    /**************************************************************************
     * Operations with Mask
     *************************************************************************/

    if (operation == "vmpopc_m") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    if (operation == "vmfirst_m") {
        Oplatency           = 1;
        is_INT              = 1;
    }

    assert(is_FP || is_FP_Comp || is_FP_to_INT ||
        is_INT_to_FP || is_INT || is_slide);
}