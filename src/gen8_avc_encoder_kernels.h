/*
 * Copyright @ 2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWAR OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *    Pengfei Qu <Pengfei.qu@intel.com>
 *
 */

#ifndef _GEN8_AVC_ENCODER_KERNELS_H
#define _GEN8_AVC_ENCODER_KERNELS_H

//ao
#define AVC_ENC_CHV_SZ  123116
extern const unsigned int chv_avc_encoder_kernels[AVC_ENC_CHV_SZ];

//cm
#define AVC_ENC_BDW_SZ  95532
extern const unsigned int bdw_avc_encoder_kernels[AVC_ENC_BDW_SZ];

//normal
#define AVC_ENC_BSW_SZ  123004
extern const unsigned int bsw_avc_encoder_kernels[AVC_ENC_BSW_SZ];

#endif//_GEN8_AVC_ENCODER_KERNELS_H
