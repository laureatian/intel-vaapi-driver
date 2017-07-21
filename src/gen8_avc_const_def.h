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

#ifndef GEN8_AVC_const_DEF_H
#define GEN8_AVC_const_DEF_H

/*#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>*/

#define GEN8_AVC_MBENC_CURBE_SIZE  88
#define GEN8_AVC_ME_CURBE_SIZE  39
const unsigned int gen8_avc_mbenc_curbe_i_frame_dist_init_data[GEN8_AVC_MBENC_CURBE_SIZE];
const unsigned int gen8_avc_mbenc_curbe_normal_i_frame_init_data[GEN8_AVC_MBENC_CURBE_SIZE];
const unsigned int gen8_avc_mbenc_curbe_normal_p_frame_init_data[GEN8_AVC_MBENC_CURBE_SIZE];
const unsigned int gen8_avc_mbenc_curbe_normal_b_frame_init_data[GEN8_AVC_MBENC_CURBE_SIZE];
const unsigned int gen8_avc_me_curbe_init_data[GEN8_AVC_ME_CURBE_SIZE];
#endif //GEN8_AVC_const_DEF_H
