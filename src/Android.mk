# Copyright (c) 2012 Intel Corporation. All Rights Reserved.
#
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sub license, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice (including the
# next paragraph) shall be included in all copies or substantial portions
# of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
# IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
# ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES :=              \
        gen6_mfc.c              \
        gen6_mfd.c              \
        gen6_vme.c              \
        gen7_mfd.c              \
        i965_avc_bsd.c          \
        i965_avc_hw_scoreboard.c\
        i965_avc_ildb.c         \
        i965_decoder_utils.c    \
        i965_drv_video.c        \
        i965_encoder.c          \
        i965_media.c            \
        i965_media_h264.c       \
        i965_media_mpeg2.c      \
        i965_post_processing.c  \
        i965_render.c           \
        intel_batchbuffer.c     \
        intel_batchbuffer_dump.c\
        intel_driver.c          \
        intel_memman.c          \
        object_heap.c

LOCAL_CFLAGS := -DLINUX -DANDROID -g -Wall -Wno-unused -fvisibility=hidden

LOCAL_C_INCLUDES :=             \
    $(TARGET_OUT_HEADERS)/libva \
    $(TARGET_OUT_HEADERS)/libdrm

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := i965_drv_video

LOCAL_SHARED_LIBRARIES := libdl libdrm libdrm_intel libcutils \
               libva libva-android libstdc++

ifeq ($(strip $(DRIVER_LOG_ENABLE)),true)
LOCAL_CFLAGS += -DDRIVER_LOG_ENABLE
LOCAL_SHARED_LIBRARIES += liblog
endif

include $(BUILD_SHARED_LIBRARY)


