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
 *    Tiantian Wang <tiantian.wang@@intel.com>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <va/va.h>

#include "intel_batchbuffer.h"
#include "intel_driver.h"

#include "i965_defines.h"
#include "i965_structs.h"
#include "i965_drv_video.h"
#include "i965_encoder.h"
#include "i965_encoder_utils.h"
#include "intel_media.h"

#include "i965_gpe_utils.h"
#include "i965_encoder_common.h"
#include "i965_avc_encoder_common.h"
#include "gen8_avc_encoder_kernels.h"
#include "gen9_avc_encoder.h"
#include "gen8_avc_encoder.h"
#include "gen9_avc_const_def.h"
#include "gen8_avc_const_def.h"


gen8_avc_set_curbe_mbenc(VADriverContextP ctx,
                         struct encode_state *encode_state,
                         struct i965_gpe_context *gpe_context,
                         struct intel_encoder_context *encoder_context,
                         void * param)
{
    struct i965_driver_data *i965 = i965_driver_data(ctx);
    gen8_avc_mbenc_curbe_data *cmd;
    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct generic_enc_codec_state * generic_state = (struct generic_enc_codec_state *)vme_context->generic_enc_state;
    struct avc_enc_state * avc_state = (struct avc_enc_state *)vme_context->private_enc_state;

    VAEncSliceParameterBufferH264 * slice_param = avc_state->slice_param[0];
    VAEncPictureParameterBufferH264  *pic_param = avc_state->pic_param;
    VASurfaceID surface_id;
    struct object_surface *obj_surface;

    struct mbenc_param * curbe_param = (struct mbenc_param *)param ;
    unsigned char qp = 0;
    unsigned char me_method = 0;
    unsigned int mbenc_i_frame_dist_in_use = curbe_param->mbenc_i_frame_dist_in_use;
    unsigned int table_idx = 0;
    unsigned int curbe_size = 0;

    unsigned int preset = generic_state->preset;
    if (IS_CHERRYVIEW(i965->intel.device_info) ||
        IS_GEN8(i965->intel.device_info)) {
        cmd = (gen8_avc_mbenc_curbe_data *)i965_gpe_context_map_curbe(gpe_context);
        if (!cmd)
            return;
        curbe_size = sizeof(gen8_avc_mbenc_curbe_data);
        memset(cmd, 0, curbe_size);

        if (mbenc_i_frame_dist_in_use) {
            memcpy(cmd, gen8_avc_mbenc_curbe_i_frame_dist_init_data, curbe_size);

        } else {
            switch (generic_state->frame_type) {
            case SLICE_TYPE_I:
                memcpy(cmd, gen8_avc_mbenc_curbe_normal_i_frame_init_data, curbe_size);
                break;
            case SLICE_TYPE_P:
                memcpy(cmd, gen8_avc_mbenc_curbe_normal_p_frame_init_data, curbe_size);
                break;
            case SLICE_TYPE_B:
                memcpy(cmd, gen8_avc_mbenc_curbe_normal_b_frame_init_data, curbe_size);
                break;
            default:
                assert(0);
            }

        }
    } else {
        /* Never get here, just silence a gcc warning */
        assert(0);

        return;
    }

    me_method = (generic_state->frame_type == SLICE_TYPE_B) ? gen9_avc_b_me_method[preset] : gen9_avc_p_me_method[preset];
    qp = pic_param->pic_init_qp + slice_param->slice_qp_delta;

    cmd->dw0.adaptive_enable = gen9_avc_enable_adaptive_search[preset];
    cmd->dw37.adaptive_enable = gen9_avc_enable_adaptive_search[preset];
    cmd->dw0.t8x8_flag_for_inter_enable = avc_state->transform_8x8_mode_enable;
    cmd->dw37.t8x8_flag_for_inter_enable = avc_state->transform_8x8_mode_enable;

    cmd->dw2.max_len_sp = gen9_avc_max_len_sp[preset];
    cmd->dw38.max_len_sp = 0;

    cmd->dw3.src_access = 0;
    cmd->dw3.ref_access = 0;

    if (avc_state->ftq_enable && (generic_state->frame_type != SLICE_TYPE_I)) {
        //disable ftq_override by now.
        if (avc_state->ftq_override) {
            cmd->dw3.ftq_enable = avc_state->ftq_enable;

        } else {
            // both gen9 and gen95 come here by now
            if (generic_state->frame_type == SLICE_TYPE_P) {
                cmd->dw3.ftq_enable = gen9_avc_max_ftq_based_skip[preset] & 0x01;

            } else {
                cmd->dw3.ftq_enable = (gen9_avc_max_ftq_based_skip[preset] >> 1) & 0x01;
            }
        }
    } else {
        cmd->dw3.ftq_enable = 0;
    }

    if (avc_state->disable_sub_mb_partion)
        cmd->dw3.sub_mb_part_mask = 0x7;

    if (mbenc_i_frame_dist_in_use) {
        cmd->dw2.pitch_width = generic_state->downscaled_width_4x_in_mb;
        cmd->dw4.picture_height_minus1 = generic_state->downscaled_height_4x_in_mb - 1;
        cmd->dw5.slice_mb_height = (avc_state->slice_height + 4 - 1) / 4;
        cmd->dw6.batch_buffer_end = 0;
        cmd->dw31.intra_compute_type = 1;

    } else {
        cmd->dw2.pitch_width = generic_state->frame_width_in_mbs;
        cmd->dw4.picture_height_minus1 = generic_state->frame_height_in_mbs - 1;
        cmd->dw5.slice_mb_height = (avc_state->arbitrary_num_mbs_in_slice) ? generic_state->frame_height_in_mbs : avc_state->slice_height;

        {
            memcpy(&(cmd->dw8), gen9_avc_mode_mv_cost_table[slice_type_kernel[generic_state->frame_type]][qp], 8 * sizeof(unsigned int));
            if ((generic_state->frame_type == SLICE_TYPE_I) && avc_state->old_mode_cost_enable) {
                //cmd.g9->dw8 = gen9_avc_old_intra_mode_cost[qp];
            } else if (avc_state->skip_bias_adjustment_enable) {
                /* Load different MvCost for P picture when SkipBiasAdjustment is enabled
                // No need to check for P picture as the flag is only enabled for P picture */
                cmd->dw11.value = gen9_avc_mv_cost_p_skip_adjustment[qp];

            }
        }

        table_idx = (generic_state->frame_type == SLICE_TYPE_B) ? 1 : 0;
        memcpy(&(cmd->dw16), table_enc_search_path[table_idx][me_method], 16 * sizeof(unsigned int));
    }
    //cmd->dw4.enable_fbr_bypass = avc_state->fbr_bypass_enable;
    cmd->dw4.enable_intra_cost_scaling_for_static_frame = avc_state->sfd_enable && generic_state->hme_enabled;
    cmd->dw4.field_parity_flag = 0;//bottom field
    cmd->dw4.enable_cur_fld_idr = 0;//field realted
    cmd->dw4.contrained_intra_pred_flag = pic_param->pic_fields.bits.constrained_intra_pred_flag;
    cmd->dw4.hme_enable = generic_state->hme_enabled;
    cmd->dw4.picture_type = slice_type_kernel[generic_state->frame_type];
    cmd->dw4.use_actual_ref_qp_value = generic_state->hme_enabled && (gen9_avc_mr_disable_qp_check[preset] == 0);


    cmd->dw7.intra_part_mask = avc_state->transform_8x8_mode_enable ? 0 : 0x02;
    cmd->dw7.src_field_polarity = 0;//field related

    /*ftq_skip_threshold_lut set,dw14 /15*/

    /*r5 disable NonFTQSkipThresholdLUT*/
    if (generic_state->frame_type == SLICE_TYPE_P) {
        cmd->dw32.skip_val = gen9_avc_skip_value_p[avc_state->block_based_skip_enable][avc_state->transform_8x8_mode_enable][qp];

    } else if (generic_state->frame_type == SLICE_TYPE_B) {
        cmd->dw32.skip_val = gen9_avc_skip_value_b[avc_state->block_based_skip_enable][avc_state->transform_8x8_mode_enable][qp];

    }

    cmd->dw13.qp_prime_y = qp;
    cmd->dw13.qp_prime_cb = qp;
    cmd->dw13.qp_prime_cr = qp;
    cmd->dw13.target_size_in_word = 0xff;//hardcode for brc disable

    if ((generic_state->frame_type != SLICE_TYPE_I) && avc_state->multi_pre_enable) {
        switch (gen9_avc_multi_pred[preset]) {
        case 0:
            cmd->dw32.mult_pred_l0_disable = 128;
            cmd->dw32.mult_pred_l1_disable = 128;
            break;
        case 1:
            cmd->dw32.mult_pred_l0_disable = (generic_state->frame_type == SLICE_TYPE_P) ? 1 : 128;
            cmd->dw32.mult_pred_l1_disable = 128;
            break;
        case 2:
            cmd->dw32.mult_pred_l0_disable = (generic_state->frame_type == SLICE_TYPE_B) ? 1 : 128;
            cmd->dw32.mult_pred_l1_disable = (generic_state->frame_type == SLICE_TYPE_B) ? 1 : 128;
            break;
        case 3:
            cmd->dw32.mult_pred_l0_disable = 1;
            cmd->dw32.mult_pred_l1_disable = (generic_state->frame_type == SLICE_TYPE_B) ? 1 : 128;
            break;

        }

    } else {
        cmd->dw32.mult_pred_l0_disable = 128;
        cmd->dw32.mult_pred_l1_disable = 128;
    }

    /*field setting for dw33 34, ignored*/

    if (avc_state->adaptive_transform_decision_enable) {
        if (generic_state->frame_type != SLICE_TYPE_I) {
            cmd->dw34.enable_adaptive_tx_decision = 1;
        }

        // cmd->dw58.mb_texture_threshold = 1024;
        // cmd->dw58.tx_decision_threshold = 128;
    }


    if (generic_state->frame_type == SLICE_TYPE_B) {
        cmd->dw34.list1_ref_id0_frm_field_parity = 0; //frame only
        cmd->dw34.list1_ref_id0_frm_field_parity = 0;
        cmd->dw34.b_direct_mode = slice_param->direct_spatial_mv_pred_flag;
    }

    cmd->dw34.b_original_bff = 0; //frame only
    cmd->dw34.enable_mb_flatness_check_optimization = avc_state->flatness_check_enable;
    cmd->dw34.roi_enable_flag = curbe_param->roi_enabled;
    cmd->dw34.mad_enable_falg = avc_state->mad_enable;
    cmd->dw34.mb_brc_enable = avc_state->mb_qp_data_enable || generic_state->mb_brc_enabled;
    cmd->dw34.arbitray_num_mbs_per_slice = avc_state->arbitrary_num_mbs_in_slice;

    cmd->dw34.force_non_skip_check = avc_state->mb_disable_skip_map_enable;

    if (cmd->dw34.force_non_skip_check) {
        cmd->dw34.disable_enc_skip_check = avc_state->skip_check_disable;
    }


    cmd->dw36.check_all_fractional_enable = avc_state->caf_enable;
    cmd->dw38.ref_threshold = 400;
    cmd->dw39.hme_ref_windows_comb_threshold = (generic_state->frame_type == SLICE_TYPE_B) ? gen9_avc_hme_b_combine_len[preset] : gen9_avc_hme_combine_len[preset];

    /* Default:2 used for MBBRC (MB QP Surface width and height are 4x downscaled picture in MB unit * 4  bytes)
       0 used for MBQP data surface (MB QP Surface width and height are same as the input picture size in MB unit * 1bytes)
       starting GEN9, BRC use split kernel, MB QP surface is same size as input picture */
    cmd->dw47.mb_qp_read_factor = (avc_state->mb_qp_data_enable || generic_state->mb_brc_enabled) ? 0 : 2;

    if (mbenc_i_frame_dist_in_use) {
        cmd->dw13.qp_prime_y = 0;
        cmd->dw13.qp_prime_cb = 0;
        cmd->dw13.qp_prime_cr = 0;
        cmd->dw33.intra_16x16_nondc_penalty = 0;
        cmd->dw33.intra_8x8_nondc_penalty = 0;
        cmd->dw33.intra_4x4_nondc_penalty = 0;

    }
    if (cmd->dw4.use_actual_ref_qp_value) {
        cmd->dw44.actual_qp_value_for_ref_id0_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 0);
        cmd->dw44.actual_qp_value_for_ref_id1_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 1);
        cmd->dw44.actual_qp_value_for_ref_id2_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 2);
        cmd->dw44.actual_qp_value_for_ref_id3_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 3);
        cmd->dw45.actual_qp_value_for_ref_id4_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 4);
        cmd->dw45.actual_qp_value_for_ref_id5_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 5);
        cmd->dw45.actual_qp_value_for_ref_id6_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 6);
        cmd->dw45.actual_qp_value_for_ref_id7_list0 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 0, 7);
        cmd->dw46.actual_qp_value_for_ref_id0_list1 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 1, 0);
        cmd->dw46.actual_qp_value_for_ref_id1_list1 =  gen9_avc_get_qp_from_ref_list(ctx, slice_param, 1, 1);
    }

    table_idx = slice_type_kernel[generic_state->frame_type];
    cmd->dw46.ref_cost = gen9_avc_ref_cost[table_idx][qp];

    if (generic_state->frame_type == SLICE_TYPE_I) {
        cmd->dw0.skip_mode_enable = 0;
        cmd->dw37.skip_mode_enable = 0;
        cmd->dw36.hme_combine_overlap = 0;
        cmd->dw47.intra_cost_sf = 16;
        cmd->dw34.enable_direct_bias_adjustment = 0;
        cmd->dw34.enable_global_motion_bias_adjustment = 0;

    } else if (generic_state->frame_type == SLICE_TYPE_P) {
        cmd->dw1.max_num_mvs = i965_avc_get_max_mv_per_2mb(avc_state->seq_param->level_idc) / 2;
        cmd->dw3.bme_disable_fbr = 1;
        cmd->dw5.ref_width = gen9_avc_search_x[preset];
        cmd->dw5.ref_height = gen9_avc_search_y[preset];
        cmd->dw7.non_skip_zmv_added = 1;
        cmd->dw7.non_skip_mode_added = 1;
        cmd->dw7.skip_center_mask = 1;
        cmd->dw47.intra_cost_sf = (avc_state->adaptive_intra_scaling_enable) ? gen9_avc_adaptive_intra_scaling_factor[qp] : gen9_avc_intra_scaling_factor[qp];
        cmd->dw47.max_vmv_r = i965_avc_get_max_mv_len(avc_state->seq_param->level_idc) * 4;//frame onlys
        cmd->dw36.hme_combine_overlap = 1;
        cmd->dw36.num_ref_idx_l0_minus_one = (avc_state->multi_pre_enable) ? slice_param->num_ref_idx_l0_active_minus1 : 0;
        cmd->dw39.ref_width = gen9_avc_search_x[preset];
        cmd->dw39.ref_height = gen9_avc_search_y[preset];
        cmd->dw34.enable_direct_bias_adjustment = 0;
        cmd->dw34.enable_global_motion_bias_adjustment = avc_state->global_motion_bias_adjustment_enable;
        if (avc_state->global_motion_bias_adjustment_enable)
            cmd->dw58.hme_mv_cost_scaling_factor = avc_state->hme_mv_cost_scaling_factor;

    } else {
        cmd->dw1.max_num_mvs = i965_avc_get_max_mv_per_2mb(avc_state->seq_param->level_idc) / 2;
        cmd->dw1.bi_weight = avc_state->bi_weight;
        cmd->dw3.search_ctrl = 7;
        cmd->dw3.skip_type = 1;
        cmd->dw5.ref_width = gen9_avc_b_search_x[preset];
        cmd->dw5.ref_height = gen9_avc_b_search_y[preset];
        cmd->dw7.skip_center_mask = 0xff;
        cmd->dw47.intra_cost_sf = (avc_state->adaptive_intra_scaling_enable) ? gen9_avc_adaptive_intra_scaling_factor[qp] : gen9_avc_intra_scaling_factor[qp];
        cmd->dw47.max_vmv_r = i965_avc_get_max_mv_len(avc_state->seq_param->level_idc) * 4;//frame only
        cmd->dw36.hme_combine_overlap = 1;
        surface_id = slice_param->RefPicList1[0].picture_id;
        obj_surface = SURFACE(surface_id);
        if (!obj_surface) {
            WARN_ONCE("Invalid backward reference frame\n");
            return;
        }
        cmd->dw36.is_fwd_frame_short_term_ref = !!(slice_param->RefPicList1[0].flags & VA_PICTURE_H264_SHORT_TERM_REFERENCE);

        cmd->dw36.num_ref_idx_l0_minus_one = (avc_state->multi_pre_enable) ? slice_param->num_ref_idx_l0_active_minus1 : 0;
        cmd->dw36.num_ref_idx_l1_minus_one = (avc_state->multi_pre_enable) ? slice_param->num_ref_idx_l1_active_minus1 : 0;
        cmd->dw39.ref_width = gen9_avc_b_search_x[preset];
        cmd->dw39.ref_height = gen9_avc_b_search_y[preset];
        cmd->dw40.dist_scale_factor_ref_id0_list0 = avc_state->dist_scale_factor_list0[0];
        cmd->dw40.dist_scale_factor_ref_id1_list0 = avc_state->dist_scale_factor_list0[1];
        cmd->dw41.dist_scale_factor_ref_id2_list0 = avc_state->dist_scale_factor_list0[2];
        cmd->dw41.dist_scale_factor_ref_id3_list0 = avc_state->dist_scale_factor_list0[3];
        cmd->dw42.dist_scale_factor_ref_id4_list0 = avc_state->dist_scale_factor_list0[4];
        cmd->dw42.dist_scale_factor_ref_id5_list0 = avc_state->dist_scale_factor_list0[5];
        cmd->dw43.dist_scale_factor_ref_id6_list0 = avc_state->dist_scale_factor_list0[6];
        cmd->dw43.dist_scale_factor_ref_id7_list0 = avc_state->dist_scale_factor_list0[7];

        cmd->dw34.enable_direct_bias_adjustment = avc_state->direct_bias_adjustment_enable;
        if (cmd->dw34.enable_direct_bias_adjustment) {
            cmd->dw7.non_skip_zmv_added = 1;
            cmd->dw7.non_skip_mode_added = 1;
        }

        cmd->dw34.enable_global_motion_bias_adjustment = avc_state->global_motion_bias_adjustment_enable;
        if (avc_state->global_motion_bias_adjustment_enable)
            cmd->dw58.hme_mv_cost_scaling_factor = avc_state->hme_mv_cost_scaling_factor;

    }

    avc_state->block_based_skip_enable = cmd->dw3.block_based_skip_enable;

    if (avc_state->rolling_intra_refresh_enable) {
        /*by now disable it*/
        if (generic_state->brc_enabled) {
            cmd->dw4.enable_intra_refresh = false;
            cmd->dw34.widi_intra_refresh_en = avc_state->rolling_intra_refresh_enable;
            cmd->dw48.widi_intra_refresh_mbx = 0;
            cmd->dw58.widi_intra_refresh_mby = 0;
        } else {
            cmd->dw4.enable_intra_refresh = true;
            cmd->dw34.widi_intra_refresh_en = avc_state->rolling_intra_refresh_enable;
            //cmd->dw48.widi_intra_refresh_mbx = 0;
            //cmd->dw58.widi_intra_refresh_mby = 0;
        }
        cmd->dw32.mult_pred_l0_disable = 128;
        /* Pass the same IntraRefreshUnit to the kernel w/o the adjustment by -1, so as to have an overlap of one MB row or column of Intra macroblocks
         across one P frame to another P frame, as needed by the RollingI algo */
        cmd->dw48.widi_intra_refresh_mbx = 0;
        cmd->dw48.widi_intra_refresh_unit_in_mb_minus1 = 0;
        cmd->dw48.widi_intra_refresh_qp_delta = 0;

    } else {
        cmd->dw34.widi_intra_refresh_en = 0;
    }

    //cmd->dw34.enable_per_mb_static_check = avc_state->sfd_enable && generic_state->hme_enabled;
    //cmd->dw34.enable_adaptive_search_window_size = avc_state->adaptive_search_window_enable;

    /*roi set disable by now. 49-56*/
    if (curbe_param->roi_enabled) {
        cmd->dw49.roi_1_x_left   = generic_state->roi[0].left;
        cmd->dw49.roi_1_y_top    = generic_state->roi[0].top;
        cmd->dw50.roi_1_x_right  = generic_state->roi[0].right;
        cmd->dw50.roi_1_y_bottom = generic_state->roi[0].bottom;

        cmd->dw51.roi_2_x_left   = generic_state->roi[1].left;
        cmd->dw51.roi_2_y_top    = generic_state->roi[1].top;
        cmd->dw52.roi_2_x_right  = generic_state->roi[1].right;
        cmd->dw52.roi_2_y_bottom = generic_state->roi[1].bottom;

        cmd->dw53.roi_3_x_left   = generic_state->roi[2].left;
        cmd->dw53.roi_3_y_top    = generic_state->roi[2].top;
        cmd->dw54.roi_3_x_right  = generic_state->roi[2].right;
        cmd->dw54.roi_3_y_bottom = generic_state->roi[2].bottom;

        cmd->dw55.roi_4_x_left   = generic_state->roi[3].left;
        cmd->dw55.roi_4_y_top    = generic_state->roi[3].top;
        cmd->dw56.roi_4_x_right  = generic_state->roi[3].right;
        cmd->dw56.roi_4_y_bottom = generic_state->roi[3].bottom;

        cmd->dw36.enable_cabac_work_around = 0;

        if (!generic_state->brc_enabled) {
            char tmp = 0;
            tmp = generic_state->roi[0].value;
            CLIP(tmp, -qp, AVC_QP_MAX - qp);
            cmd->dw57.roi_1_dqp_prime_y = tmp;
            tmp = generic_state->roi[1].value;
            CLIP(tmp, -qp, AVC_QP_MAX - qp);
            cmd->dw57.roi_2_dqp_prime_y = tmp;
            tmp = generic_state->roi[2].value;
            CLIP(tmp, -qp, AVC_QP_MAX - qp);
            cmd->dw57.roi_3_dqp_prime_y = tmp;
            tmp = generic_state->roi[3].value;
            CLIP(tmp, -qp, AVC_QP_MAX - qp);
            cmd->dw57.roi_4_dqp_prime_y = tmp;
        } else {
            cmd->dw34.roi_enable_flag = 0;
        }
    }

    cmd->dw65.mb_data_surf_index = GEN8_AVC_MBENC_MFC_AVC_PAK_OBJ_CM_G8;
    cmd->dw66.mv_data_surf_index =  GEN8_AVC_MBENC_IND_MV_DATA_CM_G8;
    cmd->dw67.i_dist_surf_index = GEN8_AVC_MBENC_BRC_DISTORTION_CM_G8;
    cmd->dw68.src_y_surf_index = GEN8_ GEN8__AVC_MBENC_CURR_Y_CM_G8;
    cmd->dw69.mb_specific_data_surf_index = GEN8_AVC_MBENC_MB_SPECIFIC_DATA_CM_G8;
    cmd->dw70.aux_vme_out_surf_index = GEN8_AVC_MBENC_AUX_VME_OUT_CM_G8;
    cmd->dw71.curr_ref_pic_sel_surf_index = GEN8_AVC_MBENC_REFPICSELECT_L0_CM_G8;
    cmd->dw72.hme_mv_pred_fwd_bwd_surf_index = GEN8_AVC_MBENC_MV_DATA_FROM_ME_CM_G8;
    cmd->dw73.hme_dist_surf_index = GEN8_AVC_MBENC_4xME_DISTORTION_CM_G8;
    cmd->dw74.slice_map_surf_index = GEN8_AVC_MBENC_SLICEMAP_DATA_CM_G8;
    cmd->dw75.fwd_frm_mb_data_surf_index = GEN8_AVC_MBENC_FWD_MB_DATA_CM_G8;
    cmd->dw76.fwd_frm_mv_surf_index = GEN8_AVC_MBENC_FWD_MV_DATA_CM_G8;
    cmd->dw77.mb_qp_buffer = GEN8_AVC_MBENC_MBQP_CM_G8;
    cmd->dw78.mb_brc_lut = GEN8_AVC_MBENC_MBBRC_CONST_DATA_CM_G8;
    cmd->dw79.vme_inter_prediction_surf_index = GEN8_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_0_CM_G8;
    cmd->dw80.vme_inter_prediction_mr_surf_index = GEN8_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_1_CM_G8;
    cmd->dw81.flatness_chk_surf_index = GEN8_AVC_MBENC_FLATNESS_CHECK_CM_G8;
    cmd->dw82.mad_surf_index = GEN8_AVC_MBENC_MAD_DATA_CM_G8;
    cmd->dw83.force_non_skip_mb_map_surface = GEN8_AVC_MBENC_FORCE_NONSKIP_MB_MAP_CM_G8;
    cmd->dw84.widi_wa_surf_index = GEN8_AVC_MBENC_WIDI_WA_DATA_CM_G8;
    cmd->dw85.brc_curbe_surf_index = GEN8_AVC_MBENC_BRC_CURBE_DATA_CM_G8;
    cmd->dw86.static_detection_cost_table_index = GEN8_AVC_MBENC_STATIC_FRAME_DETECTION_OUTPUT_CM_G8;

    i965_gpe_context_unmap_curbe(gpe_context);

    return;
}

static void
gen8_avc_set_curbe_scaling4x(VADriverContextP ctx,
                             struct encode_state *encode_state,
                             struct i965_gpe_context *gpe_context,
                             struct intel_encoder_context *encoder_context,
                             void *param)
{
    gen8_avc_scaling4x_curbe_data *curbe_cmd;
    struct scaling_param *surface_param = (struct scaling_param *)param;

    curbe_cmd = i965_gpe_context_map_curbe(gpe_context);

    if (!curbe_cmd)
        return;

    memset(curbe_cmd, 0, sizeof(gen9_avc_scaling4x_curbe_data));

    curbe_cmd->dw0.input_picture_width  = surface_param->input_frame_width;
    curbe_cmd->dw0.input_picture_height = surface_param->input_frame_height;

    curbe_cmd->dw1.input_y_bti = GEN8_SCALING_FRAME_SRC_Y_CM;
    curbe_cmd->dw2.output_y_bti = GEN8_SCALING_FRAME_DST_Y_CM;


    curbe_cmd->dw5.flatness_threshold = 128;
    curbe_cmd->dw6.enable_mb_flatness_check = surface_param->enable_mb_flatness_check;
    curbe_cmd->dw6.enable_mb_variance_output = surface_param->enable_mb_variance_output;
    curbe_cmd->dw6.enable_mb_pixel_average_output = surface_param->enable_mb_pixel_average_output;

    if (curbe_cmd->dw6.enable_mb_variance_output ||
        curbe_cmd->dw6.enable_mb_pixel_average_output) {
        curbe_cmd->dw10.mbv_proc_states_bti_top_field  = GEN8_SCALING_FIELD_TOP_MBVPROCSTATS_DST_CM;
        curbe_cmd->dw11.mbv_proc_states_bti_bottom_field = GEN8_SCALING_FIELD_BOT_MBVPROCSTATS_DST_CM;
    }

    i965_gpe_context_unmap_curbe(gpe_context);
    return;
}
