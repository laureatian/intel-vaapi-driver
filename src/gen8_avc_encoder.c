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
#include "gen8_avc_encoder.h"
#include "gen8_avc_const_def.h"

static void
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
gen8_avc_send_surface_mbenc(VADriverContextP ctx,
                            struct encode_state *encode_state,
                            struct i965_gpe_context *gpe_context,
                            struct intel_encoder_context *encoder_context,
                            void * param_mbenc)
{
    struct i965_driver_data *i965 = i965_driver_data(ctx);
    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct i965_avc_encoder_context * avc_ctx = (struct i965_avc_encoder_context *)vme_context->private_enc_ctx;
    struct generic_enc_codec_state * generic_state = (struct generic_enc_codec_state *)vme_context->generic_enc_state;
    struct avc_enc_state * avc_state = (struct avc_enc_state *)vme_context->private_enc_state;
    struct object_surface *obj_surface;
    struct gen9_surface_avc *avc_priv_surface;
    struct i965_gpe_resource *gpe_resource;
    struct mbenc_param * param = (struct mbenc_param *)param_mbenc ;
    VASurfaceID surface_id;
    unsigned int mbenc_i_frame_dist_in_use = param->mbenc_i_frame_dist_in_use;
    unsigned int size = 0;
    unsigned int frame_mb_size = generic_state->frame_width_in_mbs *
                                 generic_state->frame_height_in_mbs;
    int i = 0;
    VAEncSliceParameterBufferH264 * slice_param = avc_state->slice_param[0];
    unsigned char is_g95 = 0;

    /*if (IS_SKL(i965->intel.device_info) ||
        IS_BXT(i965->intel.device_info))
        is_g95 = 0;
    else if (IS_KBL(i965->intel.device_info) ||
             IS_GLK(i965->intel.device_info))
        is_g95 = 1;*/

    obj_surface = encode_state->reconstructed_object;

    if (!obj_surface || !obj_surface->private_data)
        return;
    avc_priv_surface = obj_surface->private_data;

    /*pak obj command buffer output*/
    size = frame_mb_size * 16 * 4;
    gpe_resource = &avc_priv_surface->res_mb_code_surface;
    gen9_add_buffer_gpe_surface(ctx,
                                gpe_context,
                                gpe_resource,
                                0,
                                size / 4,
                                0,
                                GEN9_AVC_MBENC_MFC_AVC_PAK_OBJ_INDEX);

    /*mv data buffer output*/
    size = frame_mb_size * 32 * 4;
    gpe_resource = &avc_priv_surface->res_mv_data_surface;
    gen9_add_buffer_gpe_surface(ctx,
                                gpe_context,
                                gpe_resource,
                                0,
                                size / 4,
                                0,
                                GEN9_AVC_MBENC_IND_MV_DATA_INDEX);

    /*input current  YUV surface, current input Y/UV object*/
    if (mbenc_i_frame_dist_in_use) {
        obj_surface = encode_state->reconstructed_object;
        if (!obj_surface || !obj_surface->private_data)
            return;
        avc_priv_surface = obj_surface->private_data;
        obj_surface = avc_priv_surface->scaled_4x_surface_obj;
    } else {
        obj_surface = encode_state->input_yuv_object;
    }
    gen9_add_2d_gpe_surface(ctx,
                            gpe_context,
                            obj_surface,
                            0,
                            1,
                            I965_SURFACEFORMAT_R8_UNORM,
                            GEN9_AVC_MBENC_CURR_Y_INDEX);

    gen9_add_2d_gpe_surface(ctx,
                            gpe_context,
                            obj_surface,
                            1,
                            1,
                            I965_SURFACEFORMAT_R16_UINT,
                            GEN9_AVC_MBENC_CURR_UV_INDEX);

    if (generic_state->hme_enabled) {
        /*memv input 4x*/
        gpe_resource = &(avc_ctx->s4x_memv_data_buffer);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_MV_DATA_FROM_ME_INDEX);
        /* memv distortion input*/
        gpe_resource = &(avc_ctx->s4x_memv_distortion_buffer);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_4XME_DISTORTION_INDEX);
    }

    /*mbbrc const data_buffer*/
    if (param->mb_const_data_buffer_in_use) {
        size = 16 * AVC_QP_MAX * sizeof(unsigned int);
        gpe_resource = &avc_ctx->res_mbbrc_const_data_buffer;
        gen9_add_buffer_gpe_surface(ctx,
                                    gpe_context,
                                    gpe_resource,
                                    0,
                                    size / 4,
                                    0,
                                    GEN9_AVC_MBENC_MBBRC_CONST_DATA_INDEX);

    }

    /*mb qp data_buffer*/
    if (param->mb_qp_buffer_in_use) {
        if (avc_state->mb_qp_data_enable)
            gpe_resource = &(avc_ctx->res_mb_qp_data_surface);
        else
            gpe_resource = &(avc_ctx->res_mbbrc_mb_qp_data_surface);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_MBQP_INDEX);
    }

    /*input current  YUV surface, current input Y/UV object*/
    if (mbenc_i_frame_dist_in_use) {
        obj_surface = encode_state->reconstructed_object;
        if (!obj_surface || !obj_surface->private_data)
            return;
        avc_priv_surface = obj_surface->private_data;
        obj_surface = avc_priv_surface->scaled_4x_surface_obj;
    } else {
        obj_surface = encode_state->input_yuv_object;
    }
    gen9_add_adv_gpe_surface(ctx, gpe_context,
                             obj_surface,
                             GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_0_INDEX);
    /*input ref YUV surface*/
    for (i = 0; i < slice_param->num_ref_idx_l0_active_minus1 + 1; i++) {
        surface_id = slice_param->RefPicList0[i].picture_id;
        obj_surface = SURFACE(surface_id);
        if (!obj_surface || !obj_surface->private_data)
            break;

        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 obj_surface,
                                 GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_0_INDEX + i * 2 + 1);
    }
    /*input current  YUV surface, current input Y/UV object*/
    if (mbenc_i_frame_dist_in_use) {
        obj_surface = encode_state->reconstructed_object;
        if (!obj_surface || !obj_surface->private_data)
            return;
        avc_priv_surface = obj_surface->private_data;
        obj_surface = avc_priv_surface->scaled_4x_surface_obj;
    } else {
        obj_surface = encode_state->input_yuv_object;
    }
    gen9_add_adv_gpe_surface(ctx, gpe_context,
                             obj_surface,
                             GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_1_INDEX);

    for (i = 0; i < slice_param->num_ref_idx_l1_active_minus1 + 1; i++) {
        if (i > 0) break; // only  one ref supported here for B frame
        surface_id = slice_param->RefPicList1[i].picture_id;
        obj_surface = SURFACE(surface_id);
        if (!obj_surface || !obj_surface->private_data)
            break;

        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 obj_surface,
                                 GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_1_INDEX + i * 2 + 1);
        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 obj_surface,
                                 GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_0_INDEX + i * 2 + 2);
        if (i == 0) {
            avc_priv_surface = obj_surface->private_data;
            /*pak obj command buffer output(mb code)*/
            size = frame_mb_size * 16 * 4;
            gpe_resource = &avc_priv_surface->res_mb_code_surface;
            gen9_add_buffer_gpe_surface(ctx,
                                        gpe_context,
                                        gpe_resource,
                                        0,
                                        size / 4,
                                        0,
                                        GEN9_AVC_MBENC_FWD_MB_DATA_INDEX);

            /*mv data buffer output*/
            size = frame_mb_size * 32 * 4;
            gpe_resource = &avc_priv_surface->res_mv_data_surface;
            gen9_add_buffer_gpe_surface(ctx,
                                        gpe_context,
                                        gpe_resource,
                                        0,
                                        size / 4,
                                        0,
                                        GEN9_AVC_MBENC_FWD_MV_DATA_INDEX);

        }

        if (i < INTEL_AVC_MAX_BWD_REF_NUM) {
            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     obj_surface,
                                     GEN9_AVC_MBENC_VME_INTER_PRED_CURR_PIC_IDX_1_INDEX + i * 2 + 1 + INTEL_AVC_MAX_BWD_REF_NUM);
        }

    }

    /* BRC distortion data buffer for I frame*/
    if (mbenc_i_frame_dist_in_use) {
        gpe_resource = &(avc_ctx->res_brc_dist_data_surface);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_BRC_DISTORTION_INDEX);
    }

    /* as ref frame ,update later RefPicSelect of Current Picture*/
    obj_surface = encode_state->reconstructed_object;
    avc_priv_surface = obj_surface->private_data;
    if (avc_state->ref_pic_select_list_supported && avc_priv_surface->is_as_ref) {
        gpe_resource = &(avc_priv_surface->res_ref_pic_select_surface);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_REFPICSELECT_L0_INDEX);

    }

    // if (param->mb_vproc_stats_enable) {
    /*mb status buffer input*/
    /*   size = frame_mb_size * 16 * 4;
       gpe_resource = &(avc_ctx->res_mb_status_buffer);
       gen9_add_buffer_gpe_surface(ctx,
                                   gpe_context,
                                   gpe_resource,
                                   0,
                                   size / 4,
                                   0,
                                   GEN9_AVC_MBENC_MB_STATS_INDEX);

    } else*/ if (avc_state->flatness_check_enable) {

        gpe_resource = &(avc_ctx->res_flatness_check_surface);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_MB_STATS_INDEX);
    }

    if (param->mad_enable) {
        /*mad buffer input*/
        size = 4;
        gpe_resource = &(avc_ctx->res_mad_data_buffer);
        gen9_add_buffer_gpe_surface(ctx,
                                    gpe_context,
                                    gpe_resource,
                                    0,
                                    size / 4,
                                    0,
                                    GEN9_AVC_MBENC_MAD_DATA_INDEX);
        i965_zero_gpe_resource(gpe_resource);
    }

    /*brc updated mbenc curbe data buffer,it is ignored by gen9 and used in gen95*/
    /* if (avc_state->mbenc_brc_buffer_size > 0) {
         size = avc_state->mbenc_brc_buffer_size;
         gpe_resource = &(avc_ctx->res_mbenc_brc_buffer);
         gen9_add_buffer_gpe_surface(ctx,
                                     gpe_context,
                                     gpe_resource,
                                     0,
                                     size / 4,
                                     0,
                                     GEN95_AVC_MBENC_BRC_CURBE_DATA_INDEX);
     }*/

    /*artitratry num mbs in slice*/
    if (avc_state->arbitrary_num_mbs_in_slice) {
        /*slice surface input*/
        gpe_resource = &(avc_ctx->res_mbenc_slice_map_surface);
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_MBENC_SLICEMAP_DATA_INDEX);
        gen9_avc_generate_slice_map(ctx, encode_state, encoder_context);
    }

    /* BRC distortion data buffer for I frame */
    if (!mbenc_i_frame_dist_in_use) {
        if (avc_state->mb_disable_skip_map_enable) {
            gpe_resource = &(avc_ctx->res_mb_disable_skip_map_surface);
            gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                           gpe_resource,
                                           1,
                                           I965_SURFACEFORMAT_R8_UNORM,
                                           (is_g95 ? GEN95_AVC_MBENC_FORCE_NONSKIP_MB_MAP_INDEX : GEN9_AVC_MBENC_FORCE_NONSKIP_MB_MAP_INDEX));
        }

        /*  if (avc_state->sfd_enable && generic_state->hme_enabled) {
              if (generic_state->frame_type == SLICE_TYPE_P) {
                  gpe_resource = &(avc_ctx->res_sfd_cost_table_p_frame_buffer);

              } else if (generic_state->frame_type == SLICE_TYPE_B) {
                  gpe_resource = &(avc_ctx->res_sfd_cost_table_b_frame_buffer);
              }

              if (generic_state->frame_type != SLICE_TYPE_I) {
                  gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                                 gpe_resource,
                                                 1,
                                                 I965_SURFACEFORMAT_R8_UNORM,
                                                 (is_g95 ? GEN95_AVC_MBENC_SFD_COST_TABLE_INDEX : GEN9_AVC_MBENC_SFD_COST_TABLE_INDEX));
              }
          }*/
    }

    if (avc_state->sfd_enable) {
        size = 128 / sizeof(unsigned long);
        gpe_resource = &(avc_ctx->res_sfd_output_buffer);
        gen9_add_buffer_gpe_surface(ctx,
                                    gpe_context,
                                    gpe_resource,
                                    0,
                                    size / 4,
                                    0,
                                    GEN8_AVC_MBENC_STATIC_FRAME_DETECTION_OUTPUT_CM_G8);
    }

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

static void
gen8_avc_send_surface_scaling(VADriverContextP ctx,
                              struct encode_state *encode_state,
                              struct i965_gpe_context *gpe_context,
                              struct intel_encoder_context *encoder_context,
                              void *param)
{
    struct scaling_param *surface_param = (struct scaling_param *)param;
    unsigned int surface_format;
    unsigned int res_size;

    if (surface_param->scaling_out_use_32unorm_surf_fmt)
        surface_format = I965_SURFACEFORMAT_R32_UNORM;
    else if (surface_param->scaling_out_use_16unorm_surf_fmt)
        surface_format = I965_SURFACEFORMAT_R16_UNORM;
    else
        surface_format = I965_SURFACEFORMAT_R8_UNORM;

    gen9_add_2d_gpe_surface(ctx, gpe_context,
                            surface_param->input_surface,
                            0, 1, surface_format,
                            GEN9_AVC_SCALING_FRAME_SRC_Y_INDEX);

    gen9_add_2d_gpe_surface(ctx, gpe_context,
                            surface_param->output_surface,
                            0, 1, surface_format,
                            GEN9_AVC_SCALING_FRAME_DST_Y_INDEX);

    /*add buffer mv_proc_stat, here need change*/
    if (surface_param->mbv_proc_stat_enabled) {
        res_size = 16 * (surface_param->input_frame_width / 16) * (surface_param->input_frame_height / 16) * sizeof(unsigned int);

        gen9_add_buffer_gpe_surface(ctx,
                                    gpe_context,
                                    surface_param->pres_mbv_proc_stat_buffer,
                                    0,
                                    res_size / 4,
                                    0,
                                    GEN8_SCALING_FRAME_MBVPROCSTATS_DST_CM);
    } else if (surface_param->enable_mb_flatness_check) {
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       surface_param->pres_flatness_check_surface,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN8_SCALING_FRAME_MBVPROCSTATS_DST_CM);
    }

    return;
}
static void
gen8_avc_set_curbe_me(VADriverContextP ctx,
                      struct encode_state *encode_state,
                      struct i965_gpe_context *gpe_context,
                      struct intel_encoder_context *encoder_context,
                      void * param)
{
    gen8_avc_me_curbe_data *curbe_cmd;
    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct generic_enc_codec_state * generic_state = (struct generic_enc_codec_state *)vme_context->generic_enc_state;
    struct avc_enc_state * avc_state = (struct avc_enc_state *)vme_context->private_enc_state;

    VAEncSliceParameterBufferH264 * slice_param = avc_state->slice_param[0];

    struct me_param * curbe_param = (struct me_param *)param ;
    unsigned char  use_mv_from_prev_step = 0;
    unsigned char write_distortions = 0;
    unsigned char qp_prime_y = 0;
    unsigned char me_method = gen9_avc_p_me_method[generic_state->preset];
    unsigned char seach_table_idx = 0;
    unsigned char mv_shift_factor = 0, prev_mv_read_pos_factor = 0;
    unsigned int downscaled_width_in_mb, downscaled_height_in_mb;
    unsigned int scale_factor = 0;

    qp_prime_y = avc_state->pic_param->pic_init_qp + slice_param->slice_qp_delta;
    switch (curbe_param->hme_type) {
    case INTEL_ENC_HME_4x : {
        use_mv_from_prev_step = (generic_state->b16xme_enabled) ? 1 : 0;
        write_distortions = 1;
        mv_shift_factor = 2;
        scale_factor = 4;
        prev_mv_read_pos_factor = 0;
        break;
    }
    case INTEL_ENC_HME_16x : {
        use_mv_from_prev_step = (generic_state->b32xme_enabled) ? 1 : 0;
        write_distortions = 0;
        mv_shift_factor = 2;
        scale_factor = 16;
        prev_mv_read_pos_factor = 1;
        break;
    }
    case INTEL_ENC_HME_32x : {
        use_mv_from_prev_step = 0;
        write_distortions = 0;
        mv_shift_factor = 1;
        scale_factor = 32;
        prev_mv_read_pos_factor = 0;
        break;
    }
    default:
        assert(0);

    }
    curbe_cmd = i965_gpe_context_map_curbe(gpe_context);

    if (!curbe_cmd)
        return;

    downscaled_width_in_mb = ALIGN(generic_state->frame_width_in_pixel / scale_factor, 16) / 16;
    downscaled_height_in_mb = ALIGN(generic_state->frame_height_in_pixel / scale_factor, 16) / 16;

    memcpy(curbe_cmd, gen8_avc_me_curbe_init_data, sizeof(gen8_avc_me_curbe_data));

    curbe_cmd->dw3.sub_pel_mode = 3;
    if (avc_state->field_scaling_output_interleaved) {
        /*frame set to zero,field specified*/
        curbe_cmd->dw3.src_access = 0;
        curbe_cmd->dw3.ref_access = 0;
        curbe_cmd->dw7.src_field_polarity = 0;
    }
    curbe_cmd->dw4.picture_height_minus1 = downscaled_height_in_mb - 1;
    curbe_cmd->dw4.picture_width = downscaled_width_in_mb;
    curbe_cmd->dw5.qp_prime_y = qp_prime_y;

    curbe_cmd->dw6.use_mv_from_prev_step = use_mv_from_prev_step;
    curbe_cmd->dw6.write_distortions = write_distortions;
    curbe_cmd->dw6.super_combine_dist = gen9_avc_super_combine_dist[generic_state->preset];
    curbe_cmd->dw6.max_vmvr = i965_avc_get_max_mv_len(avc_state->seq_param->level_idc) * 4;//frame only

    if (generic_state->frame_type == SLICE_TYPE_B) {
        curbe_cmd->dw1.bi_weight = 32;
        curbe_cmd->dw13.num_ref_idx_l1_minus1 = slice_param->num_ref_idx_l1_active_minus1;
        me_method = gen9_avc_b_me_method[generic_state->preset];
        seach_table_idx = 1;
    }

    if (generic_state->frame_type == SLICE_TYPE_P ||
        generic_state->frame_type == SLICE_TYPE_B)
        curbe_cmd->dw13.num_ref_idx_l0_minus1 = slice_param->num_ref_idx_l0_active_minus1;

    curbe_cmd->dw15.prev_mv_read_pos_factor = prev_mv_read_pos_factor;
    curbe_cmd->dw15.mv_shift_factor = mv_shift_factor;

    memcpy(&curbe_cmd->dw16, table_enc_search_path[seach_table_idx][me_method], 14 * sizeof(int));

    curbe_cmd->dw32._4x_memv_output_data_surf_index = GEN8_AVC_ME_MV_DATA_SURFACE_INDEX;
    curbe_cmd->dw33._16x_32x_memv_input_data_surf_index = (curbe_param->hme_type == INTEL_ENC_HME_32x) ? GEN8_AVC_32XME_MV_DATA_SURFACE_INDEX : GEN8_AVC_16XME_MV_DATA_SURFACE_INDEX ;
    curbe_cmd->dw34._4x_me_output_dist_surf_index = GEN8_AVC_ME_DISTORTION_SURFACE_INDEX;
    curbe_cmd->dw35._4x_me_output_brc_dist_surf_index = GEN8_AVC_ME_BRC_DISTORTION_INDEX;
    curbe_cmd->dw36.vme_fwd_inter_pred_surf_index = GEN8_AVC_ME_CURR_FOR_FWD_REF_INDEX;
    curbe_cmd->dw37.vme_bdw_inter_pred_surf_index = GEN8_AVC_ME_CURR_FOR_BWD_REF_INDEX;
    curbe_cmd->dw38.reserved = 0;

    i965_gpe_context_unmap_curbe(gpe_context);
    return;
}

static void
gen8_avc_send_surface_me(VADriverContextP ctx,
                         struct encode_state *encode_state,
                         struct i965_gpe_context *gpe_context,
                         struct intel_encoder_context *encoder_context,
                         void * param)
{
    struct i965_driver_data *i965 = i965_driver_data(ctx);

    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct generic_enc_codec_state * generic_state = (struct generic_enc_codec_state *)vme_context->generic_enc_state;
    struct i965_avc_encoder_context * avc_ctx = (struct i965_avc_encoder_context *)vme_context->private_enc_ctx;
    struct avc_enc_state * avc_state = (struct avc_enc_state *)vme_context->private_enc_state;

    struct object_surface *obj_surface, *input_surface;
    struct gen9_surface_avc *avc_priv_surface;
    struct i965_gpe_resource *gpe_resource;
    struct me_param * curbe_param = (struct me_param *)param ;

    VAEncSliceParameterBufferH264 * slice_param = avc_state->slice_param[0];
    VASurfaceID surface_id;
    int i = 0;

    /* all scaled input surface stored in reconstructed_object*/
    obj_surface = encode_state->reconstructed_object;
    if (!obj_surface || !obj_surface->private_data)
        return;
    avc_priv_surface = obj_surface->private_data;


    switch (curbe_param->hme_type) {
    case INTEL_ENC_HME_4x : {
        /*memv output 4x*/
        gpe_resource = &avc_ctx->s4x_memv_data_buffer;
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_ME_MV_DATA_SURFACE_INDEX);

        /*memv input 16x*/
        if (generic_state->b16xme_enabled) {
            gpe_resource = &avc_ctx->s16x_memv_data_buffer;
            gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                           gpe_resource,
                                           1,
                                           I965_SURFACEFORMAT_R8_UNORM,
                                           GEN9_AVC_16XME_MV_DATA_SURFACE_INDEX);
        }
        /* brc distortion  output*/
        gpe_resource = &avc_ctx->res_brc_dist_data_surface;
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_ME_BRC_DISTORTION_INDEX);
        /* memv distortion output*/
        gpe_resource = &avc_ctx->s4x_memv_distortion_buffer;
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_ME_DISTORTION_SURFACE_INDEX);
        /*input current down scaled YUV surface*/
        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_4x_surface_obj;
        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX);
        /*input ref scaled YUV surface*/
        for (i = 0; i < slice_param->num_ref_idx_l0_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList0[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_4x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX + i * 2 + 1);
        }

        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_4x_surface_obj;

        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX);

        for (i = 0; i < slice_param->num_ref_idx_l1_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList1[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_4x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX + i * 2 + 1);
        }
        break;

    }
    case INTEL_ENC_HME_16x : {
        gpe_resource = &avc_ctx->s16x_memv_data_buffer;
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_ME_MV_DATA_SURFACE_INDEX);

        if (generic_state->b32xme_enabled) {
            gpe_resource = &avc_ctx->s32x_memv_data_buffer;
            gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                           gpe_resource,
                                           1,
                                           I965_SURFACEFORMAT_R8_UNORM,
                                           GEN9_AVC_32XME_MV_DATA_SURFACE_INDEX);
        }

        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_16x_surface_obj;
        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX);

        for (i = 0; i < slice_param->num_ref_idx_l0_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList0[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_16x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX + i * 2 + 1);
        }

        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_16x_surface_obj;

        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX);

        for (i = 0; i < slice_param->num_ref_idx_l1_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList1[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_16x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX + i * 2 + 1);
        }
        break;
    }
    case INTEL_ENC_HME_32x : {
        gpe_resource = &avc_ctx->s32x_memv_data_buffer;
        gen9_add_buffer_2d_gpe_surface(ctx, gpe_context,
                                       gpe_resource,
                                       1,
                                       I965_SURFACEFORMAT_R8_UNORM,
                                       GEN9_AVC_ME_MV_DATA_SURFACE_INDEX);

        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_32x_surface_obj;
        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX);

        for (i = 0; i < slice_param->num_ref_idx_l0_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList0[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_32x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_FWD_REF_INDEX + i * 2 + 1);
        }

        obj_surface = encode_state->reconstructed_object;
        avc_priv_surface = obj_surface->private_data;
        input_surface = avc_priv_surface->scaled_32x_surface_obj;

        gen9_add_adv_gpe_surface(ctx, gpe_context,
                                 input_surface,
                                 GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX);

        for (i = 0; i < slice_param->num_ref_idx_l1_active_minus1 + 1; i++) {
            surface_id = slice_param->RefPicList1[i].picture_id;
            obj_surface = SURFACE(surface_id);
            if (!obj_surface || !obj_surface->private_data)
                break;
            avc_priv_surface = obj_surface->private_data;

            input_surface = avc_priv_surface->scaled_32x_surface_obj;

            gen9_add_adv_gpe_surface(ctx, gpe_context,
                                     input_surface,
                                     GEN9_AVC_ME_CURR_FOR_BWD_REF_INDEX + i * 2 + 1);
        }
        break;
    }
    default:
        assert(0);

    }
}

static VAStatus
static void
gen8_avc_set_curbe_brc_frame_update(VADriverContextP ctx,
                                    struct encode_state *encode_state,
                                    struct i965_gpe_context *gpe_context,
                                    struct intel_encoder_context *encoder_context,
                                    void * param)
{
    gen8_avc_frame_brc_update_curbe_data *cmd;
    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct generic_enc_codec_state * generic_state = (struct generic_enc_codec_state *)vme_context->generic_enc_state;
    struct avc_enc_state * avc_state = (struct avc_enc_state *)vme_context->private_enc_state;
    struct object_surface *obj_surface;
    struct gen9_surface_avc *avc_priv_surface;
    struct avc_param common_param;
    VAEncSequenceParameterBufferH264 * seq_param = avc_state->seq_param;

    obj_surface = encode_state->reconstructed_object;

    if (!obj_surface || !obj_surface->private_data)
        return;
    avc_priv_surface = obj_surface->private_data;

    cmd = i965_gpe_context_map_curbe(gpe_context);

    if (!cmd)
        return;

    memcpy(cmd, &gen8_avc_frame_brc_update_curbe_init_data, sizeof(gen8_avc_frame_brc_update_curbe_data));

    cmd->dw5.target_size_flag = 0 ;
    if (generic_state->brc_init_current_target_buf_full_in_bits > (double)generic_state->brc_init_reset_buf_size_in_bits) {
        /*overflow*/
        generic_state->brc_init_current_target_buf_full_in_bits -= (double)generic_state->brc_init_reset_buf_size_in_bits;
        cmd->dw5.target_size_flag = 1 ;
    }

    if (generic_state->skip_frame_enbale) {
        cmd->dw6.num_skip_frames = generic_state->num_skip_frames ;
        cmd->dw7.size_skip_frames = generic_state->size_skip_frames;

        generic_state->brc_init_current_target_buf_full_in_bits += generic_state->brc_init_reset_input_bits_per_frame * generic_state->num_skip_frames;

    }
    cmd->dw0.target_size = (unsigned int)generic_state->brc_init_current_target_buf_full_in_bits ;
    cmd->dw1.frame_number = generic_state->seq_frame_number ;
    cmd->dw2.size_of_pic_headers = generic_state->herder_bytes_inserted << 3 ;
    cmd->dw5.cur_frame_type = generic_state->frame_type ;
    cmd->dw5.brc_flag = 0 ;
    cmd->dw5.brc_flag |= (avc_priv_surface->is_as_ref) ? INTEL_ENCODE_BRCUPDATE_IS_REFERENCE : 0 ;

    if (avc_state->multi_pre_enable) {
        cmd->dw5.brc_flag  |= INTEL_ENCODE_BRCUPDATE_IS_ACTUALQP ;
        cmd->dw14.qp_index_of_cur_pic = avc_priv_surface->frame_idx ; //do not know this. use -1
    }

    cmd->dw5.max_num_paks = generic_state->num_pak_passes ;
    if (avc_state->min_max_qp_enable) {
        switch (generic_state->frame_type) {
        case SLICE_TYPE_I:
            cmd->dw6.minimum_qp = avc_state->min_qp_i ;
            cmd->dw6.maximum_qp = avc_state->max_qp_i ;
            break;
        case SLICE_TYPE_P:
            cmd->dw6.minimum_qp = avc_state->min_qp_p ;
            cmd->dw6.maximum_qp = avc_state->max_qp_p ;
            break;
        case SLICE_TYPE_B:
            cmd->dw6.minimum_qp = avc_state->min_qp_b ;
            cmd->dw6.maximum_qp = avc_state->max_qp_b ;
            break;
        }
    } else {
        cmd->dw6.minimum_qp = 0 ;
        cmd->dw6.maximum_qp = 0 ;
    }
    cmd->dw6.enable_force_skip = avc_state->enable_force_skip ;
    cmd->dw6.enable_sliding_window = 0 ;

    generic_state->brc_init_current_target_buf_full_in_bits += generic_state->brc_init_reset_input_bits_per_frame;

    if (generic_state->internal_rate_mode == INTEL_BRC_AVBR) {
        cmd->dw3.start_gadj_frame0 = (unsigned int)((10 *   generic_state->avbr_convergence) / (double)150);
        cmd->dw3.start_gadj_frame1 = (unsigned int)((50 *   generic_state->avbr_convergence) / (double)150);
        cmd->dw4.start_gadj_frame2 = (unsigned int)((100 *  generic_state->avbr_convergence) / (double)150);
        cmd->dw4.start_gadj_frame3 = (unsigned int)((150 *  generic_state->avbr_convergence) / (double)150);
        cmd->dw11.g_rate_ratio_threshold_0 = (unsigned int)((100 - (generic_state->avbr_curracy / (double)30) * (100 - 40)));
        cmd->dw11.g_rate_ratio_threshold_1 = (unsigned int)((100 - (generic_state->avbr_curracy / (double)30) * (100 - 75)));
        cmd->dw12.g_rate_ratio_threshold_2 = (unsigned int)((100 - (generic_state->avbr_curracy / (double)30) * (100 - 97)));
        cmd->dw12.g_rate_ratio_threshold_3 = (unsigned int)((100 + (generic_state->avbr_curracy / (double)30) * (103 - 100)));
        cmd->dw12.g_rate_ratio_threshold_4 = (unsigned int)((100 + (generic_state->avbr_curracy / (double)30) * (125 - 100)));
        cmd->dw12.g_rate_ratio_threshold_5 = (unsigned int)((100 + (generic_state->avbr_curracy / (double)30) * (160 - 100)));

    }
    //cmd->dw15.enable_roi = generic_state->brc_roi_enable ;

    memset(&common_param, 0, sizeof(common_param));
    common_param.frame_width_in_pixel = generic_state->frame_width_in_pixel;
    common_param.frame_height_in_pixel = generic_state->frame_height_in_pixel;
    common_param.frame_width_in_mbs = generic_state->frame_width_in_mbs;
    common_param.frame_height_in_mbs = generic_state->frame_height_in_mbs;
    common_param.frames_per_100s = generic_state->frames_per_100s;
    common_param.vbv_buffer_size_in_bit = generic_state->vbv_buffer_size_in_bit;
    common_param.target_bit_rate = generic_state->target_bit_rate;

    //cmd->dw19.user_max_frame = i965_avc_get_profile_level_max_frame(&common_param, seq_param->level_idc);
    i965_gpe_context_unmap_curbe(gpe_context);

    return;
}

static void
gen8_avc_kernel_init_brc(VADriverContextP ctx,
                         struct generic_encoder_context *generic_context,
                         struct gen_avc_brc_context *kernel_context)
{
    struct i965_driver_data *i965 = i965_driver_data(ctx);
    struct i965_gpe_table *gpe = &i965->gpe_table;
    struct i965_gpe_context *gpe_context = NULL;
    struct encoder_kernel_parameter kernel_param ;
    struct encoder_scoreboard_parameter scoreboard_param;
    struct i965_kernel common_kernel;
    int i = 0;

    const int brc_curbe_size[NUM_GEN9_AVC_KERNEL_BRC - 1] = {
        (sizeof(gen9_avc_brc_init_reset_curbe_data)),
        (sizeof(gen8_avc_frame_brc_update_curbe_data)),
        (sizeof(gen9_avc_brc_init_reset_curbe_data)),
        (sizeof(gen8_avc_mbenc_curbe_data)),
        0,
    };

    kernel_param.inline_data_size = 0;
    kernel_param.sampler_size = 0;

    memset(&scoreboard_param, 0, sizeof(scoreboard_param));
    scoreboard_param.mask = 0xFF;
    scoreboard_param.enable = generic_context->use_hw_scoreboard;
    scoreboard_param.type = generic_context->use_hw_non_stalling_scoreboard;
    scoreboard_param.walkpat_flag = 0;

    for (i = 0; i < NUM_GEN9_AVC_KERNEL_BRC - 1; i++) {
        kernel_param.curbe_size = brc_curbe_size[i];
        gpe_context = &kernel_context->gpe_contexts[i];
        gen9_init_gpe_context_avc(ctx, gpe_context, &kernel_param);
        gen9_init_vfe_scoreboard_avc(gpe_context, &scoreboard_param);

        memset(&common_kernel, 0, sizeof(common_kernel));

        intel_avc_get_kernel_header_and_size((void *)(generic_context->enc_kernel_ptr),
                                             generic_context->enc_kernel_size,
                                             INTEL_GENERIC_ENC_BRC,
                                             i,
                                             &common_kernel);

        gpe->load_kernels(ctx,
                          gpe_context,
                          &common_kernel,
                          1);
    }

}

static void
gen8_avc_kernel_init(VADriverContextP ctx,
                     struct intel_encoder_context *encoder_context)
{
    struct i965_driver_data *i965 = i965_driver_data(ctx);
    struct encoder_vme_mfc_context * vme_context = (struct encoder_vme_mfc_context *)encoder_context->vme_context;
    struct i965_avc_encoder_context * avc_ctx = (struct i965_avc_encoder_context *)vme_context->private_enc_ctx;
    struct generic_encoder_context * generic_ctx = (struct generic_encoder_context *)vme_context->generic_enc_ctx;

    gen9_avc_kernel_init_scaling(ctx, generic_ctx, &avc_ctx->context_scaling);
    gen8_avc_kernel_init_brc(ctx, generic_ctx, &avc_ctx->context_brc);
    gen9_avc_kernel_init_me(ctx, generic_ctx, &avc_ctx->context_me);
    gen9_avc_kernel_init_mbenc(ctx, generic_ctx, &avc_ctx->context_mbenc);
    gen9_avc_kernel_init_wp(ctx, generic_ctx, &avc_ctx->context_wp);
    gen9_avc_kernel_init_sfd(ctx, generic_ctx, &avc_ctx->context_sfd);

    //function pointer
    generic_ctx->pfn_set_curbe_scaling2x = gen9_avc_set_curbe_scaling2x;
    generic_ctx->pfn_set_curbe_scaling4x = gen8_avc_set_curbe_scaling4x;
    generic_ctx->pfn_set_curbe_me = gen8_avc_set_curbe_me;
    generic_ctx->pfn_set_curbe_mbenc = gen8_avc_set_curbe_mbenc;
    generic_ctx->pfn_set_curbe_brc_init_reset = gen9_avc_set_curbe_brc_init_reset;
    generic_ctx->pfn_set_curbe_brc_frame_update = gen8_avc_set_curbe_brc_frame_update;
    //generic_ctx->pfn_set_curbe_brc_mb_update = gen9_avc_set_curbe_brc_mb_update;
    generic_ctx->pfn_set_curbe_sfd = gen9_avc_set_curbe_sfd;
    //generic_ctx->pfn_set_curbe_wp = gen9_avc_set_curbe_wp;

    generic_ctx->pfn_send_scaling_surface = gen8_avc_send_surface_scaling;
    generic_ctx->pfn_send_me_surface = gen8_avc_send_surface_me;
    generic_ctx->pfn_send_mbenc_surface = gen8_avc_send_surface_mbenc;
    generic_ctx->pfn_send_brc_init_reset_surface = gen9_avc_send_surface_brc_init_reset;
    generic_ctx->pfn_send_brc_frame_update_surface = gen9_avc_send_surface_brc_frame_update;
    //generic_ctx->pfn_send_brc_mb_update_surface = gen9_avc_send_surface_brc_mb_update;
    generic_ctx->pfn_send_sfd_surface = gen9_avc_send_surface_sfd;
    //generic_ctx->pfn_send_wp_surface = gen9_avc_send_surface_wp;

    /*if (IS_SKL(i965->intel.device_info) ||
        IS_BXT(i965->intel.device_info))
        generic_ctx->pfn_set_curbe_scaling4x = gen9_avc_set_curbe_scaling4x;
    else if (IS_KBL(i965->intel.device_info) ||
             IS_GLK(i965->intel.device_info))
        generic_ctx->pfn_set_curbe_scaling4x = gen95_avc_set_curbe_scaling4x;
     */
}
