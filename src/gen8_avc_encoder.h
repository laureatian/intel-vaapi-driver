typedef struct _gen8_avc_encoder_kernel_header {
    int nKernelCount;

    // Quality mode for Frame/Field
    kernel_header mbenc_quality_I;
    kernel_header mbenc_quality_P;
    kernel_header mbenc_quality_B;
    // Normal mode for Frame/Field
    kernel_header mbenc_normal_I;
    kernel_header mbenc_normal_P;
    kernel_header mbenc_normal_B;
    // Performance modes for Frame/Field
    kernel_header mbenc_performance_I;
    kernel_header mbenc_performance_P;
    kernel_header mbenc_performance_B;
    // WiDi modes for Frame/Field
    kernel_header mbenc_widi_I;
    kernel_header mbenc_widi_P;
    kernel_header mbenc_widi_B;

    // HME
    kernel_header me_p;
    kernel_header me_b;

    // DownScaling
    kernel_header ply_dscale_ply;
    kernel_header ply_dscale_2f_ply_2f;
    // BRC Init frame
    kernel_header frame_brc_init;

    // FrameBRC Update
    kernel_header frame_brc_update;

    // BRC Reset frame
    kernel_header frame_brc_reset;

    // BRC I Frame Distortion
    kernel_header frame_brc_i_dist;

    //BRC Block Copy
    kernel_header brc_block_copy;

    // 2x DownScaling
    kernel_header ply_2xdscale_ply;
    kernel_header ply_2xdscale_2f_ply_2f;

    // Static frame detection Kernel
    kernel_header static_detection;
} gen8_avc_encoder_kernel_header;


typedef struct _gen8_avc_mbenc_curbe_data {
    struct {
        uint32_t skip_mode_enable: 1;
        uint32_t adaptive_enable: 1;
        uint32_t bi_mix_dis: 1;
        uint32_t reserved0: 2;
        uint32_t early_ime_success_enable: 1;
        uint32_t reserved1: 1;
        uint32_t t8x8_flag_for_inter_enable: 1;
        uint32_t reserved2: 16;
        uint32_t early_ime_stop: 8;
    } dw0;

    struct {
        uint32_t max_num_mvs: 6;
        uint32_t reserved0: 10;
        uint32_t bi_weight: 6;
        uint32_t reserved1: 6;
        uint32_t uni_mix_disable: 1;
        uint32_t reserved2: 3;
    } dw1;

    struct {
        uint32_t max_len_sp: 8;
        uint32_t max_num_su: 8;
        uint32_t pitch_width: 16;
    } dw2;

    struct {
        uint32_t src_size: 2;
        uint32_t reserved0: 2;
        uint32_t mb_type_remap: 2;
        uint32_t src_access: 1;
        uint32_t ref_access: 1;
        uint32_t search_ctrl: 3;
        uint32_t dual_search_path_option: 1;
        uint32_t sub_pel_mode: 2;
        uint32_t skip_type: 1;
        uint32_t disable_field_cache_allocation: 1;
        uint32_t inter_chroma_mode: 1;
        uint32_t ftq_enable: 1;
        uint32_t bme_disable_fbr: 1;
        uint32_t block_based_skip_enable: 1;
        uint32_t inter_sad: 2;
        uint32_t intra_sad: 2;
        uint32_t sub_mb_part_mask: 7;
        uint32_t reserved1: 1;
    } dw3;

    struct {
        uint32_t picture_height_minus1: 16;
        uint32_t mv_restriction_in_slice_enable: 1;
        uint32_t delta_mv_enable: 1;
        uint32_t true_distortion_enable: 1;
        uint32_t enable_wavefront_optimization: 1;
        uint32_t reserved0: 1;
        uint32_t enable_intra_cost_scaling_for_static_frame: 1;
        uint32_t enable_intra_refresh: 1;
        uint32_t enable_widi_wa_surf: 1;
        uint32_t enable_widi_dirty_rect: 1;
        uint32_t enable_cur_fld_idr: 1;
        uint32_t contrained_intra_pred_flag: 1;
        uint32_t field_parity_flag: 1;
        uint32_t hme_enable: 1;
        uint32_t picture_type: 2;
        uint32_t use_actual_ref_qp_value: 1;
    } dw4;

    struct {
        uint32_t slice_mb_height: 16;
        uint32_t ref_width: 8;
        uint32_t ref_height: 8;
    } dw5;

    struct {
        uint32_t batch_buffer_end;
    } dw6;

    struct {
        uint32_t intra_part_mask: 5;
        uint32_t non_skip_zmv_added: 1;
        uint32_t non_skip_mode_added: 1;
        uint32_t luma_intra_src_corner_swap: 1;
        uint32_t reserved0: 8;
        uint32_t mv_cost_scale_factor: 2;
        uint32_t bilinear_enable: 1;
        uint32_t src_field_polarity: 1;
        uint32_t weightedsad_harr: 1;
        uint32_t ac_only_haar: 1;
        uint32_t ref_id_cost_mode: 1;
        uint32_t reserved1: 1;
        uint32_t skip_center_mask: 8;
    } dw7;

    struct {
        uint32_t mode_0_cost: 8;
        uint32_t mode_1_cost: 8;
        uint32_t mode_2_cost: 8;
        uint32_t mode_3_cost: 8;
    } dw8;

    struct {
        uint32_t mode_4_cost: 8;
        uint32_t mode_5_cost: 8;
        uint32_t mode_6_cost: 8;
        uint32_t mode_7_cost: 8;
    } dw9;

    struct {
        uint32_t mode_8_cost: 8;
        uint32_t mode_9_cost: 8;
        uint32_t ref_id_cost: 8;
        uint32_t chroma_intra_mode_cost: 8;
    } dw10;

    union {
        struct {
            uint32_t mv_0_cost: 8;
            uint32_t mv_1_cost: 8;
            uint32_t mv_2_cost: 8;
            uint32_t mv_3_cost: 8;
        };
        uint32_t value;
    } dw11;

    struct {
        uint32_t mv_4_cost: 8;
        uint32_t mv_5_cost: 8;
        uint32_t mv_6_cost: 8;
        uint32_t mv_7_cost: 8;
    } dw12;

    struct {
        uint32_t qp_prime_y: 8;
        uint32_t qp_prime_cb: 8;
        uint32_t qp_prime_cr: 8;
        uint32_t target_size_in_word: 8;
    } dw13;

    struct {
        uint32_t sic_fwd_transcoeff_threshold_0: 16;
        uint32_t sic_fwd_transcoeff_threshold_1: 8;
        uint32_t sic_fwd_transcoeff_threshold_2: 8;
    } dw14;

    struct {
        uint32_t sic_fwd_transcoeff_threshold_3: 8;
        uint32_t sic_fwd_transcoeff_threshold_4: 8;
        uint32_t sic_fwd_transcoeff_threshold_5: 8;
        uint32_t sic_fwd_transcoeff_threshold_6: 8;
    } dw15;

    struct {
        struct generic_search_path_delta sp_delta_0;
        struct generic_search_path_delta sp_delta_1;
        struct generic_search_path_delta sp_delta_2;
        struct generic_search_path_delta sp_delta_3;
    } dw16;

    struct {
        struct generic_search_path_delta sp_delta_4;
        struct generic_search_path_delta sp_delta_5;
        struct generic_search_path_delta sp_delta_6;
        struct generic_search_path_delta sp_delta_7;
    } dw17;

    struct {
        struct generic_search_path_delta sp_delta_8;
        struct generic_search_path_delta sp_delta_9;
        struct generic_search_path_delta sp_delta_10;
        struct generic_search_path_delta sp_delta_11;
    } dw18;

    struct {
        struct generic_search_path_delta sp_delta_12;
        struct generic_search_path_delta sp_delta_13;
        struct generic_search_path_delta sp_delta_14;
        struct generic_search_path_delta sp_delta_15;
    } dw19;

    struct {
        struct generic_search_path_delta sp_delta_16;
        struct generic_search_path_delta sp_delta_17;
        struct generic_search_path_delta sp_delta_18;
        struct generic_search_path_delta sp_delta_19;
    } dw20;

    struct {
        struct generic_search_path_delta sp_delta_20;
        struct generic_search_path_delta sp_delta_21;
        struct generic_search_path_delta sp_delta_22;
        struct generic_search_path_delta sp_delta_23;
    } dw21;

    struct {
        struct generic_search_path_delta sp_delta_24;
        struct generic_search_path_delta sp_delta_25;
        struct generic_search_path_delta sp_delta_26;
        struct generic_search_path_delta sp_delta_27;
    } dw22;

    struct {
        struct generic_search_path_delta sp_delta_28;
        struct generic_search_path_delta sp_delta_29;
        struct generic_search_path_delta sp_delta_30;
        struct generic_search_path_delta sp_delta_31;
    } dw23;

    struct {
        struct generic_search_path_delta sp_delta_32;
        struct generic_search_path_delta sp_delta_33;
        struct generic_search_path_delta sp_delta_34;
        struct generic_search_path_delta sp_delta_35;
    } dw24;

    struct {
        struct generic_search_path_delta sp_delta_36;
        struct generic_search_path_delta sp_delta_37;
        struct generic_search_path_delta sp_delta_38;
        struct generic_search_path_delta sp_delta_39;
    } dw25;

    struct {
        struct generic_search_path_delta sp_delta_40;
        struct generic_search_path_delta sp_delta_41;
        struct generic_search_path_delta sp_delta_42;
        struct generic_search_path_delta sp_delta_43;
    } dw26;

    struct {
        struct generic_search_path_delta sp_delta_44;
        struct generic_search_path_delta sp_delta_45;
        struct generic_search_path_delta sp_delta_46;
        struct generic_search_path_delta sp_delta_47;
    } dw27;

    struct {
        struct generic_search_path_delta sp_delta_48;
        struct generic_search_path_delta sp_delta_49;
        struct generic_search_path_delta sp_delta_50;
        struct generic_search_path_delta sp_delta_51;
    } dw28;

    struct {
        struct generic_search_path_delta sp_delta_52;
        struct generic_search_path_delta sp_delta_53;
        struct generic_search_path_delta sp_delta_54;
        struct generic_search_path_delta sp_delta_55;
    } dw29;

    struct {
        uint32_t intra_4x4_mode_mask: 9;
        uint32_t reserved0: 7;
        uint32_t intra_8x8_mode_mask: 9;
        uint32_t reserved1: 7;
    } dw30;

    struct {
        uint32_t intra_16x16_mode_mask: 4;
        uint32_t intra_chroma_mode_mask: 4;
        uint32_t intra_compute_type: 2;
        uint32_t reserved0: 22;
    } dw31;

    struct {
        uint32_t skip_val: 16;
        uint32_t mult_pred_l0_disable: 8;
        uint32_t mult_pred_l1_disable: 8;
    } dw32;

    struct {
        uint32_t intra_16x16_nondc_penalty: 8;
        uint32_t intra_8x8_nondc_penalty: 8;
        uint32_t intra_4x4_nondc_penalty: 8;
        uint32_t reserved0: 8;
    } dw33;

    struct {
        uint32_t list0_ref_id0_field_parity: 1;
        uint32_t list0_ref_id1_field_parity: 1;
        uint32_t list0_ref_id2_field_parity: 1;
        uint32_t list0_ref_id3_field_parity: 1;
        uint32_t list0_ref_id4_field_parity: 1;
        uint32_t list0_ref_id5_field_parity: 1;
        uint32_t list0_ref_id6_field_parity: 1;
        uint32_t list0_ref_id7_field_parity: 1;
        uint32_t list1_ref_id0_frm_field_parity: 1;
        uint32_t list1_ref_id1_frm_field_parity: 1;
        uint32_t widi_intra_refresh_en: 2;
        uint32_t arbitray_num_mbs_per_slice: 1;
        uint32_t force_non_skip_check: 1;
        uint32_t disable_enc_skip_check: 1;
        uint32_t enable_direct_bias_adjustment: 1;
        uint32_t enable_global_motion_bias_adjustmnent: 1;
        uint32_t b_force_to_skip: 1;
        uint32_t reserved0: 6;
        uint32_t list1_ref_id0_field_parity: 1;
        uint32_t list1_ref_id1_field_parity: 1;
        uint32_t mad_enable_falg: 1;
        uint32_t roi_enable_flag: 1;
        uint32_t enable_mb_flatness_check_optimization: 1;
        uint32_t b_direct_mode: 1;
        uint32_t mb_brc_enable: 1;
        uint32_t b_original_bff: 1;
    } dw34;

    struct {
        uint32_t panic_mode_mb_threshold: 16;
        uint32_t small_mb_size_in_word: 8;
        uint32_t large_mb_size_in_word: 8;
    } dw35;

    struct {
        uint32_t num_ref_idx_l0_minus_one: 8;
        uint32_t hme_combined_extra_sus: 8;
        uint32_t num_ref_idx_l1_minus_one: 8;
        uint32_t enablE_cabac_work_around: 1;
        uint32_t reserved0: 4;
        uint32_t is_fwd_frame_short_term_ref: 1;
        uint32_t check_all_fractional_enable: 1;
        uint32_t hme_combine_overlap: 2;
    } dw36;

    struct {
        uint32_t skip_mode_enable: 1;
        uint32_t adaptive_enable: 1;
        uint32_t bi_mix_dis: 1;
        uint32_t reserved0: 2;
        uint32_t early_ime_success_enable: 1;
        uint32_t reserved1: 1;
        uint32_t t8x8_flag_for_inter_enable: 1;
        uint32_t reserved2: 16;
        uint32_t early_ime_stop: 8;
    } dw37;

    /* reserved */
    struct {
        uint32_t max_len_sp: 8;
        uint32_t max_num_su: 8;
        uint32_t ref_threshold: 16;
    } dw38;

    struct {
        uint32_t reserved0: 8;
        uint32_t hme_ref_windows_comb_threshold: 8;
        uint32_t ref_width: 8;
        uint32_t ref_height: 8;
    } dw39;

    struct {
        uint32_t dist_scale_factor_ref_id0_list0: 16;
        uint32_t dist_scale_factor_ref_id1_list0: 16;
    } dw40;

    struct {
        uint32_t dist_scale_factor_ref_id2_list0: 16;
        uint32_t dist_scale_factor_ref_id3_list0: 16;
    } dw41;

    struct {
        uint32_t dist_scale_factor_ref_id4_list0: 16;
        uint32_t dist_scale_factor_ref_id5_list0: 16;
    } dw42;

    struct {
        uint32_t dist_scale_factor_ref_id6_list0: 16;
        uint32_t dist_scale_factor_ref_id7_list0: 16;
    } dw43;

    struct {
        uint32_t actual_qp_value_for_ref_id0_list0: 8;
        uint32_t actual_qp_value_for_ref_id1_list0: 8;
        uint32_t actual_qp_value_for_ref_id2_list0: 8;
        uint32_t actual_qp_value_for_ref_id3_list0: 8;
    } dw44;

    struct {
        uint32_t actual_qp_value_for_ref_id4_list0: 8;
        uint32_t actual_qp_value_for_ref_id5_list0: 8;
        uint32_t actual_qp_value_for_ref_id6_list0: 8;
        uint32_t actual_qp_value_for_ref_id7_list0: 8;
    } dw45;

    struct {
        uint32_t actual_qp_value_for_ref_id0_list1: 8;
        uint32_t actual_qp_value_for_ref_id1_list1: 8;
        uint32_t ref_cost: 16;
    } dw46;

    struct {
        uint32_t mb_qp_read_factor: 8;
        uint32_t intra_cost_sf: 8;
        uint32_t max_vmv_r: 16;
    } dw47;

    struct {
        uint32_t widi_intra_refresh_mbx: 16;
        uint32_t widi_intra_refresh_unit_in_mb_minus1: 8;
        uint32_t widi_intra_refresh_qp_delta: 8;
    } dw48;

    struct {
        uint32_t roi_1_x_left: 16;
        uint32_t roi_1_y_top: 16;
    } dw49;

    struct {
        uint32_t roi_1_x_right: 16;
        uint32_t roi_1_y_bottom: 16;
    } dw50;

    struct {
        uint32_t roi_2_x_left: 16;
        uint32_t roi_2_y_top: 16;
    } dw51;

    struct {
        uint32_t roi_2_x_right: 16;
        uint32_t roi_2_y_bottom: 16;
    } dw52;

    struct {
        uint32_t roi_3_x_left: 16;
        uint32_t roi_3_y_top: 16;
    } dw53;

    struct {
        uint32_t roi_3_x_right: 16;
        uint32_t roi_3_y_bottom: 16;
    } dw54;

    struct {
        uint32_t roi_4_x_left: 16;
        uint32_t roi_4_y_top: 16;
    } dw55;

    struct {
        uint32_t roi_4_x_right: 16;
        uint32_t roi_4_y_bottom: 16;
    } dw56;

    struct {
        uint32_t roi_1_dqp_prime_y: 8;
        uint32_t roi_2_dqp_prime_y: 8;
        uint32_t roi_3_dqp_prime_y: 8;
        uint32_t roi_4_dqp_prime_y: 8;
    } dw57;

    struct {
        uint32_t hme_mv_cost_scaling_factor: 8;
        int32_t reserved0: 8;
        int32_t wide_intra_refresh_mby: 16;
    } dw58;

    struct {
        uint32_t reserved;
    } dw59;

    struct {
        uint32_t cabac_wa_zone0_threshold: 16;
        uint32_t cabac_wa_zone1_threshold: 16;
    } dw60;

    struct {
        uint32_t cabac_wa_zone2_threshold: 16;
        uint32_t cabac_wa_zone3_threshold: 16;
    } dw61;

    struct {
        uint32_t cabac_wa_zone0_intra_min_qp: 8;
        uint32_t cabac_wa_zone1_intra_min_qp: 8;
        uint32_t cabac_wa_zone2_intra_min_qp: 8;
        uint32_t cabac_wa_zone3_intra_min_qp: 8;
    } dw62;

    struct {
        uint32_t reserved;
    } dw63;

    struct {
        uint32_t reserved;
    } dw64;

    struct {
        uint32_t mb_data_surf_index;
    } dw65;

    struct {
        uint32_t mv_data_surf_index;
    } dw66;

    struct {
        uint32_t i_dist_surf_index;
    } dw67;

    struct {
        uint32_t src_y_surf_index;
    } dw68;

    struct {
        uint32_t mb_specific_data_surf_index;
    } dw69;

    struct {
        uint32_t aux_vme_out_surf_index;
    } dw70;

    struct {
        uint32_t curr_ref_pic_sel_surf_index;
    } dw71;

    struct {
        uint32_t hme_mv_pred_fwd_bwd_surf_index;
    } dw72;

    struct {
        uint32_t hme_dist_surf_index;
    } dw73;

    struct {
        uint32_t slice_map_surf_index;
    } dw74;

    struct {
        uint32_t fwd_frm_mb_data_surf_index;
    } dw75;

    struct {
        uint32_t fwd_frm_mv_surf_index;
    } dw76;

    struct {
        uint32_t mb_qp_buffer;
    } dw77;

    struct {
        uint32_t mb_brc_lut;
    } dw78;

    struct {
        uint32_t vme_inter_prediction_surf_index;
    } dw79;

    struct {
        uint32_t vme_inter_prediction_mr_surf_index;
    } dw80;

    struct {
        uint32_t flatness_chk_surf_index;
    } dw81;

    struct {
        uint32_t mad_surf_index;
    } dw82;

    struct {
        uint32_t force_non_skip_mb_map_surface;
    } dw83;

    struct {
        uint32_t widi_wa_surf_index;
    } dw84;

    struct {
        uint32_t brc_curbe_surf_index;
    } dw85;

    struct {
        uint32_t static_detection_cost_table_index;
    } dw86;

    struct {
        uint32_t reserved0;
    } dw87;

} gen8_avc_mbenc_curbe_data;

typedef struct _gen8_avc_me_curbe_data {
    struct {
        uint32_t skip_mode_enable: 1;
        uint32_t adaptive_enable: 1;
        uint32_t bi_mix_dis: 1;
        uint32_t reserved0: 2;
        uint32_t early_ime_success_enable: 1;
        uint32_t reserved1: 1;
        uint32_t t8x8_flag_for_inter_enable: 1;
        uint32_t reserved2: 16;
        uint32_t early_ime_stop: 8;
    } dw0;

    struct {
        uint32_t max_num_mvs: 6;
        uint32_t reserved0: 10;
        uint32_t bi_weight: 6;
        uint32_t reserved1: 6;
        uint32_t uni_mix_disable: 1;
        uint32_t reserved2: 3;
    } dw1;

    struct {
        uint32_t len_sp: 8;
        uint32_t max_num_su: 8;
        uint32_t picture_wicth: 16;
    } dw2;

    struct {
        uint32_t src_size: 2;
        uint32_t reserved0: 2;
        uint32_t mb_type_remap: 2;
        uint32_t src_access: 1;
        uint32_t ref_access: 1;
        uint32_t search_ctrl: 3;
        uint32_t dual_search_path_option: 1;
        uint32_t sub_pel_mode: 2;
        uint32_t skip_type: 1;
        uint32_t disable_field_cache_allocation: 1;
        uint32_t inter_chroma_mode: 1;
        uint32_t ft_enable: 1;
        uint32_t bme_disable_fbr: 1;
        uint32_t block_based_skip_enable: 1;
        uint32_t inter_sad: 2;
        uint32_t intra_sad: 2;
        uint32_t sub_mb_part_mask: 7;
        uint32_t reserved1: 1;
    } dw3;

    struct {
        uint32_t picture_height_minus1: 16;
        uint32_t mv_restriction_in_slice_enbale: 1;
        uint32_t delta_mv_enable: 1;
        uint32_t true_distortin_enable: 1;
        uint32_t enable_wave_front_optimization: 1;
        uint32_t reserved0: 1;
        uint32_t enable_intra_cost_scaling_for_static_frame: 1;
        uint32_t enable_intra_refresh: 1;
        uint32_t enable_widi_wa_surf: 1;
        uint32_t enable_widi_dirty_rect: 1;
        uint32_t bcur_fld_idr: 1;
        uint32_t constrained_intra_pred_flag: 1;
        uint32_t filed_parity_flag: 1;
        uint32_t hme_enable: 1;
        uint32_t picture_type: 2;
        uint32_t use_actual_ref_qp_value: 1;
    } dw4;

    struct {
        uint32_t slice_mb_height: 16;
        uint32_t ref_width: 8;
        uint32_t ref_height: 8;
    } dw5;

    struct {
        uint32_t batch_buffer_end;
    } dw6;

    struct {
        uint32_t intra_part_mask: 5;
        uint32_t non_skip_amv_added: 1;
        uint32_t non_skip_mode_added: 1;
        uint32_t luma_intra_src_corner_swap: 1;
        uint32_t reserved0: 8;
        uint32_t mv_cost_scale_factor: 2;
        uint32_t bilinear_enable: 1;
        uint32_t src_field_polarity: 1;
        uint32_t weightedsad_harr: 1;
        uint32_t ac_only_haar: 1;
        uint32_t ref_id_cost_mode: 1;
        uint32_t reserved1: 1;
        uint32_t skip_center_mask: 8;
    } dw7;

    struct {
        uint32_t mode_0_cost: 8;
        uint32_t mode_1_cost: 8;
        uint32_t mode_2_cost: 8;
        uint32_t mode_3_cost: 8;
    } dw8;

    struct {
        uint32_t mode_4_cost: 8;
        uint32_t mode_5_cost: 8;
        uint32_t mode_6_cost: 8;
        uint32_t mode_7_cost: 8;
    } dw9;

    struct {
        uint32_t mode_8_cost: 8;
        uint32_t mode_9_cost: 8;
        uint32_t ref_id_cost: 8;
        uint32_t chroma_intra_mode_cost: 8;
    } dw10;

    struct {
        uint32_t mv_0_cost: 8;
        uint32_t mv_1_cost: 8;
        uint32_t mv_2_cost: 8;
        uint32_t mv_3_cost: 8;
    } dw11;

    struct {
        uint32_t mv_4_cost: 8;
        uint32_t mv_5_cost: 8;
        uint32_t mv_6_cost: 8;
        uint32_t mv_7_cost: 8;
    } dw12;

    struct {
        uint32_t qp_prime_y: 8;
        uint32_t qp_prime_cb: 8;
        uint32_t qp_prime_cr: 8;
        uint32_t target_size_in_word: 8;
    } dw13;

    struct {
        uint32_t sic_fwd_trans_coeff_threshold0: 16;
        uint32_t sic_fwd_trans_coeff_threshold1: 8;
        uint32_t sic_fwd_trans_coeff_threshold2: 8;
    } dw14;

    struct {
        uint32_t sic_fwd_trans_coeff_threshold3: 8;
        uint32_t sic_fwd_trans_coeff_threshold4: 8;
        uint32_t sic_fwd_trans_coeff_threshold5: 8;
        uint32_t sic_fwd_trans_coeff_threshold6: 8;
    } dw15;

    struct {
        struct generic_search_path_delta sp_delta_0;
        struct generic_search_path_delta sp_delta_1;
        struct generic_search_path_delta sp_delta_2;
        struct generic_search_path_delta sp_delta_3;
    } dw16;

    struct {
        struct generic_search_path_delta sp_delta_4;
        struct generic_search_path_delta sp_delta_5;
        struct generic_search_path_delta sp_delta_6;
        struct generic_search_path_delta sp_delta_7;
    } dw17;

    struct {
        struct generic_search_path_delta sp_delta_8;
        struct generic_search_path_delta sp_delta_9;
        struct generic_search_path_delta sp_delta_10;
        struct generic_search_path_delta sp_delta_11;
    } dw18;

    struct {
        struct generic_search_path_delta sp_delta_12;
        struct generic_search_path_delta sp_delta_13;
        struct generic_search_path_delta sp_delta_14;
        struct generic_search_path_delta sp_delta_15;
    } dw19;

    struct {
        struct generic_search_path_delta sp_delta_16;
        struct generic_search_path_delta sp_delta_17;
        struct generic_search_path_delta sp_delta_18;
        struct generic_search_path_delta sp_delta_19;
    } dw20;

    struct {
        struct generic_search_path_delta sp_delta_20;
        struct generic_search_path_delta sp_delta_21;
        struct generic_search_path_delta sp_delta_22;
        struct generic_search_path_delta sp_delta_23;
    } dw21;

    struct {
        struct generic_search_path_delta sp_delta_24;
        struct generic_search_path_delta sp_delta_25;
        struct generic_search_path_delta sp_delta_26;
        struct generic_search_path_delta sp_delta_27;
    } dw22;

    struct {
        struct generic_search_path_delta sp_delta_28;
        struct generic_search_path_delta sp_delta_29;
        struct generic_search_path_delta sp_delta_30;
        struct generic_search_path_delta sp_delta_31;
    } dw23;

    struct {
        struct generic_search_path_delta sp_delta_32;
        struct generic_search_path_delta sp_delta_33;
        struct generic_search_path_delta sp_delta_34;
        struct generic_search_path_delta sp_delta_35;
    } dw24;

    struct {
        struct generic_search_path_delta sp_delta_36;
        struct generic_search_path_delta sp_delta_37;
        struct generic_search_path_delta sp_delta_38;
        struct generic_search_path_delta sp_delta_39;
    } dw25;

    struct {
        struct generic_search_path_delta sp_delta_40;
        struct generic_search_path_delta sp_delta_41;
        struct generic_search_path_delta sp_delta_42;
        struct generic_search_path_delta sp_delta_43;
    } dw26;

    struct {
        struct generic_search_path_delta sp_delta_44;
        struct generic_search_path_delta sp_delta_45;
        struct generic_search_path_delta sp_delta_46;
        struct generic_search_path_delta sp_delta_47;
    } dw27;

    struct {
        struct generic_search_path_delta sp_delta_48;
        struct generic_search_path_delta sp_delta_49;
        struct generic_search_path_delta sp_delta_50;
        struct generic_search_path_delta sp_delta_51;
    } dw28;

    struct {
        struct generic_search_path_delta sp_delta_52;
        struct generic_search_path_delta sp_delta_53;
        struct generic_search_path_delta sp_delta_54;
        struct generic_search_path_delta sp_delta_55;
    } dw29;

    struct {
        uint32_t reserved;
    } dw30;

    struct {
        uint32_t reserved;
    } dw31;

    struct {
        uint32_t _4x_me_output_data_surf_index;
    } dw32;

    struct {
        uint32_t _16xor32xme_input_data_surf_index;
    } dw33;
    struct {
        uint32_t _4x_me_output_dist_surf_index;
    } dw34;

    struct {
        uint32_t _4x_me_output_brc_dist_surf_index;
    } dw35;

    struct {
        uint32_t vme_fwd_inter_pred_surf_index;
    } dw36;

    struct {
        uint32_t vme_bdw_inter_pred_surf_index;
    } dw37;

    /* reserved */
    struct {
        uint32_t reserved;
    } dw38;
} gen8_avc_me_curbe_data;

