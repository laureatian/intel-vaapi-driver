#include <assert.h>

#include "intel_batchbuffer.h"
#include "intel_driver.h"

#include "i965_defines.h"
#include "i965_structs.h"
#include "i965_drv_video.h"
#include "i965_encoder.h"
#include "i965_encoder_utils.h"
#include "intel_media.h"

#include "i965_gpe_utils.h"
#include "i965_avc_encoder_common.h"
#include "gen9_avc_encoder.h"

#include "common_debug.h"

#define DEBUG_FRAME_NUM_MAX 1000

struct gen9_surface_avc;
static char * path = "/home/tiantian/doc/vaapi_log/";
//static char * path="./debug_log/";

void debug_map_surface(dri_bo *bo, char* file_name, int debug_log_enable)
{
    int i;
    if (!debug_log_enable) return;
    unsigned int tiling, swizzle;
    if (bo != NULL) {
        unsigned int bo_size = bo->size;

        dri_bo_get_tiling(bo, &tiling, &swizzle);

        if (tiling != I915_TILING_NONE)
            drm_intel_gem_bo_map_gtt(bo);
        else
            dri_bo_map(bo, 1);

        //  FILE* fp_image = fopen(file_name, "wb+");
        unsigned int *start = (unsigned int*)bo->virtual;
        FILE* fp_image = fopen(file_name, "w");
//        fwrite(bo->virtual, 1, bo_size, fp_image);
        if (bo_size % 4 != 0) {
            printf("This buffer is not 4 aligned %s\n", file_name);
        }
        for (i = 0; i < bo_size / 4; i++) {
            fprintf(fp_image, "%08x ", start[i]);
            if (i % 4 == 3)
                fprintf(fp_image, " \n");
        }

        fclose(fp_image);
        dri_bo_unmap(bo);
    }
}

void debug_map_yuv_surface(dri_bo *bo, char* file_name, int debug_log_enable, int width, int height, int stride_w, int stride_h)
{

    if (!debug_log_enable) return;
    unsigned int tiling, swizzle;
    unsigned char * pdata = NULL;
    int i = 0;
    if (bo != NULL) {
        dri_bo_get_tiling(bo, &tiling, &swizzle);

        if (tiling != I915_TILING_NONE)
            drm_intel_gem_bo_map_gtt(bo);
        else
            dri_bo_map(bo, 1);

        FILE* fp_image = fopen(file_name, "wb+");
        pdata = bo->virtual;
        for (i = 0; i < height; i++) {
            fwrite(pdata, 1, width, fp_image);
            pdata += stride_w;

        }
        pdata = bo->virtual + stride_w * stride_h;
        for (i = 0; i < height / 2; i++) {
            fwrite(pdata, 1, width, fp_image);
            pdata += stride_w;

        }
        fclose(fp_image);
        dri_bo_unmap(bo);
    }
}
void debug_dumpobjsurface(VADriverContextP ctx, struct object_surface * debug_surface, char * prefix, char * name, int debug_log_enable, int num)
{
    if (!debug_log_enable) return;
    struct object_surface *obj_surface;

    char fname[100];

    if (num > DEBUG_FRAME_NUM_MAX) return;


    obj_surface = debug_surface;
    assert(obj_surface && obj_surface->bo);
    sprintf(fname, "%s%03d_%s_%s_%dx%d_%dx%d.yuv", path, num, prefix, name, obj_surface->width, obj_surface->height, obj_surface->orig_width, obj_surface->orig_height);
    debug_map_yuv_surface(obj_surface->bo, fname, debug_log_enable, obj_surface->orig_width, obj_surface->orig_height, obj_surface->width, obj_surface->height);

}

void debug_dump_gpe(VADriverContextP ctx, struct i965_gpe_resource *gpe_resource, char * prefix, char * name, int debug_log_enable, int num)
{
    if (!debug_log_enable) return;

    char fname[100];

    if (num > DEBUG_FRAME_NUM_MAX) return;

    sprintf(fname, "%s%03d_gpe_%s_%s.bin", path, num, prefix, name);

    assert(gpe_resource && gpe_resource->bo);
    debug_map_surface(gpe_resource->bo, fname, debug_log_enable);

}

void debug_dump_bo(VADriverContextP ctx, dri_bo *bo, char * prefix, char * name, int debug_log_enable, int num)
{
    if (!debug_log_enable) return;

    char fname[100];

    if (num > DEBUG_FRAME_NUM_MAX) return;

    sprintf(fname, "%s%03d_bo_%s_%s.bin", path, num, prefix, name);

    assert(bo);
    debug_map_surface(bo, fname, debug_log_enable);

}

void debug_dump_curbe(VADriverContextP ctx, struct i965_gpe_context *gpe_context, char * prefix, char * name, int debug_log_enable, int num, int size)
{
    if (!debug_log_enable) return;

    char fname[100];
    unsigned char * pdata = NULL;

    if (num > DEBUG_FRAME_NUM_MAX) return;

    sprintf(fname, "%s%03d_curbe_%s_%s.bin", path, num, prefix, name);
    if (gpe_context == NULL)return;
    pdata = (unsigned char *)i965_gpe_context_map_curbe(gpe_context);
    if (pdata == NULL || size == 0)
        return;

    FILE* fp_image = fopen(fname, "wb+");
    fwrite(pdata, 1, size, fp_image);
    fclose(fp_image);
    i965_gpe_context_unmap_curbe(gpe_context);
}

void debug_dump_avc_mbcode_mv(VADriverContextP ctx, struct object_surface * debug_surface, char * prefix, char * name, int debug_log_enable, int num)
{
    if (!debug_log_enable) return;
    struct object_surface *obj_surface;
    struct gen9_surface_avc *avc_priv_surface;

    char fname[100];

    if (num > DEBUG_FRAME_NUM_MAX) return;

    sprintf(fname, "%s%03d_%s_%s.bin", path, num, prefix, name);

    obj_surface = debug_surface;
    if (!obj_surface || !obj_surface->private_data)
        return;
    avc_priv_surface = obj_surface->private_data;
    debug_map_surface(avc_priv_surface->res_mb_code_surface.bo, fname, debug_log_enable);
    sprintf(fname, "%s%03d_%s_%s.bin", path, num, prefix, "mv_data");
    debug_map_surface(avc_priv_surface->res_mv_data_surface.bo, fname, debug_log_enable);

}

