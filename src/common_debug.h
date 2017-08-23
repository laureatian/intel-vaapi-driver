extern void debug_map_surface(dri_bo *bo, char* file_name, int debug_log_enable);
extern void debug_dumpobjsurface(VADriverContextP ctx, struct object_surface * debug_surface, char * prefix, char * name, int debug_log_enable, int num);
extern void debug_dump_gpe(VADriverContextP ctx, struct i965_gpe_resource *gpe_resource, char * prefix, char * name, int debug_log_enable, int num);
extern void debug_dump_bo(VADriverContextP ctx, dri_bo *bo, char * prefix, char * name, int debug_log_enable, int num);
extern void debug_dump_curbe(VADriverContextP ctx, struct i965_gpe_context *gpe_context, char * prefix, char * name, int debug_log_enable, int num, int size);
extern void debug_dump_avc_mbcode_mv(VADriverContextP ctx, struct object_surface * debug_surface, char * prefix, char * name, int debug_log_enable, int num);
