#ifndef AURALITE_H
#define AURALITE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*AuraliteLogCallback)(uint32_t level, const char* msg);
typedef void (*AuraliteDebugDrawLineCallback)(float x1, float y1, float z1, float x2, float y2, float z2, uint32_t color_rgb);

uint32_t auralite_api_version(void);
uint32_t auralite_abi_version(void);
const char* auralite_last_error(void);
int32_t auralite_set_log_callback(AuraliteLogCallback cb);
int32_t auralite_set_debug_draw_line_callback(AuraliteDebugDrawLineCallback cb);
int32_t auralite_world2_create(uint64_t* out);
int32_t auralite_world3_create(uint64_t* out);
int32_t auralite_world2_step(uint64_t token, float dt);
int32_t auralite_world3_step(uint64_t token, float dt);
int32_t auralite_world2_destroy(uint64_t token);
int32_t auralite_world3_destroy(uint64_t token);
uint32_t auralite_world_count(void);
int32_t auralite_world2_add_body(uint64_t token, uint8_t kind, float px, float py, float vx, float vy, float mass, uint64_t* out_body_id);
int32_t auralite_world3_add_body(uint64_t token, uint8_t kind, float px, float py, float pz, float vx, float vy, float vz, float mass, uint64_t* out_body_id);
int32_t auralite_world2_body_query(uint64_t token, uint64_t body_id, float* out_px, float* out_py, float* out_vx, float* out_vy, uint8_t* out_sleeping);
int32_t auralite_world3_body_query(uint64_t token, uint64_t body_id, float* out_px, float* out_py, float* out_pz, float* out_vx, float* out_vy, float* out_vz, uint8_t* out_sleeping);
int32_t auralite_world2_body_apply_impulse(uint64_t token, uint64_t body_id, float ix, float iy);
int32_t auralite_world3_body_apply_impulse(uint64_t token, uint64_t body_id, float ix, float iy, float iz);
int32_t auralite_world3_batch_query_positions(uint64_t token, const uint64_t* body_ids, uint32_t count, float* out_positions);
#ifdef __cplusplus
}
#endif
#endif /* AURALITE_H */
