#ifndef AURALITE_H
#define AURALITE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t auralite_api_version(void);
int32_t auralite_world2_create(uint64_t *out);
int32_t auralite_world2_step(uint64_t token, float dt);
int32_t auralite_world2_destroy(uint64_t token);
#ifdef __cplusplus
}
#endif
#endif
