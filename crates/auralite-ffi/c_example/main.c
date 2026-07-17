#include <stdio.h>
#include <assert.h>
#include "../auralite.h"

int main(void) {
    printf("AuraLite C FFI verification example starting...\n");
    uint32_t api = auralite_api_version();
    assert((api >> 16) == 1);
    
    uint64_t wtoken = 0;
    int32_t res = auralite_world2_create(&wtoken);
    assert(res == 0);
    assert(wtoken != 0);
    
    uint64_t body_id = 0;
    res = auralite_world2_add_body(wtoken, 2, 0.0f, 10.0f, 0.0f, -1.0f, 1.0f, &body_id);
    assert(res == 0);
    assert(body_id != 0);
    
    float px = 0.0f, py = 0.0f, vx = 0.0f, vy = 0.0f;
    uint8_t sleeping = 0;
    res = auralite_world2_body_query(wtoken, body_id, &px, &py, &vx, &vy, &sleeping);
    assert(res == 0);
    assert(px == 0.0f);
    assert(py == 10.0f);
    
    for (int i = 0; i < 60; ++i) {
        res = auralite_world2_step(wtoken, 0.016666668f);
        assert(res == 0);
    }
    
    res = auralite_world2_body_query(wtoken, body_id, &px, &py, &vx, &vy, &sleeping);
    assert(res == 0);
    assert(py < 10.0f);
    
    res = auralite_world2_destroy(wtoken);
    assert(res == 0);
    printf("AuraLite C FFI verification example completed successfully!\n");
    return 0;
}
