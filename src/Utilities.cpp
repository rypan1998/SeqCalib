#include "Utilities.h"

uint64_t ImageIdsToPairId(uint32_t id1, uint32_t id2) {
    if(id1 > id2) {
        return 2147483647ll * id2 + id1;
    }
    return 2147483647ll * id1 + id2;
}
