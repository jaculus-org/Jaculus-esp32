#include "hub75Feature.h"

bool operator==(const Color &lhs, const Color &rhs) {
    return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a;
}

bool operator!=(const Color &lhs, const Color &rhs) { return !(lhs == rhs); }
