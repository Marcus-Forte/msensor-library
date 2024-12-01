#pragma once

#include <cstdint>

namespace timing {
    /**
     * @brief Get current time in microsseconds. Uses system clock.
     * 
     * @return uint64_t time in microsseconds.
     */
    uint64_t getNowUs();
}