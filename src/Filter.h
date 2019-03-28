#pragma once

#include <string>
#include <atomic>
#include "ofMain.h"
#include "rs.hpp"

class Filter {
 public:
        Filter(const string name, rs2::processing_block& filter);
        Filter(Filter&& other);
        string name;
        rs2::processing_block& filterBlock;
        std::atomic_bool is_enabled;
};
