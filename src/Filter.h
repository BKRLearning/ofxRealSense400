#pragma once

#include <string>
#include <atomic>
#include "ofMain.h"
#include "rs.hpp"

class Filter {
 public:
        Filter(const string name, rs2::filter& filter);
        Filter(Filter&& other);
        string name;
        rs2::filter& filterBlock;
        std::atomic_bool is_enabled;
};
