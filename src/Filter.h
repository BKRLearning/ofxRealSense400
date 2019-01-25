#pragma once

#include "ofMain.h"
#include "rs.hpp"

class Filter {
    public:
        Filter(rs2::processing_block& filter);

        rs2::processing_block& filterBlock;
        bool is_enabled;
};
