#include "Filter.h"

Filter::Filter(const string name, rs2::filter& filter) :
    name(name),
    filterBlock(filter),
    is_enabled(false) {
//
}

Filter::Filter(Filter&& other) :
    name(std::move(other.name)),
    filterBlock(other.filterBlock),
    is_enabled(other.is_enabled.load()) {
//
}
