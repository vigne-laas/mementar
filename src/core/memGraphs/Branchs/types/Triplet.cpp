#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

std::regex Triplet::regex("(\\w)\\|(\\w+)\\|(\\w+)\\|(\\w+)");
std::regex Triplet::regex2("\\[([^\\]]+)\\]([^|]+)\\|([^|]+)\\|([^|]+)");
std::smatch Triplet::match;

} // namespace mementar
