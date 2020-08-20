#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

std::regex Triplet::regex("(\\w)\\|(\\w+)\\|(\\w+)\\|(\\w+)");
std::smatch Triplet::match;

} // namespace mementar
