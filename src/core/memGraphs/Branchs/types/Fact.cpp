#include "mementar/core/memGraphs/Branchs/types/Fact.h"

namespace mementar {

std::regex Fact::regex("(\\w)\\|(\\w+)\\|(\\w+)\\|(\\w+)");
std::smatch Fact::match;

} // namespace mementar
