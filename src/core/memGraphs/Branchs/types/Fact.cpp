#include "mementar/core/memGraphs/Branchs/types/Fact.h"

namespace mementar {

std::regex Fact::regex("\\[(\\d*)(,(\\d*))?\\]\\{([^\\}]*)\\}");
std::smatch Fact::match;

} // namespace mementar
