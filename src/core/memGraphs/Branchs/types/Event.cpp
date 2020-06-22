#include "mementar/core/memGraphs/Branchs/types/Event.h"

namespace mementar {

std::regex Event::regex("\\[(\\d*)(,(\\d*))?\\]\\{([^\\}]*)\\}");
std::smatch Event::match;

} // namespace mementar
