#pragma once

#include <vector>
#include <path_planning/utils/graph_utils.h>

/**
 * Breadth-First Search (BFS)
 * Performs BFS on the given GridGraph from start to goal.
 * Returns a vector of Cells representing the path.
 */
std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal);
