#include "graph_utils.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

// ------------------- Constants -------------------
const double PI = 3.14159265358979323846;
const float ROBOT_RADIUS = 0.25f;  // example value
const float HIGH = 1e6f;           // high score placeholder

// ------------------- Helper Functions -------------------
int cellToIdx(int i, int j, const GridGraph& graph) {
    return i * graph.width + j;  // row-major
}

Cell idxToCell(int idx, const GridGraph& graph) {
    int i = idx / graph.width;
    int j = idx % graph.width;
    return {i, j};
}

Cell posToCell(float x, float y, const GridGraph& graph) {
    int i = static_cast<int>(floor((y - graph.origin_y) / graph.meters_per_cell));
    int j = static_cast<int>(floor((x - graph.origin_x) / graph.meters_per_cell));
    return {i, j};
}

vector<float> cellToPos(int i, int j, const GridGraph& graph) {
    float x = graph.origin_x + (j + 0.5f) * graph.meters_per_cell;
    float y = graph.origin_y + (i + 0.5f) * graph.meters_per_cell;
    return {x, y};
}

bool isCellInBounds(int i, int j, const GridGraph& graph) {
    return i >= 0 && i < graph.height && j >= 0 && j < graph.width;
}

bool isIdxOccupied(int idx, const GridGraph& graph) {
    return graph.cell_odds[idx] >= graph.threshold;
}

bool isCellOccupied(int i, int j, const GridGraph& graph) {
    int idx = cellToIdx(i, j, graph);
    if (idx < 0 || idx >= graph.width * graph.height) return true;
    return isIdxOccupied(idx, graph);
}

// ------------------- Core Functions -------------------
bool isLoaded(const GridGraph& graph) {
    return graph.width > 0 && graph.height > 0 &&
           graph.meters_per_cell > 0 &&
           (int)graph.cell_odds.size() == graph.width * graph.height;
}

bool loadFromFile(const string& file_path, GridGraph& graph) {
    ifstream in(file_path);
    if (!in.is_open()) {
        cerr << "ERROR: loadFromFile: Failed to open " << file_path << endl;
        return false;
    }

    // Read header: origin_x origin_y width height meters_per_cell
    in >> graph.origin_x >> graph.origin_y >> graph.width >> graph.height >> graph.meters_per_cell;
    if (graph.width <= 0 || graph.height <= 0 || graph.meters_per_cell <= 0) return false;

    int num_cells = graph.width * graph.height;
    graph.cell_odds.assign(num_cells, 0);
    graph.obstacle_distances.assign(num_cells, numeric_limits<float>::infinity());
    graph.isObstacle.assign(num_cells, false);

    int odds;
    for (int idx = 0; idx < num_cells; ++idx) {
        if (!(in >> odds)) odds = 0;
        graph.cell_odds[idx] = odds;
        graph.isObstacle[idx] = odds >= graph.threshold;
    }

    graph.collision_radius = ROBOT_RADIUS + graph.meters_per_cell;
    initGraph(graph);

    return true;
}

string mapAsString(const GridGraph& graph) {
    ostringstream oss;
    oss << graph.origin_x << " " << graph.origin_y << " " 
        << graph.width << " " << graph.height << " " << graph.meters_per_cell << "\n";
    for (int i = 0; i < graph.height; ++i) {
        for (int j = 0; j < graph.width; ++j) {
            oss << int(graph.cell_odds[cellToIdx(i, j, graph)]) << " ";
        }
        oss << "\n";
    }
    return oss.str();
}

// ------------------- Initialization -------------------
void initGraph(GridGraph& graph) {
    if (graph.width <= 0 || graph.height <= 0) return;

    int N = graph.width * graph.height;

    if ((int)graph.cell_odds.size() != N) 
        graph.cell_odds.assign(N, 0);
    if ((int)graph.obstacle_distances.size() != N) 
        graph.obstacle_distances.assign(N, numeric_limits<float>::infinity());

    graph.parent_idx.assign(N, -1);
    graph.score.assign(N, HIGH);
    graph.visited_flag.assign(N, false);
    graph.visited_cells.clear();

    graph.nodes.clear();
    for (int i = 0; i < graph.height; ++i)
        for (int j = 0; j < graph.width; ++j)
            graph.nodes.push_back(CellNode(i, j));
}

// ------------------- Neighbors -------------------
vector<int> findNeighbors(int idx, const GridGraph& graph, bool eightConnected = false) {
    vector<int> neighbors;
    Cell c = idxToCell(idx, graph);
    vector<pair<int,int>> offsets = {{-1,0},{1,0},{0,-1},{0,1}};

    if (eightConnected) {
        offsets.push_back({-1,-1}); offsets.push_back({-1,1});
        offsets.push_back({1,-1});  offsets.push_back({1,1});
    }

    for (auto [di,dj] : offsets) {
        int ni = c.i + di;
        int nj = c.j + dj;
        if (isCellInBounds(ni, nj, graph)) {
            int nidx = cellToIdx(ni, nj, graph);
            if (!graph.isObstacle[nidx]) neighbors.push_back(nidx);
        }
    }

    return neighbors;
}

// ------------------- Collision -------------------
bool checkCollisionFast(int idx, const GridGraph& graph) {
    return graph.obstacle_distances[idx] * graph.meters_per_cell <= graph.collision_radius;
}

bool checkCollision(int idx, const GridGraph& graph) {
    if (isIdxOccupied(idx, graph)) return true;

    Cell c0 = idxToCell(idx, graph);
    auto state = cellToPos(c0.i, c0.j, graph);
    double dtheta = graph.meters_per_cell / graph.collision_radius;
    for (double theta = 0; theta < 2*PI; theta += dtheta) {
        double x = state[0] + graph.collision_radius * cos(theta);
        double y = state[1] + graph.collision_radius * sin(theta);
        Cell c = posToCell(x, y, graph);
        if (!isCellInBounds(c.i, c.j, graph) || isCellOccupied(c.i, c.j, graph))
            return true;
    }
    return false;
}

// ------------------- Pathfinding Helpers -------------------
int getParent(int idx, const GridGraph& graph) {
    if (idx < 0 || idx >= (int)graph.parent_idx.size()) return -1;
    return graph.parent_idx[idx];
}

float getScore(int idx, const GridGraph& graph) {
    if (idx < 0 || idx >= (int)graph.score.size()) return HIGH;
    return graph.score[idx];
}

int findLowestScore(const vector<int>& node_list, const GridGraph& graph) {
    if (node_list.empty()) return -1;
    int min_idx = node_list[0];
    for (int i = 1; i < node_list.size(); ++i) {
        if (getScore(node_list[i], graph) < getScore(min_idx, graph)) {
            min_idx = node_list[i];
        }
    }
    return min_idx;
}

vector<Cell> tracePath(int goal, const GridGraph& graph) {
    vector<Cell> path;
    int current = goal;
    while (current >= 0) {
        path.push_back(idxToCell(current, graph));
        current = getParent(current, graph);
    }
    reverse(path.begin(), path.end());
    return path;
}
