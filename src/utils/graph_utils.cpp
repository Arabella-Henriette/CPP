#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

// ------------------- Constants -------------------
const double PI = 3.14159265358979323846;
const float ROBOT_RADIUS = 0.25f;  // example value, change as needed
const float HIGH = 1e6f;           // high score placeholder

// ------------------- Structs -------------------
struct Cell {
    int i;
    int j;
};

struct GridGraph {
    int width;
    int height;
    float meters_per_cell;
    float origin_x;
    float origin_y;
    float collision_radius;
    float threshold;              // for occupied cells
    vector<int> cell_odds;        // odds or occupancy
    vector<float> obstacle_distances;
    vector<bool> isObstacle;      // true if blocked
};

// ------------------- Helper Functions -------------------
int cellToIdx(int i, int j, const GridGraph& graph) {
    return i + j * graph.width;
}

Cell idxToCell(int idx, const GridGraph& graph) {
    Cell c;
    c.i = idx % graph.width;
    c.j = idx / graph.width;
    return c;
}

Cell posToCell(float x, float y, const GridGraph& graph) {
    int i = static_cast<int>(floor((x - graph.origin_x) / graph.meters_per_cell));
    int j = static_cast<int>(floor((y - graph.origin_y) / graph.meters_per_cell));
    return {i, j};
}

vector<float> cellToPos(int i, int j, const GridGraph& graph) {
    float x = (i + 0.5f) * graph.meters_per_cell + graph.origin_x;
    float y = (j + 0.5f) * graph.meters_per_cell + graph.origin_y;
    return {x, y};
}

bool isCellInBounds(int i, int j, const GridGraph& graph) {
    return i >= 0 && j >= 0 && i < graph.width && j < graph.height;
}

bool isIdxOccupied(int idx, const GridGraph& graph) {
    return graph.cell_odds[idx] >= graph.threshold;
}

bool isCellOccupied(int i, int j, const GridGraph& graph) {
    return isIdxOccupied(cellToIdx(i, j, graph), graph);
}

// ------------------- Core Functions -------------------
bool isLoaded(const GridGraph& graph) {
    bool correct_size = graph.cell_odds.size() == graph.width * graph.height;
    bool positive_size = graph.width > 0 && graph.height > 0;
    bool positive_m_per_cell = graph.meters_per_cell > 0;
    return correct_size && positive_size && positive_m_per_cell;
}

bool loadFromFile(const string& file_path, GridGraph& graph) {
    ifstream in(file_path);
    if (!in.is_open()) {
        cerr << "ERROR: loadFromFile: Failed to load from " << file_path << endl;
        return false;
    }

    // Read header
    in >> graph.origin_x >> graph.origin_y >> graph.width >> graph.height >> graph.meters_per_cell;

    // Check sanity
    if (graph.width <= 0 || graph.height <= 0 || graph.meters_per_cell <= 0.0f) {
        return false;
    }

    graph.collision_radius = ROBOT_RADIUS + graph.meters_per_cell;

    int num_cells = graph.width * graph.height;
    graph.cell_odds.clear();
    graph.cell_odds.resize(num_cells);
    graph.obstacle_distances = vector<float>(num_cells, 0);
    graph.isObstacle = vector<bool>(num_cells, false);

    int odds;
    for (int idx = 0; idx < num_cells; ++idx) {
        in >> odds;
        graph.cell_odds[idx] = odds;
        graph.isObstacle[idx] = odds >= graph.threshold; // mark obstacle
    }

    initGraph(graph);

    return true;
}

string mapAsString(GridGraph& graph) {
    ostringstream oss;
    oss << graph.origin_x << " " << graph.origin_y << " ";
    oss << graph.width << " " << graph.height << " " << graph.meters_per_cell << " ";
    for (int j = 0; j < graph.height; j++) {
        for (int i = 0; i < graph.width; i++) {
            oss << graph.cell_odds[cellToIdx(i, j, graph)] << " ";
        }
    }
    return oss.str();
}

void initGraph(GridGraph& graph) {
    // TODO (P3): Initialize graph nodes if needed
}

// ------------------- Neighbors -------------------
vector<int> findNeighbors(int idx, const GridGraph& graph, bool eightConnected = false) {
    vector<int> neighbors;
    Cell c = idxToCell(idx, graph);

    vector<pair<int,int>> offsets = {{-1,0}, {1,0}, {0,-1}, {0,1}};
    if (eightConnected) {
        offsets.push_back({-1,-1});
        offsets.push_back({-1,1});
        offsets.push_back({1,-1});
        offsets.push_back({1,1});
    }

    for (auto [di,dj] : offsets) {
        int ni = c.i + di;
        int nj = c.j + dj;
        if (isCellInBounds(ni, nj, graph)) {
            int neighborIdx = cellToIdx(ni, nj, graph);
            if (!graph.isObstacle[neighborIdx]) {
                neighbors.push_back(neighborIdx);
            }
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

    double dtheta = graph.meters_per_cell / graph.collision_radius;
    double theta = 0;

    Cell c0 = idxToCell(idx, graph);
    auto state = cellToPos(c0.i, c0.j, graph);

    while (theta < 2 * PI) {
        double x = state[0] + graph.collision_radius * cos(theta);
        double y = state[1] + graph.collision_radius * sin(theta);
        Cell c = posToCell(x, y, graph);

        if (!isCellInBounds(c.i, c.j, graph)) return true;
        if (isCellOccupied(c.i, c.j, graph)) return true;

        theta += dtheta;
    }

    return false;
}

// ------------------- Pathfinding Helpers -------------------
int getParent(int idx, const GridGraph& graph) {
    // TODO (P3)
    return -1;
}

float getScore(int idx, const GridGraph& graph) {
    // TODO (P3)
    return HIGH;
}

int findLowestScore(const vector<int>& node_list, const GridGraph& graph) {
    if (node_list.empty()) return -1;
    int min_idx = 0;
    for (int i = 1; i < node_list.size(); ++i) {
        if (getScore(node_list[i], graph) < getScore(node_list[min_idx], graph)) {
            min_idx = i;
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
