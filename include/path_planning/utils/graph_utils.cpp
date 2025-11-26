#include "graph_utils.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

// -------------------- Basic checks & init --------------------

bool isLoaded(const GridGraph& graph)
{
    return graph.width > 0 && graph.height > 0 &&
           (int)graph.cell_odds.size() == graph.width * graph.height;
}

bool loadFromFile(const std::string& file_path, GridGraph& graph)
{
    // Minimal example loader: expects a simple text format:
    // First line: width height meters_per_cell origin_x origin_y
    // Then width*height integers for cell odds (row-major)
    ifstream in(file_path);
    if (!in.is_open()) return false;

    int w, h;
    float mpc, ox, oy;
    if (!(in >> w >> h >> mpc >> ox >> oy)) return false;

    graph.width = w;
    graph.height = h;
    graph.meters_per_cell = mpc;
    graph.origin_x = ox;
    graph.origin_y = oy;

    graph.cell_odds.assign(w*h, 0);
    for (int i = 0; i < w*h; ++i) {
        int v;
        if (!(in >> v)) v = 0;
        graph.cell_odds[i] = static_cast<int8_t>(v);
    }

    // Initialize obstacle_distances to a large value (will need computation)
    graph.obstacle_distances.assign(w*h, std::numeric_limits<float>::infinity());

    // initialize node arrays
    graph.parent_idx.assign(w*h, -1);
    graph.score.assign(w*h, HIGH);
    graph.visited_flag.assign(w*h, false);

    return true;
}

std::string mapAsString(GridGraph& graph)
{
    std::ostringstream oss;
    oss << graph.width << " " << graph.height << " " << graph.meters_per_cell << " "
        << graph.origin_x << " " << graph.origin_y << "\n";
    for (int i = 0; i < graph.width * graph.height; ++i) {
        oss << int(graph.cell_odds[i]) << ( (i+1) % graph.width == 0 ? "\n" : " ");
    }
    return oss.str();
}

void initGraph(GridGraph& graph)
{
    if (graph.width <= 0 || graph.height <= 0) return;
    int N = graph.width * graph.height;
    if ((int)graph.cell_odds.size() != N) graph.cell_odds.assign(N, 0);
    if ((int)graph.obstacle_distances.size() != N) graph.obstacle_distances.assign(N, std::numeric_limits<float>::infinity());

    graph.parent_idx.assign(N, -1);
    graph.score.assign(N, HIGH);
    graph.visited_flag.assign(N, false);
    graph.visited_cells.clear();
}

// -------------------- Conversions --------------------

int cellToIdx(int i, int j, const GridGraph& graph)
{
    if (!isCellInBounds(i,j,graph)) return -1;
    return i * graph.width + j; // row-major
}

Cell idxToCell(int idx, const GridGraph& graph)
{
    if (idx < 0 || idx >= graph.width * graph.height) return Cell();
    int r = idx / graph.width;
    int c = idx % graph.width;
    return Cell(r, c);
}

Cell posToCell(float x, float y, const GridGraph& graph)
{
    // Convert world coords to cell indices
    float rel_x = x - graph.origin_x;
    float rel_y = y - graph.origin_y;
    int j = static_cast<int>(floor(rel_x / graph.meters_per_cell + 0.5f));
    int i = static_cast<int>(floor(rel_y / graph.meters_per_cell + 0.5f));
    if (!isCellInBounds(i, j, graph)) return Cell(-1,-1);
    return Cell(i, j);
}

std::vector<float> cellToPos(int i, int j, const GridGraph& graph)
{
    std::vector<float> pos(2, 0.0f);
    if (!isCellInBounds(i, j, graph)) return pos;
    // Center of the cell
    pos[0] = graph.origin_x + (j + 0.5f) * graph.meters_per_cell;
    pos[1] = graph.origin_y + (i + 0.5f) * graph.meters_per_cell;
    return pos;
}

// -------------------- Bounds & occupancy --------------------

bool isCellInBounds(int i, int j, const GridGraph& graph)
{
    return i >= 0 && j >= 0 && i < graph.height && j < graph.width;
}

bool isIdxOccupied(int idx, const GridGraph& graph)
{
    if (idx < 0 || idx >= graph.width * graph.height) return true;
    // If cell_odds > threshold => occupied
    return graph.cell_odds[idx] >= graph.threshold;
}

bool isCellOccupied(int i, int j, const GridGraph& graph)
{
    int idx = cellToIdx(i, j, graph);
    if (idx < 0) return true;
    return isIdxOccupied(idx, graph);
}

// -------------------- Neighbors --------------------

std::vector<int> findNeighbors(int idx, const GridGraph& graph)
{
    std::vector<int> neighbors;
    if (idx < 0) return neighbors;
    int r = idx / graph.width;
    int c = idx % graph.width;

    // 4-connected neighbors (up, right, down, left). Add diagonals if desired.
    constexpr int dr[4] = {-1, 0, 1, 0};
    constexpr int dc[4] = {0, 1, 0, -1};

    for (int k = 0; k < 4; ++k) {
        int nr = r + dr[k];
        int nc = c + dc[k];
        if (isCellInBounds(nr, nc, graph)) {
            int nidx = cellToIdx(nr, nc, graph);
            // skip occupied cells
            if (!isIdxOccupied(nidx, graph)) neighbors.push_back(nidx);
        }
    }
    return neighbors;
}

// -------------------- Collision checks --------------------

bool checkCollisionFast(int idx, const GridGraph& graph)
{
    if (idx < 0) return true;
    if (graph.obstacle_distances.empty()) {
        // fallback to brute-force check
        return checkCollision(idx, graph);
    }
    // compare distance at cell with collision_radius (plus robot radius)
    return graph.obstacle_distances[idx] < graph.collision_radius;
}

bool checkCollision(int idx, const GridGraph& graph)
{
    if (idx < 0) return true;
    // brute-force check: iterate cells within radius and see if any are occupied
    int N = graph.width * graph.height;
    int r = idx / graph.width;
    int c = idx % graph.width;

    int maxCellOffset = static_cast<int>(ceil(graph.collision_radius / graph.meters_per_cell));
    for (int dr = -maxCellOffset; dr <= maxCellOffset; ++dr) {
        for (int dc = -maxCellOffset; dc <= maxCellOffset; ++dc) {
            int nr = r + dr;
            int nc = c + dc;
            if (!isCellInBounds(nr, nc, graph)) continue;
            int nidx = cellToIdx(nr, nc, graph);
            auto posA = cellToPos(r, c, graph);
            auto posB = cellToPos(nr, nc, graph);
            float dx = posA[0] - posB[0];
            float dy = posA[1] - posB[1];
            float d = sqrt(dx*dx + dy*dy);
            if (d <= graph.collision_radius) {
                if (isIdxOccupied(nidx, graph)) return true;
            }
        }
    }
    return false;
}

// -------------------- Parent / score helpers --------------------

int getParent(int idx, const GridGraph& graph)
{
    if (idx < 0 || idx >= (int)graph.parent_idx.size()) return -1;
    return graph.parent_idx[idx];
}

float getScore(int idx, const GridGraph& graph)
{
    if (idx < 0 || idx >= (int)graph.score.size()) return HIGH;
    return graph.score[idx];
}

int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph)
{
    if (node_list.empty()) return -1;
    int best_idx = -1;
    float best_score = std::numeric_limits<float>::infinity();
    for (int n : node_list) {
        if (n < 0 || n >= (int)graph.score.size()) continue;
        float s = graph.score[n];
        if (s < best_score) {
            best_score = s;
            best_idx = n;
        }
    }
    return best_idx;
}

std::vector<Cell> tracePath(int goal, const GridGraph& graph)
{
    std::vector<Cell> path;
    if (goal < 0 || goal >= graph.width * graph.height) return path;
    int cur = goal;
    while (cur >= 0) {
        Cell c = idxToCell(cur, graph);
        path.push_back(c);
        cur = getParent(cur, graph);
    }
    // currently path is goal->start, reverse to start->goal
    reverse(path.begin(), path.end());
    return path;
}
