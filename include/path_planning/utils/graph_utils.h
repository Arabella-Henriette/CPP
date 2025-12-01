#ifndef PATH_PLANNING_GRAPH_SEARCH_GRAPH_UTILS_H
#define PATH_PLANNING_GRAPH_SEARCH_GRAPH_UTILS_H

#include <vector>
#include <string>
#include <array>
#include <cstdint>   // FIXED include error

#define HIGH 1e6
#define ROBOT_RADIUS 0.137

// -----------------------------------------------------------------------------
// Visualization Cell (USED ONLY FOR DISPLAY AND PATH OUTPUT)
// -----------------------------------------------------------------------------
struct Cell
{
    int row, col;          // Row and column
    bool visited;          // Whether it has been visited (for debugging/visual)
    int cost;              // Cost for visualization (not used in planning)

    Cell* parent;          // Pointer to parent (visualization only)

    Cell(int r = 0, int c = 0)
        : row(r), col(c), visited(false), cost(0), parent(nullptr) {}
};

// -----------------------------------------------------------------------------
// P3.2: CellNode — used for path planning (A*, BFS, Dijkstra)
// -----------------------------------------------------------------------------
struct CellNode
{
    int row, col;                 // Location of the cell
    bool visited;                 // Has it been visited during search?
    float cost;                   // Cost-so-far (g-score)
    int parent;                   // Parent index (NOT a pointer)

    CellNode(int r = 0, int c = 0)
        : row(r), col(c), visited(false),
          cost(std::numeric_limits<float>::infinity()),
          parent(-1) {}
};

// -----------------------------------------------------------------------------
// GridGraph structure
// -----------------------------------------------------------------------------
struct GridGraph
{
    GridGraph()
        : width(-1),
          height(-1),
          origin_x(0.0f),
          origin_y(0.0f),
          meters_per_cell(0.0f),
          collision_radius(0.15f),
          threshold(-100)
    {}

    int width, height;
    float origin_x, origin_y;
    float meters_per_cell;
    float collision_radius;
    int8_t threshold;

    std::vector<int8_t> cell_odds;
    std::vector<float> obstacle_distances;

    std::vector<Cell> visited_cells; // used only for visualization

    // P3.2: Vector of search nodes
    std::vector<CellNode> nodes;
};

// -----------------------------------------------------------------------------
// Function declarations (unchanged)
// -----------------------------------------------------------------------------

bool isLoaded(const GridGraph& graph);
bool loadFromFile(const std::string& file_path, GridGraph& graph);
std::string mapAsString(GridGraph& graph);
void initGraph(GridGraph& graph);

int cellToIdx(int i, int j, const GridGraph& graph);
Cell idxToCell(int idx, const GridGraph& graph);
Cell posToCell(float x, float y, const GridGraph& graph);
std::vector<float> cellToPos(int i, int j, const GridGraph& graph);

bool isCellInBounds(int i, int j, const GridGraph& graph);
bool isIdxOccupied(int idx, const GridGraph& graph);
bool isCellOccupied(int i, int j, const GridGraph& graph);

std::vector<int> findNeighbors(int idx, const GridGraph& graph);

bool checkCollisionFast(int idx, const GridGraph& graph);
bool checkCollision(int idx, const GridGraph& graph);

int getParent(int idx, const GridGraph& graph);
float getScore(int idx, const GridGraph& graph);
int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph);

std::vector<Cell> tracePath(int goal, const GridGraph& graph);

static std::vector<std::array<float, 3>> cellsToPoses(std::vector<Cell>& path,
                                                      GridGraph& graph)
{
    std::vector<std::array<float, 3>> pose_path;

    for (Cell& cell : path)
    {
        std::array<float, 3> pose;
        auto position = cellToPos(cell.row, cell.col, graph);
        pose[0] = position[0];
        pose[1] = position[1];
        pose[2] = 0;
        pose_path.push_back(pose);
    }

    return pose_path;
};

#endif
