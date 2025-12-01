#include <iostream>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>

#include <mbot_bridge/robot.h>

#include <path_planning/utils/graph_utils.h>
#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/viz_utils.h>
#include <path_planning/graph_search/graph_search.h>
#include <path_planning/graph_search/distance_transform.h>

int main(int argc, char const *argv[])
{
    float goal_x = 0, goal_y = 0;

    if (argc < 2)
    {
        std::cerr << "Please provide the path to a map file as input.\n";
        return -1;
    }

    if (argc == 4)
    {
        goal_x = std::stof(argv[2]);
        goal_y = std::stof(argv[3]);
    }

    std::string map_file = argv[1];
    GridGraph graph;
    loadFromFile(map_file, graph);

    // If using checkCollisionFast, compute distance transform
    computeDistanceTransform(graph);

    // Convert goal position to cell
    Cell goal = posToCell(goal_x, goal_y, graph);

    // Initialize the robot
    mbot_bridge::MBot robot;

    // Get the robot's SLAM pose
    std::vector<float> pose = robot.readSlamPose();
    if (pose.size() == 0)
    {
        std::cerr << "No pose information! Can't plan." << std::endl;
        return -1;
    }

    // Convert start position to cell
    Cell start = posToCell(pose[0], pose[1], graph);

    // Plan path using BFS
    std::vector<Cell> path = breadthFirstSearch(graph, start, goal);

    if (path.empty())
    {
        std::cerr << "No path found to the goal." << std::endl;
        return -1;
    }

    std::cout << "Path found with " << path.size() << " cells. Sending to robot..." << std::endl;

    // Send path to robot
    robot.drivePath(cellsToPoses(path, graph));

    // Optional: Wait until robot reaches the goal
    while (!robot.isAtGoal())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Robot has reached the goal!" << std::endl;

    // Save the path output file for visualization
    generatePlanFile(start, goal, path, graph);

    return 0;
}
