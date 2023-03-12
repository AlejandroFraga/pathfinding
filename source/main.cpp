#include "PathFinding/PathFinder.hpp"
#include "PathFinding/Algorithms/AStar.hpp"
#include "PathFinding/Algorithms/BestFirstSearch.hpp"
#include "PathFinding/Algorithms/BreadthFirstSearch.hpp"
#include "PathFinding/Algorithms/DepthFirstSearch.hpp"


int main()
{
    using namespace PathFinding;
    
    
    // Store the Board parameters
    Coordinate size { 25, 5 };
    Coordinate start { 1, size.second / 2 };
    Coordinate goal { size.first - 2, size.second / 2 };
    
    
    // Store the Run Mode
    RunMode runMode = RunMode::Timer;
    
    
    // Store the PathFinding Algorithm parameters
    Heuristic heuristic = Heuristic::Manhattan;
    bool diagonals = true;
    bool cutCorners = true;
    
    
    // Init the A* Algorithm
    PathFinder pathFinder(std::make_unique<AStar>(size, start, goal));
    
    while (true)
    {
        // Set rand obstacles and run it
        pathFinder.setRandObstacles();
        pathFinder.run(runMode, heuristic, diagonals, cutCorners);
        
        
        // Set the Best First Search Algorithm and run it with same obstacles as last algorithm
        pathFinder.setAlgorithm(std::make_unique<BestFirstSearch>(size, start, goal));
        pathFinder.run(runMode, heuristic, diagonals, cutCorners);
        
        
        // Set the Breadth First Search Algorithm and run it with same obstacles as last algorithm
        pathFinder.setAlgorithm(std::make_unique<BreadthFirstSearch>(size, start, goal));
        pathFinder.run(runMode, heuristic, diagonals, cutCorners);
        
        
        // Set the Depth First Search Algorithm and run it with same obstacles as last algorithm
        pathFinder.setAlgorithm(std::make_unique<DepthFirstSearch>(size, start, goal));
        pathFinder.run(runMode, heuristic, diagonals, cutCorners);
        
        
        // Set the A* Algorithm
        pathFinder.setAlgorithm(std::make_unique<AStar>(size, start, goal));
    }
}
