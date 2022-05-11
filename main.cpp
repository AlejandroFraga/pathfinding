#ifdef _DEBUG
#include <chrono>
#include <thread>
#endif
#include "Timer.h"
#include "ConsoleOutput.h"
#include "Utils.h"
#include "Board.h"
#include "AStar.h"


/**
 * Sets obstacles on the board.
 *
 * @param board Board on which the obstacles are set.
 */
template <Derived<PathFinding::NodeBase>T>
void setRandObstacles(PathFinding::Board<T>& board)
{
    srand(time(0));
    auto n = rand() % (board.size() / 2);

    for (size_t i = 0; i < n; ++i)
    {
        PathFinding::Coordinate pos = { rand() % board.max.first, rand() % board.max.second };
        if (pos != board.start && pos != board.goal)
            board[pos].obstacle = true;
    }
}

int main()
{
    // To avoid putting PathFinding many times over the example main
    using namespace PathFinding;

    // We create a board with A* nodes
    Board<AStar::Node> board({ 25, 5 }, { 1, 2 }, { 23, 2 });
    setRandObstacles(board);

    // Init the A* with the board of A* nodes
    AStar aStar(board);
    aStar.init(Heuristic::Manhattan);

#ifndef _DEBUG
    // We create this scope to make the timer call its destructor
    {
        // If we are not debugging, execute the complete A* and benchmark the time it takes
        Benchmark::Timer timer("A*");
        aStar.complete();
    }
#else
    // If we are debugging, execute the A* step by step with 500ms stops between steps
    while (!aStar.nextStep())
    {
        ConsoleOutput::Print(aStar);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ConsoleOutput::ClearScreen();
    }
#endif
    
    // Print the solution if there is any
    ConsoleOutput::Print(aStar, false, true);
}
