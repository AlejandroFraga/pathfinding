#ifndef PATH_FINDER_H
#define PATH_FINDER_H
#pragma once

#include <chrono>
#include <thread>
#include <unordered_map>
#include "Algorithms/AlgorithmBase.hpp"
#include "Output/ConsoleOutput.hpp"
#include "../Timer.hpp"


namespace PathFinding
{
    enum class RunMode
    {
        Timer,
        StopStepByStep,
        AutoStepByStep
    };

    class PathFinder
    {
    private:
        
        /// Hash function for the Coordinate, so a unordered map can be used for the obstacles
        struct hash_pair {
            size_t operator()(const Coordinate& p) const
            {
                auto hash1 = std::hash<int>{}(p.first);
                auto hash2 = std::hash<int>{}(p.second);

                if (hash1 != hash2)
                {
                    return hash1 ^ hash2;
                }

                return hash1;
            }
        };
        
        /// Algorithm to run
        std::unique_ptr<AlgorithmBase> m_algorithm;
        
        /// Collection of obstacles
        std::unordered_map<Coordinate, bool, hash_pair> m_obstacles;
        
    public:
        
        explicit PathFinder(std::unique_ptr<AlgorithmBase> &&algorithm = {}) : m_algorithm(std::move(algorithm))
        {
            initObstacles();
        }
        
        /**
         * Set the algorithm to run.
         *
         * @param algorithm Algorithm to run.
         */
        void setAlgorithm(std::unique_ptr<AlgorithmBase> &&algorithm)
        {
            m_algorithm = std::move(algorithm);
            
            initObstacles();
        }
        
        /**
         * Returns a collection of random obstacles with size [0, total size / 2).
         *
         * @return Collection of random obstacles.
         */
        auto getRandObstacles()
        {
            std::vector<Coordinate> obstacles;
            
            srand((unsigned int)time(0));
            auto n = rand() % (m_algorithm.get()->getTotalSize() / 2);

            for (size_t i = 0; i < n; ++i)
            {
                Coordinate pos{rand() % m_algorithm.get()->getSize().first, rand() % m_algorithm.get()->getSize().second};
                obstacles.emplace_back(pos);
            }
            return obstacles;
        }
        
        /**
         * Sets the node at the coordinate given as an obstacle or not.
         *
         * @param c Coordinate in which the obstacle field is modified.
         * @param obstacle Wether we set it to be an obstacle or not.
         */
        void setObstacle(const Coordinate& c, const bool obstacle = true)
        {
            m_algorithm.get()->setObstacle(c, obstacle);
            m_obstacles[c] = obstacle;
        }
        
        /**
         * Sets a collection of nodes at given coordinates as an obstacle or not.
         *
         * @param cs Collection of coordinate in which the obstacle field is modified.
         * @param obstacle Wether we set them to be an obstacle or not.
         */
        void setObstacles(const std::vector<Coordinate>& cs, const bool obstacle = true)
        {
            for (auto c : cs)
            {
                setObstacle(c);
            }
        }
        
        /**
         * Runs the algorithm with the parameters given.
         *
         * @param runMode Mode.
         * @param heuristic Heuristic to use when calculating distances.
         * @param diagonals The algorithm can use diagonal movements.
         * @param cutCorners The algorithm can cut corners when passing next to an obstacle.
         */
        void run(RunMode runMode = RunMode::Timer, Heuristic heuristic = Heuristic::Manhattan, bool diagonals = true, bool cutCorners = true)
        {
            // Init the Algorithm
            m_algorithm->init(heuristic, diagonals, cutCorners);
            
            switch (runMode)
            {
                case RunMode::Timer:
                    runModeTimer();
                    break;
                    
                case RunMode::StopStepByStep:
                    runModeStopStepByStep();
                    break;
                    
                case RunMode::AutoStepByStep:
                    runModeAutoStepByStep();
                    break;
                    
                default:
                    std::cout << "Mode not supported yet";
                    break;
            }
            
            // Print the solution if there is any
            ConsoleOutput::Print(*m_algorithm, true, true);
        }
        
        /**
         * Sets random obstacles.
         *
         * @param resetBefore Resets the obstacles before setting them randomly.
         */
        void setRandObstacles(bool resetBefore = true)
        {
            if (resetBefore)
            {
                m_algorithm.get()->reset();
                resetObstacles();
            }
            
            setObstacles(getRandObstacles());
        }
        
    private:
        
        /**
         * Runs the Algorithm on Timer mode.
         */
        void runModeTimer()
        {
            // We create this scope to make the timer call its destructor
            {
                Benchmark::Timer timer(m_algorithm->name);
                m_algorithm->complete();
            }
        }
        
        /**
         * Runs the Algorithm on StopStepByStep mode.
         */
        void runModeStopStepByStep()
        {
            // Execute the algorithm with stops between steps
            while (!m_algorithm->nextStep())
            {
                ConsoleOutput::Print(*m_algorithm, true, true);
            }
        }
        
        /**
         * Runs the Algorithm on AutoStepByStep mode.
         */
        void runModeAutoStepByStep()
        {
            // Execute the algorithm with 500ms stops between steps
            while (!m_algorithm->nextStep())
            {
                ConsoleOutput::Print(*m_algorithm);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        /**
         * Init the obstacles of the Algorithm with the ones stored.
         */
        void initObstacles()
        {
            for (auto& c: m_obstacles)
            {
                m_algorithm.get()->setObstacle(c.first);
            }
        }
        
        /**
         * Resets the obstacles stored.
         */
        void resetObstacles()
        {
            m_obstacles.clear();
        }
    };
}

#endif
