#ifndef ALGORITHM_BASE_H
#define ALGORITHM_BASE_H
#pragma once

#include <cmath>
#include <vector>
#include "../Board.hpp"


namespace PathFinding
{
    /// Heuristic to calculate the distance between nodes
    enum class Heuristic
    {
        Manhattan,
        Diagonal,
        Euclidean,
        Chebyshev
    };

    /// Base clase for PathFinding algorithms
    class AlgorithmBase
    {
    public:
        
        /// Name of the algorithm
        std::string name;
        
        /// Collection of solution coordinates
        std::vector<Coordinate> solution;
        
        /// Number of nodes opened
        unsigned long nodesOpened = 0;
        
        /// Number of nodes closed
        unsigned long nodesClosed = 0;
        
    protected:
        
        /// Heuristic to calculate the distance between nodes
        Heuristic m_heuristic = Heuristic::Manhattan;
        
        /// Whether you can move diagonally or not
        bool m_diagonals = true;
        
        ///Whether you can cut obstacles corners or not
        bool m_cutCorners = true;

    public:
        explicit AlgorithmBase(std::string &&name = {}) : name(std::move(name)) {}
        
        virtual ~AlgorithmBase() {}
        
        /**
         * Instantiates the variables of the algorithm
         *
         * @param heuristic Heuristic to use when calculating distances
         * @param diagonals The algorithm can use diagonal movements
         * @param cutCorners The algorithm can cut corners when passing next to an obstacle
         */
        virtual void init(const Heuristic heuristic, const bool diagonals = true, const bool cutCorners = true)
        {
            m_heuristic = heuristic;
            m_diagonals = diagonals;
            m_cutCorners = cutCorners;
            
            nodesOpened = 0;
            
            solution.clear();
        }
        
        /**
         * Resets the algorithm
         */
        virtual void reset()
        {
            nodesOpened = 0;
            
            solution.clear();
        }
        
        /**
         * Gets the size of the board
         *
         * @return The size of the board
         */
        virtual inline const size_t getTotalSize() const = 0;
        
        /**
         * Gets the size by axis of the board
         *
         * @return The size by axis of the board
         */
        virtual inline const Coordinate getSize() const = 0;
        
        /**
         * Gets the coordinate of the of the board's start
         *
         * @return The coordinate of the board's start
         */
        virtual inline const Coordinate getStart() const = 0;
        
        /**
         * Gets the coordinate of the of the board's goal
         *
         * @return The coordinate of the board's goal
         */
        virtual inline const Coordinate getGoal() const = 0;
        
        /**
         * Gets the coordinate of the of the board's goal
         *
         * @return The coordinate of the board's goal
         */
        virtual inline const NodeBase* getGoalNode() const = 0;
        
        /**
         * Checks if a given coordinate is an obstacle or not
         *
         * @param c Coordinate in which the obstacle field is modified
         * @return If the given coordinate is an obstacle
         */
        virtual inline const bool isObstacle(const Coordinate& c) const = 0;
        
        /**
         * Sets the node at a given coordinate as an obstacle or not
         *
         * @param c Coordinate in which the obstacle field is modified
         * @param obstacle Wether we set it to be an obstacle or not
         */
        virtual inline const void setObstacle(const Coordinate& c, const bool obstacle = true) const = 0;
        
        //TODO: Remove
        virtual inline double getValue1(const Coordinate& c) const = 0;
        
        //TODO: Remove
        virtual inline double getValue2(const Coordinate& c) const = 0;
        
        /**
         * Calculates the distance of the solution
         *
         * @return Distance of the solution
         */
        const double getSolutionDistance() const
        {
            double distance = 0.f;
            
            if (solution.size())
            {
                for (int i = 0; i < solution.size() - 1; ++i)
                {
                    distance += isDiagonal(solution[i], solution[i + 1]) ? 1.414f : 1.f;
                }
            }
            return distance;
        }
        
        /**
         * Runs the next step of the algorithm
         *
         * @return Distance of the solution
         */
        virtual bool nextStep() = 0;
        
        /**
         * Runs the algorithm until completion
         */
        inline void complete()
        {
            while(!nextStep());
        }

    protected:
        /**
         * Checks if the given coordinate is the goal, if so, the solution is stored if found
         *
         * @param c Coordinate to check if is the goal or not
         * @return If the given node is the goal or not
         */
        bool checkGoal(const Coordinate& c)
        {
            if (c == getGoal())
            {
                storeSolution();
                return true;
            }
            return false;
        }
        
        /**
         * Stores the solution if found
         */
        void storeSolution()
        {
            // Start on the goal and go back by parents
            auto* node = getGoalNode();
            
            while (node)
            {
                solution.push_back(node->position);
                node = node->parent;
            }
        }
        
        /**
         * Opens a node
         *
         * @param node Node to open
         * @param parent Node to set as parent of the node to open
         */
        virtual void openNode(NodeBase* node, NodeBase* parent = nullptr)
        {
            ++nodesOpened;
            
            node->parent = parent;
            node->opened = true;
        }
        
        /**
         * Checks if the coordinates are diagonally connected or not
         *
         * @param c1 First coodinate
         * @param c2 Second coordinate
         * @return Wheter the coordinates are diagonally connected or not
         */
        const bool isDiagonal(const Coordinate& c1, const Coordinate& c2) const
        {
            return c1.first != c2.first && c1.second != c2.second;
        }
        
        /**
         * Calculates the distance between the two given coordinates with the Heuristic given
         *
         * @param from First coordinate
         * @param to Second coordinate
         * @return Distance between the two given coordinates with the Heuristic given
         */
        const double distance(const Coordinate& from, const Coordinate& to) const
        {
            switch (m_heuristic)
            {
                case Heuristic::Manhattan:
                    return manhattanDistance(from, to);

                case Heuristic::Diagonal:
                    return diagonalDistance(from, to);

                case Heuristic::Euclidean:
                    return euclideanDistance(from, to);
                        
                case Heuristic::Chebyshev:
                    return chebyshevDistance(from, to);
            }
            return -1.f;
        }
        
        /**
         * Calculates the distance between the two given coordinates with the Manhattan Heuristic
         *
         * @param from First coordinate
         * @param to Second coordinate
         * @return Distance between the two given coordinates with the Manhattan Heuristic
         */
        const double manhattanDistance(const Coordinate& from, const Coordinate& to) const
        {
            return std::abs(from.first - to.first) + std::abs(from.second - to.second);
        }
        
        /**
         * Calculates the distance between the two given coordinates with the Diagonal Heuristic
         *
         * @param from First coordinate
         * @param to Second coordinate
         * @return Distance between the two given coordinates with the Diagonal Heuristic
         */
        const double diagonalDistance(const Coordinate& from, const Coordinate& to) const
        {
            //auto D = 1;
            //auto D2 = std::sqrt(2);
            auto D2 = 1.414f;
            auto dx = std::abs(from.first - to.first);
            auto dy = std::abs(from.second - to.second);
            // return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
            return (dx + dy) + (D2 - 2) * std::min(dx, dy);
        }
        
        /**
         * Calculates the distance between the two given coordinates with the Euclidean Heuristic
         *
         * @param from First coordinate
         * @param to Second coordinate
         * @return Distance between the two given coordinates with the Euclidean Heuristic
         */
        const double euclideanDistance(const Coordinate& from, const Coordinate& to) const
        {
            return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
        }
        
        /**
         * Calculates the distance between the two given coordinates with the Chebyshev Heuristic
         *
         * @param from First coordinate
         * @param to Second coordinate
         * @return Distance between the two given coordinates with the Chebyshev Heuristic
         */
        const double chebyshevDistance(const Coordinate& from, const Coordinate& to) const
        {
            //TODO: Implement
            return std::abs(from.first - to.first) + std::abs(from.second - to.second);
        }
    };
}

#endif
