#ifndef ASTAR_H
#define ASTAR_H
#pragma once

#include <vector>
#include "Board.h"


namespace PathFinding
{
    // Define the posible Heuristics
    enum class Heuristic
    {
        Manhattan, Diagonal, Euclidean
    };

	class AStar
	{
    public:
        struct Node : public NodeBase
        {
            // Whether the node has been evaluated or not.
            bool closed = false;

            // The previous node from which the shortest path reaches this node
            Node* parent = nullptr;

            // Heuristics calculations
            double h = std::numeric_limits<double>::max();
            double g = 0;
        };

		Board<Node>& board;
        
        // Collection of coordinates that create the solution of the A* algorithm.
		std::vector<Coordinate> solution;

	private:
        // Heuristic to calculate the distance between nodes
		Heuristic m_heuristic = Heuristic::Manhattan;

        // Whether you can move diagonally or not.
		bool m_diagonals = true;

        // Whether you can cut obstacles corners or not.
        bool m_cutCorners = true;

        // Collection of nodes yet to be evaluated.
        std::vector<Node*> m_openNodes;

	public:
		AStar(Board<Node>& board) : board(board)
        {
            m_openNodes.emplace_back(&board.getNode(board.start));
        }

        /**
         * Instantiates the variables of the A* algorithm.
         *
         * @param board Board on which the nodes are stored.
         * @param heuristic Heuristic to calculate the distance between nodes.
         */
        void init(const Heuristic& heuristic, const bool& diagonals = true, const bool& cutCorners = true)
        {
            solution.clear();
            m_heuristic = heuristic;
            m_diagonals = diagonals;
            m_cutCorners = cutCorners;
        }

        /**
         * Executes the next step in the A* algorithm if it hasn't finished yet.
         *
         * @return if the A* algorithm has finished its execution.
         */
		bool nextStep()
        {
            while(!evaluateOpenNodes());

            // Store the solution
            if (finished())
            {
                auto* node = &board.getNode(board.goal);
                while (node && (node->parent || node->position != board.goal))
                {
                    solution.emplace_back(node->position);
                    node = node->parent;
                }
                return true;
            }
            return false;
        }

        /**
         * Completes the execution of the A* algorithm.
         */
        void complete()
        {
            while (!nextStep());
        }

        /**
         * Checks if the A* algorithm has finished its execution.
         *
         * @return if the A* algorithm has finished its execution.
         */
        bool finished()
        {
            return board[board.goal].parent || !m_openNodes.size();
        }

	private:
        bool evaluateOpenNodes()
        {
            if (finished()) return true;

            bool changes = false;

            sortNodes(m_openNodes);

            auto* parent = m_openNodes.back();
            m_openNodes.pop_back();
            parent->closed = true;

            return evaluateNeighbors(parent);
        }

        bool evaluateNeighbors(Node* parent)
        {
            bool changes = false;

            std::vector<Coordinate> neighbors;
            board.getNeighbors(parent->position, neighbors, m_diagonals);

            for (auto& position : neighbors)
            {
                if (isNodeAccesible(parent->position, position) && !board[position].closed)
                {
                    auto& neighbor = board[position];
                    updateNode(parent, neighbor);

                    // It it's not in the open list
                    if (!contains(neighbor, m_openNodes))
                        m_openNodes.push_back(&neighbor);

                    changes = true;
                }

                if (position == board.goal) return true;
            }

            return changes;
        }

        bool isNodeAccesible(const Coordinate& from, const Coordinate& to)
        {
            if (!board.validNode(to) || !board.validNode(from)) return false;

            // Store the evaluations in variables to simplify the check visualization
            const bool contiguous1 = !board[{ from.first, to.second }].obstacle;
            const bool contiguous2 = !board[{ to.first, from.second }].obstacle;
            const bool contiguousClear = contiguous1 && contiguous2;
            const bool canCutCorner = m_cutCorners && (contiguous1 || contiguous2);


            // The node has to be valid and not a obstacle, not a diagonal
            // Or we can use diagonals and not both contiguous are obstacles
            // Or we can use diagonals, cut on corners and at least one contiguous is not an obstacle
            return !board[to].obstacle && (!Utils::isDiagonal(from, to) || (m_diagonals && (contiguousClear || canCutCorner)));
        }

        void updateNode(Node* parent, Node& neighbor)
        {
            auto h = distance(neighbor.position, board.goal);
            auto g = parent->g + distance(parent->position, neighbor.position);

            if (h + g < neighbor.h + neighbor.g)
            {
                neighbor.parent = parent;
                neighbor.h = h;
                neighbor.g = g;
            }
        }

        bool contains(const Node& node, const std::vector<Node*>& nodes)
        {
            for (auto* nodeAux : nodes)
                if (&node == nodeAux)
                    return true;

            return false;
        }

        void sortNodes(std::vector<Node*>& openNodes)
        {
            std::sort(openNodes.begin(), openNodes.end(), [](const Node* node1, const Node* node2)
                {
                    if (node1->h + node1->g == node2->h + node2->g)
                        return node1->h > node2->h;
                    else
                        return node1->h + node1->g > node2->h + node2->g;
                });
        }

        double distance(const Coordinate& curr, const Coordinate& goal)
        {
            switch (m_heuristic)
            {
            case Heuristic::Manhattan:
                return manhattanDistance(curr, goal);

            case Heuristic::Diagonal:
                return diagonalDistance(curr, goal);

            case Heuristic::Euclidean:
                return euclideanDistance(curr, goal);
            }
            return -1.f;
        }

        double manhattanDistance(const Coordinate& curr, const Coordinate& goal)
        {
            return std::abs(curr.first - goal.first) + std::abs(curr.second - goal.second);
        }

        double diagonalDistance(const std::pair<int, int>& curr, const std::pair<int, int>& goal)
        {
            //auto D = 1;
            //auto D2 = std::sqrt(2);
            auto D2 = 1.414f;
            auto dx = std::abs(curr.first - goal.first);
            auto dy = std::abs(curr.second - goal.second);
            // return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
            return (dx + dy) + (D2 - 2) * std::min(dx, dy);
        }

        double euclideanDistance(const Coordinate& curr, const Coordinate& goal)
        {
            return std::sqrt(std::pow(curr.first - goal.first, 2) + std::pow(curr.second - goal.second, 2));
        }
	};
}

#endif