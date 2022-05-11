#ifndef BOARD_H
#define BOARD_H
#pragma once

#include <vector>
#include "Utils.h"


namespace PathFinding
{
    // Define the coordinate type
    typedef std::pair<int, int>         Coordinate;

    // Define the node base struct
    struct NodeBase
    {
        bool obstacle = false;
        Coordinate position{ 0, 0 };
    };

    template <Derived<NodeBase>T>
    class Board
    {
    public:

        const Coordinate max;
        const Coordinate start;
        const Coordinate goal;

    protected:
        const size_t m_size = max.first * max.second;
        std::vector<std::vector<T>> m_nodes;

    public:
        Board(Coordinate max, Coordinate start, Coordinate goal)
            : max(max), m_size(max.first* max.second), m_nodes(max.first), start(start), goal(goal)
        {
            for (size_t i = 0; i < max.first; ++i)
            {
                m_nodes[i] = std::vector<T>(max.second);
                for (size_t j = 0; j < max.second; ++j)
                {
                    m_nodes[i][j].position = {i, j};
                }
            }
        }

        auto& operator[](const size_t& i) { return m_nodes[i]; }

        const auto& operator[](const size_t& i) const { return m_nodes[i]; }

        T& operator[](const Coordinate& c) { return m_nodes[c.first][c.second]; }

        const T& operator[](const Coordinate& c) const { return m_nodes[c.first][c.second]; }

        size_t size() const { return m_size; };

        const bool validNode(const Coordinate& c) const {
            return c.first >= 0 && c.first < max.first&& c.second >= 0 && c.second < max.second;
        }

        T& getNode(const Coordinate& c) { return m_nodes[c.first][c.second]; }

        const auto& getNode(const Coordinate& c) const { return m_nodes[c.first][c.second]; }

        void getNeighbors(const Coordinate& node, std::vector<Coordinate>& neighbors, const bool& diagonals)
        {
            if (diagonals)
            {
                neighbors.reserve(8);
                getNeighborsWithDiagonals(node, neighbors);
            }
            else
            {
                neighbors.reserve(4);
                getNeighborsWithoutDiagonals(node, neighbors);
            }
        }

    protected:
        void getNeighborsWithoutDiagonals(const Coordinate& node, std::vector<Coordinate>& neighbors)
        {
            // Direct neighbors
            neighbors.emplace_back(node.first, node.second - 1);        // North
            neighbors.emplace_back(node.first + 1, node.second);        // East
            neighbors.emplace_back(node.first, node.second + 1);        // South
            neighbors.emplace_back(node.first - 1, node.second);        // West
        }

        void getNeighborsWithDiagonals(const Coordinate& node, std::vector<Coordinate>& neighbors)
        {
            getNeighborsWithoutDiagonals(node, neighbors);

            // Diagonals
            neighbors.emplace_back(node.first + 1, node.second - 1);    // North East
            neighbors.emplace_back(node.first + 1, node.second + 1);    // South East
            neighbors.emplace_back(node.first - 1, node.second + 1);    // South West
            neighbors.emplace_back(node.first - 1, node.second - 1);    // North West
        }
    };
}

#endif