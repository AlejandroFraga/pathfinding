#ifndef BOARD_H
#define BOARD_H
#pragma once

#include <vector>


namespace PathFinding
{
    template<class T, class U>
    concept Derived = std::is_base_of<U, T>::value;

    /// Define the coordinate type
    typedef std::pair<int, int> Coordinate;

    /// Define the node base struct
    struct NodeBase
    {
        /// The previous node from which the shortest path reaches this node
        NodeBase* parent = nullptr;
        
        /// Position of the Node
        Coordinate position{ 0, 0 };
        
        /// Whether the node is an obstacle or not
        bool obstacle = false;
        
        /// Whether the node has been stored to be evaluated or not
        bool opened = false;
        
        inline bool operator==(const NodeBase& node) { return position == node.position; }
        
        inline bool operator!=(const NodeBase& node) { return !(*this == node); }
        
        virtual void init()
        {
            parent = nullptr;
            opened = false;
        }
        
        virtual void reset()
        {
            init();
            obstacle = false;
        }
    };
    
    template <Derived<NodeBase>T>
    class Board
    {
        
    protected:
        /// Total size of the Board
        const size_t m_totalSize = m_size.first * m_size.second;
        
        /// Collection of all nodes of the Board
        std::vector<std::vector<T*>> m_nodes;
        
    private:
        /// Whether the node has been stored to be evaluated or not
        const Coordinate m_size;
        
        /// Whether the node has been stored to be evaluated or not
        const Coordinate m_start;
        
        /// Whether the node has been stored to be evaluated or not
        const Coordinate m_goal;

    public:
        /**
         * Create a board with the size, start, and goal position given
         *
         * @param size Size of the board
         * @param start Start position of the board
         * @param goal Goal position of the board
         */
        Board(const Coordinate& size, const Coordinate& start, const Coordinate& goal)
            : m_size(size), m_totalSize(size.first * size.second), m_nodes(size.first), m_start(start), m_goal(goal)
        {
            for (size_t i = 0; i < m_size.first; ++i)
            {
                // Create the rows
                m_nodes[i] = std::vector<T*>(m_size.second);
                for (size_t j = 0; j < m_size.second; ++j)
                {
                    // Create the nodes and set its position
                    m_nodes[i][j] = new T();
                    m_nodes[i][j]->position = {i, j};
                }
            }
        }
        
        virtual ~Board()
        {
            for (size_t i = 0; i < m_size.first; ++i)
            {
                for (size_t j = 0; j < m_size.second; ++j)
                {
                    // Free the memory of every node on the board
                    delete(m_nodes[i][j]);
                }
            }
        }
        
        /**
         * Call init on every node of the Board
         */
        void init()
        {
            for (size_t i = 0; i < m_size.first; ++i)
            {
                for (size_t j = 0; j < m_size.second; ++j)
                {
                    m_nodes[i][j]->init();
                }
            }
        }
        
        /**
         * Call reset on every node of the Board
         */
        void reset()
        {
            for (size_t i = 0; i < m_size.first; ++i)
            {
                for (size_t j = 0; j < m_size.second; ++j)
                {
                    m_nodes[i][j]->reset();
                }
            }
        }

        auto& operator[](const size_t i) { return i < m_size.first ? m_nodes[i] : nullptr; }

        const auto& operator[](const size_t i) const { return i < m_size.first ? m_nodes[i] : nullptr; }

        T* operator[](const Coordinate& c)
        {
            return isValidNode(c) ? m_nodes[c.first][c.second] : nullptr;
        }

        const T* operator[](const Coordinate& c) const
        {
            return isValidNode(c) ? m_nodes[c.first][c.second] : nullptr;
        }
        
        /**
         * Get the size of the board
         *
         * @return The size of the board
         */
        size_t getTotalSize() const { return m_totalSize; };
        
        /**
         * Returns the size of the board by axis
         *
         * @return Size of the board by axis
         */
        inline const Coordinate getSize() const { return m_size; }
        
        /**
         * Returns the position of the start node
         *
         * @return Position of the start node
         */
        inline const Coordinate getStart() const { return m_start; }
        
        /**
         * Returns the start node
         *
         * @return The start node
         */
        inline T* getStartNode()
        {
            return isValidNode(m_start) ? m_nodes[m_start.first][m_start.second] : nullptr;
        }
        
        /**
         * Returns the start node
         *
         * @return The start node
         */
        inline const T* getStartNode() const
        {
            return isValidNode(m_start) ? m_nodes[m_start.first][m_start.second] : nullptr;
            
        }
        
        /**
         * Returns the position of the goal node
         *
         * @return Position of the goal node
         */
        inline const Coordinate getGoal() const { return m_goal; }
        
        /**
         * Returns the goal node
         *
         * @return The goal node
         */
        inline T* getGoalNode()
        {
            return isValidNode(m_goal) ? m_nodes[m_goal.first][m_goal.second] : nullptr;
        }
        
        /**
         * Returns the goal node
         *
         * @return The goal node
         */
        inline const T* getGoalNode() const
        {
            return isValidNode(m_goal) ? m_nodes[m_goal.first][m_goal.second] : nullptr;
            
        }
        
        /**
         * Checks if the position is within the limits of the board
         *
         * @return If the position is within the limits of the board
         */
        inline const bool isValidNode(const Coordinate& c) const {
            return c.first >= 0 && c.first < m_size.first && c.second >= 0 && c.second < m_size.second;
        }
        
        /**
         * Checks if the position is an obstacle
         *
         * @return If the position is an obstacle
         */
        inline const bool isObstacle(const Coordinate& c) const {
            return isValidNode(c) && operator[](c)->obstacle;
        }
        
        /**
         * Sets the node at the coordinate given as an obstacle or not
         *
         * @param c Coordinate in which the obstacle field is modified
         * @param obstacle Wether we set it to be an obstacle or not
         */
        const void setObstacle(const Coordinate& c, const bool obstacle = true) const
        {
            if (isValidNode(c) && c != m_start && c != m_goal)
            {
                m_nodes[c.first][c.second]->obstacle = obstacle;
            }
        }
        
        /**
         * Adds the accessible neighbors to the collection given taking into account if diagonals can be used and/or can cut corners
         *
         * @param node Node from which the neighbors are returned
         * @param neighbors Reference to the collection of neighbors to which they are added
         * @param diagonals The algorithm can use diagonal movements
         * @param cutCorners The algorithm can cut corners when passing next to an obstacle
         */
        void getNeighbors(T* node, std::vector<T*>& neighbors, const bool diagonals, const bool cutCorners) const
        {
            if (!diagonals)
            {
                neighbors.reserve(4);
                getNeighborsWithoutDiagonals(node, neighbors);
            }
            else
            {
                neighbors.reserve(8);
                getNeighborsWithDiagonals(node, neighbors, cutCorners);
            }
        }

    protected:
        /**
         * Get the neighbors of the given node given this order:
         *
         *      Order:
         *  +---+---+---+
         *  |   | 1 |   |
         *  +---+---+---+
         *  | 4 |   | 2 |
         *  +---+---+---+
         *  |   | 3 |   |
         *  +---+---+---+
         *
         * @param node Node from which the neighbors are returned
         * @param neighbors Reference to the collection of neighbors to which they are added
         */
        void getNeighborsWithoutDiagonals(T* node, std::vector<T*>& neighbors) const
        {
            auto first = node->position.first;
            auto second = node->position.second;
            
            addIfAccessible(node, {first, second - 1}, neighbors);  // North
            addIfAccessible(node, {first + 1, second}, neighbors);  // East
            addIfAccessible(node, {first, second + 1}, neighbors);  // South
            addIfAccessible(node, {first - 1, second}, neighbors);  // West
        }
        
        /**
         * Get the neighbors including diagonals given this order:
         *
         *      Order:
         *  +---+---+---+
         *  | 5 | 1 | 6 |
         *  +---+---+---+
         *  | 4 |   | 2 |
         *  +---+---+---+
         *  | 8 | 3 | 7 |
         *  +---+---+---+
         *
         * @param node Node from which the neighbors are returned
         * @param neighbors Reference to the collection of neighbors to which they are added
         * @param cutCorners If neighbors to which we have to cut corners are also added to the collection
         */
        void getNeighborsWithDiagonals(T* node, std::vector<T*>& neighbors, const bool cutCorners) const
        {
            getNeighborsWithoutDiagonals(node, neighbors);
            
            auto first = node->position.first;
            auto second = node->position.second;
            
            addIfAccessible(node, {first - 1, second - 1}, neighbors, cutCorners);  // North West
            addIfAccessible(node, {first + 1, second - 1}, neighbors, cutCorners);  // North East
            addIfAccessible(node, {first + 1, second + 1}, neighbors, cutCorners);  // South East
            addIfAccessible(node, {first - 1, second + 1}, neighbors, cutCorners);  // South West
        }
        
    private:
        /**
         * Sets the node at the coordinate given as an obstacle or not
         *
         * @param from First node
         * @param toPosition Second coordinate
         * @param neighbors Reference to the collection of neighbors to which they are added
         * @param cutCorners If neighbors to which we have to cut corners are also added to the collection
         */
        void addIfAccessible(T* from, const Coordinate& toPosition, std::vector<T*>& neighbors, const bool cutCorners = true) const
        {
            if (isValidNode(toPosition) && isAccessible(from->position, toPosition, cutCorners))
            {
               neighbors.emplace_back(m_nodes[toPosition.first][toPosition.second]);
            }
        }
        
        /**
         * Checks if the coordinates are not on the same x nor y axis, so, if they are neighbors, they are diagonally connected
         *
         * @param c1 First coordinate
         * @param c2 Second coordinate
         * @return If they are neighbors, it returns if they are diagonally connected
         */
        const bool isDiagonal(const Coordinate& c1, const Coordinate& c2) const
        {
            return c1.first != c2.first && c1.second != c2.second;
        }
        
        /**
         * Checks if the position is accessible from the first coordinate into the second taking into account if you can cut corners
         *
         * @param from Initial coordinate
         * @param to Final coordinate
         * @param cutCorners If we can cut corners
         * @return If the position is accessible from the first coordinate into the second taking into account if you can cut corners
         */
        const bool isAccessible(const Coordinate& from, const Coordinate& to, const bool cutCorners = true) const
        {
            // Not accesible if any of the nodes is not valid, or the node to go to is an obstacle
            if (!isValidNode(to) || !isValidNode(from) || m_nodes[to.first][to.second]->obstacle) return false;
            
            // If it is not a diagonal, it is accessible
            if (!isDiagonal(from, to)) return true;

            // If both contiguous are not obstacles, or only one is and we can cut corners, it is accesible
            const bool contiguous1 = !m_nodes[from.first][to.second]->obstacle;
            const bool contiguous2 = !m_nodes[to.first][from.second]->obstacle;
            return (contiguous1 && contiguous2) || (cutCorners && (contiguous1 || contiguous2));
        }
    };
}

#endif
