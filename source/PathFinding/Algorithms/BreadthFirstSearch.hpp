#ifndef BREADTH_FIRST_SEARCH_H
#define BREADTH_FIRST_SEARCH_H
#pragma once

#include <queue>
#include "AlgorithmBase.hpp"


namespace PathFinding
{
    class BreadthFirstSearch : public AlgorithmBase
    {
        struct Node : NodeBase {};
        
    protected:
        /// Collection of nodes yet to be evaluated.
        std::queue<Node*> m_openNodes;
        
        /// Node's board
        Board<Node> m_board;

    public:
        /**
         * Create a board with the size, start, and goal position given
         *
         * @param size Size of the board
         * @param start Start position of the board
         * @param goal Goal position of the board
         */
        BreadthFirstSearch(const Coordinate& size, const Coordinate& start, const Coordinate& goal)
        : m_board{size, start, goal}, AlgorithmBase("Breadth First Search") {}
        
        virtual ~BreadthFirstSearch(){}
        
        /**
         * Instantiates the variables of the algorithm.
         *
         * @param heuristic Heuristic to calculate the distance between nodes.
         * @param diagonals Board on which the nodes are stored.
         * @param cutCorners Board on which the nodes are stored.
         */
        void init(const Heuristic heuristic, const bool diagonals = true, const bool cutCorners = true) override
        {
            AlgorithmBase::init(heuristic, diagonals, cutCorners);
            
            while(m_openNodes.size())
            {
                m_openNodes.pop();
            }
            
            m_board.init();
            
            openNode(m_board.getStartNode());
        }
        
        virtual void reset() override
        {
            AlgorithmBase::reset();
            
            while(m_openNodes.size())
            {
                m_openNodes.pop();
            }
            
            m_board.reset();
        }
        
        inline const size_t getTotalSize() const override { return m_board.getTotalSize(); }
        
        inline const Coordinate getSize() const override { return m_board.getSize(); }
        
        inline const Coordinate getStart() const override { return m_board.getStart(); }
        
        inline const Coordinate getGoal() const override { return m_board.getGoal(); }
        
        inline const NodeBase* getGoalNode() const override { return m_board.getGoalNode(); }
        
        inline const bool isObstacle(const Coordinate& c) const override { return m_board.isObstacle(c); }
        
        inline const void setObstacle(const Coordinate& c, const bool obstacle = true) const override
        {
            m_board.setObstacle(c, obstacle);
        }
        
        //TODO: Remove
        inline double getValue1(const Coordinate& c) const override
        {
            return m_board.isValidNode(c) && m_board[c]->opened ? 111.f : 0.f;
        }
        
        //TODO: Remove
        inline double getValue2(const Coordinate& c) const override
        {
            return 0.f;
        }
        
        bool nextStep() override
        {
            if (!m_openNodes.size()) return true;
            
            auto* node = closeNode();
            
            if (checkGoal(node->position)) return true;
            
            std::vector<Node*> neighbors;
            m_board.getNeighbors(node, neighbors, m_diagonals, m_cutCorners);
            for (auto* neighbor : neighbors)
            {
                if (neighbor->opened) continue;
                
                openNode(neighbor, node);
                
                if (checkGoal(neighbor->position)) return true;
            }
            
            return false;
        }
        
    protected:
        
        void openNode(Node* node, Node* parent = nullptr)
        {
            AlgorithmBase::openNode(node, parent);
            
            m_openNodes.push(node);
        }
        
        Node* closeNode()
        {
            ++nodesClosed;
            
            auto* node = m_openNodes.front();
            m_openNodes.pop();
            
            return node;
        }
    };
}

#endif
