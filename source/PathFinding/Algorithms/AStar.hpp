#ifndef A_STAR_H
#define A_STAR_H
#pragma once

#include "AlgorithmBase.hpp"


namespace PathFinding
{
    class AStar : public AlgorithmBase
    {
        struct Node : NodeBase
        {
            /// Whether the node has been evaluated or not.
            bool closed = false;

            /// Heuristics calculations
            double h = std::numeric_limits<double>::max();
            double g = 0;
            
            virtual void init() override
            {
                NodeBase::init();
                opened = false;
                h = std::numeric_limits<double>::max();
                g = 0;
            }
            
            virtual void reset() override
            {
                NodeBase::reset();
                opened = false;
                h = std::numeric_limits<double>::max();
                g = 0;
            }
        };
        
    protected:
        /// Collection of nodes yet to be evaluated.
        std::vector<Node*> m_openNodes;
        
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
        AStar(const Coordinate& size, const Coordinate& start, const Coordinate& goal)
        : m_board{size, start, goal}, AlgorithmBase("A*") {}
        
        virtual ~AStar(){}
        
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
            
            m_openNodes.clear();
            
            m_board.init();
            
            openNode(m_board.getStartNode());
        }
        
        virtual void reset() override
        {
            AlgorithmBase::reset();
            
            m_openNodes.clear();
            
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
            return m_board.isValidNode(c) ? m_board[c]->h : 0.f;
        }
        
        //TODO: Remove
        inline double getValue2(const Coordinate& c) const override
        {
            return m_board.isValidNode(c) ? m_board[c]->g : 0.f;
        }
        
        bool nextStep() override
        {
            if (!m_openNodes.size()) return true;
            
            sortNodes(m_openNodes);
            
            auto* node = closeNode();
            
            if (checkGoal(node->position)) return true;
            
            std::vector<Node*> neighbors;
            m_board.getNeighbors(node, neighbors, m_diagonals, m_cutCorners);
            
            for (auto* neighbor : neighbors)
            {
                if (neighbor->closed) continue;
                
                updateNode(neighbor, node);
                
                if (neighbor->opened) continue;
                
                openNode(neighbor, node);
                
                if (checkGoal(neighbor->position)) return true;
            }
            
            return false;
        }
        
    protected:
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
        
        void openNode(Node* node, Node* parent = nullptr)
        {
            AlgorithmBase::openNode(node, parent);
            
            m_openNodes.push_back(node);
        }
        
        void updateNode(Node* neighbor, Node* parent)
        {
            auto h = distance(neighbor->position, m_board.getGoal());
            auto g = parent->g + distance(parent->position, neighbor->position);

            if (h + g < neighbor->h + neighbor->g)
            {
                neighbor->parent = parent;
                neighbor->h = h;
                neighbor->g = g;
            }
        }
        
        Node* closeNode()
        {
            ++nodesClosed;
            
            auto* node = m_openNodes.back();
            m_openNodes.pop_back();
            node->closed = true;
            
            return node;
        }
    };
}

#endif
