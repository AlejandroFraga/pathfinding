#ifndef CONSOLE_OUTPUT_H
#define CONSOLE_OUTPUT_H
#pragma once

#include <iostream>
#include "../Algorithms/AlgorithmBase.hpp"
#include "../Board.hpp"


namespace PathFinding
{
	enum class RelPos : int
	{
		NorthWest = 1,
		North = 2,
		NorthEast = 3,
		West = 4,
		Same = 5,
		East = 6,
		SouthWest = 7,
		South = 8,
		SouthEast = 9
	};

	enum class Char : char
	{
        // Support for MacOS terminal
#if __APPLE__
        Space = 32,
        Slash = 47,
        Zero = 48,
        Less = 60,
        Greater = 62,
        Start = 65,
        Goal = 66,
        Backslash = 92,
        Caret = 94,
        LowercaseV = 118,
        Vertical = 124,
        VerticalLeft = 43,
        UpperRight = 43,
        LowerLeft = 43,
        HorizontalUp = 43,
        HorizontalDown = 43,
        VerticalRight = 43,
        Horizontal = 45,
        Cross = 43,
        LowerRight = 43,
        UpperLeft = 43,
        Block = 35
#else
		Space = 32,
		Slash = 47,
		Zero = 48,
		Less = 60,
		Greater = 62,
		Start = 65,
		Goal = 66,
		Backslash = 92,
		Caret = 94,
		LowercaseV = 118,
		Vertical = 179,
		VerticalLeft = 180,
		UpperRight = 191,
		LowerLeft = 192,
		HorizontalUp = 193,
		HorizontalDown = 194,
		VerticalRight = 195,
		Horizontal = 196,
		Cross = 197,
		LowerRight = 217,
		UpperLeft = 218,
		Block = 219
#endif
	};

	class ConsoleOutput
	{
	private:
		/// Works better with odd numbers
		static const size_t m_nodeChars = 3;

	public:
		static void Print(const AlgorithmBase& algorithm, const bool& clear = true, const bool& enterToContinue = false)
		{
            std::cout << "Algorithm: " << algorithm.name << std::endl;
            std::cout << "Nodes opened: " << algorithm.nodesOpened << " - Nodes closed: " << algorithm.nodesClosed << std::endl;
            if (algorithm.solution.size())
            {
                std::cout << "Solution (distance: " << algorithm.getSolutionDistance() << "):" << std::endl;
            }
            else
            {
                std::cout << "No solution:" << std::endl;
            }

			// Calculate the size by axis of the board
			const Coordinate size(algorithm.getSize().first * (m_nodeChars + 1) + 1, algorithm.getSize().second * (m_nodeChars + 1) + 1);

			// Stores the total position of the std::cout
			Coordinate i;
			for (i.second = 0; i.second < size.second; ++i.second)
			{
				for (i.first = 0; i.first < size.first; ++i.first)
				{
					std::cout << getRepresentation(algorithm, size, i);
				}
				std::cout << std::endl;
			}

			if (enterToContinue)
				EnterToContinue(clear);
		}

		static void ClearScreen()
        {
#ifndef __APPLE__
            system("CLS");
#endif
        }

    private:
		static char getRepresentation(const AlgorithmBase& algorithm, const Coordinate& size, const Coordinate& coutPos)
		{
			// Actual node position of the board
			Coordinate boardPos{ coutPos.first / (m_nodeChars + 1), coutPos.second / (m_nodeChars + 1) };

			// Position inside the m_nodeChars of each node representation
			const Coordinate nodeC{ coutPos.first % (m_nodeChars + 1) - 1, coutPos.second % (m_nodeChars + 1) - 1 };

			// Try get if it's a border or a main element to represent
			auto nextChar = getBorder(coutPos, size);
			nextChar = nextChar == Char::Space ? getMain(algorithm, boardPos, nodeC) : nextChar;

			// If a solution is passed, print the solution. If not, print the value of the node
			if (algorithm.solution.size())
				return nextChar == Char::Space ? (char)getSolutionPath(algorithm.solution, boardPos, nodeC) : (char)nextChar;
			else
				return nextChar == Char::Space ? getNodeValue(algorithm.getValue1(boardPos), algorithm.getValue2(boardPos), nodeC) : (char)nextChar;

			return (char)nextChar;
		}

		static Char getBorder(const Coordinate& globalC, const Coordinate& size)
		{
			const bool oddColumn = globalC.first % (m_nodeChars + 1);
			const bool oddRow = globalC.second % (m_nodeChars + 1);

			// Upper left corner
			if (globalC.first == 0 && globalC.second == 0)
				return Char::UpperLeft;

			// Upper right corner
			else if (globalC.first == size.first - 1 && globalC.second == 0)
				return Char::UpperRight;

			// Lower left corner
			else if (globalC.first == 0 && globalC.second == size.second - 1)
				return Char::LowerLeft;

			// Lower right corner
			else if (globalC.first == size.first - 1 && globalC.second == size.second - 1)
				return Char::LowerRight;

			// Upper border even
			else if (globalC.second == 0 && !oddColumn)
				return Char::HorizontalDown;

			// Upper border odd
			else if (globalC.second == 0 && oddColumn)
				return Char::Horizontal;

			// Left border even
			else if (globalC.first == 0 && !oddRow)
				return Char::VerticalRight;

			// Left border odd
			else if (globalC.first == 0 && oddRow)
				return Char::Vertical;

			// Lower border even
			else if (globalC.second == size.second - 1 && !oddColumn)
				return Char::HorizontalUp;

			// Lower border odd
			else if (globalC.second == size.second - 1 && oddColumn)
				return Char::Horizontal;

			// Right border even
			else if (globalC.first == size.first - 1 && !oddRow)
				return Char::VerticalLeft;

			// Right border odd
			else if (globalC.first == size.first - 1 && oddRow)
				return Char::Vertical;

			// Inner horizontals
			else if (!oddRow && oddColumn)
				return Char::Horizontal;

			// Inner verticals
			else if (oddRow && !oddColumn)
				return Char::Vertical;

			// Crosses
			else if (!oddColumn)
				return Char::Cross;

			return Char::Space;
		}
        
		static Char getMain(const AlgorithmBase& algorithm, const Coordinate& localC, const Coordinate& nodeC)
		{
			// Obstacles
			if (algorithm.isObstacle(localC))
				return Char::Block;

			// Start
			else if (localC == algorithm.getStart() && nodeC.first == m_nodeChars / 2 && nodeC.first == nodeC.second)
				return Char::Start;

			// Goal
			else if (localC == algorithm.getGoal() && nodeC.first == m_nodeChars / 2 && nodeC.first == nodeC.second)
				return Char::Goal;

			return Char::Space;
		}

		static Char getSolutionPath(const std::vector<Coordinate>& solution, const Coordinate& localC, const Coordinate& nodeC)
		{
			const auto halfNodeChars = m_nodeChars / 2;
			const bool centralColumn = nodeC.first == halfNodeChars;				// Central column
			const bool centralRow = nodeC.second == halfNodeChars;					// Central row
			const bool tlDiagonal = nodeC.first == nodeC.second;					// Top left to bottom right diagonal
			const bool blDiagonal = nodeC.first == m_nodeChars - 1 - nodeC.second;	// Bottom left to top right diagonal

			auto resultIndex = solution.size();
            
            for (size_t i = 0; i < solution.size(); ++i)
            {
                if (localC == solution[i])
                {
                    resultIndex = i;
                }
            }

			// Solution path
			if (resultIndex && resultIndex < solution.size() - 1)
			{
				const Coordinate diffBefore(solution[resultIndex + 1].first - solution[resultIndex].first,
					solution[resultIndex + 1].second - solution[resultIndex].second);

				const Coordinate diffAfter(solution[resultIndex - 1].first - solution[resultIndex].first,
					solution[resultIndex - 1].second - solution[resultIndex].second);

				auto relBefore = getRelPos(diffBefore);
				auto relAfter = getRelPos(diffAfter);

				if (((relBefore == RelPos::NorthWest || relAfter == RelPos::NorthWest) && nodeC.second < halfNodeChars && tlDiagonal)
					|| ((relBefore == RelPos::SouthEast || relAfter == RelPos::SouthEast) && nodeC.second > halfNodeChars && tlDiagonal))
					return Char::Backslash;

				else if (((relBefore == RelPos::North || relAfter == RelPos::North) && nodeC.second < halfNodeChars && centralColumn)
					|| ((relBefore == RelPos::South || relAfter == RelPos::South) && nodeC.second > halfNodeChars && centralColumn))
					return Char::Vertical;

				else if (((relBefore == RelPos::NorthEast || relAfter == RelPos::NorthEast) && nodeC.second < halfNodeChars && blDiagonal)
					|| ((relBefore == RelPos::SouthWest || relAfter == RelPos::SouthWest) && nodeC.second > halfNodeChars && blDiagonal))
					return Char::Slash;

				else if (((relBefore == RelPos::East || relAfter == RelPos::East) && nodeC.first > halfNodeChars && centralRow)
					|| ((relBefore == RelPos::West || relAfter == RelPos::West) && nodeC.first < halfNodeChars && centralRow))
					return Char::Horizontal;

				// Central part
				else if (centralColumn && centralRow)
					return getSolutionPathCenter(relBefore, relAfter);
			}

			return Char::Space;
		}

		static Char getSolutionPathCenter(const RelPos& relBefore, const RelPos& relAfter)
		{
			auto relInt = 0;

			if (relBefore < relAfter)
				relInt = (int)relBefore * 10 + (int)relAfter;
			else
				relInt = (int)relAfter * 10 + (int)relBefore;

			switch (relInt)
			{
				case 11:	// NorthWest	NorthWest
				case 19:	// NorthWest	SouthEast
				case 99:	// SouthEast	SouthEast
					return Char::Backslash;

				case 12:	// NorthWest	North
				case 13:	// NorthWest	NorthEast
				case 23:	// North		NorthEast
					return Char::LowercaseV;

				case 14:	// NorthWest	West
				case 17:	// NorthWest	SouthWest
				case 47:	// West			SouthWest
					return Char::Less;

				case 16:	// NorthWest	East
				case 34:	// NorthEast	West
				case 44:	// West			West
				case 46:	// West			East
				case 49:	// West			SouthEast
				case 66:	// East			East
				case 67:	// East			SouthWest
					return Char::Horizontal;

				case 18:	// NorthWest	South
				case 22:	// North		North
				case 27:	// North		SouthWest
				case 28:	// North		South
				case 29:	// North		SouthEast
				case 38:	// NorthEast	South
				case 88:	// South		South
					return Char::Vertical;

				case 24:	// North		West
					return Char::LowerRight;

				case 26:	// North		East
					return Char::LowerLeft;

				case 33:	// NorthEast	NorthEast
				case 37:	// NorthEast	SouthWest
				case 77:	// SouthWest	SouthWest
					return Char::Slash;

				case 36:	// NorthEast	East
				case 39:	// NorthEast	SouthEast
				case 69:	// East			SouthEast
					return Char::Less;

				case 48:	// West			South
					return Char::UpperRight;

				case 68:	// East			South
					return Char::UpperLeft;

				case 78:	// SouthWest	South
				case 79:	// SouthWest	SouthEast
				case 89:	// South		SouthEast
					return Char::Caret;
			}

			return Char::Space;
		}

		static RelPos getRelPos(const Coordinate& diff)
		{
			const int first = diff.first ? diff.first / std::abs(diff.first) : 0;
			const int second = diff.second ? diff.second / std::abs(diff.second) : 0;

			switch (first)
			{
				case -1:
					switch (second)
					{
						case -1:
							return RelPos::NorthWest;
						case 0:
							return RelPos::West;
						case 1:
							return RelPos::SouthWest;
					}
				case 0:
					switch (diff.second)
					{
						case -1:
							return RelPos::North;
						case 0:
							return RelPos::Same;
						case 1:
							return RelPos::South;
					}
				case 1:
					switch (diff.second)
					{
						case -1:
							return RelPos::NorthEast;
						case 0:
							return RelPos::East;
						case 1:
							return RelPos::SouthEast;
					}
			}

			return RelPos::Same;
		}
        
        //TODO: Improve
		static char getNodeValue(double value1, double value2, const Coordinate& NodeC)
		{
            if (value1 == 0.f && value2 == 0.f)
            {
                return (int)Char::Space;
            }
            
			auto units = (int)std::pow(10, m_nodeChars - 1 - NodeC.first);

			// Regular nodes -> h X value
			if (NodeC.second == 0 && value1 <= 999)
			{
				if (NodeC.first == m_nodeChars - 1)
                    return ((int)Char::Zero + ((int)value1 % 10));
				else
                    return ((int)Char::Zero + ((int)value1 % (units * 10) - (int)value1 % units) / units);
			}
            else if (NodeC.second == m_nodeChars - 1 && value2 > 0)
            {
                if (NodeC.first == m_nodeChars - 1)
                    return ((int)Char::Zero + (int)value2 % 10);
                else
                    return ((int)Char::Zero + ((int)value2 % (units * 10) - (int)value2 % units) / units);
            }

			return (int)Char::Space;
		}

		static void EnterToContinue(bool clear = false)
		{
			std::cout << "Press enter to continue...";
			std::cin.get();

			if (clear)
				ClearScreen();
		}
	};
}
#endif
