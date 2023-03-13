# Pathfinding

Documentation is a WIP.

Implementation of different pathfinding algorithms.

## Table of contents

- [How to use it](#how-to-use-it)
- [Algorithms](#algorithms)
  - [A*](#a)
  - [IDA*](#ida)
  - [Best First Search](#best-first-search)
  - [Breadth First Search](#breadth-first-search)
  - [Depth First Search](#depth-first-search)
  - [Dijkstra](#dijkstra)
  - [Jump Point Search](#jump-point-search)
  - [Orthogonal Jump Point Search](#orthogonal-jump-point-search)
  - [Trace](#trace)
- [Parameters](#parameters)
  - [Size](#size)
  - [Start](#start)
  - [Goal](#goal)
  - [Heuristics](#heuristics)
  - [Diagonals](#diagonals)
  - [Cut Corners](#cut-corners)
- [Outputs](#outputs)
  - [Console Output](#console-output)
- [To Do](#to-do)

## How to use it

## Algorithms

### A*

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/AStar.hpp#L114-L141

#### Bi-directional

Not yet implemented

### IDA*

Not yet implemented

#### Bi-directional

Not yet implemented

### Best First Search

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/BestFirstSearch.hpp#L106-L129

#### Bi-directional

Not yet implemented

### Breadth First Search

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/BreadthFirstSearch.hpp#L97-L117

#### Bi-directional

Not yet implemented

### Depth First Search

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/DepthFirstSearch.hpp#L97-L125

#### Bi-directional

Not yet implemented

### Dijkstra

Not yet implemented

#### Bi-directional

Not yet implemented

### Jump Point Search

Not yet implemented

### Orthogonal Jump Point Search

Not yet implemented

### Trace

Not yet implemented

## Parameters

### Size

### Start

### Goal

### Heuristics

| Algorithm | Affects |
| --- | --- |
| A* | ✓ |
| IDA* | ✓ |
| Best First Search | ✓ |
| Breadth First Search | ✗ |
| Depth First Search | ✗ |
| Dijkstra | ✗ |
| Jump Point Search | ✓ |
| Orthogonal Jump Point Search | ✓ |
| Trace | ✓ |

#### Manhattan

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/AlgorithmBase.hpp#L264-L267

#### Diagonal

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/AlgorithmBase.hpp#L276-L285

#### Euclidean

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/AlgorithmBase.hpp#L294-L297

#### Chebyshev

Not yet implemented

https://github.com/AlejandroFraga/pathfinding/blob/dcc955a94872d111c8f6c1ad306ba8c2163b8651/source/PathFinding/Algorithms/AlgorithmBase.hpp#L306-L310

### Diagonals

| Algorithm | Affects |
| --- | --- |
| A* | ✓ |
| IDA* | ✓ |
| Best First Search | ✓ |
| Breadth First Search | ✓ |
| Depth First Search | ✓ |
| Dijkstra | ✓ |
| Jump Point Search | ✗ |
| Orthogonal Jump Point Search | ✗ |
| Trace | ✓ |

### Cut Corners

| Algorithm | Affects |
| --- | --- |
| A* | ✓ |
| IDA* | ✓ |
| Best First Search | ✓ |
| Breadth First Search | ✓ |
| Depth First Search | ✓ |
| Dijkstra | ✓ |
| Jump Point Search | ✗ |
| Orthogonal Jump Point Search | ✗ |
| Trace | ✓ |

## Outputs

All the implemented interfaces to display the output 

### Console output

<img src="https://github.com/AlejandroFraga/pathfinding/blob/main/images/Console_Output.jpg" width="720"/>

#### No Solution Output

<img src="https://github.com/AlejandroFraga/pathfinding/blob/main/images/No_Solution.png" width="720"/>

#### Diagonals And Cut Corners Output

<img src="https://github.com/AlejandroFraga/pathfinding/blob/main/images/Diagonals_And_Cut_Corners.png" width="720"/>

#### Diagonals No Cut Corners Output

<img src="https://github.com/AlejandroFraga/pathfinding/blob/main/images/Diagonals_No_Cut_Corners.png" width="720"/>

#### No Diagonals Output

<img src="https://github.com/AlejandroFraga/pathfinding/blob/main/images/No_Diagonals.png" width="720"/>

## To Do

List of improvements to do

### Algorithms implementations

- [X] A*
- [ ] IDA*
- [X] Best First Search
- [X] Breadth First Search
- [X] Depth First Search
- [ ] Dijkstra
- [ ] Jump Point Search
- [ ] Orthogonal Jump Point Search
- [ ] Trace

### Algorithms bi-directional implementations

- [ ] Bi-directional A*
- [ ] Bi-directional IDA*
- [ ] Bi-directional Best First Search
- [ ] Bi-directional Breadth First Search
- [ ] Bi-directional Depth First Search
- [ ] Bi-directional Dijkstra

### Outputs

- [ ] Improve Console Output
- [ ] Web based output

### Miscellaneous

- [ ] Benchmark mode
