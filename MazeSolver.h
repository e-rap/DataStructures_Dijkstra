#pragma once

#include <vector>



// Maze is a 2D array containing walls, and passes with the different levels of difficulty.
// A wall square has a value -999
// Other squares can be a grass, trees, mud, secret passages etc.
// For each square there is assigned difficulty to cross it.
using Maze = std::vector<std::vector<int>>;

// Coordinates of a square (row, column)
using Coord = std::pair<int, int>;
// Path is a sequence of square coordinates.
using Path = std::vector<Coord>;


// Function should return the shortest path in a maze from a start to the goal square.
// The traveller can move in eight directions.
//
// The output should include the start and the goal squares.
// Example:
//
// Given a maze with (0,0) at the left top corner
//  1  3    4
//  2 -999  -1
//  4  1    0
// The shortest path from (0, 0) to (2, 2) is {{0,0}, {0, 1}, {1, 2}, {2, 2} }

Path GetShortestPath(const Maze& maze, const Coord& start, const Coord& goal);