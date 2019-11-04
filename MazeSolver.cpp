#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>
#include <limits>
#include <algorithm>

#include<iostream>

#include "MazeSolver.h"


namespace Helpers
{
  const int WALL = -999;
  const Coord UNDEFINED{ -1,-1 };
  struct Node;

  struct Node
  {
    Node()
      : position{}, distance{}, previous{ UNDEFINED }
    {
    }
    Node(Coord position, int distance)
      : position{ position }, distance{ distance }, previous{ UNDEFINED }
    {
    }

    Coord position;
    int distance;
    Coord previous;

  };

  // function decls
  std::vector<Coord> get_neighbours(const Coord& pos_in, const Maze& grid);
  std::string coord_to_string(Coord coord);
  void update_distance(const Node& a, Node& b, const Maze& grid);
  void print_graph(const std::unordered_map<std::string, Node>& graph, const Maze& maze);

  void update_distance(const Node& a, Node& b, const Maze& grid)
  {
    int distance = a.distance + grid[b.position.first][b.position.second];
    if (distance < b.distance)
    {
      b.previous = a.position;
      b.distance = distance;
    }
  }

  /// returns all neighbouring positions
  std::vector<Coord> get_neighbours(const Coord& pos_in, const Maze& grid)
  {
    std::vector<Coord> output{};
    int m = grid.size();
    int n = grid[0].size();
    for (int i{ -1 }; i <= 1; ++i)
    {
      for (int j{ -1 }; j <= 1; ++j)
      {
        int y = pos_in.first + i;
        int x = pos_in.second + j;

        if (!((y < 0) || (y >= m) || (x < 0) || (x >= n) || (i == 0 && j == 0)))
        {
          if (grid[x][y] != WALL || grid[x][y] >= 0)
          {
            output.push_back(std::make_pair(y, x));
          }
        }
      }
    }
    return output;
  }

  /// converts coordinate to string value
  ///
  /// used to make coords hashible
  std::string coord_to_string(Coord coord)
  {
    std::stringstream ss;
    ss << "[" << coord.first << " " << coord.second << "]";
    return ss.str();
  }

  void print_graph(const std::unordered_map<std::string, Node>& graph, const Maze& maze)
  {
    for (int row = 0; row < maze.size(); ++row)
    {
      for (int col = 0; col < maze[0].size(); ++col)
      {
        std::string value = std::to_string(graph.at(coord_to_string(
          std::make_pair(row, col))).distance);
        if (value == std::to_string(std::numeric_limits<int>::max()))
        {
          value = "x";
        }
        std::cout << value << " ";
      }
      std::cout << "\n";
    }
  }

} // Namespace Helpers




Path GetShortestPath(const Maze& maze, const Coord& start, const Coord& goal)
{
  using namespace Helpers;
  std::unordered_map < std::string, Node> graph;
  std::unordered_set<std::string> unvisited{};
  Path path{};

  // build graph (map of all nodes hashed by coordinate)
  for (size_t row{ 0 }; row < maze.size(); ++row)
  {
    for (size_t col{ 0 }; col < maze[0].size(); ++col)
    {
      Coord current_coord = std::make_pair(row, col);
      Node node{
        current_coord,
        std::numeric_limits<int>::max() };
      graph[coord_to_string(current_coord)] = node;

      if (maze[row][col] != WALL)
      {
        unvisited.insert(coord_to_string(current_coord));
      }
    }
  }

  std::string start_coord_str = coord_to_string(start); // get hash of start node
  graph[start_coord_str].distance = 0; // set source distance to 0

  Node* current_node = &graph[start_coord_str]; // set current node to start
  bool found = false;
  while (!unvisited.empty())
  {
    print_graph(graph,maze);
    std::cout << "cur_node = " << coord_to_string(current_node->position) << "\n\n";
    /// if current node is goal
    if (current_node->position == goal)
    {
      found = true;
      break;
    }

    // get vector of neighbour nodes
    std::vector<Coord> neighbours = get_neighbours(
      current_node->position, maze);

    // update neighbour distances and find minimum distance neighbour 
    Node* min_distance_neighbour{ nullptr };
    int min_distance = std::numeric_limits<int>::max();
    for (const auto& coord : neighbours)
    {
      if (unvisited.find(coord_to_string(coord)) != unvisited.end())
      {
        Node* neighbour = &graph[coord_to_string(coord)];


        update_distance(*current_node, *neighbour, maze);

        if (neighbour->distance < min_distance)
        {

          min_distance = neighbour->distance;
          min_distance_neighbour = neighbour;
        }
      }
    }

    // remove current node from unvisited set
    unvisited.erase(coord_to_string(current_node->position));

    // set current node to the minimum distance neighbour
    if (min_distance_neighbour != nullptr)
    {
      current_node = min_distance_neighbour;
    }
    else
    {
      // lambda function for comparing two nodes by distance
      auto compare = [graph](const auto& l, const auto& r)
      {
        return graph.at(l).distance < graph.at(r).distance;
      };

      // find unvisited node with minimum value
      auto iter = std::min_element(unvisited.cbegin(), 
                                   unvisited.cend(),
                                   compare);
      current_node = &graph[*iter];
    }

  }
  if (found)
  {
    path.push_back(goal);
    while (path.back() != UNDEFINED)
    {
      path.push_back(graph[coord_to_string(path.back())].previous);
    }
    path.pop_back();
  }
  std::reverse(path.begin(), path.end());
  return path;

}