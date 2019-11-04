#include "MazeSolver.h"


/// test function
int test(Maze &x, Path& correct_path)
{
  int ret_val = 0;
  Path path = GetShortestPath(x, Coord{ 0, 0 }, Coord{ 2, 2 });
  if (path.size() == correct_path.size())
  {
    for (size_t i{ 0 }; i < correct_path.size(); ++i)
    {
      if (path[i] != correct_path[i])
      {
        ret_val = -1;
        break;
      }
    }
  }
  else
  {
    ret_val = -1;
  }
  return ret_val;
}


/// test cases
int main()
{
  int ret = 0;
  // Given a maze with (0,0) at the left top corner
  //  1  3    4
  //  2 -999  1
  //  4  1    0
  // The shortest path from (0, 0) to (2, 2) is:
  // {{0,0}, {1, 0}, {2, 1}, {2, 2} }
  Maze m1{ {1,3,4},{2, -999, 1},{ 4, 1 , 0 } };
  Path p1{ {0,0},{1,0},{2,1},{2,2} };
  ret += test(m1, p1);

  // Given a maze with (0,0) at the left top corner
  //  1  2    4
  //  2 -999  0
  //  4  1    0
  // The shortest path from (0, 0) to (2, 2) is:
  // {{0,0}, {1, 0}, {2, 1}, {2, 2} }
  Maze m2{ {1,2,4},{2, -999, 0},{ 4, 1 , 0 } };
  Path p2{ {0,0},{0,1},{1,2},{2,2} };
  ret += test(m2, p2);

  return (ret < 0)? -1 : ret;
}