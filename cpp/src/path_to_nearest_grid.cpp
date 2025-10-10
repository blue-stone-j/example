/*
1. divide a large square into unit squares and some unit squares are occupied. 
2. given a square, find the nearest occupied square and return the path to it.
*/

#include <algorithm>
#include <iostream>
#include <vector>
#include <queue>
#include <map>

struct Cell
{
  int x, y;
  bool operator<(const Cell &other) const
  {
    return x == other.x ? y < other.y : x < other.x;
  }
};

// Directions for moving in 4 directions (right, left, down, up)
int dx[] = {0, 0, 1, -1};
int dy[] = {1, -1, 0, 0};

// Function to retrieve path from start to destination
std::vector<Cell> reconstructPath(std::map<Cell, Cell> &parent, Cell start, Cell end)
{
  std::vector<Cell> path;
  for (Cell at = end; !(at.x == start.x && at.y == start.y); at = parent[at])
  {
    path.push_back(at);
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Cell> findPathToNearestOccupied(std::vector<std::vector<int>> &grid, int x, int y)
{
  int n = grid.size();
  if (grid[x][y] == 1) return {{x, y}}; // Already occupied

  std::queue<Cell> q;
  std::map<Cell, Cell> parent; // Stores parent cell to reconstruct path
  std::vector<std::vector<bool>> visited(n, std::vector<bool>(n, false));

  q.push({x, y});
  visited[x][y]  = true;
  parent[{x, y}] = {x, y}; // Mark starting cell

  while (!q.empty())
  {
    Cell cur = q.front();
    q.pop();

    for (int d = 0; d < 4; ++d)
    {
      int nx = cur.x + dx[d];
      int ny = cur.y + dy[d];

      if (nx >= 0 && nx < n && ny >= 0 && ny < n && !visited[nx][ny])
      {
        visited[nx][ny]  = true;
        parent[{nx, ny}] = cur; // Track the path

        if (grid[nx][ny] == 1)
        {
          return reconstructPath(parent, {x, y}, {nx, ny});
        }

        q.push({nx, ny});
      }
    }
  }
  return {}; // No occupied cell found
}

int main()
{
  int n                              = 5;
  std::vector<std::vector<int>> grid = {
      {0, 0, 0, 0, 1},
      {0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0},
      {1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}};

  int x = 2, y = 2;
  std::vector<Cell> path = findPathToNearestOccupied(grid, x, y);

  if (!path.empty())
  {
    std::cout << "Path to nearest occupied cell:\n";
    for (auto p : path)
    {
      std::cout << "(" << p.x << ", " << p.y << ") ";
    }
    std::cout << std::endl;
  }
  else
  {
    std::cout << "No occupied cell found.\n";
  }

  return 0;
}
