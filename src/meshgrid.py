#!/usr/bin/python

from digifab import *
from digifab.src.plotter import Plotter

def random_polygon_mesh(seed=None):
  if seed is not None:
    numpy.random.seed(seed)

  pgon = PolyLine(10*numpy.random.rand(20,2)).hull()

  holes = [
    PolyLine(
      10*numpy.random.rand(1,2) + 2*numpy.random.rand(20,2)
    ).hull() for i in range(5)
  ]

  for hole in holes:
    pgon -= hole

  pmesh = PolyMesh(generator=solid.linear_extrude()(pgon.get_generator()))

  triangles = PolyLine()

  for triangle in pmesh:
    if all(triangle.points[:,2] == 0.0):
      triangles += PolyLine(triangle.points[:,0:2],polygon=True)
  
  return triangles

# Plot PolyLine mesh of triangles, and boolean grid of dots
def plot_triangle_grid(triangles, cxs, cys, grid):
  plotter = Plotter()
  triangles.plot(plotter)
  for x_i in range(len(cxs)):
    for y_i in range(len(cys)):
      if grid[x_i,y_i]:
        plotter.ax.plot(cxs[x_i],cys[y_i],'g.')
      else:
        plotter.ax.plot(cxs[x_i],cys[y_i],'r.')
  plotter.show()

# Return true if given point is inside triangle
def point_in_triangle(point,triangle):
  P = numpy.array(point)
  A = triangle.points[0,:]
  B = triangle.points[1,:]
  C = triangle.points[2,:]
  
  # From http://www.blackpawn.com/texts/pointinpoly/
  # Compute vectors        
  v0 = C - A
  v1 = B - A
  v2 = P - A

  # Compute dot products
  dot00 = v0.dot(v0)
  dot01 = v0.dot(v1)
  dot02 = v0.dot(v2)
  dot11 = v1.dot(v1)
  dot12 = v1.dot(v2)

  # Compute barycentric coordinates
  invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
  u = (dot11 * dot02 - dot01 * dot12) * invDenom
  v = (dot00 * dot12 - dot01 * dot02) * invDenom

  # Check if point is in triangle
  return (u >= 0) and (v >= 0) and (u + v < 1)

# Return 2D boolean grid with the centers of squares that are completely
# inside mesh. Also returns lists of x and y coordinates
def grid_from_triangles(triangles, delta = 0.2):
  mins,maxs = triangles.bounds(delta)

  xs = numpy.arange(mins[0],maxs[0],delta)
  ys = numpy.arange(mins[1],maxs[1],delta)

  x_is = numpy.arange(len(xs))
  y_is = numpy.arange(len(ys))

  corners = numpy.zeros((len(xs),len(ys))) == 1

  for i in range(len(triangles)):
    triangle = triangles[i]
    t_mins, t_maxs = triangle.bounds()
    sub_x_is = x_is[(xs >= t_mins[0]) & (xs <= t_maxs[0])]
    sub_y_is = y_is[(ys >= t_mins[1]) & (ys <= t_maxs[1])]
    for x_i in sub_x_is:
      for y_i in sub_y_is:
        point = [xs[x_i],ys[y_i]]
        if not corners[x_i,y_i] and point_in_triangle(point,triangle):
          corners[x_i,y_i] = True
  
  cxs = xs[:-1] + delta/2
  cys = ys[:-1] + delta/2
  grid = corners[:-1,:-1] & corners[1:,:-1] & corners[:-1,1:] & corners[1:,1:]
  return cxs, cys, grid

# Returns a random [x,y] index in grid that is True
def random_grid_index(grid):
  xi,yi = numpy.meshgrid(range(grid.shape[0]),range(grid.shape[1]),indexing='ij')
  xis, yis = xi[grid], yi[grid]
  if len(xis) == 0:
    return None
  else:
    idx = numpy.random.randint(len(xis))
    return xis[idx],yis[idx]

def dummy_filt(val):
  return True

def match_neighbors(grid, cell, filt=None):
  if filt is None:
    filt = dummy_filt
  xi, yi = cell
  x_max, y_max = grid.shape
  neighbors = []
  for i in range(max(0,xi-1),min(xi+1,x_max-1)+1):
    for j in range(max(0,yi-1),min(yi+1,y_max-1)+1):
      if (i,j) != cell and filt(grid[i,j]):
        neighbors.append((i,j))
  return neighbors

def index_dist(a,b):
  return ((a[0] - b[0])**2 + (a[1]-b[1])**2)**0.5

def plan_path(grid, start=None, end=None):
  if start is None:
    start = random_grid_index(grid)
  if end is None:
    end = random_grid_index(grid)
  assert(grid[start] and grid[end])
  path_grid = numpy.inf * numpy.ones(grid.shape, dtype=numpy.int64)
  path_grid[~grid] = numpy.nan
  path_grid[start] = 0
  front = [start]
  while path_grid[end] == numpy.inf and front:
    new_front = set()
    for cell in front:
      new_front.update(match_neighbors(path_grid, cell, numpy.isinf))
    new_vals = numpy.zeros(len(new_front))
    front = list(new_front)
    for i in range(len(front)):
      nbrs = match_neighbors(path_grid, front[i], numpy.isfinite)
      dists = [path_grid[n] + index_dist(n, front[i]) for n in nbrs]
      new_vals[i] = min(dists)
    path_grid[zip(*front)] = new_vals

  if path_grid[end] == numpy.inf:
    return []
  else:
    path = [end]
    while path_grid[path[-1]] != 0:
      nbrs = match_neighbors(path_grid, path[-1])
      vals = path_grid[zip(*nbrs)]
      min_idx = numpy.nanargmin(path_grid[zip(*nbrs)])
      path.append(nbrs[min_idx])
    path.reverse()
    return path, path_grid

def show_path_grid(grid, path):
  show_grid = numpy.zeros(grid.shape) == 1
  show_grid[zip(*path)] = True
  return show_grid

def plot_path(triangles=None, start=None, end=None, delta=0.2):
  if triangles is None:
    triangles = random_polygon_mesh()
  
  cxs,cys,grid = grid_from_triangles(triangles, delta)
  
  if start is None:
    start = random_grid_index(grid)
  if end is None:
    end = random_grid_index(grid)

  path,_ = plan_path(grid,start,end)
  
  plot_triangle_grid(triangles,cxs,cys,show_path_grid(grid,path))

if __name__ == '__main__':
  plot_path(42)
