#!/usr/bin/python

from digifab import *
from digifab.src.plotter import Plotter

def random_polygon_mesh(seed=42):
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
def plot_triangle_grid(triangles, xs, ys, grid):
  plotter = Plotter()
  triangles.plot(plotter)
  for x_i in range(len(xs)):
    for y_i in range(len(ys)):
      if grid[x_i,y_i]:
        plotter.ax.plot(xs[x_i],ys[y_i],'g.')
      else:
        plotter.ax.plot(xs[x_i],ys[y_i],'r.')
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

if __name__ == '__main__':
  triangles = random_polygon_mesh()
  xs,ys,grid = grid_from_triangles(triangles)
  plot_triangle_grid(triangles, xs, ys, grid)

