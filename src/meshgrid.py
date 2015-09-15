#!/usr/bin/python

from digifab import *
from digifab.src.plotter import Plotter
import glob, os, sys
import json, numpy

from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
from matplotlib.legend_handler import HandlerLine2D

def mesh_to_triangles(mesh):
  triangles = PolyLine()

  for triangle in mesh:
    if all(triangle.points[:,2] == 0.0):
      triangles += PolyLine(triangle.points[:,0:2],polygon=True)
  
  return triangles

def region_to_triangles(region):
  mesh = PolyMesh(generator=solid.linear_extrude()(region.simplified().get_generator()))
  return mesh_to_triangles(mesh)

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

  return region_to_triangles(pgon)

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
def grid_from_triangles(triangles, delta = 0.2, grid_coords=None):
  if grid_coords is None:
    mins, maxs = triangles.bounds(delta)
    xs = numpy.arange(mins[0],maxs[0]+delta,delta)
    ys = numpy.arange(mins[1],maxs[1]+delta,delta)
  else:
    cxs,cys = grid_coords
    delta = cxs[1] - cxs[0]
    xs = numpy.arange(cxs[0] - delta/2, cxs[-1] + 3*delta/2, delta)
    ys = numpy.arange(cys[0] - delta/2, cys[-1] + 3*delta/2, delta)

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
  if not grid[start]:
    print "WARNING: start is not in grid"
  if not grid[end]:
    print "WARNING: end is not in grid"

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

  if numpy.isinf(path_grid[end]) or numpy.isnan(path_grid[end]):
    return [],path_grid
  else:
    path = [end]
    while path_grid[path[-1]] != 0:
      nbrs = match_neighbors(path_grid, path[-1])
      vals = path_grid[zip(*nbrs)]
      min_idx = numpy.nanargmin(path_grid[zip(*nbrs)])
      path.append(nbrs[min_idx])
    path.reverse()
    return path, path_grid

def full_path(path):
  if path[0] != '/':
    path = os.getcwd() + '/' + path
  return path

def stl_to_triangles(cam_H, stl_filename):
  stl_filename = full_path(stl_filename)
  gen = solid.linear_extrude() (
    solid.minkowski() (
      solid.circle(0.02),
      solid.projection() (
        solid.multmatrix(cam_H.tolist()) (
          solid.import_stl(stl_filename)
        )
      )
    )
  )

  return mesh_to_triangles(PolyMesh(generator=gen))

def load_result(stl_file, stuck_zumy_file):
  f = open(full_path(stuck_zumy_file))
  data = json.load(f)
  f.close()
  
  cam_h = numpy.array(data['cam_h'])
  
  triangles = stl_to_triangles(cam_h, stl_file)
  
  data.update({'cam_h':cam_h, 'triangles':triangles})
  
  return data

def get_circle_regions(centers,radius=0.05):
  regions = PolyLine()

  for x,y,t in centers:
    gen = solid.translate([x,y])(solid.circle(radius))
    gen.add_param('$fn',20)
    regions += PolyLine(generator=gen).simplified()

  return regions
  
def closest_grid_index(cxs, cys, point):
  xi = ((cxs - point[0])**2).argmin()
  yi = ((cys - point[1])**2).argmin()
  return (xi,yi)

def process_result(triangles, zumy, stuck, end = (0.2,0.0), delta=0.02,**kwargs):
  
  zumy_circle = get_circle_regions([zumy],0.06)[0]
  zumy_frontier = (triangles & get_circle_regions([zumy],0.2)[0]).simplified()

  zumy_region = (zumy_circle + zumy_frontier).hull().simplified()

  robot_region = triangles | zumy_region

  stuck_regions = get_circle_regions(stuck)

  safe_region = (robot_region.bounding_box(3*delta) - stuck_regions)
 
  cxs, cys, robot_grid = grid_from_triangles(region_to_triangles(robot_region), delta)
  _,_,safe_grid = grid_from_triangles(region_to_triangles(safe_region), grid_coords=(cxs,cys))
  
  plan_grid = robot_grid & safe_grid
  
  start_idx = closest_grid_index(cxs, cys, zumy[0:2])
  end_idx = closest_grid_index(cxs, cys, end)
  
  zumy_path,_ = plan_path(plan_grid, start_idx, end_idx)
  
  return {
    'robot_region':robot_region, 
    'stuck_regions':stuck_regions, 
    'cxs': cxs,
    'cys': cys,
    'safe_grid':safe_grid,
    'robot_grid':robot_grid,
    'zumy_path':zumy_path
  }

def plot_result(
  robot_region, stuck_regions, cxs, cys, safe_grid, robot_grid, roach, zumy_path,
  legend=False, **kwargs):
  plotter = Plotter()
  
  environment = PolyLine([(-0.3,-0.135),(-0.3,0.6),(0.3,0.6),(0.3,-0.135)],polygon=True)
  environment.plot(plotter,linewidth=2,label='environment_bounds')

  hazzard = PatchCollection([Rectangle((-0.02,0.095),0.32,0.275)],cmap=plt.cm.hsv,alpha=0.2)
  hazzard.set_array(numpy.array([0]))
  plotter.ax.add_collection(hazzard)

  region_outline = robot_region - PolyLine()
  region_outline.plot(plotter,label='exploration_bounds')

  stuck_regions.plot(plotter,'red')

  for i in range(len(cxs)):
    for j in range(len(cys)):
      if not safe_grid[i,j]:
        plotter.ax.plot(cxs[i],cys[j],'rx',label='stuck_points')
      elif robot_grid[i,j]:
        plotter.ax.plot(cxs[i],cys[j],'g.',label='safe_points')
  
  for path in roach:
    xy = numpy.array([[x,y] for x,y,t in path])
    plotter.add_polylines([xy.T],'blue',label='roach_paths')
 
  if len(zumy_path):
    zc = [(cxs[xi],cys[yi]) for xi,yi in zumy_path] 
    zumy_path_pl = PolyLine(zc)
    zumy_path_pl.plot(plotter,linewidth=4,label='zumy_path')

    plotter.ax.plot(zc[0][0],zc[0][1],'yo',markersize=15.0,label='zumy_start')
    plotter.ax.plot(zc[-1][0],zc[-1][1],'y*',markersize=15.0,label='zumy_goal')
  
  if legend:
    legend_map = {label:handle for handle,label in zip(*plotter.ax.get_legend_handles_labels())}
    single_handler = HandlerLine2D(numpoints=1)
    single_labels = ['stuck_points','safe_points','zumy_start','zumy_goal']
    available_labels = [l for l in single_labels if l in legend_map.keys()]
    handler_map = {legend_map[label]:single_handler for label in available_labels}
    plotter.ax.legend(legend_map.values(),legend_map.keys(),handler_map=handler_map)

  return plotter

def save_result(plotter, pdf_file):
  pdf_file = full_path(pdf_file)
  pp = PdfPages(pdf_file)
  pp.savefig(plotter.fig)
  pp.close()

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
  #plot_path(42)
  stl_file = full_path(sys.argv[1])
  json_file = full_path(sys.argv[2])
  pdf_file = full_path(sys.argv[3])

  result = load_result(stl_file, json_file)
  result.update(process_result(**result))
  save_result(plot_result(**result), pdf_file)
