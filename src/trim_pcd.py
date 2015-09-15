#!/usr/bin/python

import sys

if __name__ == '__main__':
  pcd_file = sys.argv[1]
  out_file = sys.argv[2]

  f = open(pcd_file,'r')
  lines = f.readlines()
  f.close()

  for line in lines[:]:
    vs = line.split()
    if len(vs) == 3:
      vs = [float(v) for v in vs]
      norm = sum([v**2 for v in vs])**0.5
      if vs[1] > 0.1 or norm == 0 or norm > 1.0:
        lines.remove(line)

  n_points = (len(lines)-11)
  lines[6] = 'WIDTH %d\n' % n_points
  lines[9] = 'POINTS %d\n' % n_points

  f = open(out_file, 'w')
  for line in lines:
    f.write(line)
  f.close()
