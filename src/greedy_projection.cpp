#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/impl/point_types.hpp>

#include <math.h>

using namespace std;

#define EPSILON 3.0

/* Performs meshing on an input point cloud 
 */
pcl::PolygonMesh mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
 // Normal estimation using standard method from PCL
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Need coordinates and normals in the same Point Cloud. Concatenate the XYZ and normal fields into PointNormal type point cloud
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.025);

  // Set typical values for the parameters
  /* Maximum acceptable distance for a point to be considered, 
   * relative to the distance of the nearest point (typical values 
   * are 50-100 and 2.5-3 or 1.5 for grids) 
   */
  gp3.setMu(80.0);
  /* Determines how many neighbors are searched for 
   */
  gp3.setMaximumNearestNeighbors(100);
  /* Maximum and minimum angles in each triangle. Minimum is NOT 
   * guaranteed but the maximum is. Typical values are 10 and 
   * 120 degrees (in radians) 
   */
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  /* Deals with cases where there are sharp edges or corners and 
   * where two sides of a surface run very close to each other. 
   * Points are not connected to the current point if their normals 
   * deviate more than the specified angle. This angle is computed 
   * as the angle between the lines defined by the normals if 
   * setNormalConsistency is not set. Typically, 45 degrees (in radians) 
   * and false works on most datasets. 
   */
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setNormalConsistency(false);

  // Set the input objects and perform the actual triangulation to get result in triangles
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  return triangles;
}

/* Converts an input .pcd file into a PolygonMesh structure. Saves resutling mesh as STL file.
 */
pcl::PolygonMesh initalizeMesh(string pcd_filepath, string stl_filepath, bool saveSTL){
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (pcd_filepath, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // perform the meshing on the input cloud
  pcl::PolygonMesh triangles = mesh(cloud);

  /* TO VISUALIZE THE VTK FILE: 
   * Download open-source ParaView. I used Desktop version for Linux from: http://www.paraview.org/download/
   */
  //pcl::io::saveVTKFile(vtk_filepath, triangles);
  if(saveSTL){
	pcl::io::savePolygonFileSTL(stl_filepath, triangles); 
  	//cout << "Finished constructing mesh and saving .vtk file to " << vtk_filepath << "!\n";
  	cout << "Finished constructing mesh and saving .stl file to " << stl_filepath << "!\n";
  }
  return triangles;
}

/* Computes the angle between the vector formed from p1 to p2 and p1 to p3
  */
double computeAngle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3){
	// form the two vectors
	pcl::PointXYZ v1(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
	pcl::PointXYZ v2(p3.x-p1.x, p3.y-p1.y, p3.z-p1.z);

	double len1 = sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
	double len2 = sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);

	double dot_prod = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
	double a = dot_prod/(len1 * len2);

	if(a >= 1.0){
		return 0.0;
	}else if(a <= -1.0){
		return M_PI;
	}else{
		return acos(a); // range is 0 to PI
	}
}

/* Filters out all interior points in PolygonMesh. Re-computes mesh after
 * filtering for cleaner final mesh structure. 
 */
pcl::PolygonMesh filterMesh(pcl::PolygonMesh triangles){
  vector<pcl::Vertices> polygons = triangles.polygons;

  // convert from PCLPointCloud2 to PointCloud to access point cloud
  pcl::PCLPointCloud2 pcl_pc = triangles.cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  int cloud_size = cloud.size();
  cout << "size of cloud = " << cloud_size << endl;

  // create map from each point in point cloud to the pair of points that form adjoining triangles
  vector<vector<vector<int> > > map;
  map.resize(cloud_size);
  cout << "number of map points = " << map.size() << endl;

  /* convert PolygonMesh data struct into map between each point idx and all pairs of point 
   * indices that form neighboring triangles
   */
  cout << "Printing polygon vertices..." << endl;
  for(int i = 0; i < polygons.size(); i++){
	for(int j = 0; j < polygons[i].vertices.size(); j++){
		int idx = polygons[i].vertices[j]; 

		// gather the (two) other points that form the triangle with the current point
		vector<int> neighboring_pts;
		for(int k = 0; k < polygons[i].vertices.size(); k++){
			if(k != j){
				neighboring_pts.push_back(polygons[i].vertices[k]);
			}
		}
		// if first time encountering point at this index, push new vector
		if(map[idx].empty()){
			vector<vector<int> > new_vec;
			new_vec.push_back(neighboring_pts);
			map[idx] = new_vec;
		}else{
			map[idx].push_back(neighboring_pts);
		}
	}
  }

  // make new PointCloud to store filtered cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  filt_cloud->height = 1;
  filt_cloud->width = cloud_size; 
  filt_cloud->points.resize(filt_cloud->width * filt_cloud->height);
  int filt_cloud_idx = 0;

  /* compute sum of all angles formed by triangles around a point
   * if the sum is equal to 2pi, then this point is an interior point in the mesh
   * and we remove it from the point cloud
   */
  for(int i = 0; i < map.size(); i++){
	pcl::PointXYZ p1 = cloud.at(i);
	double angle_sum = 0.0;
	// compute the angle between current point p1, and the two other vertices p2 and p3
	for(int j = 0; j < map[i].size(); j++){
		int idx_p2 = map[i][j][0]; 
		int idx_p3 = map[i][j][1];
		pcl::PointXYZ p2 = cloud.at(idx_p2);
		pcl::PointXYZ p3 = cloud.at(idx_p3);
		double angle = computeAngle(p1, p2, p3);
		cout << "angle = " << angle << endl;
		angle_sum += angle;
	}
	cout << "angle_sum = " << angle_sum << endl;
	// if the sum of the angles for all neighboring triangles is NOT 2pi 
	// (aka the point is an edge point), then add it to filtered cloud
	if(angle_sum < (2*M_PI - EPSILON)){
		filt_cloud->points[filt_cloud_idx].x = p1.x;
		filt_cloud->points[filt_cloud_idx].y = p1.y;
		filt_cloud->points[filt_cloud_idx].z = p1.z;
		filt_cloud_idx += 1;
		cout << "ADDED PT!" << endl;
	}
  }

  // perform meshing of the new filtered point cloud
  return mesh(filt_cloud);
}

int main (int argc, char** argv) {
  cout << "argc = " << argc << endl;
  /* Sanity check: Need 5 parameters because of the way ROS makes use call launch files. */
  if(argc < 5){
	cout << "Not enough arguments! Need input .pcd filepath and output .vtk filepath." << endl;
	return -1;
  }

  string pcd_filepath = argv[1]; 
  string stl_filepath = argv[2];
  cout << "pcd_filepath = " << pcd_filepath << endl;
  cout << "stl_filepath = " << stl_filepath << endl;

  // get initial mesh from raw point cloud
  pcl::PolygonMesh triangles = initalizeMesh(pcd_filepath, stl_filepath, false);
  pcl::PolygonMesh filt_triangles = filterMesh(triangles);

  pcl::io::savePolygonFileSTL(stl_filepath, filt_triangles); 
  cout << "Finished constructing mesh and saving .stl file to " << stl_filepath << "!\n";
  // Finish
  return (0);
}
