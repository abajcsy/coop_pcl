#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

using namespace std;

int main (int argc, char** argv) {
  cout << "argc = " << argc << endl;
  /* Sanity check: Need 5 parameters because of the way ROS makes use call launch files. */
  if(argc < 5){
	cout << "Not enough arguments! Need input .pcd filepath and output .vtk filepath." << endl;
	return -1;
  }

  string pcd_filepath = argv[1]; 
  string vtk_filepath = argv[2];
  cout << "pcd_filepath = " << pcd_filepath << endl;
  cout << "vtk_filepath = " << vtk_filepath << endl;

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (pcd_filepath, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

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

  /* TO VISUALIZE THE VTK FILE: 
   * Download open-source ParaView. I used Desktop version for Linux from: http://www.paraview.org/download/
   */
  //pcl::io::saveVTKFile(vtk_filepath, triangles);
  pcl::io::savePolygonFileSTL(vtk_filepath, triangles); 

  //cout << "Finished constructing mesh and saving .vtk file to " << vtk_filepath << "!\n";
  cout << "Finished constructing mesh and saving .stl file to " << vtk_filepath << "!\n";
  // Finish
  return (0);
}
