#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <string>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#define EPSILON 3.0

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudMesh {
  private: 
	ros::NodeHandle nh_;
	ros::Publisher cloud_pub_;
	ros::Publisher proj_cloud_pub_;	
		
	PointCloud::Ptr cloud_;
	PointCloud::Ptr cloud_projected_;

	pcl::PolygonMesh mesh_;
	pcl::PolygonMesh filt_mesh_;

  public: 
	CloudMesh(ros::NodeHandle &nh){
		nh_ = nh;
		cloud_pub_ = nh_.advertise<PointCloud>("points2", 1);
		proj_cloud_pub_ = nh_.advertise<PointCloud>("proj_points2", 1);

		// Fill in the cloud data
		PointCloud::Ptr tmp_cloud(new PointCloud); 	
		cloud_ = tmp_cloud;
		cloud_->width  = 15000;
		cloud_->height = 1;
		cloud_->header.frame_id = "usb_cam";
		cloud_->points.resize (cloud_->width * cloud_->height);

		PointCloud::Ptr tmp_proj_cloud(new PointCloud); 	
		cloud_projected_ = tmp_proj_cloud;
		cloud_projected_->header.frame_id = "usb_cam";
	}

	/* Projects points within input point cloud onto the plane specified by ax + by + cz + d=0
	 */
	void project_inliers(string pcd_filepath, double x_coeff, double y_coeff, double z_coeff, double d){
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile (pcd_filepath, cloud_blob);
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud_);
		cloud_->header.frame_id = "usb_cam";

		cout << "Finished reading input point cloud..." << endl;

		// Create a set of planar coefficients with X=Y=0,Z=1
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		coefficients->values.resize (4);
		coefficients->values[0] = x_coeff;
		coefficients->values[1] = y_coeff;
		coefficients->values[2] = z_coeff;
		coefficients->values[3] = d;

		// Create the filtering object
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		proj.setInputCloud (cloud_);
		proj.setModelCoefficients (coefficients);
		proj.filter (*cloud_projected_);
		cloud_projected_->header.frame_id = "usb_cam";
		pcl_conversions::toPCL(ros::Time::now(), cloud_projected_->header.stamp);

		cout << "Finished projecting points..." << endl;
		/*
		string pcd_filename_out = "/home/andrea/ros_workspace/src/coop_pcl/data/pcd/plane_projected.pcd";
		pcl::io::savePCDFileASCII(pcd_filename_out, *cloud_projected_);
		cout << "Saved projected point cloud to " << pcd_filename_out << endl;				
		*/
	}

	/* Performs meshing on an input point cloud. Using the mesh_projected flag, set if you want to mesh 
	 * from the original raw point cloud or the 2D projected cloud.
	 */
	pcl::PolygonMesh mesh(PointCloud::Ptr cloud, double searchRad, double mu, int maxNN){
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
		gp3.setSearchRadius(searchRad);

		// Set typical values for the parameters
		/* Maximum acceptable distance for a point to be considered, 
		* relative to the distance of the nearest point (typical values 
		* are 50-100 and 2.5-3 or 1.5 for grids) 
		*/
		gp3.setMu(mu);
		/* Determines how many neighbors are searched for 
		*/
		gp3.setMaximumNearestNeighbors(maxNN);
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

	/* Converts an input .pcd file into a PolygonMesh structure. 
	 * Saves resutling mesh as STL file if saveSTL flag is true.
	 * Using the mesh_projected flag, set if you want to mesh from 
	 * the original raw point cloud or the 2D projected cloud.
	 */
	void initializeMesh(string stl_filepath, double searchRad, double mu, int maxNN, bool save_STL, bool mesh_projected){
		PointCloud::Ptr input_cloud;
		if(mesh_projected){
			input_cloud = cloud_projected_;
		}else{
			input_cloud = cloud_;
		}
		// perform the meshing on the input cloud and set mesh_ class variable to computed raw mesh
		mesh_ = mesh(input_cloud, searchRad, mu, maxNN);

		/* TO VISUALIZE THE STL FILE: 
		* Download open-source ParaView. I used Desktop version for Linux from: http://www.paraview.org/download/
		*/
		if(save_STL){
			pcl::io::savePolygonFileSTL(stl_filepath, mesh_); 
			cout << "In initializeMesh(): Constructed mesh and saving .stl file to " << stl_filepath << "!\n";
		}
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
	void filterMesh(double searchRad, double mu, int maxNN){
		// TODO have check to see if mesh_ has been initialized properly?
		vector<pcl::Vertices> polygons = mesh_.polygons;

		// Convert from PCLPointCloud2 to PointCloud to access point cloud
		pcl::PCLPointCloud2 pcl_pc = mesh_.cloud;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromPCLPointCloud2(pcl_pc, cloud);
		int cloud_size = cloud.size();

		// Create map from each point in point cloud to the pair of points that form adjoining triangles
		vector<vector<vector<int> > > map;
		map.resize(cloud_size);

		/* Convert PolygonMesh data struct into map between each point idx and all pairs of point 
		* indices that form neighboring triangles
		*/
		cout << "Converting PolygonMesh to map from (ith point)-->(adjoining triangle vertices)" << endl;
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

		// Make new PointCloud to store filtered cloud
		PointCloud::Ptr filt_cloud(new PointCloud);
		filt_cloud->height = 1;
		filt_cloud->width = cloud_size; 
		filt_cloud->points.resize(filt_cloud->width * filt_cloud->height);
		int filt_cloud_idx = 0;

		/* Compute sum of all angles formed by triangles around a point
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
				//cout << "angle = " << angle << endl;
				angle_sum += angle;
			}
			//cout << "angle_sum = " << angle_sum << endl;
			// if the sum of the angles for all neighboring triangles is NOT 2pi 
			// (aka the point is an edge point), then add it to filtered cloud
			if(angle_sum < (2*M_PI - EPSILON)){
				filt_cloud->points[filt_cloud_idx].x = p1.x;
				filt_cloud->points[filt_cloud_idx].y = p1.y;
				filt_cloud->points[filt_cloud_idx].z = p1.z;
				filt_cloud_idx += 1;
				//cout << "ADDED PT!" << endl;
			}
		}

		// Perform meshing of the new filtered point cloud
		filt_mesh_ = mesh(filt_cloud, searchRad, mu, maxNN);
		cout << "Finished successfully re-meshing filtered point cloud\n";
	}

	/* Runs complete cloud to mesh conversion with following steps:
	 * 	(1) project raw point cloud data to plane (specified by input coefficients)
	 * 	(2) initialize the mesh based on projected point cloud
	 * 	(3) filter raw mesh by removing interior points and remeshing
	 * 	(4) save .stl file of final mesh
	 */
	void run(string pcd_filepath, string raw_stl_filepath, string stl_filepath){
		// (1) projection to plane
		cout << "Projecting mesh to plane...\n";
		double x_coeff = 0.0, y_coeff = 1.0, z_coeff = 0.0, d = 0.0;
		project_inliers(pcd_filepath, x_coeff, y_coeff, z_coeff, d);

		// (2) initialize mesh
		cout << "Intializing raw mesh from point cloud...\n";
		double searchRad = 0.025, searchRad_filt = 0.5;
		double mu = 80.0, mu_filt = 80.0;
		int maxNN = 100, maxNN_filt = 50;
		bool save_STL = true, mesh_projected = true;
		initializeMesh(raw_stl_filepath, searchRad, mu, maxNN, save_STL, mesh_projected);

		// (3) filter raw mesh
		cout << "Filtering interior points from raw mesh...\n";
		filterMesh(searchRad_filt, mu_filt, maxNN_filt);

		// (4) save .stl file 
		cout << "Constructed filtered mesh and saving .stl file to " << stl_filepath << "\n";
		pcl::io::savePolygonFileSTL(stl_filepath, filt_mesh_); 

		/*ros::Rate loop_rate(2);
		while(nh_.ok()){
			// publish both clouds
			cloud_pub_.publish(cloud_);
			proj_cloud_pub_.publish(cloud_projected_);

			ros::spinOnce();
			loop_rate.sleep();
		}*/
	}

};

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "cloud_mesh");
  ros::NodeHandle nh;

  cout << "argc = " << argc << endl;
  if(argc < 4){
	cout << "Not enough arguments! Need input .pcd filepath and two output .stl filepaths." << endl;
	return -1;
  }

  string pcd_filepath = argv[1]; 
  string raw_stl_filepath = argv[2];
  string stl_filepath = argv[3];
  cout << "pcd_filepath = " << pcd_filepath << endl;
  cout << "raw_stl_filepath = " << raw_stl_filepath << endl;
  cout << "stl_filepath = " << stl_filepath << endl;

  CloudMesh driver(nh);
  driver.run(pcd_filepath, raw_stl_filepath, stl_filepath);
}

