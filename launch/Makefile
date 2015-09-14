COOP_PCL_DIR=$(shell rospack find coop_pcl)
SRC_DIR=$(COOP_PCL_DIR)/src
LAUNCH_DIR=$(COOP_PCL_DIR)/launch

EXPERIMENT_CALIB_DIR=$(shell rospack find experiment_calib)

%.pdf: %.stl %.json $(SRC_DIR)/meshgrid.py
	$(SRC_DIR)/meshgrid.py $*.stl $*.json $@

%.stl: %_trim.pcd $(SRC_DIR)/cloud_to_mesh.cpp $(LAUNCH_DIR)/meshing_test.launch
	roslaunch $(LAUNCH_DIR)/meshing_test.launch pcd_filepath:=$(abspath $*_trim.pcd) raw_stl_filepath:=$(abspath $*_raw.stl) stl_filepath:=$(abspath $@)

%.json: %.bag %_avg.bag $(SRC_DIR)/bag_to_poses.py
	$(SRC_DIR)/bag_to_poses.py $*.bag $*_avg.bag $@

%_trim.pcd: %.pcd $(SRC_DIR)/trim_pcd.py
	$(SRC_DIR)/trim_pcd.py $*.pcd $@

%_avg.bag: %.bag $(EXPERIMENT_CALIB_DIR)/src/avg_bag_tf.py
	$(EXPERIMENT_CALIB_DIR)/src/avg_bag_tf.py $*.bag $@

.PRECIOUS: %.pcd %.bag %.json %.pdf
