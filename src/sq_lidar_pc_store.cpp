#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>

class SQLidarPCStore{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"sq_lidar_pc_store"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stored {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_now {new pcl::PointCloud<pcl::PointXYZI>};
		/*odom*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
		bool pc_was_added = false;
		/*frame id*/
		std::string frame_id_pc = "/base_link";
		/*tf*/
		tf::TransformListener tflistener;
		/*limit storing*/
		const bool limit_storing = true;
		int limit_num_scans = 25;
		/*
		 * [storing time] = [limit_num_scans] x [rate of /odom (=0.02s)]
		 */
		std::vector<size_t> list_num_scanpoints;

	public:
		SQLidarPCStore();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Visualizer(void);
		void Publisher(void);
};

SQLidarPCStore::SQLidarPCStore()
	: nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &SQLidarPCStore::CallbackPC, this);
	sub_odom = nh.subscribe("/odom", 1, &SQLidarPCStore::CallbackOdom, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/sq_lidar/stored", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	nhPrivate.getParam("num_scans", limit_num_scans);
}

void SQLidarPCStore::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	sensor_msgs::PointCloud pc_in;
	sensor_msgs::PointCloud pc_trans;
	sensor_msgs::PointCloud2 pc2_trans;

	sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_in);
	try{
		tflistener.waitForTransform(frame_id_pc, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud(frame_id_pc, msg->header.stamp, pc_in, msg->header.frame_id, pc_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(pc_was_added)	cloud_now->points.clear();
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(pc2_trans, *tmp_pc);
	const double laser_range = 40.0;
	for(size_t i=0;i<tmp_pc->points.size();i++){
		double laser_distance = sqrt(tmp_pc->points[i].x*tmp_pc->points[i].x + tmp_pc->points[i].y*tmp_pc->points[i].y + tmp_pc->points[i].z*tmp_pc->points[i].z);
		if(laser_distance<laser_range)	cloud_now->points.push_back(tmp_pc->points[i]);
	}
	cloud_stored->header.frame_id = frame_id_pc;
	pc_was_added = false;
}

void SQLidarPCStore::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	if(first_callback_odom)	odom_last = odom_now;
	else if(!pc_was_added){
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		quaternionMsgToTF(odom_now.pose.pose.orientation, pose_now);
		quaternionMsgToTF(odom_last.pose.pose.orientation, pose_last);
		tf::Quaternion relative_rotation = pose_last*pose_now.inverse();
		relative_rotation.normalize();
		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
		tf::Quaternion q_global_move(
				odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
				odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
				odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
				0.0);
		tf::Quaternion q_local_move = pose_last.inverse()*q_global_move*pose_last;
		Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
		pcl::transformPointCloud(*cloud_stored, *cloud_stored, offset, rotation);
		*cloud_stored  += *cloud_now;
		pc_was_added = true;
		
		odom_last = odom_now;
		
		/*limit storing*/
		if(limit_storing){
			list_num_scanpoints.push_back(cloud_now->points.size());
			if(list_num_scanpoints.size()>limit_num_scans){
				cloud_stored->points.erase(cloud_stored->points.begin(), cloud_stored->points.begin() + list_num_scanpoints[0]);
				list_num_scanpoints.erase(list_num_scanpoints.begin());
			}
			cloud_stored->width = cloud_stored->points.size();
			cloud_stored->height = 1;

			std::cout << "limit storing: true" << std::endl;
			std::cout << "number of stored scans: " << list_num_scanpoints.size() << std::endl;
		}
		Publisher();
	}
	first_callback_odom = false;

	Visualizer();
}

void SQLidarPCStore::Visualizer(void)
{
	viewer.removeAllPointClouds();

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_stored, "intensity"); 
	viewer.addPointCloud<pcl::PointXYZI>(cloud_stored, intensity_distribution, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	
	viewer.spinOnce();
}

void SQLidarPCStore::Publisher(void)
{
	sensor_msgs::PointCloud2 ros_pc_out;
	pcl::toROSMsg(*cloud_stored, ros_pc_out);
	ros_pc_out.header.stamp = odom_now.header.stamp;
	pub.publish(ros_pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_lidar_pc_store");
	
	SQLidarPCStore sq_lidar_pc_store;

	ros::spin();
}
