#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>


class SQLidarPlanarNormalEstimation{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_pc;
		ros::Publisher _pub_plane;
		ros::Publisher _pub_nc;
		/*tf*/
		tf::TransformListener _tflistener;
		/*point cloud*/
		pcl::PointCloud<pcl::InterestPoint>::Ptr _pc {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _pc_plane_now {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _pc_plane_last {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _nc {new pcl::PointCloud<pcl::PointNormal>};
		/*odom*/
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		/*viewer*/
		pcl::visualization::PCLVisualizer _viewer{"sq_lidar_planar_normal_estimation"};
		/*flag*/
		bool got_first_odom = false;
		/*parameter*/
		const double _laser_range = 40.0;
		std::string _frame_id;
		int _curvature_region;
		double _th_flatness;
		double  _th_assoc_squareddist;
		double _th_cross_angle_deg;

	public:
		SQLidarPlanarNormalEstimation();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void reset(void);
		bool transformPCWithTF(const sensor_msgs::PointCloud2& pc2_in, sensor_msgs::PointCloud2& pc2_out, std::string target_frame, ros::Time target_stamp);
		bool transformPCWithOdom(const pcl::PointCloud<pcl::PointNormal>::Ptr pc_in, pcl::PointCloud<pcl::PointNormal>::Ptr pc_out, nav_msgs::Odometry odom_last, nav_msgs::Odometry odom_now);
		void eraseNanPoint(void);
		void computeFlatness(void);
		bool estimateNormal(pcl::KdTreeFLANN<pcl::PointNormal>& kdtree, pcl::PointNormal& n);
		void publication(void);
		void visualization(void);
		double norm(double x, double y, double z);
		double angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
};

SQLidarPlanarNormalEstimation::SQLidarPlanarNormalEstimation()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("frame_id", _frame_id, std::string("/base_link"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("curvature_region", _curvature_region, 5);
	std::cout << "_curvature_region = " << _curvature_region << std::endl;
	_nhPrivate.param("th_flatness", _th_flatness, 1.0e-3);
	std::cout << "_th_flatness = " << _th_flatness << std::endl;
	_nhPrivate.param("th_assoc_squareddist",  _th_assoc_squareddist, 5.0e-1);
	std::cout << " _th_assoc_squareddist = " <<  _th_assoc_squareddist << std::endl;
	_nhPrivate.param("th_cross_angle_deg", _th_cross_angle_deg, 5.0);
	std::cout << "_th_cross_angle_deg = " << _th_cross_angle_deg << std::endl;
	/*sub*/
	_sub_pc = _nh.subscribe("/cloud", 1, &SQLidarPlanarNormalEstimation::callbackPC, this);
	_sub_odom = _nh.subscribe("/odom", 1, &SQLidarPlanarNormalEstimation::callbackOdom, this);
	/*pub*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/with_flatness", 1);
	_pub_plane = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
	_pub_nc = _nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	/*viewer*/
	_viewer.setBackgroundColor(1, 1, 1);
}

void SQLidarPlanarNormalEstimation::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom_now = *msg;
	if(!got_first_odom){
		_odom_last = _odom_now;
		got_first_odom = true;
	}
}

void SQLidarPlanarNormalEstimation::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "----------" << std::endl;
	std::cout << "msg->header.frame_id = " << msg->header.frame_id << std::endl;
	std::cout << "msg->height*msg->width = " << msg->height*msg->width << std::endl;
	if(msg->header.frame_id != "laser1_link")	return;
	if(!got_first_odom)	return;

	reset();

	sensor_msgs::PointCloud2 pc_trans;
	if(!transformPCWithTF(*msg, pc_trans, _frame_id, msg->header.stamp))	return;
	pcl::fromROSMsg(pc_trans, *_pc);
	eraseNanPoint();

	computeFlatness();
	publication();
	visualization();
}

void SQLidarPlanarNormalEstimation::reset(void)
{
	transformPCWithOdom(_pc_plane_now, _pc_plane_last, _odom_last, _odom_now);
	_odom_last = _odom_now;
	_pc_plane_now->points.clear();
	_nc->points.clear();
}

bool SQLidarPlanarNormalEstimation::transformPCWithTF(const sensor_msgs::PointCloud2& pc2_in, sensor_msgs::PointCloud2& pc2_out, std::string target_frame, ros::Time target_stamp)
{
	sensor_msgs::PointCloud pc1_in;
	sensor_msgs::PointCloud pc1_out;

	sensor_msgs::convertPointCloud2ToPointCloud(pc2_in, pc1_in);

	try{
		_tflistener.waitForTransform(target_frame, pc2_in.header.frame_id, target_stamp, ros::Duration(1.0));
		_tflistener.transformPointCloud(target_frame, target_stamp, pc1_in, pc2_in.header.frame_id, pc1_out);
		sensor_msgs::convertPointCloudToPointCloud2(pc1_out, pc2_out);
		return true;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return false;
	}
}

bool SQLidarPlanarNormalEstimation::transformPCWithOdom(const pcl::PointCloud<pcl::PointNormal>::Ptr pc_in, pcl::PointCloud<pcl::PointNormal>::Ptr pc_out, nav_msgs::Odometry odom_last, nav_msgs::Odometry odom_now)
{
	tf::Quaternion q_now;
	tf::Quaternion q_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_now);
	quaternionMsgToTF(odom_last.pose.pose.orientation, q_last);
	tf::Quaternion q_rel_rot = q_last*q_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(
		q_rel_rot.w(),
		q_rel_rot.x(),
		q_rel_rot.y(),
		q_rel_rot.z()
	);
	tf::Quaternion q_global_move(
		odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_last.inverse()*q_global_move*q_last;
	Eigen::Vector3f offset(
		q_local_move.x(),
		q_local_move.y(),
		q_local_move.z()
	);
	pcl::transformPointCloudWithNormals(*pc_in, *pc_out, offset, rotation);
}

void SQLidarPlanarNormalEstimation::eraseNanPoint(void)
{
	for(size_t i=0;i<_pc->points.size();){
		double depth = norm(_pc->points[i].x, _pc->points[i].y, _pc->points[i].z);
		if(depth > _laser_range || std::isnan(depth)){
			_pc->points.erase(_pc->points.begin() + i);
		}
		else	++i;
	}
	_pc->width = _pc->points.size();
}

void SQLidarPlanarNormalEstimation::computeFlatness(void)
{
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	if(!_pc_plane_last->points.empty())	kdtree.setInputCloud(_pc_plane_last);

	for(size_t i=0;i<_pc->points.size();++i){
		if(i < _curvature_region || i >= _pc->points.size()-_curvature_region){
			/* _pc->points[i].curvature = std::nan(""); */
			_pc->points[i].strength = -1;
		}
		else{
			Eigen::Vector3d v_sum_diff = Eigen::Vector3d::Zero();
			Eigen::Vector3d v_abs_diff = Eigen::Vector3d::Zero();
			for(size_t j=i-_curvature_region;j<=i+_curvature_region;++j){
				if(i==j)	continue;

				Eigen::Vector3d v_diff(
					_pc->points[i].x - _pc->points[j].x,
					_pc->points[i].y - _pc->points[j].y,
					_pc->points[i].z - _pc->points[j].z
				);
				v_sum_diff += v_diff;

				if(j < i)	v_abs_diff -= v_diff;
				if(j > i)	v_abs_diff += v_diff;
			}
			double depth = norm(_pc->points[i].x, _pc->points[i].y, _pc->points[i].z);
			_pc->points[i].strength = v_sum_diff.norm()/(depth*depth*2.0*_curvature_region);
			v_abs_diff.normalize();

			if(_pc->points[i].strength < _th_flatness){
				pcl::PointNormal tmp;
				tmp.x = _pc->points[i].x;
				tmp.y = _pc->points[i].y;
				tmp.z = _pc->points[i].z;
				tmp.curvature = _pc->points[i].strength;
				tmp.normal_x = v_abs_diff(0);
				tmp.normal_y = v_abs_diff(1);
				tmp.normal_z = v_abs_diff(2);
				_pc_plane_now->points.push_back(tmp);

				if(!_pc_plane_last->points.empty()){
					if(estimateNormal(kdtree, tmp))	_nc->points.push_back(tmp);
				}
			}
		}
		/* std::cout << "_nc->points.size() = " << _nc->points.size() << std::endl; */
		/* std::cout << "_pc->points[i].strength = " << _pc->points[i].strength << std::endl; */
	}
}

bool SQLidarPlanarNormalEstimation::estimateNormal(pcl::KdTreeFLANN<pcl::PointNormal>& kdtree, pcl::PointNormal& n)
{
	const int k = 1;
	std::vector<int> list_index;
	std::vector<float> list_squareddist;
	kdtree.nearestKSearch(n, k, list_index, list_squareddist);
	std::cout << "list_squareddist[0] = " << list_squareddist[0] << std::endl;
	if(list_squareddist[0] >  _th_assoc_squareddist)	return false;
	Eigen::Vector3d v_now(
		n.normal_x,
		n.normal_y,
		n.normal_z
	);
	Eigen::Vector3d v_last(
		_pc_plane_last->points[list_index[0]].normal_x,
		_pc_plane_last->points[list_index[0]].normal_y,
		_pc_plane_last->points[list_index[0]].normal_z
	);
	Eigen::Vector3d v_rel(
		n.x - _pc_plane_last->points[list_index[0]].x,
		n.y - _pc_plane_last->points[list_index[0]].y,
		n.z - _pc_plane_last->points[list_index[0]].z
	);
	double cross_angle = angleBetweenVectors(v_rel.cross(v_now), v_rel.cross(v_last));
	if(cross_angle/M_PI*180.0 > _th_cross_angle_deg)	return false;
	Eigen::Vector3d v_normal = (v_rel.cross(v_now) + v_rel.cross(v_last)).normalized();
	/*input*/
	n.normal_x = v_normal(0);
	n.normal_y = v_normal(1);
	n.normal_z = v_normal(2);
	return true;
}

void SQLidarPlanarNormalEstimation::publication(void)
{
	/*pc*/
	if(!_pc->points.empty()){
		sensor_msgs::PointCloud2 msg_pc;
		pcl::toROSMsg(*_pc, msg_pc);
		_pub_pc.publish(msg_pc);
	}
	/*plane*/
	if(!_pc_plane_now->points.empty()){
		_pc_plane_now->header.frame_id = _frame_id;
		sensor_msgs::PointCloud2 msg_pc_plane;
		pcl::toROSMsg(*_pc_plane_now, msg_pc_plane);
		_pub_plane.publish(msg_pc_plane);
	}
	/*nc*/
	if(!_nc->points.empty()){
		_nc->header.frame_id = _frame_id;
		sensor_msgs::PointCloud2 msg_nc;
		pcl::toROSMsg(*_nc, msg_nc);
		_pub_nc.publish(msg_nc);
	}
}

void SQLidarPlanarNormalEstimation::visualization(void)
{
	_viewer.removeAllPointClouds();

	/*_pc*/
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::InterestPoint> intensity_distribution(_pc, "strength"); 
	_viewer.addPointCloud<pcl::InterestPoint>(_pc, intensity_distribution, "_pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "_pc");
	/* _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "_pc"); */
	/*_pc_plane_now*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_pc_plane_now, 1, 0.5, "_pc_plane_now");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "_pc_plane_now");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "_pc_plane_now");
	/*_nc*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_nc, 1, 0.5, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "_nc");

	_viewer.spinOnce();
}

double SQLidarPlanarNormalEstimation::norm(double x, double y, double z)
{
	return sqrt(x*x + y*y + z*z);
}

double SQLidarPlanarNormalEstimation::angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
	return acos(v1.dot(v2)/v1.norm()/v2.norm());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_lidar_planar_normal_estimation");
	
	SQLidarPlanarNormalEstimation sq_lidar_planar_normal_estimation;

	ros::spin();
}
