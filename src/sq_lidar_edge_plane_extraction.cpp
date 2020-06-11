#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class SQLidarEdgePlaneExtraction{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		/*publisher*/
		ros::Publisher _pub_pc;
		ros::Publisher _pub_edge;
		ros::Publisher _pub_plane;
		/*tf*/
		tf::TransformListener tflistener;
		/*point cloud*/
		pcl::PointCloud<pcl::InterestPoint>::Ptr _pc {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr _pc_edge {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr _pc_plane {new pcl::PointCloud<pcl::InterestPoint>};
		/*parameter*/
		const double _laser_range = 40.0;
		std::string _frame_id;
		int _curvature_region;
		double _th_flatness_edge;
		double _th_flatness_plane;

	public:
		SQLidarEdgePlaneExtraction();
		void reset(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		bool transformPCFrame(const sensor_msgs::PointCloud2& pc_in, sensor_msgs::PointCloud2& pc_put);
		void eraseNanPoint(void);
		void computeFlatness(void);
		void publication(void);
		double norm(double x, double y, double z);
};

SQLidarEdgePlaneExtraction::SQLidarEdgePlaneExtraction()
	: _nhPrivate("~")
{
	_nhPrivate.param("frame_id", _frame_id, std::string("/base_link"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("curvature_region", _curvature_region, 5);
	std::cout << "_curvature_region = " << _curvature_region << std::endl;
	_nhPrivate.param("th_flatness_edge", _th_flatness_edge, 1.0);
	std::cout << "_th_flatness_edge = " << _th_flatness_edge << std::endl;
	_nhPrivate.param("th_flatness_plane", _th_flatness_plane, 1.0e-3);
	std::cout << "_th_flatness_plane = " << _th_flatness_plane << std::endl;

	_sub_pc = _nh.subscribe("/cloud", 1, &SQLidarEdgePlaneExtraction::callbackPC, this);
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/with_flatness", 1);
	_pub_edge = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/edge", 1);
	_pub_plane = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
}

void SQLidarEdgePlaneExtraction::reset(void)
{
	_pc->points.clear();
	_pc_edge->points.clear();
	_pc_plane->points.clear();
}

void SQLidarEdgePlaneExtraction::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "----------" << std::endl;

	reset();
	sensor_msgs::PointCloud2 pc_trans;
	if(!transformPCFrame(*msg, pc_trans))	return;
	pcl::fromROSMsg(pc_trans, *_pc);
	eraseNanPoint();

	std::cout << "_pc-header.frame_id = " << _pc->header.frame_id << std::endl;
	std::cout << "_pc->points.size() = " << _pc->points.size() << std::endl;

	computeFlatness();
	publication();
}

bool SQLidarEdgePlaneExtraction::transformPCFrame(const sensor_msgs::PointCloud2& pc2_in, sensor_msgs::PointCloud2& pc2_out)
{
	sensor_msgs::PointCloud pc1_in;
	sensor_msgs::PointCloud pc1_out;

	sensor_msgs::convertPointCloud2ToPointCloud(pc2_in, pc1_in);

	try{
		tflistener.waitForTransform(_frame_id, pc2_in.header.frame_id, pc2_in.header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud(_frame_id, pc2_in.header.stamp, pc1_in, pc2_in.header.frame_id, pc1_out);
		sensor_msgs::convertPointCloudToPointCloud2(pc1_out, pc2_out);
		return true;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return false;
	}
}

void SQLidarEdgePlaneExtraction::eraseNanPoint(void)
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

void SQLidarEdgePlaneExtraction::computeFlatness(void)
{
	for(size_t i=0;i<_pc->points.size();++i){
		if(i < _curvature_region || i >= _pc->points.size()-_curvature_region){
			/* _pc->points[i].strength = std::nan(""); */
			_pc->points[i].strength = -1;
		}
		else{
			double diff_x = 0.0;
			double diff_y = 0.0;
			double diff_z = 0.0;
			for(size_t j=i-_curvature_region;j<=i+_curvature_region;++j){
				diff_x += _pc->points[i].x - _pc->points[j].x;
				diff_y += _pc->points[i].y - _pc->points[j].y;
				diff_z += _pc->points[i].z - _pc->points[j].z;
			}
			double depth = norm(_pc->points[i].x, _pc->points[i].y, _pc->points[i].z);
			double flatness = norm(diff_x, diff_y, diff_z)/(depth*2*_curvature_region);
			_pc->points[i].strength = flatness;

			if(_pc->points[i].strength > _th_flatness_edge)	_pc_edge->points.push_back(_pc->points[i]);
			else if(_pc->points[i].strength < _th_flatness_plane)	_pc_plane->points.push_back(_pc->points[i]);
		}
		std::cout << "_pc->points[i].strength = " << _pc->points[i].strength << std::endl;
	}
}

void SQLidarEdgePlaneExtraction::publication(void)
{
	/*pc*/
	// _pc->header.frame_id = _frame_id;
	sensor_msgs::PointCloud2 msg_pc;
	pcl::toROSMsg(*_pc, msg_pc);
	_pub_pc.publish(msg_pc);
	/*edge*/
	_pc_edge->header.frame_id = _frame_id;
	sensor_msgs::PointCloud2 msg_pc_edge;
	pcl::toROSMsg(*_pc_edge, msg_pc_edge);
	_pub_edge.publish(msg_pc_edge);
	/*plane*/
	_pc_plane->header.frame_id = _frame_id;
	sensor_msgs::PointCloud2 msg_pc_plane;
	pcl::toROSMsg(*_pc_plane, msg_pc_plane);
	_pub_plane.publish(msg_pc_plane);
}

double SQLidarEdgePlaneExtraction::norm(double x, double y, double z)
{
	return sqrt(x*x + y*y + z*z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sqlidar_edge_plane_extraction");
	
	SQLidarEdgePlaneExtraction sqlidar_edge_plane_extraction;

	ros::spin();
}
