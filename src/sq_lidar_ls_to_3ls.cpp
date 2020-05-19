#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class SQLidarLSTo3LS{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_ls;
		/*publish*/
		std::vector<ros::Publisher> vec_pub_ls;
		/*frame id*/
		std::vector<std::string> laser_id{
			"laser1_link",
			"laser2_link",
			"laser3_link"
		};
	public:
		SQLidarLSTo3LS();
		void CallbackLS(const sensor_msgs::LaserScanConstPtr& msg);
};

SQLidarLSTo3LS::SQLidarLSTo3LS()
	: nhPrivate("~")
{
	sub_ls = nh.subscribe("/sq_lidar/scan", 1, &SQLidarLSTo3LS::CallbackLS, this);
	for(size_t i=0;i<laser_id.size();++i){
		std::cout << "laser_id[" << i << "] = " << laser_id[i] << std::endl;
		vec_pub_ls.push_back(nh.advertise<sensor_msgs::LaserScan>("/sq_lidar/scan/" + laser_id[i], 1));
	}
}

void SQLidarLSTo3LS::CallbackLS(const sensor_msgs::LaserScanConstPtr& msg)
{
	bool exists_other_frame = true;
	for(size_t i=0;i<laser_id.size();++i){
		if(msg->header.frame_id == laser_id[i]){
			sensor_msgs::LaserScan output = *msg;
			vec_pub_ls[i].publish(output);
			exists_other_frame = false;
		}
	}
	if(exists_other_frame){
		std::cout << "exists_other_frame = " << exists_other_frame << ":" << msg->header.frame_id << std::endl;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_lidar_ls_to_3ls");
	
	SQLidarLSTo3LS sq_lidar_ls_to_3ls;

	ros::spin();
}
