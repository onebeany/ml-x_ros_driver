#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include "ml/libsoslab_ml.h"

namespace mlx_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;        // x, y, z, padding
        float intensity;        
        uint32_t offset_time;   // nanoseconds
        uint16_t ring;          // ring number (row number)
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
  
POINT_CLOUD_REGISTER_POINT_STRUCT(mlx_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, offset_time, offset_time)
    (uint16_t, ring, ring)
)

typedef pcl::PointXYZRGB PointRGB_T;
typedef pcl::PointCloud<PointRGB_T> PointCloudRGB_T;
typedef pcl::PointCloud<mlx_ros::Point> PointCloud_T;

static int g_scan_frequency_hz = 20; // Frequency of LiDAR

int nCols = 0;
int nRows = 0;

int scanCount = 0;

unsigned char soslab_r[] = {3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 41, 42, 42, 43, 43, 43, 44, 44, 45, 45, 46, 46, 46, 47, 47, 48, 48, 49, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 66, 67, 67, 68, 68, 70, 71, 73, 75, 77, 79, 80, 82, 84, 86, 88, 90, 92, 93, 95, 97, 99, 101, 102, 104, 106, 108, 110, 111, 113, 115, 117, 119, 120, 122, 124, 126, 128, 129, 131, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151, 153, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 172, 174, 176, 178, 180, 181, 183, 184, 185, 187, 188, 189, 190, 191, 192, 194, 195, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 212, 213, 214, 215, 216, 217, 218, 219, 221, 222, 223, 224, 225, 226, 227, 228, 230, 231, 232, 233, 234, 235, 236, 237, 239, 240, 241, 242, 243, 244, 245, 246, 248, 249, 250, 251, 252, 253, 254, 254};
unsigned char soslab_g[] = {18, 19, 21, 23, 24, 26, 27, 29, 30, 32, 33, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 58, 60, 61, 63, 64, 66, 67, 69, 71, 72, 74, 75, 77, 78, 80, 81, 83, 84, 86, 88, 89, 91, 92, 94, 95, 97, 98, 100, 101, 103, 105, 106, 108, 109, 111, 112, 114, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 171, 172, 173, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 247, 248, 249, 250, 251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
unsigned char soslab_b[] = {107, 108, 109, 110, 111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 120, 121, 122, 123, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 138, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 157, 158, 159, 160, 161, 162, 162, 163, 163, 163, 164, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 169, 170, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 179, 180, 180, 180, 181, 181, 181, 182, 182, 181, 180, 179, 178, 177, 175, 175, 174, 172, 171, 170, 169, 168, 167, 166, 165, 164, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 121, 120, 119, 118, 117, 116, 115, 114, 113, 111, 109, 107, 105, 104, 102, 100, 98, 96, 95, 93, 91, 89, 88, 86, 84, 82, 81, 79, 77, 75, 73, 72, 70, 68, 66, 65, 63, 61, 59, 58, 56, 54, 52, 51, 49, 47, 45, 43, 42, 40, 38, 36, 35, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 15, 13, 12, 10, 8, 6, 5, 2, 1, 1};

static const char* DEFAULT_IP_ADDR_DEVICE          = "192.168.1.10";
static const unsigned short DEFAULT_IP_PORT_DEVICE = 2000;

static const char* DEFAULT_IP_ADDR_PC              = "0.0.0.0";
static const unsigned short DEFAULT_IP_PORT_PC     = 0;

static const char* DEFAULT_PACKAGE_NAME            = "ml";

// Default publisher template
static const char* DEFAULT_FRAME_ID = "map";

image_transport::Publisher pub_depth;
image_transport::Publisher pub_intensity;
image_transport::Publisher pub_ambient;
ros::Publisher pub_lidar_rgb;
ros::Publisher pub_lidar;

sensor_msgs::ImagePtr msg_ambient;
sensor_msgs::ImagePtr msg_depth;
sensor_msgs::ImagePtr msg_intensity;

PointCloudRGB_T::Ptr msg_pointcloud_rgb(new PointCloudRGB_T);
PointCloud_T::Ptr msg_pointcloud(new PointCloud_T);

int max_ambient_img_val = 30000;
int max_depth_img_val = 10000;
int max_intensity_img_val = 3500;

cv::Mat colormap(cv::Mat image)
{
    nCols = image.cols;
    nRows = image.rows;
    
    int temp_intensity = 0;

    cv::Mat rgb_image(nRows, nCols, CV_8UC3);

    for (int col = 0; col < nCols; col++){
        for (int row = 0; row < nRows; row++){
            temp_intensity = image.at<uint8_t>(row, col);
            float colormap_val0, colormap_val1, colormap_val2;
            
            colormap_val0 = soslab_r[temp_intensity];
            colormap_val1 = soslab_g[temp_intensity];
            colormap_val2 = soslab_b[temp_intensity];

            rgb_image.at<cv::Vec3b>(row, col)[0] = int(colormap_val0);
            rgb_image.at<cv::Vec3b>(row, col)[1] = int(colormap_val1);
            rgb_image.at<cv::Vec3b>(row, col)[2] = int(colormap_val2);
        }
    }
    return rgb_image;
}

void ml_scene_data_callback(void* arg, SOSLAB::LidarML::scene_t& scene)
{
	/*
	// For checking whether ptp works well or not
	uint64_t timestamp = scene.timestamp[55];
	time_t sec = timestamp >> 32;
	int nsec = timestamp & 0xFFFFFFFF;

	struct tm* ml_tm = localtime(&sec);
	std::cout << "time: " << 1900 + ml_tm->tm_year << "."
		<< ml_tm->tm_mon + 1 << "." << ml_tm->tm_mday << " ";
	std::cout << ml_tm->tm_hour << ":" << ml_tm->tm_min << ":"
		<< ml_tm->tm_sec << ":" << nsec << std::endl;
	*/

    std::vector<uint32_t> ambient;
    std::vector<uint16_t> intensity;
    std::vector<uint32_t> depth;
    std::vector<SOSLAB::point_t> pointcloud = scene.pointcloud[0];

    std::size_t height = scene.rows;
    std::size_t width = scene.cols;
    std::size_t width2 = (scene.cols == 192) ? scene.cols*3 : scene.cols;

    /* Ambient Image */
    if(!scene.ambient_image.empty()){
        ambient = scene.ambient_image;

        cv::Mat ambient_image(height, width2, CV_32SC1, ambient.data());

        ambient_image.convertTo(ambient_image, CV_8UC1, (255.0 / (max_ambient_img_val - 0)), 0);
        ambient_image = colormap(ambient_image);
        cv::normalize(ambient_image, ambient_image, 0, 255, cv::NORM_MINMAX);
        if (pub_ambient.getNumSubscribers() > 0) {
            msg_ambient = cv_bridge::CvImage(std_msgs::Header(), "rgb8", ambient_image).toImageMsg();
            pub_ambient.publish(msg_ambient);
        }
    }

    /* Depth Image */
    if(!scene.depth_image.empty()){
        depth = scene.depth_image[0];

        cv::Mat depth_image(height, width, CV_32SC1, depth.data());

        depth_image.convertTo(depth_image, CV_16U);
        depth_image.convertTo(depth_image, CV_8UC1, (255.0 / (max_depth_img_val - 0)), 0);
        
        depth_image = colormap(depth_image);
        cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);
        if(pub_depth.getNumSubscribers() > 0) {
            msg_depth = cv_bridge::CvImage(std_msgs::Header(), "rgb8", depth_image).toImageMsg();
            pub_depth.publish(msg_depth);
        }
    }

    /* Intensity Image */
    cv::Mat intensity_image;
    if(!scene.intensity_image.empty()){
        intensity = scene.intensity_image[0];
        cv::Mat intensity_image_raw(height, width, CV_16UC1, intensity.data());
        intensity_image_raw.convertTo(intensity_image, CV_8UC1, (255.0 / (max_intensity_img_val - 0)), 0);
        intensity_image = colormap(intensity_image);

        if (pub_intensity.getNumSubscribers() > 0) {
            msg_intensity = cv_bridge::CvImage(std_msgs::Header(), "rgb8", intensity_image).toImageMsg();
            pub_intensity.publish(msg_intensity);
        }
    }

    // RGB Point Cloud for visualization
    msg_pointcloud_rgb->header.frame_id = DEFAULT_FRAME_ID;
    msg_pointcloud_rgb->width = width;
    msg_pointcloud_rgb->height = height;
    msg_pointcloud_rgb->points.resize(pointcloud.size());


    // Point Cloud for real usage
    msg_pointcloud->header.frame_id = DEFAULT_FRAME_ID;
    msg_pointcloud->width = width;
    msg_pointcloud->height = height;
    msg_pointcloud->points.resize(pointcloud.size());

	int baseRow = 0;
    uint64_t base_timestamp = scene.timestamp[0];
    for (int row = 1; row < height; row++) {
        if (scene.timestamp[row] < base_timestamp) {
            base_timestamp = scene.timestamp[row];
			baseRow = row;
        }
    }

	/*
	// For checking whether there is a timestamp outlier
	// The desired timestamp of each row should be ascending order from row[0] to row[55]
	if(baseRow != 0){
		std::cout << "scan[" << scanCount << "] | base timestamp(row[" << baseRow << "]): " << base_timestamp << std::endl;
	std::cout << "Current ros time: " << ros::Time::now() << std::endl;
	}
	*/
	
	// scan period and point interval calculation
    // scan_period_ns = 1e9 / frequency
    // point_interval_ns = scan_period_ns / (rows * cols)
    uint64_t scan_period_ns    = 1000000000ULL / g_scan_frequency_hz;
    uint64_t total_points      = static_cast<uint64_t>(height) * width;
    uint32_t point_interval_ns = static_cast<uint32_t>(scan_period_ns / total_points);

    for (int row=0; row < height; row++) {

		uint64_t row_offset = scene.timestamp[row] - base_timestamp;

        for (int col = 0; col < width; col++) {

            int idx = col + (width * row);
  
            //unit : (m)
            float x = pointcloud[idx].x / 1000.0;
            float y = pointcloud[idx].y / 1000.0;
            float z = pointcloud[idx].z / 1000.0;
            
            // 1. RGB Point Cloud data setting
            msg_pointcloud_rgb->points[idx].x = x;
            msg_pointcloud_rgb->points[idx].y = y;
            msg_pointcloud_rgb->points[idx].z = z;

            if(!scene.intensity_image.empty()){
                msg_pointcloud_rgb->points[idx].r = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[0]);
                msg_pointcloud_rgb->points[idx].g = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[1]);
                msg_pointcloud_rgb->points[idx].b = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[2]);
            } else {
                msg_pointcloud_rgb->points[idx].r = 255;
                msg_pointcloud_rgb->points[idx].g = 255;
                msg_pointcloud_rgb->points[idx].b = 255;
            }
            

            // 2. Point Cloud data setting
            msg_pointcloud->points[idx].x = x;
            msg_pointcloud->points[idx].y = y;
            msg_pointcloud->points[idx].z = z;
            
            if(!scene.intensity_image.empty()) {
                msg_pointcloud->points[idx].intensity = static_cast<float>(scene.intensity_image[0][idx]);
            } else {
                msg_pointcloud->points[idx].intensity = 0.0f;
            }
            
            // time offset setting
			uint32_t offset_ns = static_cast<uint32_t>(row_offset + col * point_interval_ns);
            msg_pointcloud->points[idx].offset_time = offset_ns;
            
			// std::cout << "row[" << row << "] offset_time: " << time_offset_ns << std::endl;

            // ring setting (row number)
            msg_pointcloud->points[idx].ring = static_cast<uint16_t>(row);
        }
    }
	
	// Convert LiDAR ts to ROS ts
	ros::Time sensor_time;
	sensor_time.sec = base_timestamp >> 32;
	sensor_time.nsec = base_timestamp & 0xFFFFFFFF;

    pcl_conversions::toPCL(sensor_time, msg_pointcloud_rgb->header.stamp);
    pcl_conversions::toPCL(sensor_time, msg_pointcloud->header.stamp);
    pub_lidar_rgb.publish(msg_pointcloud_rgb);
    pub_lidar.publish(msg_pointcloud);

	scanCount++;
}

int main (int argc, char **argv)
{
    bool success;
    /* ROS node init */
    ros::init(argc, argv, DEFAULT_PACKAGE_NAME);

    /* FPS 10 */
    bool fps10_enable                     = false;

    /* Depth Completion */
    bool depth_completion_enable          = false;

    /* Data Selection */
    bool ambient_enable                   = true;
    bool depth_enable                     = true;
    bool intensity_enable                 = true;

    /* get parameters */
    ros::NodeHandle nh("~");

    nh.param<bool>("fps10", fps10_enable, false);
    nh.param<bool>("depth_completion_enable", depth_completion_enable, false);

    nh.param<bool>("ambient_enable", ambient_enable, true);
    nh.param<bool>("depth_enable", depth_enable, true);
    nh.param<bool>("intensity_enable", intensity_enable, true);

    /* publisher setting */
    image_transport::ImageTransport it(nh);

    pub_depth = it.advertise("depth_color", 1);
    pub_intensity = it.advertise("intensity_color", 1);
    pub_ambient = it.advertise("ambient_color", 1);

    pub_lidar_rgb = nh.advertise<PointCloudRGB_T>("pointcloud_rgb", 10);
    pub_lidar = nh.advertise<PointCloud_T>("pointcloud", 10);

    SOSLAB::ip_settings_t ip_settings_device;
    SOSLAB::ip_settings_t ip_settings_pc;

    nh.param<std::string>("ip_address_device", ip_settings_device.ip_address, DEFAULT_IP_ADDR_DEVICE);
    nh.param<int>("ip_port_device", ip_settings_device.port_number, DEFAULT_IP_PORT_DEVICE);
    nh.param<std::string>("ip_address_pc", ip_settings_pc.ip_address, DEFAULT_IP_ADDR_PC);
    nh.param<int>("ip_port_pc", ip_settings_pc.port_number, DEFAULT_IP_PORT_PC);

    std::shared_ptr<SOSLAB::LidarML> lidar_ml(new SOSLAB::LidarML);

    std::cout << lidar_ml->api_info() << std::endl;
    std::cout << "> ip_address_device: " << ip_settings_device.ip_address << std::endl;
    std::cout << "> ip_port_device: " << ip_settings_device.port_number << std::endl;
    std::cout << "> ip_address_pc: " << ip_settings_pc.ip_address << std::endl;
    std::cout << "> ip_port_pc: " << ip_settings_pc.port_number << std::endl;

	success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
	if (!success) {
		std::cerr << "LiDAR ML :: connection failed." << std::endl;
		return 0;
	}

    lidar_ml->ambient_enable(ambient_enable);       //Ambient enable (True / False)
    lidar_ml->depth_enable(depth_enable);           //Depth enable (True / False)
    lidar_ml->intensity_enable(intensity_enable);   //Intensity enable (True / False)

    /* FPS 10 */
    lidar_ml->fps10(fps10_enable);
	g_scan_frequency_hz = fps10_enable ? 10 : 20;
    /* Depth Completion */
    lidar_ml->depth_completion(depth_completion_enable);
/*  
	// If you want to syncronize lidar's inner clock with system clock, (the base time will be system clock) comment out following code:
    // TODO) Figure out the affection of this function with PTP
	success = lidar_ml->sync_localtime();
    if (success) {
        ROS_INFO("Lidar time synchronized with host computer");
    } else {
        ROS_WARN("Failed to synchronize lidar time");
    }
*/

    lidar_ml->register_scene_callback(ml_scene_data_callback, nullptr);

	// For syncronizing multi LiDARs
	
	bool use_sync_start = false;
	int expected_nodes = 1;
	double sync_timeout = 10.0;

	ros::param::get("use_sync_start", use_sync_start);
	ros::param::get("expected_nodes", expected_nodes); 
	ros::param::get("sync_timeout", sync_timeout);

	if(use_sync_start && expected_nodes >1){

		std::string node_name = ros::this_node::getName();
		ros::param::set(node_name + "/sync_ready", true);

		ROS_INFO("%s is ready", node_name.c_str());

		ros::Time start_wait = ros::Time::now();
	
		bool all_ready = false;

		while(!all_ready){
			all_ready = true;

			for(int i = 0 ; i < expected_nodes; i++){
				std::string check_node;
				std::string param_name = "/sync_node_list/" + std::to_string(i);

				if(ros::param::get(param_name, check_node) && check_node != node_name){
					bool node_ready = false;
					if(!ros::param::get(check_node + "/sync_ready", node_ready) || !node_ready){
						all_ready = false;
						break;
					}
				}
			}

			if((ros::Time::now() - start_wait).toSec() > sync_timeout){
				ROS_ERROR("[%s} Sync Timeout! Terminate the node.", node_name.c_str());
				ros::shutdown();
				exit(1);
			}
		}
		ROS_INFO("[%s] All nodes are ready. Start LiDAR scan...", node_name.c_str());
	}

	success = lidar_ml->run();

	if (!success) {
		std::cerr << "[" << (ros::this_node::getName()).c_str() << "] LiDAR ML :: run failed." << std::endl;
	}
	else {
		std::cout  << "[" << (ros::this_node::getName()).c_str() << "] LiDAR ML :: run." << std::endl;
	}
    std::cout << "[" << (ros::this_node::getName()).c_str() << "] LiDAR ML :: Streaming started!" << std::endl;

    /* publishing start */
    ros::Rate r(50);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    lidar_ml->stop();
    std::cout << "[" << (ros::this_node::getName()).c_str() << "] Streaming stopped!" << std::endl;

    lidar_ml->disconnect();

    std::cout << "Done." << std::endl;

    return 0;
}
