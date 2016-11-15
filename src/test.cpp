#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <typeinfo>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

cv::MatND hist_base1, hist_base2;
cv::MatND hist1, hist2;
bool noHistory = true;
int h_bins = 50;
int s_bins = 60;
int histSize[] = {h_bins,s_bins};
float hranges[] = {0, 180};
float sranges[] = {0, 256};
const float* ranges1[] = {hranges, sranges};
const float* ranges2[] = {sranges, sranges, sranges};
int channels[] = {0,1};
double result1=0.0, result2=0.0, result3 = 0.0;
double result11=0.0, result12=0.0, result13 = 0.0;


void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
	//std::cout << "# " << cloud->width << std::endl;
	//for(int i = 0; i < 10; i++){
	//std::cout << i << " " << cloud->data[i] << std::endl; 
	//}
	PointCloudT::Ptr pcl_cloud( new PointCloudT);
	pcl::fromROSMsg(*cloud, *pcl_cloud);
	ROS_INFO("Point Cloud Received! size: %lu", pcl_cloud->size());
	PointCloudT::iterator it;
	std::vector<cv::Point3d> color_vector;
	int cnt = 10000;
	for (it = pcl_cloud->begin(); it != pcl_cloud->end(); it++){
	//	std::cout << typeid(*it).name() << std::endl;
		PointT p = *it;
		uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
		uint8_t r = (rgb>>16)&0x0000ff;
		uint8_t g = (rgb>>8)&0x0000ff;
		uint8_t b = (rgb)&0x0000ff;
		if (cnt-- > 0);
		//std::cout << +r << " " << +g << " " << +b << std::endl;
		color_vector.push_back(cv::Point3d(r,g,b));
	}
	cv::Mat img_raw(color_vector, false), ycc_base, hsv_base;
	cv::Mat img;
	img_raw.convertTo(img, CV_8U);
	cvtColor(img, ycc_base, cv::COLOR_RGB2YCrCb);
	cvtColor(img, hsv_base, cv::COLOR_RGB2HSV);
	cv::calcHist(&ycc_base, 1, channels, cv::Mat(), hist1, 2, histSize, ranges1, true, false);
	cv::calcHist(&hsv_base, 1, channels, cv::Mat(), hist2, 2, histSize, ranges2, true, false);
	if (noHistory){
		hist_base1 = hist1.clone();
		hist_base2 = hist2.clone();
		noHistory = false;
		ROS_INFO("Initializing...");
	}
	else {
		double dist1 = cv::compareHist(hist1, hist_base1, CV_COMP_CORREL);
		ROS_INFO("result 1: %.5lf d: %.5lf", dist1, fabs(dist1-result1));
		double dist2 = 1- cv::compareHist(hist1, hist_base1, CV_COMP_BHATTACHARYYA);
		ROS_INFO("result 4: %.5lf d: %.5lf", dist2, fabs(dist2-result2));
		//double dist3 = (dist1+dist2)/2;
		//ROS_INFO("result c: %.5lf d: %.5lf", dist3, fabs(dist3-result3));
		result1 = dist1;
		result2 = dist2;
		//result3 = dist3;
		double dist11 = cv::compareHist(hist2, hist_base2, CV_COMP_CORREL);
		ROS_INFO("result 1: %.5lf d: %.5lf", dist11, fabs(dist11-result11));
		double dist12 = 1- cv::compareHist(hist2, hist_base2, CV_COMP_BHATTACHARYYA);
		ROS_INFO("result 4: %.5lf d: %.5lf", dist12, fabs(dist12-result12));
		//double dist13 = (dist11+dist12)/2;
		//ROS_INFO("result c: %.5lf d: %.5lf", dist13, fabs(dist13-result13));
		result11 = dist11;
		result12 = dist12;
		//result13 = dist13;
		hist_base1 = hist1.clone();
		hist_base2 = hist2.clone();
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"human_clouds_receiver");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("segbot_pcl_person_detector/human_clouds", 1000, callback);
	ROS_INFO("Started. Waiting for person pointcloud...");
	ros::spin();
	return 0;
}
