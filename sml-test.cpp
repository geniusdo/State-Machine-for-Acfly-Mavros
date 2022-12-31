//#include <iostream> //used for testing
#include "ros/ros.h"
#include "../include/state_machine.hpp"
#include "../include/locate_elg.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "tf/transform_datatypes.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
namespace sml = boost::sml;
using namespace message_filters;

cv::Mat left_image;
cv::Mat right_image;

void image_callback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img)
{
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::TYPE_8UC1);
    left_image = cv_ptr1->image;
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::TYPE_8UC1);
    right_image = cv_ptr2->image;
}

int main(int argc, char **argv)
{
    using namespace sml;
    ros::init(argc, argv, "simple_fly");
    drone::dependencies d;
    ros::NodeHandle nh;
    Eigen::Matrix<double, 3, 3> K_left, K_right;
    double angle;
    K_left << 427.2486928456254, 0, 425.06874497136005, 0, 427.74610622571066, 236.16152744508838, 0, 0, 1;
    K_right << 425.81737868037914, 0, 424.41982264160583, 0, 426.2683663190262, 235.4487746155112, 0, 0, 1;
    message_filters::Subscriber<sensor_msgs::Image> sub_left_image(nh, "/camera/infra1/image_rect_raw", 2000, ros::TransportHints().tcpNoDelay());

    message_filters::Subscriber<sensor_msgs::Image> sub_right_image(nh, "/camera/infra2/image_rect_raw", 2000, ros::TransportHints().tcpNoDelay());

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_left_image, sub_right_image);
    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    sm.process_event(drone::release{});
    drone::TFtarget target;

    bool is_detected;
    geometry_msgs::Quaternion rotate;
    tf::Quaternion tf_rotate;
    std::vector<cv::Point3d> normal;

    target.SetPosition(0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }

    target.SetTarget(0, 0, 0, 1, 0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }

    normal = zuanquan(left_image, right_image, K_left, K_right, is_detected);
    ROS_INFO("1");
    while (!is_detected)
    {
        normal = zuanquan(left_image, right_image, K_left, K_right, is_detected);
        ros::Duration(0.5).sleep();
        ROS_INFO("detecting!");
    }
    angle = -atan(normal[0].x / normal[0].z);
    double a_b = qumo(normal[0]);
    
    cv::Point3d target_pos = cv::Point3d(normal[1].x - normal[0].x / a_b * 0.5, 1.1, normal[1].z - normal[0].z / a_b * 0.5);
    rotate = tf::createQuaternionMsgFromYaw(angle);
    tf::quaternionMsgToTF(rotate, tf_rotate);
    tf::Quaternion new_rotate = tf::Quaternion(0, 0, 0, 1) * tf_rotate;
    target.SetPosition(target_pos.z, -target_pos.x, 1.1);
    ROS_INFO("x:%f, y:%f",target_pos.z, -target_pos.x);
    //target.SetPosition(0, 0, 1.1);
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }
    ros::Duration(10).sleep();
    target_pos = cv::Point3d(normal[1].x + normal[0].x / a_b * 0.5, 1.1, normal[1].z + normal[0].z / a_b * 0.5);
    target.SetPosition(target_pos.z, -target_pos.x, 1.1);
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }

    target.SetPosition(target_pos.z, -target_pos.x, -0.2);
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }

    sm.process_event(drone::lock{});
    return 0;
}
