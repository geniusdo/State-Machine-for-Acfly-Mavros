#include <boost/sml.hpp>

#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#define SphereDis 0.03

namespace sml = boost::sml;

namespace drone
{
    struct TFtarget
    {
        tf::Vector3 position = tf::Vector3(0, 0, 0);
        tf::Quaternion rotation = tf::Quaternion(0, 0, 0, 1);
        void SetPosition(double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(0, 0, 0, 1);
        }
        void SetTarget(double p, double i, double j, double k, double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(p, i, j, k);
        }
        
    };

    struct dependencies
    {
        mavros_msgs::State current_state;
        ros::NodeHandle n;
        ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, [&](const mavros_msgs::State::ConstPtr &msg)
                                                                    { current_state = *msg; });
        ros::Publisher target_pub;
        ros::ServiceClient client;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        TFtarget _target;

        void Set2Offboard()
        {
            mavros_msgs::CommandSetMode offb_set_mode;
            offb_set_mode.request.base_mode = 0;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
            client.call(offb_set_mode);
        }

        void Set2Armed(bool open)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = open;
            client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            client.call(arm_cmd);
        }

        void openTFlisten(bool open)
        {
            client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
            mavros_msgs::SetTFListen tf_listen;
            tf_listen.request.value = open;
            client.call(tf_listen);
        }

        void getTransform(std::string target, std::string base)
        {

            try
            {
                listener.lookupTransform(target, base, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        bool is_close(tf::Vector3 current_pos, tf::Vector3 target_pos, double threshold)
        {
            tfScalar dis = tf::tfDistance(current_pos, target_pos);
            if (dis <= threshold)
                return true;
            else
                return false;
        }
        bool is_rotated(tf::Quaternion current_quat, tf::Quaternion target_quat, double threshold)
        {
            tfScalar angle = current_quat.angleShortestPath(target_quat);
            if (angle <= threshold)
                return true;
            else
                return false;
        }
    };

    // event
    struct release
    {
    };
    struct lock
    {
    };
    struct switchTFmode
    {
        int mode = 0;
    };
    struct moveTo
    {
        TFtarget target;
    };
    struct tickOnce
    {
        TFtarget target;
    };

    struct icarus
    {

        auto operator()() const
        {
            using namespace sml;

            // event

            // guard
            auto is_init = [](const dependencies &d, ros::Rate _rate)
            {
                ros::spinOnce();
                _rate.sleep();
                if (ros::ok())
                {
                    while (!d.current_state.connected)
                    {
                        ros::spinOnce();
                        _rate.sleep();
                    }
                    return true;
                }
                else
                {
                    ROS_INFO("not connected");
                    return false;
                }
            };

            auto is_valid = [](dependencies &d, ros::Rate _rate)
            {
                ros::spinOnce();
                _rate.sleep();
                if (ros::ok())
                {
                    while (d.current_state.mode != "OFFBOARD")
                    {
                        ros::spinOnce();
                        _rate.sleep();
                        d.Set2Offboard();
                        d.Set2Armed(true);
                        d.openTFlisten(true);
                    }
                    return true;
                }

                else
                {
                    return false;
                }
            };

            auto is_arrive = [](dependencies &d)
            {
                d.getTransform("map", "base_link");
                if (d.is_close(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d.transform.getOrigin().z()),
                               d._target.position, SphereDis) &&
                    d.is_rotated(d.transform.getRotation(), d._target.rotation, 2))
                    return true;
                else
                    return false;
            };

            // action
            auto unlock = [](dependencies &d)
            {
                d.Set2Offboard();
                d.Set2Armed(true);
                d.openTFlisten(true);
            };

            auto setTFtarget = [](dependencies &d)
            {
                d.broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(d._target.rotation, d._target.position),
                        ros::Time::now(), "map", "target_position"));
            };

            auto delay = [](double time)
            {
                ros::Duration(time).sleep();
            };

            return make_transition_table(
                *"idle"_s + event<release>[is_init] / unlock = "ready"_s,
                "ready"_s + event<moveTo>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d._target = e.target; },
                                                       setTFtarget) = "moving"_s,
                // "moving"_s + "endmov"_e[is_valid] / [] {} = "ready"_s,
                "moving"_s + event<tickOnce>[is_arrive] / [] {} = "ready"_s,
                "moving"_s + event<tickOnce>[!is_arrive && is_valid] / setTFtarget = "moving"_s,
                "ready"_s + event<lock> / [] {} = X

            ); //
        }
    };
};
