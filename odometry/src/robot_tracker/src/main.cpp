#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <robotics_hw1/MotorSpeed.h>
#include <robot_tracker/CustomMessage.h>
#include <robot_tracker/parametersConfig.h>
#include <robot_tracker/SetPose.h>
#include <robot_tracker/ResetPose.h>

#include <math.h>
#include "SimpleRobot.h"

#define GEAR_RATIO          1.0 / 37.0
#define WHEEL_RADIUS        0.1575
#define REAL_BASELINE       0.583
#define APPARENT_BASELINE   1.08

/* This class is used for invoking ROS functionalities while hiding its details from the rest of the code */
class ROS_Interface {
public:
    ROS_Interface(SimpleRobot* robot) :
        /* Initialize the subscribers to the motor topics and the synchronizer */
        sub_fl(node, "motor_speed_fl", 1),
        sub_rl(node, "motor_speed_rl", 1),
        sub_fr(node, "motor_speed_fr", 1),
        sub_rr(node, "motor_speed_rr", 1),
        sync(SyncPolicy(10), sub_fl, sub_rl, sub_fr, sub_rr),
        /* Initialize the publishers */
        pub_estim_twist(node.advertise<geometry_msgs::TwistStamped>("estim_twist", 1000)),
        pub_estim_odom(node.advertise<nav_msgs::Odometry>("estim_odom", 1000)),
        pub_custom_message(node.advertise<robot_tracker::CustomMessage>("custom_message", 1000)),      
        /* Initialize the services */
        srv_setPose(node.advertiseService<robot_tracker::SetPose::Request, robot_tracker::SetPose::Response>("set_pose", boost::bind(&setPose_callback, _1, _2, robot))),
        srv_resetPose(node.advertiseService<robot_tracker::ResetPose::Request, robot_tracker::ResetPose::Response>("reset_pose", boost::bind(&resetPose_callback, _1, _2, robot)))
    {
        /* Set up callback for synchronizing motor subscribers */
        sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3, _4, this, robot)); 
        /* Set up callback for dynamic reconfiguration */
        param_server.setCallback(boost::bind(&param_callback, _1, _2, robot));
        /* Fetch the initial state from the static launch parameters and set up the robot */
        double initial_x, initial_y, initial_theta;
        node.getParam("/initial_x", initial_x);
        node.getParam("/initial_y", initial_y);
        node.getParam("/initial_theta", initial_theta);
        robot->set(initial_x, initial_y, initial_theta);
    }

    /* Publishes the estimated twist of the robot (forward velocity and angular velocity) */
    void publish_estim_twist(double forward_velocity, double angular_velocity) {
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = ros::Time::now();
        twist.header.frame_id = "odom";
        twist.twist.linear.x = forward_velocity;
        twist.twist.angular.z = angular_velocity;  
        pub_estim_twist.publish(twist);
    }

    /* Publishes the estimated odometry message, TF and custom message */
    void publish_estim_odom_tf(double x, double y, double theta, double forward_velocity, double angular_velocity, int integration_method) {   
        ros::Time time = ros::Time::now();  
        geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(theta);

        /* Odometry message */
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = rotation;
        odom.twist.twist.linear.x = forward_velocity;
        odom.twist.twist.angular.z = angular_velocity;
        pub_estim_odom.publish(odom);

        /* TF message, with header */
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.rotation = rotation;
        broadcaster_odom_tf.sendTransform(tf);

        /* Custom message containing odometry and integration method */
        robot_tracker::CustomMessage custom_msg;
        custom_msg.odom = odom;
        if (integration_method == INTEGRATION_EULER) {
            custom_msg.method.data = std::string("euler");
        }
        else if (integration_method == INTEGRATION_RUNGEKUTTA) {
            custom_msg.method.data = std::string("rk");
        }
        else {
            custom_msg.method.data = std::string("INVALID_METHOD (Something went wrong!)");
        }
		pub_custom_message.publish(custom_msg);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
        robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> SyncPolicy;

    ros::NodeHandle node;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rr;
    message_filters::Synchronizer<SyncPolicy> sync;

    ros::Publisher pub_estim_twist;
    ros::Publisher pub_estim_odom;
    ros::Publisher pub_custom_message;
    tf::TransformBroadcaster broadcaster_odom_tf;

    dynamic_reconfigure::Server<robot_tracker::parametersConfig> param_server;
    dynamic_reconfigure::Server<robot_tracker::parametersConfig>::CallbackType param_callback_func;

    ros::ServiceServer srv_setPose;
    ros::ServiceServer srv_resetPose;

    double timestamp;

    /* This function is called whenever the four motor speeds are received */
    static void sync_callback(
        const robotics_hw1::MotorSpeed::ConstPtr& msg_fl, const robotics_hw1::MotorSpeed::ConstPtr& msg_rl,
        const robotics_hw1::MotorSpeed::ConstPtr& msg_fr, const robotics_hw1::MotorSpeed::ConstPtr& msg_rr, 
        ROS_Interface* ros_interface, SimpleRobot* robot
    ) {
        /* Average the timestamp of the received motor messages */
        double timestamp_left = (msg_fl->header.stamp.toSec() + msg_rl->header.stamp.toSec()) / 2;
        double timestamp_right = (msg_fr->header.stamp.toSec() + msg_rr->header.stamp.toSec()) / 2;
        double timestamp = (timestamp_left + timestamp_right) / 2; 
        double deltaTime = timestamp - ros_interface->timestamp;
        
        /* Compute forward velocity and angular velocity from the four motor speeds */
        double angular_velocity_left = (msg_fl->rpm + msg_rl->rpm) * M_PI / 60.0;
        double angular_velocity_right = (msg_fr->rpm + msg_rr->rpm) * M_PI / 60.0;
        double linear_left = -angular_velocity_left * WHEEL_RADIUS * GEAR_RATIO;
        double linear_right = angular_velocity_right * WHEEL_RADIUS * GEAR_RATIO;
        double forward_velocity = (linear_left + linear_right) / 2;
        double angular_velocity = (linear_right - linear_left) / APPARENT_BASELINE;

        /* Integrate the velocities and publish the messages */
        robot->step(deltaTime, forward_velocity, angular_velocity);
        ros_interface->publish_estim_twist(robot->velocity.forward, robot->velocity.angular);
        ros_interface->publish_estim_odom_tf(robot->state.x, robot->state.y, robot->state.theta, robot->velocity.forward, robot->velocity.angular, robot->integration_method);

        /* Update last timestamp received */
        ros_interface->timestamp = timestamp;
    }

    /* This function is called whenever the parameter server is invoked for dynamic reconfigure */
    static void param_callback(robot_tracker::parametersConfig& config, uint32_t level, SimpleRobot* robot) {
        ROS_INFO("Reconfigure Request: %d", config.method);
        robot->integration_method = config.method;
    }

    /* This function is called whenever the service for changing the robot pose is invoked */
    static bool setPose_callback(robot_tracker::SetPose::Request& req, robot_tracker::SetPose::Response& res, SimpleRobot* robot) {
        res.request_successful = true;
        robot->set(req.x, req.y, req.theta);
        return true;
    }

    /* This function is called whenever the service for resetting the robot pose is invoked */
    static bool resetPose_callback(robot_tracker::ResetPose::Request& req, robot_tracker::ResetPose::Response& res, SimpleRobot* robot) {
        res.request_successful = true;
        robot->set(0.0, 0.0, 0.0);
        return true;
    }
};

int main(int argc, char** argv) {
    /* Initialization phase */
    ros::init(argc, argv, "robot_tracker_node");
    SimpleRobot robot;
    ROS_Interface ros_interface(&robot);

    /* Enter node loop */
    ros::spin();

    return 0;
}
