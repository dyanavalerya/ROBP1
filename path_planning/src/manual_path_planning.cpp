#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include "nav_msgs/Odometry.h"

bool stored = false;

class Route
{
private:
    unsigned int stops_initialized = 0;
    geometry_msgs::PointStamped from, to, to2, firstPosition;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    ros::Publisher marker_pub;
    ros::Subscriber click_sub;
    ros::Subscriber odom_sub;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position = msg->pose.pose.position;
        goal.target_pose.header.frame_id = msg->header.frame_id;
        if(stored == false) {
          firstPosition.header.frame_id = goal.target_pose.header.frame_id;
          firstPosition.point.x = goal.target_pose.pose.position.x;
          stored = true;
        }
    }

    void _send_goal(const geometry_msgs::PointStamped& goal_point)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.w = 1.0;
        client.sendGoal(goal,
            boost::bind(&Route::_target_reached_cb, this, _1, _2));
        _send_markers();
    }

    int q = 0;

    void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
      switch(q) {
      case 0  :
          _send_goal(to);
          break; 
      case 1  :
          _send_goal(from);
          break; 

      case 2  :
          _send_goal(to2);
          break;
      case 3 : 
          _send_goal(firstPosition);
          break;
        }
        q += 1;
    }

    void _send_markers()
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;
        marker.lifetime = ros::Duration();

        visualization_msgs::MarkerArray marker_array;
        if (stops_initialized > 0)
        {
            marker.header.frame_id = to.header.frame_id;
            marker.id = 0;
            marker.pose.position = to.point;
            marker.pose.position.z += marker.scale.x;
            marker_array.markers.push_back(marker);
        }
        if (stops_initialized > 1)
        {
            marker.header.frame_id = from.header.frame_id;
            marker.id = 1;
            marker.pose.position = from.point;
            marker.pose.orientation.w = -0.7071;
            marker_array.markers.push_back(marker);
        }
        if (stops_initialized > 2)
        {
            marker.header.frame_id = to2.header.frame_id;
            marker.id = 2;
            marker.pose.position = to2.point;
            marker.pose.orientation.w = -0.7071;
            marker_array.markers.push_back(marker);
        }

        marker_pub.publish(marker_array);
    }

    void _clicked_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        ROS_INFO("Clicked: %f, %f, %f", msg->point.x,
            msg->point.y, msg->point.z);
        switch(stops_initialized%3) {
        case 0  :
            to = *msg;
            break; 

        case 1  :
            from = *msg;
            break; 

        default :
            to2 = *msg;
          }
        stops_initialized += 1;

        _send_goal(to);
    }

public:
    Route() :
        stops_initialized(0), client("move_base")
    {
        ros::NodeHandle n;
        marker_pub = n.advertise<visualization_msgs::MarkerArray>(
            "pathplanning_markers", 1);
        click_sub = n.subscribe( "clicked_point", 100,
            &Route::_clicked_point_cb, this);
        odom_sub = n.subscribe("odom", 1, &Route::OdomCallback, this);
    };
    ~Route(){};


};

// This is where we start
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_planning");

    Route r;

    ros::spin();
    return 0;
}
