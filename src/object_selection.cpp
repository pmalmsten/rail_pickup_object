/*!
 * \file object_selection.cpp
 * \brief Defines a node which discovers objects and publishes the first one for the poke cup demo.
 *
 * Upon executing this node. the following actions are taken:
 *   1) The discover_objects service is called.
 *   2) The first resulting object is measured to determine its maximum height
 *   3) The topmost middle point of the object is published to the poke cup topic.
 *
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \date Nov 30, 2012
 */

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PickupGoal.h>
#include <object_manipulation_msgs/GraspableObject.h>

#include "rail_object_discovery/DiscoverObjects.h"
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"

#include "rail_object_discovery/NamedPointCloud2.h"

// #define PICKUP_INSTEAD_OF_POKE

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_selection");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rail_object_discovery::DiscoverObjects>("discover_objects");
  ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("poke_cup_point", 1000);
  if (!client.waitForExistence(ros::Duration(3.0)))
  {
    ROS_ERROR("Service 'discover_objects' does not exist.");
    return 1;
  }
  rail_object_discovery::DiscoverObjects srv;

  // Set object filter parameters
  srv.request.constraints.object_min_sensor_range = 0.4; // meters
  srv.request.constraints.object_max_sensor_range = 3.0;
  srv.request.constraints.object_min_spherical_radius = 0.01;
  srv.request.constraints.object_max_spherical_radius = 0.10;

  ROS_INFO("Calling object detection server..");
  if (client.call(srv))
  {
    if ((int)srv.response.objects.size() > 0)
    {
      ROS_INFO_STREAM("Found " << srv.response.objects.size() << " objects");

      // Print object information
      // Read sensor_msgs/PointCloud2 into pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(srv.response.objects.at(0).cloud, *cloud);

      // Compute point cloud statistics, log
      pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(cloud);
      double radius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center);

      ROS_INFO_STREAM("First object at: (" << center.x << ", " << center.y << "," << center.z << ")");
      ROS_INFO_STREAM("First object radius: " << radius);

#ifdef PICKUP_INSTEAD_OF_POKE
      // Create PickupAction message
      object_manipulation_msgs::PickupGoal pickupGoal;

      // Define the goal object to grasp
      pickupGoal.arm_name = "arm1";// <-- Needs to be verified
      pickupGoal.target.reference_frame_id = srv.response.objects.at(0).header.frame_id;
      sensor_msgs::convertPointCloud2ToPointCloud(srv.response.objects.at(0),
          pickupGoal.target.cluster);

      // Define the lift to perform
      pickupGoal.lift.direction.vector.x = 0;// <-- Should probably be verified
      pickupGoal.lift.direction.vector.y = 0;
      pickupGoal.lift.direction.vector.z = 1;
      pickupGoal.lift.desired_distance = 0.10;// 10 cm
      pickupGoal.lift.min_distance = 0.5;// 5 cm

      // Define the collision object name of the object to grasp
      pickupGoal.collision_object_name = srv.response.names.at(0);

      // Request the pickupAction
      actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickupActionClient("object_manipulator_pickup");
      pickupActionClient.waitForServer();
      pickupActionClient.sendGoal(pickupGoal);
      pickupActionClient.waitForResult(ros::Duration(40.0));
      if(pickupActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        std::cout << "Sucessfully grabbed object" << std::endl;
      }
      else
      {
        std::cout << "Unable to grab object" << std::endl;
      }
#else
      pcl::PointXYZRGB min_pt;
      pcl::PointXYZRGB max_pt;

      // Compute centered x and y points with max z point
      pcl::getMinMax3D(*cloud, min_pt, max_pt);

      geometry_msgs::Point msg;

      msg.x = (min_pt.x + max_pt.x) / 2.0; //get approx middle x y points
      msg.y = (min_pt.y + max_pt.y) / 2.0;
      msg.z = max_pt.z; //max z so we dont try and drive through an object

      //temp write pcd
      const std::string file = "test.pcd";
      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZRGB>(file, *cloud, false);

      ROS_INFO_STREAM("Computed target point: (" << msg.x << ", " << msg.y << ", " << msg.z << ")");

      // Publish resulting parget point
      point_pub.publish(msg);

      ROS_INFO("Point published. Press Ctrl-c to terminate.");
#endif
      ros::spin();
    }
    else
    {
      ROS_INFO("No objects found");
    }
  }
  else
  {
    ROS_ERROR("Failed to call object segmentation server");
    return 1;
  }

  return 0;
}
