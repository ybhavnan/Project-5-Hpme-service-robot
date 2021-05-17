/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

double pickUpX = -3.0;
double pickUpY = 2.0;
double dropOffX = -3.0;
double dropOffY = -3.0;

bool isItemPickedUp = false;
bool isItemDroppedOff = false;
bool isItFirstReach = false;

void PosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_sub = n.subscribe( "amcl_pose", 10, PosCallback );

  visualization_msgs::Marker marker;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickUpX;
  marker.pose.position.y = pickUpY;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -> blue 
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
        return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    
    if (!isItemPickedUp)
    {
      // add marker if it is not picked up 
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
    }

    else if (!isItemDroppedOff)
    {
        // delete marker if the is not deoper off yet 
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

            if (!isItFirstReach){
                isItFirstReach = true;
                ROS_INFO("Wait here for picking up!");
                ros::Duration(5.0).sleep();
             }
    }

    else
    {
        //  add back the mark
        marker.pose.position.x = dropOffX;
        marker.pose.position.y = dropOffY;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
    }
    
    ros::spinOnce();
  }
}


void PosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;
  // ROS_INFO("Received nav_msgs");

  double distanceToPickup = sqrt(pow(robotX - pickUpX, 2) + pow(robotY - pickUpY, 2));
  double distanceToDropoff = sqrt(pow(robotX - dropOffX, 2) + pow(robotY - dropOffY, 2));  

  if (distanceToPickup < 0.5) {
    isItemPickedUp = true;
    ROS_INFO("Reached the pickup zone!");
  }
  
  if (distanceToDropoff < 0.3) {
    isItemDroppedOff = true;
    ROS_INFO("Reached the dropoff zone!");
  }
}

