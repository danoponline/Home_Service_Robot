#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to pick up
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to pick up
  ROS_INFO("Sending pick up location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the pick up location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reaches the pick up location");
  else {
    ROS_INFO("The base failed to move to the pick up location");
    return 0;
  }
  
  // Wait 5 seconds after reaching pick up location
  ros::Duration(5.0).sleep();

  // Define a position and orientation for the robot to drop off
  goal.target_pose.pose.position.x = -2.0;
  goal.target_pose.pose.position.y = -1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to drop off
  ROS_INFO("Sending drop off location");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the drop off location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot reaches the drop off location");
  else {
    ROS_INFO("The base failed to move to the drop off location");
    return 0;
  }

  return 0;
}
