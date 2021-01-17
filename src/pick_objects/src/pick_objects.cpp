#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
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
  double locations[2][3]={{3.5,-1.0,1.0},{0,-2.0,3.1416}};
  for(int i=0;i<sizeof(locations)/sizeof(locations[0]);i++){
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = locations[i][0];
    goal.target_pose.pose.position.y = locations[i][1];
    goal.target_pose.pose.orientation.w = locations[i][2];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending robot to x:%lf, y:%lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the robot reached its goal successfuly");
    else
      ROS_INFO("The robot failed to reach its goal for some reason");
    ros::Duration(5.0).sleep();
  }
  return 0;
}
