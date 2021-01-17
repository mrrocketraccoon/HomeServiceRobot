#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class SubscribeAndPublish
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub;
  ros::Subscriber sub;
  visualization_msgs::Marker marker;

public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    //Topic you want to subscribe
    sub = n.subscribe("/move_base/current_goal", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped& new_goal)
  {  
    // Set our initial shape type to be a sphere
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.1445f;
    marker.color.g = 0.1211f;
    marker.color.b = 0.5117f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    //While loop ensures that there exists at least one subscriber subscribed to the published topic
    while (pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker.action = visualization_msgs::Marker::DELETE;
    pub.publish(marker);
    ROS_INFO("Marker erased");
    marker.pose.position.x = new_goal.pose.position.x;
    marker.pose.position.y = new_goal.pose.position.y;
    marker.action = visualization_msgs::Marker::ADD;
    while (pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub.publish(marker);
    ROS_INFO("Sending robot to x:%lf, y:%lf", marker.pose.position.x, marker.pose.position.y);
  }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "add_markers");
  //ros::Rate r(1);
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
