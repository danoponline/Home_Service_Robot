#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

class AddMarkers
{
public:
  
  // Constructor
  AddMarkers()
  {
    // Set up maker publisher
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set up marker subscriber
    marker_sub_ = n_.subscribe("/amcl_pose", 1, &AddMarkers::callback, this);

    // Set our initial shape type to be a cube
    shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Set lifetime to never deleted
    marker.lifetime = ros::Duration();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
  
    // Set the pose of the marker at the pick up location
    marker.pose.position.x = pick_up_pose[0];
    marker.pose.position.y = pick_up_pose[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pick_up_pose[2];
       
    while(marker_pub_.getNumSubscribers() < 1) {
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
    }

    // Publish marker
    marker_pub_.publish(marker);

    // Set already display flag to true
    already_displayed = true;
  }

  // Calback function
  void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    
    double x_pos = msg->pose.pose.position.x;
    double y_pos = msg->pose.pose.position.y;
    double distance_to_pick_up = sqrt(pow((pick_up_pose[0]-x_pos),2)+pow((pick_up_pose[1]-y_pos),2));
    double distance_to_drop_off = sqrt(pow((drop_off_pose[0]-x_pos),2)+pow((drop_off_pose[1]-y_pos),2));

    if (distance_to_pick_up < 0.2 && !picked_up){
       
       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
       marker.action = visualization_msgs::Marker::DELETE;
  
       // Set picked_up state to true
       picked_up = true;
       
       // Publish marker
       marker_pub_.publish(marker);
       
       // Set already display flag to false
       already_displayed = false;
    }
    
    else if (distance_to_drop_off < 0.2 && picked_up && !already_displayed){
       
       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
       marker.action = visualization_msgs::Marker::ADD;
  
       // Set the pose of the marker at the pick up location
       marker.pose.position.x = drop_off_pose[0];
       marker.pose.position.y = drop_off_pose[1];
       marker.pose.position.z = 0;
       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = drop_off_pose[2];
       
       // Publish marker
       marker_pub_.publish(marker);
      
       // Set already display flag to true
       already_displayed = true;
    }

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher marker_pub_;
  ros::Subscriber marker_sub_;
  uint32_t shape;
  visualization_msgs::Marker marker;
  double pick_up_pose[3] = {5.5,0.0,1.0};
  double drop_off_pose[3] = {0.0,3.5,1.0};
  bool picked_up = false;
  bool already_displayed = false;

};//End of class AddMarkers

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "add_markers");

  //Create an object of class AddMarkers that will take care of everything
  AddMarkers add_markers;

  ros::spin();

  return 0;
}
