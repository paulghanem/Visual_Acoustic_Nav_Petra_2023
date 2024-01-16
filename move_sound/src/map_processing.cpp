#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int32.h"

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;

nav_msgs::OccupancyGrid map_;
int num_map_cells_=0;
int count=0;
std_msgs::Int32 mean_index;
geometry_msgs::PoseStamped point1;

int floor0(float value)
{
  if (value < 0.0)
    return ceil( value );
  else
    return floor( value );
}

int point2cell(const geometry_msgs::PoseStamped::ConstPtr& point)
{
  int x_cell = floor0((point->pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
  int y_cell = floor0((point->pose.position.y - map_.info.origin.position.y)/map_.info.resolution);

  return(x_cell + (y_cell)*map_.info.width);
}

void setPoint(const geometry_msgs::PoseStamped::ConstPtr& point){
  // point1=*point;
  // point1.pose.position.x=point.pose.position.x;
  // point1.pose.position.y=point.pose.position.y;
  // point1.pose.position.z=point.pose.position.z;
  std::cout<<"Got new point"<<std::endl;
  mean_index.data=point2cell(point);
  std::cout<<"Mean index is:"<<mean_index;
  
}


// Occupancy Grid Callback        , const geometry_msgs::PoseStamped::ConstPtr& point
void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std::cout<<"I entered function"<<std::endl;
  ROS_INFO("Occupancy Grid data received: resolution %f cells, width %d cells, height %d cells",
  msg->info.resolution, msg->info.width, msg->info.height);
  ROS_INFO("Entered function");
  map_=*msg;
  // point1=*point;
  // point1.x=point.pose.position.x;
  // point1.y=point.pose.position.y;
  // point1.z=point.pose.position.z;
  std::cout<<"Got new point"<<std::endl;
  // mean_index.data=point2cell(point);
  // int x_cell = floor0((point1.pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
  // int y_cell = floor0((point1.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);
  // mean_index.data= x_cell + (y_cell)*map_.info.width;
  std::cout<<"Mean index is:"<<mean_index;
}

int main(int argc, char** argv)
{
  // std::cout<<"I am here;"<< std::endl;
  // ::ifstream("/home/hello-robot/gaurav_ws/src/move_sound/maps/map1.pgm");
  // // worldCost(infile, 5,5);
  ros::init(argc, argv, "exploration");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<std_msgs::Int32>("mean_index_cell", 1);

  // message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(n, "/map", 1000);
  // message_filters::Subscriber<geometry_msgs::PoseStamped> init_goal_sub(n, "/Initial_goal", 1000);
  // typedef sync_policies::ApproximateTime<nav_msgs::OccupancyGrid,geometry_msgs::PoseStamped> MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), map_sub, init_goal_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2));

  
  ros::Subscriber sub1 = n.subscribe("/Initial_goal", 1000, setPoint);
  ros::spinOnce();
  ros::Subscriber sub2 = n.subscribe("/map", 1000, callback);
  ros::spinOnce();
  ros::Rate loop_rate(1);
  while(ros::ok()){
    pub1.publish(mean_index);
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}