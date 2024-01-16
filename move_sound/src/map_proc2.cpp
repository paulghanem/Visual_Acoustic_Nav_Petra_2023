#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>


nav_msgs::OccupancyGrid map_;
//float ori_z;
//float ori_w;
// std_msgs::Int32 mean_index;
// geometry_msgs::PoseStamped point1;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32MultiArray>("mean_index_cell", 40);
    pub2_ = n_.advertise<geometry_msgs::PoseStamped>("gaussian_max_goal", 40);
    sub2_= n_.subscribe("/map", 1000, &SubscribeAndPublish::callback, this);
    sub_ = n_.subscribe("/Initial_goal", 1000, &SubscribeAndPublish::setPoint, this);
    sub3_= n_.subscribe("/goal_after_check", 1000, &SubscribeAndPublish::cell2point, this);
    // sub3_= n_.subscribe("/blind_in_bw_cal_prob", 1000, &SubscribeAndPublish::cell2point, this);
    rot_pdf_sub= n_.subscribe("/blind_rot_sig", 1000, &SubscribeAndPublish::setRotation1, this);
    //curr_orientation= n_.subscribe("/odom", 1000, &SubscribeAndPublish::OrientationIs, this)
  }
  
  int floor0(float value){
    if (value < 0.0)
        return ceil( value );
    else
        return floor( value );
  }


  std_msgs::Float32MultiArray point2cell(const geometry_msgs::PoseStamped::ConstPtr& point, bool rotation=false)
  { 
    std_msgs::Float32MultiArray output;
    int x_cell = floor0((point->pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
    int y_cell = floor0((point->pose.position.y - map_.info.origin.position.y)/map_.info.resolution);
    int index= x_cell + (y_cell)*map_.info.width;

    output.data.clear();
    // int(current_date.strftime("%Y%m%d%H%M%S"))
    // output.data.push_back();
    output.data.push_back(index);
    output.data.push_back(x_cell);
    output.data.push_back(y_cell);
    output.data.push_back(map_.info.resolution);
    output.data.push_back(map_.info.height);
    output.data.push_back(map_.info.width);
    if (rotation==true){
    output.data.push_back(point->pose.orientation.z);
    output.data.push_back(point->pose.orientation.w);
    }
    return output;
  }
  
  
  void setPoint(const geometry_msgs::PoseStamped::ConstPtr& point1){
    std_msgs::Float32MultiArray mean_index;
    mean_index.data.clear();
    std::cout<<"Received goal point from doa.py"<<std::endl;
//    ori_z= point1->pose.orientation.z;
//    ori_w= point1->pose.orientation.w;
    mean_index = point2cell(point1);
    std::cout<<" Cell value of point received is:"<<mean_index;
    pub_.publish(mean_index);
  }
  
  
  void setPointWrotation(const geometry_msgs::PoseStamped::ConstPtr& point1, bool rotation){
    std_msgs::Float32MultiArray mean_index;
    mean_index.data.clear();
    std::cout<<"Got Rotation signal"<<std::endl;
//    ori_z= point1->pose.orientation.z;
//    ori_w= point1->pose.orientation.w;
    mean_index = point2cell(point1, rotation);
    std::cout<<" Cell corresponding point in map is:"<<mean_index<<std::endl;
    pub_.publish(mean_index);
  }
  
  
  void setRotation1(const geometry_msgs::PoseStamped::ConstPtr& point1){
    std::cout<<"Calling setPointWRotation"<<std::endl;
    setPointWrotation(point1, true);
  }
  
  
  void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    std::cout<<"I entered function"<<std::endl;
    ROS_INFO("Occupancy Grid data received: resolution %f cells, width %d cells, height %d cells",
    msg->info.resolution, msg->info.width, msg->info.height);
    ROS_INFO("Entered function");
    map_=*msg;
  }
  
  //void OrientationIs(const nav_msgs::Odometry::ConstPtr& data){
  //std::cout<<"Odometry received";
  
  //}
  
  
  void cell2point(const std_msgs::Int32& cell)
  {
    geometry_msgs::PoseStamped rpyGoal;
    std::cout<<"Cell value received from bivariate is: "<< cell.data;
    rpyGoal.pose.position.x = (cell.data % map_.info.width)*map_.info.resolution + map_.info.origin.position.x;
    rpyGoal.pose.position.y = floor(cell.data /map_.info.width)*map_.info.resolution + map_.info.origin.position.y;
    rpyGoal.pose.position.z = 0;
    std::cout<<"Point in map from received cell pixel value is:"<<rpyGoal<<std::endl;
    pub2_.publish(rpyGoal);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub2_;
  ros::Subscriber sub_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber rot_pdf_sub;
  ros::Subscriber curr_orientation;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
