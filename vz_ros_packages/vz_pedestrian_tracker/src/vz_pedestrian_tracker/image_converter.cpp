#include "image_converter.hpp"
#include <chrono>

static const std::string OPENCV_WINDOW = "Image window";
std::chrono::steady_clock::time_point begin_cb;
std::chrono::steady_clock::time_point end_cb;
std::chrono::steady_clock::time_point begin_person;
std::chrono::steady_clock::time_point end_person;


int cv_index = 0;

ImageConverter::ImageConverter() : it_(nh_)
{
  
  std::string img_topic {"/D435i/image_raw_rot"};
  
  std::string pt_obj_topic {"/pedestrian/object"};
  
  // Subscribe to input video feed and publish output video feed
  image_sub_ = it_.subscribe(img_topic, 1, &ImageConverter::imageCb, this);

  pt_pub_  = nh_.advertise<vz_face_recognition::HSE_Objects>(pt_obj_topic, 1);
}

ImageConverter::~ImageConverter()
{
}

// void ImageConverter::facerecCb(const  vz_face_recognition::HSE_Objects::ConstPtr &msg)
// {
//   // Get labels and rois
//   std::vector< vz_face_recognition::HSE_Object > objects = msg->objects;
//   for(vz_face_recognition::HSE_Object &object : objects){
//     ImageConverter::labels.push_back(object.label);
//     std::vector< int > roi {object.tl.x, object.tl.y, object.width, object.height};
//     ImageConverter::rois.push_back(roi);
//   }
// }

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  begin_cb = std::chrono::steady_clock::now();
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Send Image to Paul's code
  Person person;
  vz_face_recognition::HSE_Object object;
  vz_face_recognition::HSE_Objects objects;


  begin_person = std::chrono::steady_clock::now();
  person = person_tracker.Run(cv_ptr->image, cv_index, ImageConverter::rois, ImageConverter::labels);
  end_person = std::chrono::steady_clock::now();
  // std::cout << "Time difference person_tracker.Run (inside ImageConverter::imageCb) = " << std::chrono::duration_cast<std::chrono::milliseconds>(end_person - begin_person).count() << "[ms]" << std::endl;

  // Get frame back and convert back to sensor image
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header; // empty header
  header.seq = cv_index; 
  header.stamp = msg->header.stamp; // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, person.pt_frame);
  img_bridge.toImageMsg(img_msg);
  objects.image = img_msg;

  for(int i = 0; i < person.pt_labels.size(); i++){
    object.tl.x = person.pt_rois[i][0];
    object.tl.y = person.pt_rois[i][1];
    object.width = person.pt_rois[i][2];
    object.height = person.pt_rois[i][3];
    object.label = person.pt_labels[i];
    objects.objects.push_back(object);
  }

  objects.header.stamp = msg->header.stamp;

  pt_pub_.publish(objects);

  // Increment image count
  cv_index +=1;
  end_cb = std::chrono::steady_clock::now();
  // std::cout << "Time difference ped tracker (ImageConverter::imageCb) = " << std::chrono::duration_cast<std::chrono::milliseconds>(end_cb - begin_cb).count() << "[ms]" << std::endl;
}
