/*
 * ros_caffe_test.cpp
 *
 *  Created on: Aug 31, 2015
 *      Author: Tzutalin
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros_caffe/CaffeRes.h>
#include <cv_bridge/cv_bridge.h>
#include "Classifier.h"

const std::string RECEIVE_IMG_TOPIC_NAME = "/image_raw";
const std::string PUBLISH_RET_TOPIC_NAME = "/caffe_ret";
const std::string TURTLEBOT_CONTROL_COMMAND_TOPIC_NAME = "/cmd_vel_mux/input/teleop";
const std::string CAFFE_RES_TOPIC_NAME = "/caffe_result";

const float go_straight_base_vel = 0.3;
const float go_straight_vel_offset = -0.1;
const float turning_base_vel = 1.0;

Classifier* classifier;
std::string model_path;
std::string weights_path;
std::string mean_file;
std::string label_file;
std::string image_path;

ros::Publisher gPublisher;
ros::Publisher control_command_pub;
ros::Publisher caffe_res_pub;

void publishRet(const std::vector<Prediction>& predictions);
void publishTurtlebotControlCommand(const std::vector<Prediction>& predictions);
void publishCaffeRes(const std::vector<Prediction>& predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        //cv::imwrite("rgb.png", cv_ptr->image);
		cv::Mat img = cv_ptr->image;
		std::vector<Prediction> predictions = classifier->Classify(img, 3);
		publishRet(predictions);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// TODO: Define a msg or create a service
// Try to receive : $rostopic echo /caffe_ret
void publishRet(const std::vector<Prediction>& predictions)  {
    std_msgs::String msg;
    std::stringstream ss;
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        ss << "[" << p.second << " - " << p.first << "]" << std::endl;
    }
    msg.data = ss.str();
    gPublisher.publish(msg);
    // publishTurtlebotControlCommand(predictions);
    publishCaffeRes(predictions);
}

void publishCaffeRes(const std::vector<Prediction>& predictions)
{
  ros_caffe::CaffeRes msg;
  for(size_t i = 0; i < predictions.size(); i++)
  {
      Prediction p = predictions[i];
      if(p.first == "1 go straight")
      {
        msg.res[1] = p.second;
      }
      else if(p.first == "0 turn right")
      {
        msg.res[0] = p.second;
      }
      else
      {
        msg.res[2] = p.second;
      }
  }
  caffe_res_pub.publish(msg);
}

//Added by FYP16010
void publishTurtlebotControlCommand(const std::vector<Prediction>& predictions)
{
  float go_straight_vel = 0.0;
  float turn_right_vel = 0.0;
  float turn_left_vel = 0.0;

  for (size_t i = 0; i < predictions.size(); ++i) {
      Prediction p = predictions[i];
      if(p.first == "1 go straight")
      {
        go_straight_vel = p.second * go_straight_base_vel;
        go_straight_vel += go_straight_vel_offset;
      }
      else if(p.first == "0 turn right")
      {
        // turn_right_vel = p.second * turning_base_vel;
        turn_left_vel = p.second * turning_base_vel;
      }
      else
      {
        // turn_left_vel = p.second * turning_base_vel;
        turn_right_vel = p.second * turning_base_vel;
      }

      geometry_msgs::Twist turtlebot_control_command;
      turtlebot_control_command.linear.x = go_straight_vel;
      turtlebot_control_command.linear.y = 0.0;
      turtlebot_control_command.linear.z = 0.0;
      turtlebot_control_command.angular.x = 0.0;
      turtlebot_control_command.angular.y = 0.0;
      turtlebot_control_command.angular.z = - turn_right_vel + turn_left_vel;

      control_command_pub.publish(turtlebot_control_command);
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_caffe_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // To receive an image from the topic, PUBLISH_RET_TOPIC_NAME
    image_transport::Subscriber sub = it.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, imageCallback);
	  gPublisher = nh.advertise<std_msgs::String>(PUBLISH_RET_TOPIC_NAME, 100);
    control_command_pub = nh.advertise<geometry_msgs::Twist>(TURTLEBOT_CONTROL_COMMAND_TOPIC_NAME, 1);
    caffe_res_pub = nh.advertise<ros_caffe::CaffeRes>(CAFFE_RES_TOPIC_NAME, 1);
    // const std::string ROOT_SAMPLE = ros::package::getPath("ros_caffe");
    const std::string ROOT_SAMPLE = "/home/fyp/FYP/caffe-master";
    model_path = ROOT_SAMPLE + "/models/fyp_cv_label/deploy.prototxt";
    // model_path = ROOT_SAMPLE + "/data/deploy.prototxt";
    weights_path = ROOT_SAMPLE + "/models/fyp_cv_label/fyp_cv_label_iter_100000.caffemodel";
    // weights_path = ROOT_SAMPLE + "/data/bvlc_reference_caffenet.caffemodel";
    // mean_file = ROOT_SAMPLE + "/data/imagenet_mean.binaryproto";
    mean_file = "/home/fyp/FYP/fyp_ws/data/image_mean.binaryproto";
    // label_file = ROOT_SAMPLE + "/data/synset_words.txt";
    label_file = "/home/fyp/FYP/fyp_ws/data/labels.txt";
    // image_path = ROOT_SAMPLE + "/data/cat.jpg";
    image_path = "/home/fyp/catkin_ws/src/ros_caffe/data/cat.jpg";

    classifier = new Classifier(model_path, weights_path, mean_file, label_file);

    // Test data/cat.jpg
    cv::Mat img = cv::imread(image_path, -1);
    std::vector<Prediction> predictions = classifier->Classify(img);
    /* Print the top N predictions. */
    std::cout << "Test default image under /data/cat.jpg" << std::endl;
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        std::cout << std::fixed << std::setprecision(4) << p.second << " - \"" << p.first << "\"" << std::endl;
    }
	publishRet(predictions);

    ros::spin();
    delete classifier;
    ros::shutdown();
    return 0;
}
