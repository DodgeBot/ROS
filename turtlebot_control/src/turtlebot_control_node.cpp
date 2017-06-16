#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/package.h>
#include <ros_caffe/CaffeRes.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

//ros topic names
const std::string CAFFE_PEDESTRIAN_AVOID_RES_TOPIC_NAME = "/caffe_pedestrian_avoid_res";
const std::string CAFFE_PATH_FOLLOW_RES_TOPIC_NAME = "/caffe_path_follower_res";
const std::string ULTRASON_SENSOR_TOPIC_NAME = "/ultrason_dis";
const std::string TURTLEBOT_CONTROL_COMMAND_TOPIC_NAME = "/cmd_vel_mux/input/teleop";

// ros publishers
ros::Publisher control_command_pub;

// caffe prediction to speed
const float go_straight_base_vel = 0.3;
const float go_straight_vel_offset = 0.1;
const float turning_base_vel = 0.4;

// ultrason constants
const float gf_ultrason_block_thres = 50.0;

// global variables for controls
float gfa_pedestrian_avoid_predictions[3];
float gf_ultrason_front_right = 0.0;
float gf_ultrason_front_left = 0.0;
float gf_ultrason_left = 0.0;
float gf_ultrason_back = 0.0;
float gf_ultrason_right = 0.0;

//global variables for path follow
float gfa_path_follow_predictions[3];
float turning_thres = 0.1;

float gf_ultrason_front_right_old = 0.0;
float gf_ultrason_front_left_old = 0.0;
float gf_ultrason_left_old = 0.0;
float gf_ultrason_back_old = 0.0;
float gf_ultrason_right_old = 0.0;

float gfa_velocity_cmd[3];
float speed_straight = 0.4;
/*
Encoding:
0 - turn right
1 - go straight
2 - turn left
*/
void CaffePedestrianAvoidResCallback(const ros_caffe::CaffeRes::ConstPtr & caffe_res_msg)
{
  //ROS_INFO("Caffe result: %f, %f, %f", caffe_res_msg->res[0], caffe_res_msg->res[1], caffe_res_msg->res[2]);
  for (int i = 0; i < 3; i++) {
    gfa_pedestrian_avoid_predictions[i] = caffe_res_msg->res[i];
  }
}

/*
Encoding:
0 - turn right
1 - go straight
2 - turn left
*/
void CaffePathFollowResCallback(const ros_caffe::CaffeRes::ConstPtr & caffe_res_msg)
{
  for (int i = 0; i < 3; i++)
  {
    gfa_path_follow_predictions[i] = caffe_res_msg->res[i];
  }
}

/*
Encoding:
Counter-clockwise increasing from 0 to 4;
Facing camera dock, front-left is 0;
*/
void UltrasonSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& ultrason_data_msg)
{
  // ROS_INFO("ultrason sensor reading: %f, %f, %f, %f, %f", ultrason_data_msg->data[0], ultrason_data_msg->data[1], ultrason_data_msg->data[2], ultrason_data_msg->data[3], ultrason_data_msg->data[4]);

  gf_ultrason_front_right = ultrason_data_msg->data[0];
  gf_ultrason_front_left = ultrason_data_msg->data[1];
  gf_ultrason_left = ultrason_data_msg->data[2];
  gf_ultrason_back = ultrason_data_msg->data[3];
  gf_ultrason_right = ultrason_data_msg->data[4];
}

bool frontLeftBlocked()
{
  return gf_ultrason_front_left < gf_ultrason_block_thres;
}

bool frontRightBlocked()
{
  return gf_ultrason_front_right < gf_ultrason_block_thres;
}

bool leftBlocked()
{
  return gf_ultrason_left < gf_ultrason_block_thres;
}

bool rightBlocked()
{
  return gf_ultrason_right < gf_ultrason_block_thres;
}

bool backBlocked()
{
  return gf_ultrason_back < gf_ultrason_block_thres;
}

void turnRight()
{
  gfa_velocity_cmd[0] = 1.0;
  std::cout << "turn right" << std::endl;
}

void turnLeft()
{
  gfa_velocity_cmd[2] = 1.0;
  std::cout << "turn left" << std::endl;
}

void goStraight()
{
  gfa_velocity_cmd[1] = speed_straight;
  std::cout << "go straight" << std::endl;
}

void goBack()
{
  gfa_velocity_cmd[1] = -speed_straight;
}

void outputLabel(float p[]) {
  int idx = 0;
  if (p[0] > p[1]) {
    idx = 0;
    if (p[0] < p[2]) {
      idx = 2;
    }
  } else {
    idx = 1;
    if (p[1] < p[2]) {
      idx = 2;
    }
  }

  switch (idx) {
    case 0:
      std::cout << "turn right" << std::endl;
      break;
    case 1:
      std::cout << "go straight" << std::endl;
      break;
    case 2:
      std::cout << "turn left" << std::endl;
      break;
    default:
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_controller");

  ros::NodeHandle nh;

//subscribers
  ros::Subscriber ultrason_sub = nh.subscribe(ULTRASON_SENSOR_TOPIC_NAME, 10, UltrasonSensorCallback);
  ros::Subscriber caffe_res_sub = nh.subscribe(CAFFE_PEDESTRIAN_AVOID_RES_TOPIC_NAME, 10, CaffePedestrianAvoidResCallback);
  ros::Subscriber caffe_path_follow_res_sub = nh.subscribe(CAFFE_PATH_FOLLOW_RES_TOPIC_NAME, 10, CaffePathFollowResCallback);

//publishers
  control_command_pub = nh.advertise<geometry_msgs::Twist>(TURTLEBOT_CONTROL_COMMAND_TOPIC_NAME, 1);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist turtlebot_control_command;
  /*
  Encoding:
  0 - turn_right_vel;
  1 - go_straight_vel;
  2 - turn_left_vel;
  */


  while (ros::ok()) {
    // reset speed
    for (int i = 0; i < 3; i++) {
      gfa_velocity_cmd[i] = 0.0;
    }

    bool ultra_sonic_warning = true;
    bool pedestrian_warning = false;

    // call all callback functions
    ros::spinOnce();

    /*
    General control principle:
    If non UltraSonic sensors blocked, follow caffe prediction
    Else use UltraSonic
    */

    /**************** CAFFE **************/
    // TODO: use array? for the base vels
    if (frontRightBlocked() || frontLeftBlocked() || leftBlocked() || rightBlocked() || backBlocked()) {
      ultra_sonic_warning = true;
    }
    else
    {
      ultra_sonic_warning = false;
    }

    if(gfa_pedestrian_avoid_predictions[0] < turning_thres && gfa_pedestrian_avoid_predictions[2] < turning_thres)
    {
      pedestrian_warning = false;
    }
    else
    {
      pedestrian_warning = true;
    }

    // pedestrian_warning = false;
    // ultra_sonic_warning = false;

    // if (!ultra_sonic_warning) {
    //   gfa_velocity_cmd[1] = gfa_pedestrian_avoid_predictions[1] * go_straight_base_vel;
    //   gfa_velocity_cmd[1] += go_straight_vel_offset;
    //   gfa_velocity_cmd[0] = gfa_pedestrian_avoid_predictions[0] * turning_base_vel;
    //   gfa_velocity_cmd[2] = gfa_pedestrian_avoid_predictions[2] * turning_base_vel;
    // }
    /*************** UltraSonic ***************/
    // if straight way blocked

    if (ultra_sonic_warning) {
      // ROS_INFO("Ultrasonic working!");
      std::cout << "Ultrasonic\t\t";
      float old_rate = 0.1;
      float new_rate = 0.9;
      // filter
      gf_ultrason_front_right = old_rate * gf_ultrason_front_right_old + new_rate * gf_ultrason_front_right;
      gf_ultrason_front_left = old_rate * gf_ultrason_front_left_old + new_rate * gf_ultrason_front_left;
      gf_ultrason_left = old_rate * gf_ultrason_left_old + new_rate * gf_ultrason_left;
      gf_ultrason_right = old_rate * gf_ultrason_right_old + new_rate * gf_ultrason_right;
      gf_ultrason_back = old_rate * gf_ultrason_back_old + new_rate * gf_ultrason_back;


      if((frontRightBlocked() || rightBlocked()) && !leftBlocked())
      {
        turnLeft();
      }
      else if((frontLeftBlocked() || leftBlocked()) && !rightBlocked())
      {
        turnRight();
      }
      else if((frontLeftBlocked() || frontRightBlocked()) && leftBlocked() && rightBlocked())
      {
        // goBack();
        if(!backBlocked())
        {
          goBack();
        }
        else
        {
          turnLeft();
        }
      }
      else
      {
        goStraight();
      }

      gf_ultrason_front_right_old = gf_ultrason_front_right;
      gf_ultrason_front_left_old = gf_ultrason_front_left;
      gf_ultrason_left_old = gf_ultrason_left;
      gf_ultrason_back_old = gf_ultrason_right;
      gf_ultrason_right_old = gf_ultrason_back;
    }
    else if(pedestrian_warning)
    {
      // ROS_INFO("Avoid people!");
      std::cout << "Pedestrian Avoidance\t";
      outputLabel(gfa_pedestrian_avoid_predictions);
      gfa_velocity_cmd[1] = gfa_pedestrian_avoid_predictions[1] * go_straight_base_vel;
      gfa_velocity_cmd[1] += go_straight_vel_offset;
      gfa_velocity_cmd[0] = gfa_pedestrian_avoid_predictions[0] * turning_base_vel;
      gfa_velocity_cmd[2] = gfa_pedestrian_avoid_predictions[2] * turning_base_vel;
    }
    else
    {
      //follow path
      // ROS_INFO("Follow path");
      std::cout << "Path Following\t\t";
      outputLabel(gfa_path_follow_predictions);
      gfa_velocity_cmd[1] = gfa_path_follow_predictions[1] * go_straight_base_vel;
      gfa_velocity_cmd[1] += go_straight_vel_offset;
      gfa_velocity_cmd[0] = gfa_path_follow_predictions[0] * turning_base_vel * 4;
      gfa_velocity_cmd[2] = gfa_path_follow_predictions[2] * turning_base_vel * 4;
    }
    // ROS_INFO("ultrason sensor reading: %f, %f, %f, %f, %f", gf_ultrason_front_left, gf_ultrason_front_right, gf_ultrason_left, gf_ultrason_right, gf_ultrason_back);

    /**************** FINAL COMMAND **************/
    turtlebot_control_command.linear.x = gfa_velocity_cmd[1];
    turtlebot_control_command.linear.y = 0.0;
    turtlebot_control_command.linear.z = 0.0;
    turtlebot_control_command.angular.x = 0.0;
    turtlebot_control_command.angular.y = 0.0;
    turtlebot_control_command.angular.z = - gfa_velocity_cmd[0] + gfa_velocity_cmd[2];

    control_command_pub.publish(turtlebot_control_command);

    loop_rate.sleep();
  }
  return 0;
}
