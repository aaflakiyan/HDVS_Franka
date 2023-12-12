#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher robot_2_joint_vel_pub;
ros::Publisher robot_1_joint_vel_pub;
// double robot_2_first_desired_positions[] = { 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_2 };
// double robot_2_second_desired_positions[] = {
//   0.0, -M_PI_2 / 3.0, 0.0, -2.0 * M_PI_2 / 3.0, 0.0, 0.85 * M_PI_4, M_PI_2
// };

double robot_2_first_desired_positions[] = { 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, -1.5 * M_PI_4, M_PI_2, M_PI_2 };
// double robot_2_desired_positions[] = { 0.0,
//                                        (-M_PI_4 - M_PI_2 / 3.0) / 2,
//                                        -M_PI_2 / 15,
//                                        (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
//                                        0.4 * M_PI_4 + M_PI_2 / 4,
//                                        (M_PI_2 + 0.85 * M_PI_4) / 2,
//                                        (M_PI_2 + M_PI_2) / 2 + M_PI_2 / 3.5 };
double robot_2_desired_positions[] = { 0.0,
                                       (-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 6,
                                       0.0,
                                       (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
                                       0.4 * M_PI_4 - M_PI_2,
                                       (M_PI_2 + 0.85 * M_PI_4) / 2 + M_PI_2 / 4,
                                       (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 8 };
// double robot_2_desired_positions[] = { M_PI_2 / 3,
//                                        (-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 6,
//                                        0.0,
//                                        (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
//                                        0.4 * M_PI_4 - 1.3 * M_PI_2,
//                                        (M_PI_2 + 0.85 * M_PI_4) / 2,
//                                        (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 6 - M_PI_2 / 8 };// ratated 90

// double robot_2_desired_positions[] = { 0.0,
//                                        (-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 6,
//                                        0.0,
//                                        (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
//                                        0.4 * M_PI_4 - M_PI_2,
//                                        (M_PI_2 + 0.85 * M_PI_4) / 2 + M_PI_2 / 4,
//                                        (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 8 - M_PI_2 / 14 };// good for
//                                        ibvs other singular

// double robot_2_desired_positions[] = { M_PI_4 / 4,
//                                        (-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 6,
//                                        0.0,
//                                        (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
//                                        0.4 * M_PI_4 - M_PI_2,
//                                        (M_PI_2 + 0.85 * M_PI_4) / 2 + 0.2 * M_PI_2,
//                                        (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 8 - M_PI_2 / 14 }; // all
//                                        singular instead of HDVS

// double robot_2_desired_positions[] = { M_PI_4 / 4,
//                                        (-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 2.5,
//                                        0,
//                                        (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2 + M_PI_4 / 1.5,
//                                        0.4 * M_PI_4 - M_PI_2,
//                                        (M_PI_2 + 0.85 * M_PI_4) / 2 + 0.2 * M_PI_2,
//                                        (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 8 - M_PI_2 / 10 };// large
//                                        motion for other methods

double robot_1_desired_positions[] = { 0.0,
                                       (-M_PI_4 - M_PI_2 / 3.0) / 2,
                                       0.0,
                                       (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2,
                                       0,
                                       M_PI_2 + 1.2 * M_PI_4,
                                       -(M_PI_2 + M_PI_2) / 2 + M_PI_2 };

double kp = 1.0, kd = 0.1;
double loop_duration = 20.0;
ros::Time start_time;

void robot_2_joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std_msgs::Float64MultiArray joint_vel_msg;

  ros::Time current_time = ros::Time::now();
  ros::Duration time_passed = current_time - start_time;
  double seconds_passed = (double)time_passed.sec + ((double)time_passed.nsec / 1e9);
  double loop_point = seconds_passed / loop_duration - (long)(seconds_passed / loop_duration);
  // std::cout << loop_point << std::endl;

  std::vector<double> positions = msg->position;
  std::vector<double> velocities = msg->velocity;
  for (int i = 0; i < positions.size(); i++)
  {
    double current_desired_position = robot_2_desired_positions[i] * 2.0 * std::fabs(0.5 - loop_point) +
                                      robot_2_desired_positions[i] * (1.0 - (2.0 * std::fabs(0.5 - loop_point)));
    joint_vel_msg.data.push_back(kp * (current_desired_position - positions[i]) + kd * -velocities[i]);
  }

  robot_2_joint_vel_pub.publish(joint_vel_msg);
}

void robot_1_joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std_msgs::Float64MultiArray joint_vel_msg1;

  ros::Time current_time = ros::Time::now();
  ros::Duration time_passed = current_time - start_time;
  double seconds_passed = (double)time_passed.sec + ((double)time_passed.nsec / 1e9);
  double loop_point = seconds_passed / loop_duration - (long)(seconds_passed / loop_duration);
  // std::cout << loop_point << std::endl;

  std::vector<double> positions = msg->position;
  std::vector<double> velocities = msg->velocity;
  for (int i = 0; i < positions.size(); i++)
  {
    double current_desired_position = robot_1_desired_positions[i] * 2.0 * std::fabs(0.5 - loop_point) +
                                      robot_1_desired_positions[i] * (1.0 - (2.0 * std::fabs(0.5 - loop_point)));
    joint_vel_msg1.data.push_back(kp * (current_desired_position - positions[i]) + kd * -velocities[i]);
  }

  robot_1_joint_vel_pub.publish(joint_vel_msg1);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "moving_marker");
  ros::NodeHandle nh;

  robot_2_joint_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("robot_2/panda_arm_controller/command", 1000);
  robot_1_joint_vel_pub = nh.advertise<std_msgs::Float64MultiArray>("robot_1/panda_arm_controller/command", 1000);

  ros::Subscriber robot_2_joint_state_sub = nh.subscribe("robot_2/joint_states", 1000, robot_2_joint_state_callback);
  ros::Subscriber robot_1_joint_state_sub = nh.subscribe("robot_1/joint_states", 1000, robot_1_joint_state_callback);

  start_time = ros::Time::now();

  ros::spin();
}