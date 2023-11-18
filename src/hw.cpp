#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#define TICKS_PER_REVOLUTION 960
#define MAX_VELOCITY_RPM 160.0
#define MAX_VELOCITY_RAD_PER_SEC (MAX_VELOCITY_RPM / 60.0 * 2.0 * M_PI)

static double parsePosition(int32_t ticks)
{
  ticks = ticks % TICKS_PER_REVOLUTION;
  double radians = ((double) ticks) / ((double) TICKS_PER_REVOLUTION) * 2.0 * M_PI;
  radians = fmod(radians, 2.0 * M_PI);
  if (radians < 0.0)
  {
    radians += 2.0 * M_PI;
  }
  return radians;
}

static double parseVelocity(int32_t ticksPerSecond)
{
  double radiansPerSecond = ((double) ticksPerSecond) / ((double) TICKS_PER_REVOLUTION) * 2.0 * M_PI;
  return radiansPerSecond;
}

static int16_t velocityToCommand(double velocity)
{
  // command range -255 to 255
  int16_t command = (int16_t) (velocity / MAX_VELOCITY_RAD_PER_SEC * 255.0);
  if (command > 255)
  {
    command = 255;
  }
  else if (command < -255)
  {
    command = -255;
  }
  return command;
}

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle &nh)
  : _nh(nh)
  , _hw_motor_a_cmd_pub(nh.advertise<std_msgs::Int16>("hw/motor_a/command", 1))
  , _hw_motor_b_cmd_pub(nh.advertise<std_msgs::Int16>("hw/motor_b/command", 1))
  , _hw_motor_a_pos_sub(nh.subscribe("hw/motor_a/position", 1, &MyRobot::motor_a_pos_cb, this))
  , _hw_motor_a_vel_sub(nh.subscribe("hw/motor_a/velocity", 1, &MyRobot::motor_a_vel_cb, this))
  , _hw_motor_b_pos_sub(nh.subscribe("hw/motor_b/position", 1, &MyRobot::motor_b_pos_cb, this))
  , _hw_motor_b_vel_sub(nh.subscribe("hw/motor_b/velocity", 1, &MyRobot::motor_b_vel_cb, this))
  {
    hardware_interface::JointStateHandle state_handle_a("wheel_left_joint", &_pos[0], &_vel[0], &_eff[0]);
    _jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("wheel_right_joint", &_pos[1], &_vel[1], &_eff[1]);
    _jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&_jnt_state_interface);

    hardware_interface::JointHandle vel_handle_a(_jnt_state_interface.getHandle("wheel_left_joint"), &_cmd[0]);
    _jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(_jnt_state_interface.getHandle("wheel_right_joint"), &_cmd[1]);
    _jnt_vel_interface.registerHandle(vel_handle_b);

    registerInterface(&_jnt_vel_interface);
  }

  void read(const ros::Time& time, const ros::Duration& period) override
  {
    _pos[0] = parsePosition(_posRecv[0]);
    _pos[1] = parsePosition(_posRecv[1]);

    _vel[0] = parseVelocity(_velRecv[0]);
    _vel[1] = parseVelocity(_velRecv[1]);

    _eff[0] = 0.0;
    _eff[1] = 0.0;
  }

  void write(const ros::Time& time, const ros::Duration& period) override
  {
    std_msgs::Int16 msg;
    msg.data = velocityToCommand(_cmd[0]);
    _hw_motor_a_cmd_pub.publish(msg);

    msg.data = velocityToCommand(_cmd[1]);
    _hw_motor_b_cmd_pub.publish(msg);
  }

private:
  ros::NodeHandle _nh;

  hardware_interface::JointStateInterface _jnt_state_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  
  double _cmd[2];
  
  double _pos[2];
  int32_t _posRecv[2];
  
  double _vel[2];
  int32_t _velRecv[2];
  
  double _eff[2];

  ros::Publisher _hw_motor_a_cmd_pub;
  ros::Publisher _hw_motor_b_cmd_pub;

  ros::Subscriber _hw_motor_a_pos_sub;
  ros::Subscriber _hw_motor_a_vel_sub;

  ros::Subscriber _hw_motor_b_pos_sub;
  ros::Subscriber _hw_motor_b_vel_sub;

  void motor_a_pos_cb(const std_msgs::Int32::ConstPtr& msg)
  {
    _posRecv[0] = msg->data;
  }

  void motor_a_vel_cb(const std_msgs::Int32::ConstPtr& msg)
  {
    _velRecv[0] = msg->data;
  }

  void motor_b_pos_cb(const std_msgs::Int32::ConstPtr& msg)
  {
    _posRecv[1] = msg->data;
  }

  void motor_b_vel_cb(const std_msgs::Int32::ConstPtr& msg)
  {
    _velRecv[1] = msg->data;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "theclaw_hw");
  ros::NodeHandle nh;

  MyRobot robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(50.0);
  rate.sleep();

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    robot.read(time, period);
    cm.update(time, period);
    robot.write(time, period);
    rate.sleep();
  }

  return 0;
}
