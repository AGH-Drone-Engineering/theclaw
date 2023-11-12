#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
  {
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    registerInterface(&jnt_vel_interface);
  }

  void read(const ros::Time& time, const ros::Duration& period) override
  {
    pos[0] = 0;
    pos[1] = 0;

    vel[0] = 0;
    vel[1] = 0;

    eff[0] = 0;
    eff[1] = 0;
  }

  void write(const ros::Time& time, const ros::Duration& period) override
  {

  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  
  double cmd[2];
  
  double pos[2];
  double vel[2];
  double eff[2];

  ros::Publisher pub_left_command_int;
  ros::Publisher pub_right_command_int;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "theclaw_hw");
  ros::NodeHandle nh;

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(50.0);
  rate.sleep();

  while (ros::ok())
  {
    const ros::Time     time   = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    robot.read(time, period);
    cm.update(time, period);
    robot.write(time, period);
    rate.sleep();
  }

  return 0;
}
