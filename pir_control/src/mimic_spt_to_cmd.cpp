#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Mimic
{
  public:
    Mimic();

  private:
    void twistCallback(const geometry_msgs::TwistConstPtr& twist);
    ros::Publisher cmd_pub_;
    ros::Subscriber spt_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  cmd_pub_ = output_nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
  spt_sub_ = input_nh.subscribe<geometry_msgs::Twist> ("/spt_vel", 10, &Mimic::twistCallback, this);
}

void Mimic::twistCallback(const geometry_msgs::TwistConstPtr& twist)
{

  geometry_msgs::Twist cmd_msg;

  float left = twist->linear.y;
  float right = twist->angular.y;
  float separation = 0.287;

  cmd_msg.linear.x = (left + right) / 2.0;
  cmd_msg.angular.z = (right - left) / 2.0 * separation;

  cmd_pub_.publish(cmd_msg);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mimic_spt_to_cmd");

  Mimic mimic;

  ros::spin();

}
