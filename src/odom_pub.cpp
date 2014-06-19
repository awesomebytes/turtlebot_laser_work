#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

class OdomFromWheels{
  public:
    OdomFromWheels();
    ~OdomFromWheels();
    geometry_msgs::Vector3 last_vels;
    bool new_vel_msg;
    void wheelsCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void computeOdoms();
};


void OdomFromWheels::wheelsCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    this->last_vels = *msg;
    this->new_vel_msg = true;
}

OdomFromWheels::OdomFromWheels(){
    this->last_vels = geometry_msgs::Vector3();
    this->new_vel_msg = false;
}

OdomFromWheels::~OdomFromWheels(){}

void OdomFromWheels::computeOdoms(){
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    this->last_vels = geometry_msgs::Vector3();
    this->new_vel_msg = false;
    ros::Subscriber vels_sub = n.subscribe("wheels_vel", 100, &OdomFromWheels::wheelsCallback, this);

    // initial conditions
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(n.ok()){

      while(! this->new_vel_msg)
      {
          ros::spinOnce();
      } // check for incoming messages
      this->new_vel_msg = false;

      double wr = this->last_vels.x; // left wheel
      double wl = this->last_vels.y; // right wheel
      double S = 0.26; // separation beween wheels

      double vtotal = (wr + wl) / 2.0;
      double theta_dot = (wl - wr) / (2.0*S);

      double vth = theta_dot;

      current_time = ros::Time::now();

      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double delta_th = vth * dt;

      double vx = vtotal * cos(delta_th);
      double vy = vtotal * sin(delta_th);

      double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

      x += delta_x;
      y += delta_y;
      th += delta_th;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  OdomFromWheels o = OdomFromWheels();
  o.computeOdoms();

}
