#include <joy_vehicle_cmd_sender/joy_vehicle_cmd_sender.h>

joy_vehicle_cmd_sender::joy_vehicle_cmd_sender()
{
    joy_cmd_= boost::none;
    vehicle_cmd_pub_ = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd",10);
    joy_sub_ = nh_.subscribe("/joy", 10, &joy_vehicle_cmd_sender::joy_callback_, this);
}

joy_vehicle_cmd_sender::~joy_vehicle_cmd_sender()
{

}

void joy_vehicle_cmd_sender::run()
{
    boost::thread publish_vehicle_cmd_thread_(&joy_vehicle_cmd_sender::publish_vehicle_cmd_, this);
    return;
}

double sigmoid(double gain, double x) {
    return 1.0 / (1.0 + exp(-gain * x));
}


void joy_vehicle_cmd_sender::publish_vehicle_cmd_()
{
    ros::Rate rate(30);
    while(ros::ok())
    {
        if(joy_cmd_)
        {
            double accel_ratio = 10.0;
            double brake_ratio = 10.0;
            double steering_ratio = 1.0;
            double accel_input;
            double brake_input;
            double steer_input;

            accel_input = joy_cmd_->axes[3];
            //steer_input = 0.05*(2.0 * sigmoid(10.0, joy_cmd_->axes[0]) - 1.0);
            steer_input = 0.1 * joy_cmd_->axes[0];
            //steer_input = 0.05 * joy_cmd_->axes[0] * (2 - joy_cmd_->axes[0]);
            geometry_msgs::Twist twist_cmd;
            twist_cmd.linear.x = accel_input * accel_ratio;

            twist_cmd.linear.y = 0;
            twist_cmd.linear.z = 0;
            twist_cmd.angular.x = 0;
            twist_cmd.angular.y = 0;
            twist_cmd.angular.z = twist_cmd.linear.x * steering_ratio * steer_input;
            autoware_msgs::VehicleCmd vehicle_cmd;
            vehicle_cmd.twist_cmd.twist = twist_cmd;
            vehicle_cmd_pub_.publish(vehicle_cmd);
        }
        rate.sleep();
    }
    return;
}

void joy_vehicle_cmd_sender::joy_callback_(sensor_msgs::Joy msg)
{
    joy_cmd_ = msg;
    return;
}
