
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define GRAVITY 9.8

class AddGravity
{
public:
    AddGravity(){
        bool init_result = init();
        ROS_ASSERT(init_result);
    }

    ~AddGravity(){}

    bool init(){
        publisher_  = nh_.advertise<sensor_msgs::Imu>("imu_out", 10);
        subscriber_ = nh_.subscribe("imu_in", 150, &AddGravity::msgCallback, this);
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::Time last_published_time_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    void msgCallback(const sensor_msgs::ImuConstPtr imu_in){
        if (last_published_time_.isZero() || imu_in->header.stamp > last_published_time_)
        {
            last_published_time_ = imu_in->header.stamp;
            sensor_msgs::Imu imu_out = *imu_in;
            imu_out.header.frame_id = "imu_link";
            imu_out.linear_acceleration.x = imu_in->linear_acceleration.x;
            imu_out.linear_acceleration.y = imu_in->linear_acceleration.y;
            imu_out.linear_acceleration.z = imu_in->linear_acceleration.z + GRAVITY;
            publisher_.publish(imu_out);
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "flat_world_imu_node");

    AddGravity add_g;

    ros::spin();

    return 0;
}