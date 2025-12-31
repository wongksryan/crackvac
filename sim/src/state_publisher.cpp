#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

using namespace std::chrono;

// a mock state publisher. not used in the main project.
class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
            // create a publisher to tell robot_state_publisher the JointState information.
            // robot_state_publisher will deal with this transformation
            pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

            // create a broadcaster to tell the tf2 state information
            // this broadcaster will determine the position of coordinate system 'axis' in coordinate system 'odom'
            //broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            //loop_rate =std::make_shared<rclcpp::Rate>(33ms);

            timer = this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));

            joint_names = {
                "fl_steer_joint", "fl_wheel_joint",
                "fr_steer_joint", "fr_wheel_joint",
                "bl_steer_joint", "bl_wheel_joint",
                "br_steer_joint", "br_wheel_joint",
            };
            joint_positions.resize(joint_names.size(), 0.0);
        }

    void publish();

    private:
    const double degree = M_PI / 180.0;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    //std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    double dt;
    rclcpp::Time last_time;
};

void StatePublisher::publish(){
    // create the necessary messages
    geometry_msgs::msg::TransformStamped t; 
    rclcpp::Time now = this->get_clock()->now();
    dt = now.seconds() - last_time.seconds(); 
    last_time = now;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now;
    joint_state.name = joint_names;

    //rotate wheel and steer
    for (int i = 0; i < 8; i++) {
        joint_positions[i] += 2.0 * dt; 
    }
    joint_state.position = joint_positions; 
    
    pub->publish(joint_state);
    RCLCPP_INFO(this->get_logger(), "Publishing States ...");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}