#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/name.hpp"
#include "my_robot_interfaces/srv/trigger.hpp"
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

/*
    turtle1_controller.cpp

    This is the main turtle robot controller, that will
    make turtle1 move towards randomly spawned turtles,
    which will trigger a despawn event for the caught turtle
    and begin seeking the next turtle that was spawned sequentially
*/


class turtle1_controller : public rclcpp::Node{

    public:

        std::string topic_leading_turtle = "leading_turtle";
        std::string topic_turtle1_cmd_vel = "turtle1/cmd_vel";
        std::string topic_turtle1_pose = "turtle1/pose";
        std::string service_trigger_despawn = "trigger_despawn";


        turtle1_controller() : Node("turtle1_controller"){

            // making a callback group that is NOT added to the executor automatically
            // ensures that wait_for_message acquires the pose from the dynamic subscription creation at subtoNextTurtle
            auto unspun_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
            sub_options.callback_group = unspun_group;

            pub_moveTurtle1 = this->create_publisher<geometry_msgs::msg::Twist>(topic_turtle1_cmd_vel, 10);
            
            sub_turtle1Pose = this->create_subscription<turtlesim::msg::Pose>(topic_turtle1_pose, 10, 
                                                                              std::bind(&turtle1_controller::getMyPose, this, _1));

            sub_leadingTurtle = this->create_subscription<my_robot_interfaces::msg::Name>(topic_leading_turtle, 10,
                                                                                            std::bind(&turtle1_controller::getLeadTurtle, this, _1));
            
            client_TriggerDespawn = this->create_client<my_robot_interfaces::srv::Trigger>(service_trigger_despawn);

            // turtlesim runs at 30hz = 0.033s per frame
            timer_moveTurtle1 = this->create_wall_timer(0.02s, std::bind(&turtle1_controller::moveTurtle1, this));

            RCLCPP_INFO(this->get_logger(), "Controller has been started!");
        }

        void moveTurtle1(){

            auto msgVelocity = geometry_msgs::msg::Twist();
            auto request_despawn = std::make_shared<my_robot_interfaces::srv::Trigger::Request>();
    
            double controlSignal_angle;

            // dt matches the timer rate of moveTurtle1
            double dt = 0.02;

            if (nameCurrentTurtleTarget != "" && !despawn_is_running && !reachedTurtle() && !reverse_command_running){

                // first we get the desired heading with atan2 so that quadrant logic is handled
                // the order used is a must, otherwise you get an offset of pi degrees from the true desired heading
                double desiredHeading = std::atan2(target_yCoord - my_yCoord, target_xCoord - my_xCoord);

                // now we can grab heading error safely, with coordinate signs accounted for
                double headingError = desiredHeading - my_theta;

                // to ensure the robot only covers the shortest angular distance required, we wrap the angle
                // otherwise, we can have situations when the robot unnecessarily takes a longer time to turn
                headingError = std::fmod(headingError + M_PI, 2.0 * M_PI) - M_PI;
                
                // standard P calculation from the heading error for getting the control signal
                double headingP = Kp_angle * headingError;

                // standard D calculation using the last heading error
                double headingD = Kd_angle * ((headingError - lastHeadingError) / dt);

                controlSignal_angle = headingP + headingD;

                // for realistic behaviour (and safety, in real life), a maximum angular velocity is imposed
                controlSignal_angle = std::fmax(-maxAngularVel, std::fmin(controlSignal_angle, maxAngularVel));

                // now we publish the angular velocity command, and begin moving forward
                msgVelocity.angular.z = static_cast<float>(controlSignal_angle);
                msgVelocity.linear.x = 3;

                // the D term needs the last heading error
                lastHeadingError = headingError;

            } else if (reachedTurtle() && !despawn_is_running){

                // the robot will stop until the despawn process is complete, and a new turtle is acquired
                msgVelocity.linear.x = 0;
                msgVelocity.angular.z = 0;

                request_despawn->name = nameCurrentTurtleTarget;
                client_TriggerDespawn->async_send_request(request_despawn, 
                                                          std::bind(&turtle1_controller::callbackClientReceiveTrigger, this, _1));

                despawn_is_running = true;
            }

            // to prevent the robot from hitting the walls
            if (my_xCoord >= 11.0 || my_xCoord <= 0.5 || my_yCoord <= 0.5 || my_yCoord >= 11.0){

                // once theta resets close to desired heading, move forward
                double desiredHeading = std::atan2(target_yCoord - my_yCoord, target_xCoord - my_xCoord);
                double resetError = my_theta - desiredHeading;

                // for a positive reset error, my_theta is bigger than desiredHeading
                // so turn right, otherwise turn left
                resetError > 0 ? msgVelocity.angular.z = - maxAngularVel + 1.0 : msgVelocity.angular.z = maxAngularVel - 1.0; 
                
                if (resetError < 0) {resetError *= -1;}

                if (resetError <= 0.1) {
                    msgVelocity.linear.x = 3;
                    msgVelocity.angular.z = 0;
                } else {
                    msgVelocity.linear.x = 0;
                }
                
                reverse_command_running = true;
            } else {

                reverse_command_running = false;
            }
            
            pub_moveTurtle1->publish(msgVelocity);

        }

    private:

        float target_xCoord;
        float target_yCoord;
        int targetQuadrant;
        float my_xCoord;
        float my_yCoord;
        float my_theta;
        double Kp_angle = 4;
        double Kd_angle = 1;
        double maxAngularVel = 4;
        double lastHeadingError = 0;
        int myQuadrant = 1;
        double r;
        double thetaTarget;
        float error_coordThreshold = 0.75;
        bool despawn_is_running = false;
        bool reverse_command_running = false;
        std::string nameCurrentTurtleTarget;
        
        // publishing velocity commands for turtle1
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_moveTurtle1;

        // will dynamically subscribe to the pose of the latest turtle that is spawned
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_nextTurtlePose;

        // to get turlte1's own pose
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1Pose;

        // to acquire the leading turtle in despawner node's deque, which is the next turtle to reach
        rclcpp::Subscription<my_robot_interfaces::msg::Name>::SharedPtr sub_leadingTurtle;

        // client for triggering a despawn event, only when a target is reached
        rclcpp::Client<my_robot_interfaces::srv::Trigger>::SharedPtr client_TriggerDespawn;

        // timer to drive movement
        rclcpp::TimerBase::SharedPtr timer_moveTurtle1;

        // for attaching the un-added callback group to the dynamic subscription
        rclcpp::SubscriptionOptions sub_options;

        
        void dummyCallback(const turtlesim::msg::Pose dummyPose){
            RCLCPP_DEBUG(this->get_logger(), "Dummy pose x coord: %f", dummyPose.x);
            return;
        }

        // function for subscribing to the next sequential turtle that was spawned
        // this function only gets called when the despawner node publishes the leading turtle name
        // that published name will be in a subscription callback, which is where this function is called
        void subtoNextTurtle(std::string turtleName){
            
            auto msgOut = turtlesim::msg::Pose();

            sub_nextTurtlePose = this->create_subscription<turtlesim::msg::Pose>
                (turtleName + "/pose", 10, std::bind(&turtle1_controller::dummyCallback, this, _1), sub_options);

            auto context = this->get_node_base_interface()->get_context();

            // waiting indefinitely until a message is received from turtlesim
            // this ensures the dynamic topic creation is fully live and breathing, before updating target coords
            rclcpp::wait_for_message<turtlesim::msg::Pose>(msgOut, sub_nextTurtlePose, context, -1s);

            target_xCoord = msgOut.x;
            target_yCoord = msgOut.y;
        }

        // the target quadrant has the origin reference placed at turtle1's coordinates
        // we also calculate the theta target
        void getTargetQuadrant(){

            if (target_xCoord > my_xCoord && target_yCoord > my_yCoord){

                targetQuadrant = 1;
            } else if(target_xCoord < my_xCoord && target_yCoord > my_yCoord){

                targetQuadrant = 2;
            } else if (target_xCoord < my_xCoord && target_yCoord < my_yCoord){

                targetQuadrant = 3;
            } else {

                targetQuadrant = 4;
            }

            float x_error = std::abs(my_xCoord - target_xCoord);

            // we first get the separation
            r = std::sqrt(std::pow(x_error, 2) + std::pow((my_yCoord - target_yCoord), 2));

            // from first principles, the target angle is determined using the arccos of error and r
            thetaTarget = std::acos(x_error / r);

            RCLCPP_INFO(this->get_logger(), "[getTargetQuadrant] Theta target: %f", thetaTarget);
        }

        void getLeadTurtle(const my_robot_interfaces::msg::Name nextTurtle){

            nameCurrentTurtleTarget = nextTurtle.name;

            RCLCPP_INFO(this->get_logger(), "New target named %s!", nameCurrentTurtleTarget.c_str());

            subtoNextTurtle(nameCurrentTurtleTarget);

            getTargetQuadrant();
        }


        void getMyPose(const turtlesim::msg::Pose myPose){

            my_xCoord = myPose.x;
            my_yCoord = myPose.y;
            my_theta = myPose.theta;

            if (myPose.theta >= 0 && myPose.theta < (M_PI/2)){

                myQuadrant = 1;
            } else if(myPose.theta > (M_PI/2) && myPose.theta < M_PI){

                myQuadrant = 2;
            } else if (myPose.theta < (-M_PI/2) && myPose.theta > -M_PI){

                myQuadrant = 3;
            } else {

                myQuadrant = 4;
            }
        }

        // the next turtle is reached when the coordinates of the target and turtle1 meet an error threshold
        bool reachedTurtle(){

            if (nameCurrentTurtleTarget != ""){

                float xcoord_error = std::abs(target_xCoord - my_xCoord);
                float ycoord_error = std::abs(target_yCoord - my_yCoord);

                if ((ycoord_error <= error_coordThreshold) && 
                    (xcoord_error <= error_coordThreshold)){

                    return true;
                }

                return false;
            
            } else {

                return false;
            }
        }

        void callbackClientReceiveTrigger(rclcpp::Client<my_robot_interfaces::srv::Trigger>::SharedFuture future){

            auto response = future.get();

            // subscription for getLeadTurtle should now run
            // despawner should now give the new turtle name, and target quadrant is reset to 0
            if (response){

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Successfully despawned! Getting next turtle...");
                despawn_is_running = false;
            }
        }
            
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<turtle1_controller>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
