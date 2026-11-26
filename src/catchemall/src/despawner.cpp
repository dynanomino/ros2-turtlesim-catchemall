#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/name.hpp"
#include "my_robot_interfaces/srv/trigger.hpp"
#include "turtlesim/srv/kill.hpp"
#include <deque>

using namespace std::placeholders;
using namespace std::chrono_literals;

class despawner : public rclcpp::Node{

    public:

        std::string topic_leading_turtle = "leading_turtle";
        std::string topic_new_turtle = "new_turtle";
        std::string service_trigger_despawn = "trigger_despawn";
        std::string service_kill = "kill";

        despawner() : Node("despawner"){

            pub_leadingTurtle = this->create_publisher<my_robot_interfaces::msg::Name>(topic_leading_turtle, 10);
            
            sub_newTurtle = this->create_subscription<my_robot_interfaces::msg::Name>(topic_new_turtle, 10, 
                                                                                      std::bind(&despawner::getNextTurtle, this, _1));
            
            server_trigger = this->create_service<my_robot_interfaces::srv::Trigger>(service_trigger_despawn, 
                                                                                     std::bind(&despawner::callbackServerRespondTrigger, this, _1, _2));
            
            client_despawn = this->create_client<turtlesim::srv::Kill>(service_kill);

            RCLCPP_INFO(this->get_logger(), "Despawner has started!");
        }

    private:

        rclcpp::Publisher<my_robot_interfaces::msg::Name>::SharedPtr pub_leadingTurtle;

        rclcpp::Subscription<my_robot_interfaces::msg::Name>::SharedPtr sub_newTurtle;

        rclcpp::Service<my_robot_interfaces::srv::Trigger>::SharedPtr server_trigger;

        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_despawn;

        std::deque<std::string> allTurtles;
        
        // adding new turtle to the deque
        void getNextTurtle(const my_robot_interfaces::msg::Name newTurtle){

            // if empty, then the leading turtle is the current new turtle
            if (allTurtles.empty()){

                auto msg = my_robot_interfaces::msg::Name();
                msg.name = newTurtle.name;
                pub_leadingTurtle->publish(msg);
            }

            allTurtles.push_back(newTurtle.name);
            RCLCPP_INFO(this->get_logger(), "Added turtle %s to deque! Current size: %d", newTurtle.name.c_str(), (int)allTurtles.size());
        }

        // once a given target is reached, the despawn is ran and only after turtlesim is processed, does the controller get a response
        void callbackServerRespondTrigger(const my_robot_interfaces::srv::Trigger::Request::SharedPtr requestTrigger,
                                          const my_robot_interfaces::srv::Trigger::Response::SharedPtr responseTrigger){
        
            // the despawn target must match the head of the deque
            if (allTurtles.front() == requestTrigger->name){

                RCLCPP_INFO(this->get_logger(), "Despawn match confirmed! Despawning now...");
            
                auto requestDespawn = std::make_shared<turtlesim::srv::Kill::Request>();
                requestDespawn->name = requestTrigger->name;

                while (!client_despawn->wait_for_service(1s)){
                    
                    // prevents very weird closing behaviour when you ctrl+c the client
                    if(!rclcpp::ok()){
                        RCLCPP_INFO(this->get_logger(), "Shutting down...");
                        return;
                    }

                    RCLCPP_WARN(this->get_logger(), "Service [%s] not yet live...", service_kill.c_str());
                }

                // turtlesim doesn't respond for this service
                // so no callback for feedback is declared
                client_despawn->async_send_request(requestDespawn);

                allTurtles.pop_front();

                // th new leading turtle is given
                if (!allTurtles.empty()){

                    auto msg = my_robot_interfaces::msg::Name();
                    msg.name = allTurtles.front();
                    pub_leadingTurtle->publish(msg);
                    
                    RCLCPP_INFO(this->get_logger(), "Next turtle: %s", msg.name.c_str());

                } else {

                    RCLCPP_WARN(this->get_logger(), "Deque is empty! Waiting for spawner...");
                }

                responseTrigger->success = true;
               
            // if there's a mismatch between the front and the requested trigger, an old request was being processed
            } else if (!allTurtles.empty()) {

                RCLCPP_INFO(this->get_logger(), "Front of queue: %s. Request: %s", allTurtles.front().c_str(), requestTrigger->name.c_str());
                responseTrigger->success = false;
            }
        }
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<despawner>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}