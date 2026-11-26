#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/name.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>

using namespace std::chrono_literals;
using namespace std::placeholders;


class spawner : public rclcpp::Node{

    public:

        std::string service_spawn = "spawn";
        std::string topic_new_turtle = "new_turtle";

        // rng_ and dist_ are for a true random spawn
        spawner() : Node("spawner"), rng_(std::random_device{}()), dist_(1, 10){

            client_spawn = this->create_client<turtlesim::srv::Spawn>(service_spawn);

            pub_newTurtle = this->create_publisher<my_robot_interfaces::msg::Name>(topic_new_turtle, 10);

            timer_spawn = this->create_wall_timer(2s, std::bind(&spawner::runSpawn, this));

            RCLCPP_INFO(this->get_logger(), "Spawner has started!");
        }

        void runSpawn(){

            // spawning a new turtle within the xcoord and ycoord bounds of 10, minimum coord of 1
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = dist_(rng_);
            request->y = dist_(rng_);
            request->theta = 0;

            while(!client_spawn->wait_for_service(1s)){

                // prevents very weird closing behaviour when you ctrl+c the client
                if(!rclcpp::ok()){
                    RCLCPP_INFO(this->get_logger(), "Shutting down...");
                    return;
                }

                RCLCPP_WARN(this->get_logger(), "Service [%s] not yet live...", service_spawn.c_str());
            }

            client_spawn->async_send_request(request, std::bind(&spawner::callbackClientReceiveSpawn, this, _1));
        }

    private:

        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn;
        
        rclcpp::Publisher<my_robot_interfaces::msg::Name>::SharedPtr pub_newTurtle;

        rclcpp::TimerBase::SharedPtr timer_spawn;

        // for a truly random spawn
        std::mt19937 rng_;
        std::uniform_int_distribution<int> dist_;

        void callbackClientReceiveSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){

            auto msg = my_robot_interfaces::msg::Name();

            auto response = future.get();

            // we then publish this new turtle to the /new_turtle topic for the depsawner's deque
            msg.name = response->name;
            pub_newTurtle->publish(msg);
        }
};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<spawner>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
