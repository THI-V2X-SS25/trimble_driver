#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <regex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

class NMEA_HDT_Parser : public rclcpp::Node
{
public:
    NMEA_HDT_Parser() : Node("nmea_hdt_parser")
    {
        // Publisher for heading (degrees)
        heading_publisher_ = this->create_publisher<std_msgs::msg::Float64>("heading", 10);

        // UDP Socket setup
        setup_udp_socket("0.0.0.0", 6019); // Bind to all IP addresses for the port
    }

    ~NMEA_HDT_Parser()
    {
        close(sockfd_);
    }

private:
    void setup_udp_socket(const std::string &ip, int port)
    {
        struct sockaddr_in server_addr;

        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error creating UDP socket");
            return;
        }

        // Setup server address structure
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
        server_addr.sin_port = htons(port); // Port number

        // Bind the socket to the IP and port
        if (bind(sockfd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error binding UDP socket");
            close(sockfd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d", port);
        receive_udp_data();
    }

    void receive_udp_data()
    {
        char buffer[1024];
        ssize_t len;
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        while (rclcpp::ok())
        {
            // Receive data from UDP socket
            len = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);
            if (len < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Error receiving data");
                continue;
            }

            buffer[len] = '\0'; // Null-terminate the received data

            std::string sentence(buffer);

            // Process the NMEA sentence
            process_nmea_sentence(sentence);
        }
    }

    void process_nmea_sentence(const std::string &sentence)
    {
        // Regular expression to match the HDT sentence
        std::regex hdt_regex("\\$GPHDT,([\\d.]+),T\\*.*");
        std::smatch match;
        std::cout << sentence << std::endl;

        if (std::regex_match(sentence, match, hdt_regex))
        {
            // Extract the heading value
            double heading = std::stod(match[1].str());

            // Create a message and publish the heading in degrees
            auto heading_msg = std::make_shared<std_msgs::msg::Float64>();
            heading_msg->data = heading;
            heading_publisher_->publish(*heading_msg);
            RCLCPP_INFO(this->get_logger(), "Published Heading: %.2f°", heading);
        }
        else
        {
            ;//RCLCPP_WARN(this->get_logger(), "No HDT sentence found in the received data.");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher_;
    int sockfd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMEA_HDT_Parser>());
    rclcpp::shutdown();
    return 0;
}
