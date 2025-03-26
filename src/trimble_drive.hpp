#ifndef TRIMBLE_DRIVER_H
#define TRIMBLE_DRIVER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <regex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace trimble_driver
{
class NMEA_Parser : public rclcpp::Node
{
public:
    NMEA_Parser(const rclcpp::NodeOptions &options);
    void process_sentence(const std::string &sentence);


private:
    void initialize(const std::string &ip, int port);
    double convert2Degrees(const double &value, std::string direction);
    std::vector<std::string> splitByDelimiter(const char* charArray, char delimiter);

    rclcpp::Node::SharedPtr node_hc;
    rclcpp::Node::SharedPtr node_ht;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gga;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_hdt;

    int sockfd_;
};
} // namespace trimble_driver
#endif // TRIMBLE_DRIVER_H