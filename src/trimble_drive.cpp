#include "trimble_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <regex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace trimble_driver
{
NMEA_Parser::NMEA_Parser(const rclcpp::NodeOptions &options)
    : Node("gps_publisher", options)
{
    RCLCPP_INFO(this->get_logger(), "GPSListener constructor start");

    // Publisher for heading (degrees)
    publisher_hdt = this->create_publisher<std_msgs::msg::Float64>("heading", 10);

    publisher_gga = this->create_publisher<sensor_msgs::msg::NavSatFix>("NavSatFix", 10);

    // UDP Socket setup
    initialize("0.0.0.0", 6019); 
}

void NMEA_Parser::initialize (const std::string &ip, int port) 
{
    (void)ip;  // Suppress unused parameter warning

    struct sockaddr_in server_addr;

    // Create UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error creating UDP socket");
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
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error binding UDP socket");
        close(sockfd_);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Listening for UDP packets on port %d", port);

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
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error receiving data");
            continue;
        }

        buffer[len] = '\0'; // Null-terminate the received data

        std::string sentence(buffer);

        // Process the NMEA sentence
        process_sentence(sentence);
    }
}

void NMEA_Parser::process_sentence(const std::string &sentence)
{
    std::vector sentenceData = NMEA_Parser::splitByDelimiter(sentence.c_str(), ',');

    
   
    if (sentenceData.size() > 2 && sentenceData[0] == "$GPHDT"){
        double heading = std::stod(sentenceData[1]);        
        
        // Create a message and publish the heading in degrees
        auto heading_msg = std::make_shared<std_msgs::msg::Float64>();
        heading_msg->data = heading;
        publisher_hdt->publish(*heading_msg);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Published Heading: %.2f°", heading);
    } 
    
    if (sentenceData.size() > 4 && sentenceData[0] == "$GPGGA") {        

        auto gps_fix = std::make_shared<sensor_msgs::msg::NavSatFix>();

        gps_fix->latitude = NMEA_Parser::convert2Degrees(std::stod(sentenceData[2]),sentenceData[3]);
        gps_fix->longitude = NMEA_Parser::convert2Degrees(std::stod(sentenceData[4]),sentenceData[5]);

        publisher_gga->publish(*gps_fix);
    }  
}

double NMEA_Parser::convert2Degrees(const double &value, std::string direction){
    
    double degrees, minutes, result;

    try{
        degrees = value / 100;
        minutes = value - (degrees * 100);    
    }
    catch(const std::exception & excpt){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in degree conversion due to: %s", excpt.what());
        throw;
    }
    
    result = degrees + (minutes / 60);

    if(direction == "S" || direction == "W"){
        return -result;
    }

    return result;
}

std::vector<std::string> NMEA_Parser::splitByDelimiter(const char* charArray, char delimiter){
    
    std::vector<std::string> result;
    std::string token;

    for(int i = 0; charArray[i] != '\0'; i++){
        if(charArray[i] == delimiter){
            if(!token.empty()){
                result.push_back(token);
                token.clear();
            }
        }else{
            token += charArray[i];
        }
    }

    if(!token.empty()){
        result.push_back(token);
    }
    
    return result;
}
} // namespace trimble_driver

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // Create rclcpp::NodeOptions instance
    rclcpp::NodeOptions options;

    // Create shared pointer to NMEA_Parser, passing the options to the constructor
    rclcpp::spin(std::make_shared<trimble_driver::NMEA_Parser>(options));

    rclcpp::shutdown();
    return 0;
}
