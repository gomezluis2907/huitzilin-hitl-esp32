#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// No padding
#pragma pack(push, 1) 
struct ImuDataPacket 
{
    // Magic Bytes
    uint8_t header1;
    uint8_t header2;

    float pitch;
    float roll;
    float yaw;
};
#pragma pack(pop)

class HuitzilinSerialNode : public rclcpp::Node 
{
public:
    HuitzilinSerialNode() : Node("huitzilin_serial") 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/huitzilin/imu", 10,
                                                                        std::bind(&HuitzilinSerialNode::callbackHuitzilinImu, 
                                                                        this, std::placeholders::_1));
        
        // Open the USB port and assign it to the CLASS variable, not a local variable
        usb_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

        // Crashes if the cable isn't plugged in
        if (usb_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0.");
            return; 
        }

        // Grab the current default USB settings
        struct termios tty;
        tcgetattr(usb_port_, &tty);

        // Set Baud Rate to 115200
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag &= ~PARENB; // Disable parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;  // Clear size mask
        tty.c_cflag |= CS8;     // Force 8 data bits

        // Tell Linux not to format our bytes as text characters
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;

        // Apply the settings immediately (TCSANOW)
        tcsetattr(usb_port_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Huitzilin Serial has been started");
        RCLCPP_INFO(this->get_logger(), "/dev/ttyUSB0 port opened at 115200 baud.");
    }
 
private:
    // Declare member variables here so the whole class can see them
    int usb_port_; 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

    // Callback 
    void callbackHuitzilinImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Load IMU data into a Quaternion object
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // Matrix 3x3 is created from the quaternion
        tf2::Matrix3x3 m(q);

        // Euler angles
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

        // Struct instance
        ImuDataPacket my_packet;

        // Magic Bytes
        my_packet.header1 = 0xAA;
        my_packet.header2 = 0xBB;

        // Explicitly cast down to float to prevent compiler warnings
        my_packet.roll = (float)roll;
        my_packet.pitch = (float)pitch;
        my_packet.yaw = (float)yaw;

        // Cast &my_packet as uint8_t in order for the compiler to read byte by byte
        uint8_t* raw_bytes = (uint8_t*)&my_packet;
        
        // Write using the class variable
        write(usb_port_, raw_bytes, sizeof(ImuDataPacket));
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuitzilinSerialNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}