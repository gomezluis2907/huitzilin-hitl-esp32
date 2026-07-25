#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
#include <actuator_msgs/msg/actuators.hpp>
#include <cstring>


#pragma pack(push, 1) //No padding 

struct ImuDataPacket 
{
    // Magic Bytes
    uint8_t header1;
    uint8_t header2;

    float pitch;
    float roll;
    float yaw;
};
#pragma pack(pop) //No padding

#pragma pack(push, 1)
struct KeysDataPacket
{
    // Magic Bytes
    uint8_t header1;
    uint8_t header2;

    float pitch;
    float roll;
    float yaw;
    float throttle;
};
#pragma pack(pop) // No padding

#pragma pack(push, 1)
struct RpmPayload
{

    float rotor_0;
    float rotor_1;
    float rotor_2;
    float rotor_3;

};
#pragma pack(pop)

enum SerialState {

    WAIT_FOR_EE,
    WAIT_FOR_FF,
    READ_RPM_PAYLOAD

};

class HuitzilinSerialNode : public rclcpp::Node 
{
public:
    HuitzilinSerialNode() : Node("huitzilin_serial") 
    {
        // /huitzilin/imu subscriber
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/huitzilin/odom", 10,
                                                                        std::bind(&HuitzilinSerialNode::callbackHuitzilinOdom, 
                                                                        this, std::placeholders::_1));
        
        // /cmd_vel subscriber
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
                                                                            std::bind(&HuitzilinSerialNode::callbackCmdVel, 
                                                                            this, std::placeholders::_1));
        
        // /huitzilin/motor_speed publisher
        huitzilin_motor_speed_publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>("/huitzilin/motor_speed", 10);
        
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


        // Thread
        rx_thread_ = std::thread(&HuitzilinSerialNode::rxThread, this);
        rx_thread_.detach();
    }
 
private:

    // Declare member variables here so the whole class can see them
    int usb_port_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr huitzilin_motor_speed_publisher_;

    // Thread
    std::thread rx_thread_;
    

    // Odometry callback 
    void callbackHuitzilinOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Load IMU data into a Quaternion object
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        // Matrix 3x3 is created from the quaternion
        tf2::Matrix3x3 m(q);

        // Euler angles
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

        // Struct instance
        ImuDataPacket imu_packet;

        // Magic Bytes
        imu_packet.header1 = 0xAA;
        imu_packet.header2 = 0xBB;

        // Explicitly cast down to float to prevent compiler warnings
        imu_packet.roll = (float)roll;
        imu_packet.pitch = (float)pitch;
        imu_packet.yaw = (float)yaw;

        // Cast &imu_packet as uint8_t in order for the compiler to read byte by byte
        uint8_t* imu_bytes = (uint8_t*)&imu_packet;
        
        // Write 
        write(usb_port_, imu_bytes, sizeof(ImuDataPacket));
    }

    void callbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Struct instance 
        KeysDataPacket key_packet;

        // Magic Bytes
        key_packet.header1 = 0xCC;
        key_packet.header2 = 0xDD;

        key_packet.pitch = msg->linear.x;
        key_packet.roll = msg->linear.y;
        key_packet.yaw = msg->angular.z;
        key_packet.throttle = msg->linear.z;

        // Cast &key_packet as uint8_t in order for the compiler to read byte by byte
        uint8_t* key_bytes = (uint8_t*)&key_packet;

        // Write
        write(usb_port_, key_bytes, sizeof(KeysDataPacket));

    };

    // PID thread
    void rxThread()
    {

        uint8_t raw_bytes[256];
        SerialState current_state = WAIT_FOR_EE;
        uint8_t payload[16];
        int payload_index = 0;


        while(rclcpp::ok())
        {

            // Total number of bytes
            int len = read(usb_port_, raw_bytes, 256);

            for (int i = 0; i < len; i++) {

                uint8_t byte = raw_bytes[i];

                switch(current_state) {

                    case WAIT_FOR_EE:
                    if (byte == 0xEE) current_state = WAIT_FOR_FF;
                    else current_state = WAIT_FOR_EE;
                    break;

                    case WAIT_FOR_FF:
                    if (byte == 0xFF) { 
                        current_state = READ_RPM_PAYLOAD;
                        payload_index = 0;
                    } else if (byte == 0xEE) {current_state = WAIT_FOR_FF;
                    } else current_state = WAIT_FOR_EE;
                    break;

                    case READ_RPM_PAYLOAD:
                    payload[payload_index] = byte;
                    payload_index++;

                    if (payload_index == 16){

                        RpmPayload rpm_data;
                        std::memcpy(&rpm_data, payload, sizeof(RpmPayload));

                        // Detect corrupted packets
                        if (std::isnan(rpm_data.rotor_0) || std::abs(rpm_data.rotor_0) > 15000.0f ||
                            std::isnan(rpm_data.rotor_1) || std::abs(rpm_data.rotor_1) > 15000.0f ||
                            std::isnan(rpm_data.rotor_2) || std::abs(rpm_data.rotor_2) > 15000.0f ||
                            std::isnan(rpm_data.rotor_3) || std::abs(rpm_data.rotor_3) > 15000.0f) {
                            
                            // Throw away the corrupted packet
                            current_state = WAIT_FOR_EE;
                            break;
                        }

                        auto actuator_msg = actuator_msgs::msg::Actuators();

                        actuator_msg.velocity = {
                            rpm_data.rotor_0, 
                            rpm_data.rotor_1, 
                            rpm_data.rotor_2, 
                            rpm_data.rotor_3};              
                        
                        huitzilin_motor_speed_publisher_->publish(actuator_msg);

                        current_state = WAIT_FOR_EE;

                    }
                    break;


                }
            }

        }

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