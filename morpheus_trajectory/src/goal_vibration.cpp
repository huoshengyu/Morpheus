#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ContactInformation.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

#include <fcntl.h>      // File control (open)
#include <termios.h>    // Terminal control
#include <unistd.h>     // Write, close
#include <errno.h>      // Error number
#include <string.h>     // String operations
#include <iostream>     // IO Stream


class serialTrajecSender
{
public:
    ros::NodeHandle nh;
    ros::Subscriber goal_dist_sub;
    ros::Subscriber goal_direc_sub;

    int serial_port;
    double traj_distance;
    geometry_msgs::Vector3 latest_direction;
    struct termios tty;
    serialTrajecSender()
    {
        // Open serial port
        serial_port = open("/dev/ttyUSB0", O_RDWR); // << Change to your Arduino port!
        if (serial_port < 0)
        {
            ROS_ERROR("Error %i from open: %s", errno, strerror(errno));
            exit(1);
        }

        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port, &tty) != 0)
        {
            ROS_ERROR("Error %i from tcgetattr: %s", errno, strerror(errno));
            exit(1);
        }

        // Setup serial parameters
        tty.c_cflag &= ~PARENB; // No parity bit
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;// No flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore control lines

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, SUSP

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; 

        tty.c_cc[VTIME] = 10;    // Wait for up to 1 second (10 deciseconds)
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            ROS_ERROR("Error %i from tcsetattr: %s", errno, strerror(errno));
            exit(1);
        }

        ROS_INFO("Serial port configured successfully!");

        // Subscribe to nearest collision contact
        goal_dist_sub = nh.subscribe("/vx300s/trajectory/goal/distance", 10, &serialTrajecSender::distanceCallback, this);
        goal_direc_sub = nh.subscribe("/vx300s/trajectory/goal/vector", 10, &serialTrajecSender::directionCallback, this);

    }
    void directionCallback(const geometry_msgs::Vector3& msg)
    {
        // Just store the latest direction
        latest_direction = msg;
    }

    void distanceCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        traj_distance = msg->data;
        ROS_INFO_STREAM("Received forward distance: " << traj_distance);

        if (traj_distance <= 0.1)
        {
            char send_char = '1';  // Inactivation
            write(serial_port, &send_char, 1);
            ROS_INFO("Sent '1' to Arduino (inactive)");
        }
        else
        {
            char send_char = '0';  // Activation
            write(serial_port, &send_char, 1);
            usleep(150000);  // 150 ms
            ROS_INFO("Sent '0' to Arduino (active)");

            // Send direction sequentially
            sendDirectionComponent(latest_direction.x,latest_direction.y,latest_direction.z);
            // sendDirectionComponent(latest_direction.y);
            // sendDirectionComponent(latest_direction.z);
        }
    }

    void sendDirectionComponent(float x, float y, float z)
    {
        std::string out = "START X:" + std::to_string(x) + " Y:" + std::to_string(y) + " Z:" + std::to_string(z) + " END\n";
        write(serial_port, out.c_str(), out.length());
        usleep(100000);
        tcflush(serial_port, TCOFLUSH); // Flush buffer
        ROS_INFO_STREAM("Sent to Arduino: " << out);
        // usleep(200000); // 100 ms
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_trajec_sender");
    serialTrajecSender node;
    ros::spin();
    return 0;
}