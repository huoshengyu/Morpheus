#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ContactInformation.h>

#include <fcntl.h>      // File control (open)
#include <termios.h>    // Terminal control
#include <unistd.h>     // Write, close
#include <errno.h>      // Error number
#include <string.h>     // String operations
#include <iostream>     // IO Stream

class SerialCollisionSender
{
public:
    ros::NodeHandle nh;
    ros::Subscriber collision_sub;
    int serial_port;
    struct termios tty;
    std::string last_contact_link = ""; // <--- NEW: Remember last contact
    SerialCollisionSender()
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
        collision_sub = nh.subscribe("/vx300s/collision/nearest/contact", 10, &SerialCollisionSender::collisionCallback, this);
    }

    ~SerialCollisionSender()
    {
        close(serial_port);
    }

    void collisionCallback(const moveit_msgs::ContactInformation& msg)
    {
        std::string link1 = msg.contact_body_1;
        std::string link2 = msg.contact_body_2;
        
        if (link1 != last_contact_link && link2 != last_contact_link)
        {
            last_contact_link = link1 + "_" + link2; // optional: track both
        
            if (link1 == "vx300s/gripper_bar_link" || link2 == "vx300s/gripper_bar_link")
            {
                ROS_INFO_STREAM("Collision with gripper_bar_link detected! Sending '2' to Arduino.");
                char send_char = '2'; // Send 2
                write(serial_port, &send_char, 1);
                std::string pos_str = std::to_string(msg.position.x) + "," + std::to_string(msg.position.z) + "\n";
                write(serial_port, pos_str.c_str(), pos_str.length());
            }
            else
            {
                ROS_INFO_STREAM("Collision between: " << link1 << " and " << link2 << ". Sending '0'.");
                char send_char = '0';
                write(serial_port, &send_char, 1);
            }
        }
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_collision_sender");
    SerialCollisionSender node;
    ros::spin();
    return 0;
}
