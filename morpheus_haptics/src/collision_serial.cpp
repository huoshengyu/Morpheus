#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ContactInformation.h>

#include <fcntl.h>      // File control (open)
#include <termios.h>    // Terminal control
#include <unistd.h>     // Write, close
#include <errno.h>      // Error number
#include <string.h>     // String operations
#include <iostream>     // IO Stream
#include <sensor_msgs/JointState.h>
#include <map>
#include <cmath>
#include <sys/ioctl.h>  // ✅ Add this line to fix your build

class SerialCollisionSender
{
public:
    ros::NodeHandle nh;
    ros::Subscriber collision_sub;
    ros::Subscriber joint_state_sub;

    int serial_port;
    struct termios tty;
    std::string last_contact_link = ""; // <--- NEW: Remember last contact
    std::map<std::string, double> current_joint_state;
    
    std::map<std::string, double> sleep_joint_state = {
        {"elbow", 1.55},
        {"forearm_roll", 0.0},
        {"shoulder", -1.85},
        {"waist", 0.0},
        {"wrist_angle", 0.8},
        {"wrist_rotate", 0.0}
    };

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
            // === Force reset Arduino via DTR toggle ===
        int flags;
        ioctl(serial_port, TIOCMGET, &flags);     // Get current modem bits
        flags &= ~TIOCM_DTR;                      // Clear DTR
        ioctl(serial_port, TIOCMSET, &flags);
        usleep(100000);                           // Wait 100 ms
        flags |= TIOCM_DTR;                       // Set DTR
        ioctl(serial_port, TIOCMSET, &flags);

        // === Flush garbage and wait for Arduino to boot ===
        tcflush(serial_port, TCIOFLUSH);          // Clean buffer
        ros::Duration(2.0).sleep();               // Wait for Arduino to be ready

        ROS_INFO("Serial port configured and Arduino reset successfully!");

        ROS_INFO("Serial port configured successfully!");

        // Subscribe to nearest collision contact
        collision_sub = nh.subscribe("/vx300s/collision/nearest/contact", 10, &SerialCollisionSender::collisionCallback, this);
        joint_state_sub = nh.subscribe("/vx300s/joint_states", 10, &SerialCollisionSender::jointStateCallback, this);
    }

    ~SerialCollisionSender()
    {
        close(serial_port);
    }

    // === Joint State Callback ===
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
            current_joint_state[msg->name[i]] = msg->position[i];
    }
    // === Check if robot is in Sleep posture ===
    bool isInSleepState()
    {
        if (current_joint_state.empty()) {
            return false;
        }

        double tolerance = 0.08;

        for (const auto& it : sleep_joint_state) {
            const std::string& joint = it.first;
            double target_val = it.second;

            auto jt = current_joint_state.find(joint);
            if (jt != current_joint_state.end()) {
                double current_val = jt->second;
                double diff = std::fabs(current_val - target_val);
                if (diff > tolerance) {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    void collisionCallback(const moveit_msgs::ContactInformation& msg)
    {
        if (isInSleepState()) {
            ROS_INFO_STREAM("Robot is in Sleep state. Skipping haptic feedback.");
            return;
        }
        std::string link1 = msg.contact_body_1;
        std::string link2 = msg.contact_body_2;

        static const std::set<std::string> end_effector_links = {
            "vx300s/gripper_bar_link", "vx300s/gripper_link",
            "vx300s/gripper_prop_link", "vx300s/right_finger_link", "vx300s/left_finger_link"
        };

        static const std::set<std::string> lower_arm_links = {
            "vx300s/wrist_link", "vx300s/lower_forearm_link"
        };

        if (link1 != last_contact_link && link2 != last_contact_link)
        {
            last_contact_link = link1 + "_" + link2;

            float distancex = msg.normal.x * msg.depth;
            float distancey = msg.normal.y * msg.depth;
            std::string pos_str = std::to_string(distancex) + "," + std::to_string(distancey) + "\n";

            if (end_effector_links.count(link1) || end_effector_links.count(link2)) {
                ROS_INFO_STREAM("Collision with end effector. Sending '1'");
                char send_char = '1';
                write(serial_port, &send_char, 1);
                write(serial_port, pos_str.c_str(), pos_str.length());
            }
            else if (lower_arm_links.count(link1) || lower_arm_links.count(link2)) {
                ROS_INFO_STREAM("Collision with lower arm. Sending '2'");
                char send_char = '2';
                write(serial_port, &send_char, 1);
                write(serial_port, pos_str.c_str(), pos_str.length());
            }
            else if (link1 == "vx300s/upper_forearm_link" || link2 == "vx300s/upper_forearm_link") {
                ROS_INFO_STREAM("Collision with upper forearm. Sending '3'");
                char send_char = '3';
                write(serial_port, &send_char, 1);
                write(serial_port, pos_str.c_str(), pos_str.length());
            }
            else if (link1 == "vx300s/upper_arm_link" || link2 == "vx300s/upper_arm_link") {
                ROS_INFO_STREAM("Collision with upper arm. Sending '4'");
                char send_char = '4';
                write(serial_port, &send_char, 1);
                write(serial_port, pos_str.c_str(), pos_str.length());
            }
            else {
                ROS_INFO_STREAM("Other collision: " << link1 << " vs " << link2 << ". Sending '0'");
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
