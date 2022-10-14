#include <ros/ros.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Pilot
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    uint8_t flags = 1;
    std::string printer;
    std_msgs::Char Czarus;

public:
    Pilot(ros::NodeHandle *nh)
    {
        pub = nh->advertise<std_msgs::Char>("/Pilot", 1);
        sub = nh->subscribe("/predator/ToPilot", 1, &Pilot::predator_cb, this);
    }
    void keyLoop();
    void predator_cb(const std_msgs::String::ConstPtr &msg)
    {
        flags = 1;
        printer = msg->data;
        ROS_INFO("Why doesnt it work?");
        Czarus.data = 'k';
        pub.publish(Czarus);
    }
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{

    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_input");
    ros::NodeHandle user_input("~");
    Pilot pilot = Pilot(&user_input);

    signal(SIGINT, quit);
    pilot.keyLoop();
    return (0);
}

void Pilot::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    raw.c_cc[VTIME] = 1; //TIMEOUT
    ros::Rate rate(4.0);
    while (ros::ok())
    {
        ros::spinOnce();

        if (flags == 1)
        {
            system("clear");
            puts("           PILOT           ");
            puts("---------------------------");
            puts(printer.c_str());
            flags = 0;
        }
        // get the next event from the keyboard
    
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);

        switch (c)
        {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            Czarus.data = 'a';
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            Czarus.data = 'b';
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            Czarus.data = 'c';
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            Czarus.data = 'd';
            dirty = true;
            break;
        }

        if (dirty == true)
        {
            pub.publish(Czarus);
            dirty = false;
        }
    }

    return;
}
