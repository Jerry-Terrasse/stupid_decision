#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "robot_msgs/Autoaim_Info.h"
#include "robot_msgs/Walk_CMD.h"
#include "robot_msgs/Orient_Info.h"

enum Status
{
    INIT,
    WANDER,
    FIGHT,
    DEFEND,
};