#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "robot_msgs/Autoaim_Info.h"
#include "robot_msgs/Walk_CMD.h"
#include "robot_msgs/Orient_Info.h"
#include "robot_msgs/Build_State.h"
#include "robot_msgs/Robot_Blood_Info.h"

enum Status
{
    INIT,
    WANDER,
    FIGHT,
    DEFEND,
    RECOVER,
};