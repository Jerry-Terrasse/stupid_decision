#include "decision_node.hpp"

Status status = INIT;

nav_msgs::Odometry odom;
robot_msgs::Autoaim_Info enemies;
std_msgs::Int32 game_time;

double init_x = 8.5, init_y = 5;
double now_x = 0, now_y = 0;
double aim1_x = 9, aim1_y = 5;
double aim2_x = 14, aim2_y = 6;
double home_x = 7, home_y = 6;

int now_target = 1;
double reach_thresh = 1;
double target_radium = 0.8;

int low_hp_thresh = 400;
int build_low_hp_thresh = 50;

int hp = 1000;
int build_hp = 1500;

int rotate_when_move = 0;

void odom_callback(nav_msgs::Odometry::ConstPtr odom_)
{
    ROS_INFO("Odometry: %f, %f", odom_->pose.pose.position.x, odom_->pose.pose.position.y);
    odom = *odom_;
    now_x = odom.pose.pose.position.x + init_x;
    now_y = odom.pose.pose.position.y + init_y;
}

void enemy_callback(robot_msgs::Autoaim_Info::ConstPtr enemies_)
{
    ROS_INFO("Enemy: %d", enemies_->data.size());
    enemies = *enemies_;
}

void game_callback(std_msgs::Int32::ConstPtr game_time_)
{
    ROS_INFO("Game time: %d", game_time_->data);
    game_time = *game_time_;
}

bool reach(double x, double y, double thresh)
{
    return (x - now_x) * (x - now_x) + (y - now_y) * (y - now_y) < thresh * thresh;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stupid_decision_node");
    ros::NodeHandle nh;

    nh.getParam("init_x", init_x);
    nh.getParam("init_y", init_y);
    nh.getParam("aim1_x", aim1_x);
    nh.getParam("aim1_y", aim1_y);
    nh.getParam("aim2_x", aim2_x);
    nh.getParam("aim2_y", aim2_y);
    nh.getParam("reach_thresh", reach_thresh);
    nh.getParam("target_radium", target_radium);
    nh.getParam("rotate_when_move", rotate_when_move);
    nh.getParam("home_x", home_x);
    nh.getParam("home_y", home_y);

    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 1, odom_callback);
    ros::Subscriber enemy_sub = nh.subscribe("/autoaim2decision", 1, enemy_callback);
    ros::Subscriber game_sub = nh.subscribe("/game_time", 1, game_callback);
    // ros::Subscriber 

    ros::Publisher aim_pub = nh.advertise<robot_msgs::Walk_CMD>("/decision2pathplan", 1);
    ros::Publisher rotate_pub = nh.advertise<std_msgs::Int32>("/decision2ECbasespin", 1);
    ros::Publisher decision2autoaim_pub = nh.advertise<robot_msgs::Orient_Info>("/sentry/aim_control", 1);

    std_msgs::Int32 Zero;
    std_msgs::Int32 One;
    Zero.data = 0;
    One.data = 1;

    ros::Rate loop_rate(50);
    Status next_status = INIT;
    ros::spinOnce();
    for(; ros::ok(); ros::spinOnce(), loop_rate.sleep())
    {
        status = next_status;
        switch (status)
        {
        case INIT:
            if(game_time.data > 0) {
                ROS_INFO("Game start!");
                next_status = WANDER;
            } else {
                next_status = INIT;
            }
            break;

        case WANDER:
            if(hp < low_hp_thresh) {
                ROS_INFO("Low HP!");
                next_status = DEFEND;
                break;
            } else if(build_hp < build_low_hp_thresh) {
                ROS_INFO("Low BUILD HP!");
                next_status = DEFEND;
                break;
            } else if(enemies.data.size() > 0) {
                ROS_INFO("Enemy found!");
                next_status = FIGHT;
                break;
            } else {
                next_status = WANDER;
            }
            if(now_target == 1) {
                if(reach(aim1_x, aim1_y, reach_thresh)) {
                    now_target = 2;
                }
            } else if(now_target == 2) {
                if(reach(aim2_x, aim2_y, reach_thresh)) {
                    now_target = 1;
                }
            }
            rotate_pub.publish(rotate_when_move ? One : Zero);
            if(now_target == 1) {
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = target_radium;
                cmd.pos.x = aim1_x;
                cmd.pos.y = aim1_y;
                aim_pub.publish(cmd);
            } else if(now_target == 2) {
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = target_radium;
                cmd.pos.x = aim2_x;
                cmd.pos.y = aim2_y;
                aim_pub.publish(cmd);
            }
            break;

        case FIGHT:
            if(hp < low_hp_thresh) {
                ROS_INFO("Low HP!");
                next_status = DEFEND;
                break;
            } else if(build_hp < build_low_hp_thresh) {
                ROS_INFO("Low BUILD HP!");
                next_status = DEFEND;
                break;
            } else if(enemies.data.size() == 0) {
                ROS_INFO("Enemy lost!");
                next_status = WANDER;
                break;
            } else {
                next_status = FIGHT;
            }
            {
                rotate_pub.publish(One);
                // stop
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = 9999;
                cmd.pos.x = now_x;
                cmd.pos.y = now_y;
                aim_pub.publish(cmd);

                robot_msgs::Orient_Info orient;
                
                robot_msgs::Robot_Info enemy = enemies.data[0];
                orient.id = enemy.id;
                orient.ori.x = atan2(enemy.pos.y, enemy.pos.x);
                decision2autoaim_pub.publish(orient);
            }
            break;

        case DEFEND:
            if(reach(home_x, home_y, reach_thresh)) {
                ROS_INFO("Reach home!");
                next_status = DEFEND;
                rotate_pub.publish(One);
            } else {
                next_status = DEFEND;
                rotate_pub.publish(rotate_when_move ? One : Zero);
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = target_radium / 2;
                cmd.pos.x = home_x;
                cmd.pos.y = home_y;
                aim_pub.publish(cmd);
            }
            break;
        default:
            ROS_ERROR("Unknown status!");
            break;
        }
    }
    return 0;
}