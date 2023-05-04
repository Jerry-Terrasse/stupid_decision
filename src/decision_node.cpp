#include "decision_node.hpp"

Status status = INIT;

nav_msgs::Odometry odom;
robot_msgs::Autoaim_Info enemies;
int vitual_mode = 0;
std_msgs::Int32 game_time;

double init_x = 8.5, init_y = 5;
double now_x = 0, now_y = 0;
double yaw;
double sensorOffsetX = 0.195, sensorOffsetY = 0;
double aim1_x = 9, aim1_y = 5;
double aim2_x = 14, aim2_y = 6;
double home_x = 7, home_y = 6;
double recover_x = 7, recover_y = 6;

double buff_x_1 = 6, buff_y_1 = 3.5;
double buff_x_2 = 6, buff_y_2 = 4.5;
double patrol_x_1 = 7.230, patrol_y_1 = 1.5;
double patrol_x_2 = 6.240, patrol_y_2 = 1.020;

double min_x=0, max_x=0, min_y=0, max_y=0, min_z=0, max_z=0;

int now_target = 1;
bool running = false;
double reach_thresh = 1;
double target_radium = 0.8;

int low_hp_thresh = 300;
int build_low_hp_thresh = 800;

int hp = 1000;
int build_hp = 1500;

int rotate_when_move = 0;
bool use_build_blood = true;

double enemy_ts = 0;
double exit_fight_time = 2;

int recover_cnt = 3;

void odom_callback(nav_msgs::Odometry::ConstPtr odom_)
{
    // ROS_INFO("Odometry: %f, %f", odom_->pose.pose.position.x, odom_->pose.pose.position.y);
    odom = *odom_;
    yaw = tf::getYaw(odom.pose.pose.orientation);
    now_x = odom.pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY + init_x;
    now_y = odom.pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY + init_y;
    // now_x = odom.pose.pose.position.x + init_x;
    // now_y = odom.pose.pose.position.y + init_y;
}

void enemy_callback(robot_msgs::Autoaim_Info::ConstPtr enemies_)
{
    ROS_INFO("Enemy: %ld", enemies_->data.size());
    enemies = *enemies_;
    if(enemies.data.size() > 0) {
        enemy_ts = ros::Time::now().toSec();
    }
}

void vitual_callback(std_msgs::Int32::ConstPtr vitual_)
{
    // ROS_INFO("Vitual: %d", vitual_->data);
    vitual_mode = vitual_->data;
    if(vitual_->data > 1) {
        enemy_ts = ros::Time::now().toSec();
    }
}

void game_callback(std_msgs::Int32::ConstPtr game_time_)
{
    // ROS_INFO("Game time: %d", game_time_->data);
    game_time = *game_time_;
}

void hp_callback(robot_msgs::Robot_Blood_Info::ConstPtr team_blood_)
{
    // ROS_INFO("HP: %d", team_blood_->data);
    for(int i = 0; i < team_blood_->data.size(); i++)
        if(team_blood_->data[i].id == 0)
        {
            hp = team_blood_->data[i].blood;
            // ROS_INFO("HP: %d", hp);
        }
}

void build_hp_callback(robot_msgs::Build_State::ConstPtr build_hp_)
{
    // ROS_INFO("Build HP: %d", build_hp_->data);
    build_hp = build_hp_->blood;
}

bool reach(double x, double y, double thresh)
{
    ROS_INFO("dis: %f", (x - now_x) * (x - now_x) + (y - now_y) * (y - now_y));
    return (x - now_x) * (x - now_x) + (y - now_y) * (y - now_y) < thresh * thresh;
}

bool pos_not_valid(double x, double y, double z)
{
    return x < min_x || x > max_x || y < min_y || y > max_y || z < min_z || z > max_z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;

    nh.getParam("init_x", init_x);
    nh.getParam("init_y", init_y);
    // nh.getParam("aim1_x", aim1_x);
    // nh.getParam("aim1_y", aim1_y);
    // nh.getParam("aim2_x", aim2_x);
    // nh.getParam("aim2_y", aim2_y);
    nh.getParam("reach_thresh", reach_thresh);
    nh.getParam("target_radium", target_radium);
    nh.getParam("rotate_when_move", rotate_when_move);
    nh.getParam("use_build_blood", use_build_blood);
    nh.getParam("home_x", home_x);
    nh.getParam("home_y", home_y);
    nh.getParam("recover_x", recover_x);
    nh.getParam("recover_y", recover_y);
    nh.getParam("sensorOffsetX", sensorOffsetX);
    nh.getParam("sensorOffsetY", sensorOffsetY);
    
    nh.getParam("buff_x_1", buff_x_1);
    nh.getParam("buff_y_1", buff_y_1);
    nh.getParam("buff_x_2", buff_x_2);
    nh.getParam("buff_y_2", buff_y_2);
    nh.getParam("patrol_x_1", patrol_x_1);
    nh.getParam("patrol_y_1", patrol_y_1);
    nh.getParam("patrol_x_2", patrol_x_2);
    nh.getParam("patrol_y_2", patrol_y_2);

    nh.getParam("min_x", min_x);
    nh.getParam("max_x", max_x);
    nh.getParam("min_y", min_y);
    nh.getParam("max_y", max_y);
    nh.getParam("min_z", min_z);
    nh.getParam("max_z", max_z);

    nh.getParam("exit_fight_time", exit_fight_time);

    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 1, odom_callback);
    ros::Subscriber enemy_sub = nh.subscribe("/autoaim2decision", 1, enemy_callback);
    ros::Subscriber autoaim_sub = nh.subscribe("/vitual_mode", 1, vitual_callback);
    ros::Subscriber game_sub = nh.subscribe("/game_time", 1, game_callback);
    ros::Subscriber hp_sub = nh.subscribe("/sentry/team_blood", 1, hp_callback);
    ros::Subscriber build_hp_sub = nh.subscribe("/sentry/build_state", 1, build_hp_callback);
    // ros::Subscriber 

    ros::Publisher aim_pub = nh.advertise<robot_msgs::Walk_CMD>("/decision2pathplan", 1);
    ros::Publisher rotate_pub = nh.advertise<std_msgs::Int32>("/decision2ECbasespin", 1);
    ros::Publisher decision2autoaim_pub = nh.advertise<robot_msgs::Orient_Info>("/sentry/aim_control", 1);

    std_msgs::String state_msg;
    ros::Publisher state_pub = nh.advertise<std_msgs::String>("/state", 1);

    std_msgs::Int32 Zero;
    std_msgs::Int32 One;
    std_msgs::Int32 Five;
    Zero.data = 0;
    One.data = 1;
    Five.data = 5;

    ros::Rate loop_rate(2);
    Status next_status = INIT;
    ros::spinOnce();
    double ts=0;
    for(; ros::ok(); ros::spinOnce(), loop_rate.sleep())
    {
        ROS_WARN("pos: %lf %lf", now_x, now_y);
        if(pos_not_valid(now_x, now_y, odom.pose.pose.position.z)) {
            for(; ros::ok(); ros::spinOnce(), loop_rate.sleep())
            {
                ROS_WARN("!!! ======== PROTECTING MODE ======== !!!");
                rotate_pub.publish(Five);
                state_msg.data = "PROTECTING";
                state_pub.publish(state_msg);
            }
            break;
        }
        status = next_status;
        switch (status)
        {
        case INIT:
            state_msg.data = "INIT";
            state_pub.publish(state_msg);
            ROS_INFO("Status: INIT");
            if(game_time.data > 0) {
                ROS_INFO("Game start!");
                next_status = WANDER;

                // fast start
                rotate_pub.publish(rotate_when_move ? One : Zero);
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = target_radium;
                cmd.pos.x = patrol_x_1;
                cmd.pos.y = patrol_y_1;
                aim_pub.publish(cmd);
            } else {
                next_status = INIT;
            }
            break;

        case WANDER:
            state_msg.data = "WANDER";
            state_pub.publish(state_msg);
            ROS_INFO("Status: WANDER %d", now_target);

            if(game_time.data > 56 && game_time.data < 74) {
                aim1_x = buff_x_1;
                aim1_y = buff_y_1;
                aim2_x = buff_x_2;
                aim2_y = buff_y_2;
            } else {
                aim1_x = patrol_x_1;
                aim1_y = patrol_y_1;
                aim2_x = patrol_x_2;
                aim2_y = patrol_y_2;
            }

            if(game_time.data > 300) {
                ROS_INFO("Game time late");
                next_status = DEFEND;
                running = false;
                break;
            } else if(hp < low_hp_thresh && recover_cnt > 0) {
                ROS_INFO("Low HP!");
                --recover_cnt;
                next_status = RECOVER;
                running = false;
                break;
            } else if(build_hp < build_low_hp_thresh && (use_build_blood)) {
                ROS_INFO("Low BUILD HP!");
                next_status = DEFEND;
                running = false;
                break;
            } else if(enemies.data.size() > 0 || vitual_mode > 1) {
                ROS_INFO("Enemy found!");
                enemy_ts = ros::Time::now().toSec();
                next_status = FIGHT;
                running = false;
                break;
            } else {
                next_status = WANDER;
            }
            if(now_target == 1) {
                if(reach(aim1_x, aim1_y, reach_thresh)) {
                    running = false;
                    now_target = 2;
                }
            } else if(now_target == 2) {
                if(reach(aim2_x, aim2_y, reach_thresh)) {
                    running = false;
                    now_target = 1;
                }
            }
            rotate_pub.publish(rotate_when_move ? One : Zero);
            if(true) {
                running = true;
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
            }
            break;

        case FIGHT:
            state_msg.data = "FIGHT";
            state_pub.publish(state_msg);
            ROS_INFO("Status: FIGHT");
            if(hp < low_hp_thresh && recover_cnt > 0) {
                ROS_INFO("Low HP!");
                --recover_cnt;
                next_status = RECOVER;
                break;
            } else if(build_hp < build_low_hp_thresh  && (use_build_blood)) {
                ROS_INFO("Low BUILD HP!");
                next_status = DEFEND;

                break;
            } else if(vitual_mode <= 1 && enemies.data.size() == 0 && ros::Time::now().toSec() - enemy_ts > exit_fight_time) {
                ROS_INFO("Enemy lost!");
                next_status = WANDER;
                break;
            } else {
                next_status = FIGHT;
            }
            {
                rotate_pub.publish(Five);
                // stop
                robot_msgs::Walk_CMD cmd;
                cmd.opt = 0;
                cmd.radium = 9999;
                cmd.pos.x = now_x;
                cmd.pos.y = now_y;
                aim_pub.publish(cmd);

                // robot_msgs::Orient_Info orient;
                
                // robot_msgs::Robot_Info enemy = enemies.data[0];
                // orient.id = enemy.id;
                // orient.ori.x = atan2(enemy.pos.y, enemy.pos.x);
                // decision2autoaim_pub.publish(orient);
            }
            break;

        case DEFEND:
            state_msg.data = "DEFEND";
            state_pub.publish(state_msg);
            ROS_INFO("Status: DEFEND");
            if(reach(home_x, home_y, reach_thresh)) {
                ROS_INFO("Reach home!");
                running = false;
                next_status = DEFEND;
                rotate_pub.publish(One);
            } else {
                next_status = DEFEND;
                rotate_pub.publish(rotate_when_move ? One : Zero);
                if(true) {
                    running = true;
                    robot_msgs::Walk_CMD cmd;
                    cmd.opt = 0;
                    cmd.radium = target_radium / 2;
                    cmd.pos.x = home_x;
                    cmd.pos.y = home_y;
                    aim_pub.publish(cmd);
                }
            }
            break;
        
        case RECOVER:
            state_msg.data = "RECOVER";
            state_pub.publish(state_msg);
            ROS_INFO("Status: RECOVER");
            if(reach(recover_x, recover_y, reach_thresh / 0.8 * 0.45)) {
                ROS_INFO("Reach recover!");
                running = false;
                // spin for 5 seconds
                for(int i = 0; ros::ok() && i < 6; ++i) {
                    rotate_pub.publish(Five);
                    ros::Duration(1).sleep();
                }
                next_status = WANDER;
            } else {
                next_status = RECOVER;
                rotate_pub.publish(rotate_when_move ? One : Zero);
                if(true) {
                    running = true;
                    robot_msgs::Walk_CMD cmd;
                    cmd.opt = 0;
                    cmd.radium = target_radium / 2 / 3 * 0.9;
                    cmd.pos.x = recover_x;
                    cmd.pos.y = recover_y;
                    aim_pub.publish(cmd);
                }
            }
            break;
        default:
            ROS_ERROR("Unknown status!");
            break;
        }
    }
    return 0;
}