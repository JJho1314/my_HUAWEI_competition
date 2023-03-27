#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <stack>
#include <algorithm>
#include <unordered_map>
#include <map>

using namespace std;
#define PI 3.1415926
#define line_size 1024

// 用于坐标计算
struct Point
{
    double x_;
    double y_;
    Point(double x, double y) : x_(x), y_(y) {}
    Point() : x_(0), y_(0) {}
};

typedef Point LinearVelocity;
typedef Point Position;
typedef Point Vector;

// 由坐标计算出向量
Vector operator-(const Point &a, const Point &b)
{
    return Vector(a.x_ - b.x_, a.y_ - b.y_);
}

// 向量放缩
Vector operator*(const Vector &a, const double &b)
{
    return Vector(a.x_ * b, a.y_ * b);
}

// 点加向量
Position operator+(const Point &a, const Vector &b)
{
    return Position(a.x_ + b.x_, a.y_ + b.y_);
}

//---------------------------------struct---------------------------------
// 定义判题器返回数据工作台结构体
typedef struct
{
    int id;           // 工作台类型
    float x;          // 工作台x坐标
    float y;          // 工作台y坐标
    int left_time;    // 剩余生产时间
    int raw_material; // 原材料格状态，二进制位表描述，48(110000) => 拥有物品4, 5
    int product;      // 产品格状态,0=>无，1=>有
    int arr_idx;      // 工作台在数组中的顺序
} WorkBench;

class WORKBENCH
{
public:
    // key对应当前工作台需要的材料工作台类型，value对应的是材料工作台vector数组，其中存放的是其在自身类别数组中的index
    unordered_map<int, vector<int>> need_material_map;
    WorkBench wb;
};

// 定义判题器返回数据机器人结构体
typedef struct
{
    int work_id;           // 机器人目前所处工作台ID
    int product_type;      // 机器人携带物品类型
    float time_value;      // 时间价值系数
    float collision_value; // 碰撞价值系数
    float angle_speed;     // 角速度
    float line_speed_x;    // x轴线速度
    float line_speed_y;    // y轴线速度
    float direction;       // 机器人当前朝向
    float x;               // 机器人当前x坐标
    float y;               // 机器人当前y坐标
} Robot;

//---------------------------------utils------------------------------------------------------------
//---------------------------------struct---------------------------------
struct CarState
{
    float x;
    float y;
    float yaw;
    float speed;
    float angular_speed;
    CarState(){};
    CarState(float x_, float y_, float yaw_, float speed_, float angular_speed_):
        x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_)
    {}
};

extern vector<Robot> Barrier; // 障碍物
//---------------------------------------------------------------------------------------------------
// 定义的机器人移动类
class ROBOT
{
public:
    // ROBOT();
    void update_motion(int robot_id, Robot robot);
    int point_tracking(const float &x, const float &y);
    void move(float lineSpeed, float angleSpeed);
    void Buy();
    void Sell();
    void Destroy();

    int isRobotProductNull();
    void robotToBuy(WorkBench wb);
    void robotToSell(WorkBench wb, int workbench_index);
    void planning(CarState destination);
    int calc_obstacle_dis();

    Robot state;
    int robot_ID;
    float distance;
    int Buy_pos = -1;
    int sell_pos = -1;

    int flag = 0;           // 0表示机器人空状态，1表示机器人该去买状态，2机器人该去卖状态
    int hasDestination = 0; // 0表示没有目的地, 1表示有目的地
    int curr_idx = -1;      // 表示记录当前小车去往的工作台index

    CarState destinationState;
    vector<float> DW;
    vector<float> target_speed;
    float final_cost;
    float goal_cost;
    float speed_cost = 0;
    float obstacle_cost = 0;
    float distance_cost = 0;
    float Distance;
    vector<CarState> TrajectoryTmp;
    bool has_temp_destination_;
    Position temp_destination_;

private:
    vector<float> dwa_control(const CarState &carstate);
    vector<float> calc_dw();
    vector<float> calc_best_speed(const CarState &carstate, const vector<float> &dw);
    void predict_trajectory(const CarState &carstate, const float &speed, const float &angular_speed, vector<CarState> &trajectory);
    CarState motion_model(const CarState &carstate, const float &speed, const float &angular_speed);
    float calc_goal_cost(const vector<CarState> &trajectory);
    float calc_obstacle_cost(const vector<CarState> &trajectory);   
    
    vector<vector<CarState>> trajectory;
    float v_resolution = 0.01;     // 速度采样分辨率
    float yaw_rate_resolution = 0.1 * PI;
    float dt = 0.02;                //运动学模型预测时间
    float max_accel = 20.0;
    float predict_time = 3.0;
    float goal_cost_gain = 0.2;
    float speed_cost_gain = 2.0;
    float obstacle_cost_gain = 1.0;
    
    float pre_error = 0;
    float radius = 0.45;
    float Radius = 0.53;
    float max_forward_v = 6.0; // 最大前进速度
    float max_back_v = -2.0;    //
    float max_angleSpeed = PI;
    float max_angular_speed_rate = PI;
    float angleSpeed;
    float vel;
};

// 实现机器人移动类的内部函数
void ROBOT::update_motion(int robot_id, Robot robot)
{
    robot_ID = robot_id;
    state = robot;
}

void ROBOT::move(float lineSpeed, float angleSpeed)
{
    printf("forward %d %f\n", robot_ID, lineSpeed);
    printf("rotate %d %f\n", robot_ID, angleSpeed);
}

void ROBOT::Buy()
{
    printf("buy %d\n", robot_ID);
}

void ROBOT::Sell()
{
    printf("sell %d\n", robot_ID);
}

void ROBOT::Destroy()
{
    printf("destroy %d\n", robot_ID);
}

//路径规划
void ROBOT::planning(CarState destination)
{
    CarState currentState(state.x, state.y, state.direction, sqrt(pow(state.line_speed_x,2) + pow(state.line_speed_y,2)), state.angle_speed);
    destinationState = destination;
    vector<CarState> currentTrajectory;

    vector<float> speed(2);     //v[0]为速度, v[1]角速度
    speed = dwa_control(currentState);  
    target_speed = speed;    
    move(speed[0],speed[1]);
    // cout << "speed:" << speed[0] << ", " << speed[1] << endl;s
    currentTrajectory.clear();
    // aaa = false;
    predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
    // aaa = false;
    trajectory.push_back(currentTrajectory);
    currentState = currentTrajectory.back();
    //判断是否到达终点
    if(pow(currentState.x - destinationState.x, 2) + pow(currentState.y - destinationState.y, 2) <= 0.1)
    {
        // cout << "Done" << endl;
        return;
    }
}

// 计算动态窗口
vector<float> ROBOT::calc_dw()
{
    // 机器人速度属性限制的动态窗口
    vector<float> dw_robot_state{max_back_v, max_forward_v, -max_angleSpeed, max_angleSpeed};
    // 机器人模型限制的动态窗口
    vector<float> dw_robot_model(4);
    dw_robot_model[0] = 0;
    dw_robot_model[1] = sqrt(pow(state.line_speed_x,2)+pow(state.line_speed_y,2)) + max_accel * 0.02;
    dw_robot_model[2] = state.angle_speed - max_angular_speed_rate * 0.02;
    dw_robot_model[3] = state.angle_speed + max_angular_speed_rate * 0.02;
    vector<float> dw{max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3])};
    return dw;
}

// 动态窗口法
vector<float> ROBOT::dwa_control(const CarState &carstate)
{
    vector<float> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw();
    DW = dw;
    //计算最佳（v, w）
    vector<float> v_w(2);
    v_w = calc_best_speed(carstate, dw);
    return v_w;
}

//在dw中计算最佳速度和角速度
vector<float> ROBOT::calc_best_speed(const CarState &carstate, const vector<float> &dw)
{
    // ofstream logFile;
    // logFile.open("calc_best_speed.txt", ofstream::app);
    vector<float> best_speed{0, 0};
    vector<CarState> trajectoryTmp;
    float min_cost = 10000;

    for(float i = dw[0]; i < dw[1]; i += 0.01)
    {
        for (float j = dw[2]; j < dw[3]; j +=  PI/100)
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            // for(int m = 0; m < trajectoryTmp.size(); m++)
            // {
            //     logFile <<trajectoryTmp[m].x << ", " << trajectoryTmp[m].y << endl;
            // }

            //计算代价
            goal_cost = goal_cost_gain * calc_goal_cost(trajectoryTmp);
            speed_cost = speed_cost_gain * (max_forward_v - trajectoryTmp.back().speed);
            obstacle_cost = obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp);
            distance_cost = 0.1 * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));
            final_cost =  speed_cost + obstacle_cost + distance_cost + goal_cost;
            // logFile << "goal_cost: " << goal_cost << ", " << trajectoryTmp.back().yaw << endl;
            if(final_cost < min_cost)
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
            }
            if(best_speed[0] < 0.001 && carstate.speed < 0.001)
                best_speed[1] = -max_angular_speed_rate;
        }
    }
    // logFile << "=========================================================================" << endl;
    // logFile.close();
    // cout << "best_speed:" << best_speed[0] << ",   " << best_speed[1] << endl;
    return best_speed;
}
// 在一段时间内预测轨迹
void ROBOT::predict_trajectory(const CarState &carstate, const float &speed, const float &angular_speed, vector<CarState> &trajectory)
{
    // ofstream logFile;
    // logFile.open("predict_trajectory.txt", ofstream::app);
    float time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while(time < predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        trajectory.push_back(nextState);
        // logFile << "trajectory: " << nextState.x << ", " << nextState.y << ", " << nextState.yaw << ", " << nextState.speed << ", " << nextState.angular_speed<< endl;
        time += 0.2;
    }
    // logFile << "=============================" << endl;
    // logFile.close();
}
//根据动力学模型计算下一时刻状态
CarState ROBOT::motion_model(const CarState &carstate, const float &speed, const float &angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + speed * cos(carstate.yaw) * 0.2;
    nextState.y = carstate.y + speed * sin(carstate.yaw) * 0.2;
    nextState.yaw = carstate.yaw + angular_speed * 0.2;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    return nextState;
}
// 计算方位角代价
float ROBOT::calc_goal_cost(const vector<CarState> &trajectory)
{
    float target_angle = atan2(destinationState.y - trajectory.back().y, destinationState.x - trajectory.back().x);
    float theta_error = target_angle - trajectory.back().yaw;
    if (target_angle<-PI/2 && state.direction>PI/2)
    {
        theta_error = 2*PI + target_angle - state.direction;
    }
    else if(target_angle>PI/2 && state.direction<-PI/2)
    {
        theta_error =  target_angle - state.direction - 2*PI;
    }
    else
    {
        theta_error = target_angle - state.direction;
    }
    goal_cost = abs(theta_error);

    if(goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}

int ROBOT::calc_obstacle_dis()
{
    float distance;
    for (int i = 0; i < Barrier.size(); i ++) {
        if(i != robot_ID)
        {
            // logFile << "i and robot_ID: " << i  << ", " << robot_ID << ", " << state.x << ", " << state.y << endl;
            distance = sqrt(pow(Barrier[i].x - state.x, 2) + pow(Barrier[i].y - state.y, 2));
            // logFile << "trajectory: " << Barrier[i].x  << ", " << Barrier[i].y << ", " << trajectory[j].x << ", " << trajectory[j].y << endl;
            if(distance <= 4 * Radius)          
                return 1;
        }  
    }
    return 0;
}

// 计算障碍代价
float ROBOT::calc_obstacle_cost(const vector<CarState> &trajectory)
{

    float distance;
    for (int i = 0; i < Barrier.size(); i ++) {
        for (int j = 0; j < trajectory.size(); j ++) {
            if(i != robot_ID)
            {
                // logFile << "i and robot_ID: " << i  << ", " << robot_ID << ", " << state.x << ", " << state.y << endl;
                distance = sqrt(pow(Barrier[i].x - trajectory[j].x, 2) + pow(Barrier[i].y - trajectory[j].y, 2));
                // logFile << "trajectory: " << Barrier[i].x  << ", " << Barrier[i].y << ", " << trajectory[j].x << ", " << trajectory[j].y << endl;
                if(distance <= 3 * radius)          
                    return 10000.0;
            }  
        }
    }
    return 0;
}


int ROBOT::point_tracking(const float &x, const float &y)
{
    
    destinationState.x = x;
    destinationState.y = y;
    float dx = x - state.x;
    float dy = y - state.y;
    vector<float> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw();
    
    vector<float> speed;

    float distance = sqrt(dy * dy + dx * dx);
    float target_angle = atan2(dy, dx);
    
    
    float theta_error;
    if (target_angle <= -PI / 2 && state.direction > PI / 2)
    {
        theta_error = 2 * PI + target_angle - state.direction;
    }
    else if (target_angle >= PI / 2 && state.direction < -PI / 2)
    {
        theta_error = target_angle - state.direction - 2 * PI;
    }
    else{
        theta_error = target_angle - state.direction;
    }

    if (distance <= 0.1)
    {
        move(0.0, 0.0);
        return 0;
    }
    
    if (abs(theta_error) >= PI / 1.5)
    {
        angleSpeed = 5 * abs(theta_error) + 0.5 * (abs(theta_error) - pre_error);
        move(0, angleSpeed);
        return -1;
    }

    if (distance > 0.1)
    {
        // if(!calc_obstacle_cost(trajectoryTmp))
        // {

            angleSpeed = 5 * theta_error + 1 * (theta_error - pre_error);
            vel = 2 * distance;
        // }
        // else{
        //     speed = dwa_control(currentState);
        //     vel = speed[0];
        //     angleSpeed = speed[1];
        // }
        move(vel, angleSpeed);
    }
    pre_error = theta_error;

    return -1;
}

// 点积
double Dot(const Vector &a, const Vector &b)
{
    return a.x_ * b.x_ + a.y_ * b.y_;
}

// 向量长度
double Length(const Vector &a)
{
    return sqrt(Dot(a, a));
}

// 获取单位向量
Vector calc_Vector(const Vector &a)
{
    double t = 1 / Length(a);
    return a * t;
}

// 判断是否会相撞
bool calc_Collide(const Position &pos1, const Position &des1, const Position &pos2, const Position &des2)
{
    Vector direction1 = des1 - pos1;
    Vector direction2 = des2 - pos2;

    // 夹角为锐角，直接跳过
    if (Dot(direction1, direction2) > 0) return false;

    double len1 = Length(direction1) - 0.4;
    double len2 = Length(direction2) - 0.4;

    double len = 0;
    for (; len < 5; len += 0.5)
    {
        Position tpos1 = pos1 + calc_Vector(direction1) * len;
        Position tpos2 = pos2 + calc_Vector(direction2) * len;

        if (len > len1 || len > len2)
            return false;

        if (Length(tpos1 - tpos2) < 0.9)
            return true;
    }

    return false;
}

// 计算避障用的临时坐标点
void calc_TempDes(const Position &pos1, const Position &pos2, Position &temp_des1, Position &temp_des2) {
    Position mid_point = Position{(pos1.x_ + pos2.x_) / 2, (pos1.y_ + pos2.y_) / 2};
    Vector direction = pos2 - pos1;
    Vector vertical_direction = Vector{-direction.y_, direction.x_};

    double len = Length(direction);

    // 目前是向两侧偏移单位向量长度， 可以修改， 例如乘以1.5倍
    temp_des1 = mid_point + calc_Vector(vertical_direction) * 1.0;
    temp_des2 = mid_point - calc_Vector(vertical_direction) * 1.0; 
}



#endif