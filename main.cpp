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


std::vector<Robot> Barrier; // 障碍物
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

private:
    vector<float> dwa_control(const CarState &carstate);
    vector<float> calc_dw(const CarState &carstate);
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
    float predict_time = 2.0;
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
vector<float> ROBOT::calc_dw(const CarState &carstate)
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
    dw = calc_dw(carstate);
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
            goal_cost = goal_cost_gain * abs(calc_goal_cost(trajectoryTmp));
            speed_cost = goal_cost_gain * (max_forward_v - trajectoryTmp.back().speed);
            obstacle_cost = obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp);
            distance_cost = 0.1 * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost;
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

// 计算障碍代价
float ROBOT::calc_obstacle_cost(const vector<CarState> &trajectory)
{
    //float obstacle_cost;
    // ofstream logFile;
    // logFile.open("calc_obstacle_cost.txt", ofstream::app);
    float distance;
    for (int i = 0; i < Barrier.size(); i ++) {
        for (int j = 0; j < trajectory.size(); j ++) {
            if(i != robot_ID)
            {
                // logFile << "i and robot_ID: " << i  << ", " << robot_ID << ", " << state.x << ", " << state.y << endl;
                distance = sqrt(pow(Barrier[i].x - trajectory[j].x, 2) + pow(Barrier[i].y - trajectory[j].y, 2));
                // logFile << "trajectory: " << Barrier[i].x  << ", " << Barrier[i].y << ", " << trajectory[j].x << ", " << trajectory[j].y << endl;
                if(distance <= 2 * radius)          
                    return 10000.0;
            }  
        }
    }
    // logFile << "=============================" << endl;
    // logFile.close();
    return 0;
}

int ROBOT::point_tracking(const float &x, const float &y)
{
    CarState det;
    det.x = x;
    det.y = y;
    float dx = x - state.x;
    float dy = y - state.y;

    float distance = sqrt(dy * dy + dx * dx);

    float target_angle = atan2(dy, dx);
    det.yaw = target_angle;
    det.speed = 0.0;
    det.angular_speed = 0;

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
        angleSpeed = 2 * theta_error + 0.5 * (theta_error - pre_error);
        vel = 2 * distance;
        // planning(det);
        move(vel, angleSpeed);
    }
    pre_error = theta_error;

    return -1;
}

//-------------------------------------------------设置----------------------------------------------------------
// 设置读取函数
bool readUntilOK()
{
    char line[1024];
    while (fgets(line, sizeof line, stdin))
    {
        if (line[0] == 'O' && line[1] == 'K')
        {
            return true;
        }
        // do something
    }
    return false;
}

// ====================================================购买逻辑================================================
// 判断工作台工作台1 2 3是否可以被买
int isWorkBenchCanBeBuy(WorkBench wb)
{
    if ((wb.left_time <= 50 && (wb.id == 1 || wb.id == 2 || wb.id == 3)) || ((wb.left_time <= 50 && wb.left_time != -1) || wb.product == 1))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// 判断工作台4 5 6是否可以被买
int isWorkCanBeBuyffs(WorkBench need_buy_wb, vector<WorkBench> work_bench_v7)
{
    // 先检查v7工作台是否需要4 5 6
    for (int i = 0; i < work_bench_v7.size(); i++)
    {
        WorkBench wb = work_bench_v7[i];
        // 对wb.id=4，5，6的情况进行判断
        if (need_buy_wb.id == 4 && ((need_buy_wb.left_time <= 50 && need_buy_wb.left_time != -1) || need_buy_wb.product == 1) && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 64 || wb.raw_material == 96) && wb.left_time == -1)
        {
            return 1;
        }
        else if (need_buy_wb.id == 5 && ((need_buy_wb.left_time <= 50 && need_buy_wb.left_time != -1) || need_buy_wb.product == 1) && (wb.raw_material == 0 || wb.raw_material == 64 || wb.raw_material == 16 || wb.raw_material == 80) && wb.left_time == -1)
        {
            return 1;
        }
        else if (need_buy_wb.id == 6 && ((need_buy_wb.left_time <= 50 && need_buy_wb.left_time != -1) || need_buy_wb.product == 1) && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 16 || wb.raw_material == 48) && wb.left_time == -1)
        {
            return 1;
        }
    }

    return 0;
}

// 机器人是否可以买
int ROBOT::isRobotProductNull()
{
    if (state.product_type == 0)
    {
        return 1; // 表示为空
    }
    else
    {
        return 0; // 表示为不空
    }
}

// 调用哪个机器人去哪里买
void ROBOT::robotToBuy(WorkBench wb)
{ // 参数为机器人对象和工作台对象
    if (state.work_id != wb.arr_idx && isRobotProductNull())
    {
        point_tracking(wb.x, wb.y);
        hasDestination = wb.id; // 去id这个类型的目的地
    }
    else
    {    
        if (state.product_type == wb.id)
        {
            hasDestination = 0; // 表示到达地方，需要下一个目的地
            flag = 2; // 该去卖状态
            return;
        }                   // 机器人已经到工作台附近且已经可以买了
        Buy();


    }
}

//================================================售卖逻辑==================================================
// 判断当前机器人携带的产品type作为材料的工作台是否需要
bool isWorkBenchNeedRobotProType(ROBOT robot, WorkBench wb)
{
    if (wb.id == 4)
    {
        if (robot.state.product_type == 1 && (wb.raw_material == 0 || wb.raw_material == 4) && wb.left_time == -1)
        {
            return true;
        }
        else if (robot.state.product_type == 2 && (wb.raw_material == 0 || wb.raw_material == 2) && wb.left_time == -1)
        {
            return true;
        }
    }
    else if (wb.id == 5)
    {
        if (robot.state.product_type == 1 && (wb.raw_material == 0 || wb.raw_material == 8) && wb.left_time == -1)
        {
            return true;
        }
        else if (robot.state.product_type == 3 && (wb.raw_material == 0 || wb.raw_material == 2) && wb.left_time == -1)
        {
            return true;
        }
    }
    else if (wb.id == 6)
    {
        if (robot.state.product_type == 2 && (wb.raw_material == 0 || wb.raw_material == 8) && wb.left_time == -1)
        {
            return true;
        }
        else if (robot.state.product_type == 3 && (wb.raw_material == 0 || wb.raw_material == 4) && wb.left_time == -1)
        {
            return true;
        }
    }
    else if (wb.id == 7)
    {
        if (robot.state.product_type == 4 && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 64 || wb.raw_material == 96) && wb.left_time == -1)
        {
            return true;
        }
        else if (robot.state.product_type == 5 && (wb.raw_material == 0 || wb.raw_material == 64 || wb.raw_material == 16 || wb.raw_material == 80) && wb.left_time == -1)
        {
            return true;
        }
        else if (robot.state.product_type == 6 && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 16 || wb.raw_material == 48) && wb.left_time == -1)
        {
            return true;
        }
    }

    return false;
}

// 机器人奔向对应的workbench进行出售
void ROBOT::robotToSell(WorkBench wb, int workbench_index)
{
    // 机器人已经携带物品1，可以运动到
    if (state.work_id != workbench_index && state.product_type != 0)
    {
        // 机器人向工作台跑去
        hasDestination = wb.id; // 去往id对应的目的地
        point_tracking(wb.x, wb.y);
    }
    else if (state.work_id == workbench_index)
    {
        hasDestination = 0; // 没有目的地
        Sell();
        // 机器人该去买状态
        Buy_pos = -1;
        sell_pos = -1;
        flag = 1;
    }
}

// =======================================================utils工具函数====================================================
float distance_between_workbench(WorkBench curr, WorkBench target)
{
    float dx = curr.x - target.x;
    float dy = curr.y - target.y;
    return sqrt(dy * dy + dx * dx);
}

float dis_wb_robot(Robot rb, WorkBench wb)
{
    float dx = rb.x - wb.x;
    float dy = rb.y - wb.y;
    return sqrt(dy * dy + dx * dx);
}

//-----------------------------------main-----------------------------
int main()
{
    readUntilOK();
    puts("OK");     // 初始化完毕
    fflush(stdout); // 清空标准输出中的缓存，防止输出到判题器中的数据出错
    ofstream logFile;

    // 定义日志输出文件
    logFile.open("log.txt");

    // 定义每一帧需要的变量
    int frameID;

    // 定义保存机器人移动类的数组
    ROBOT robot_array[4];
    
    // 定义当前总钱数
    int currMoney = 200000;
    // 定义工作台数量
    int K;

    // 创建对应的保存map的WORKBENCH数组
    vector<WORKBENCH> WB1;
    vector<WORKBENCH> WB2;
    vector<WORKBENCH> WB3;
    vector<WORKBENCH> WB4;
    vector<WORKBENCH> WB5;
    vector<WORKBENCH> WB6;
    vector<WORKBENCH> WB7;
    vector<WORKBENCH> WB8;
    vector<WORKBENCH> WB9;
    vector<WORKBENCH> WB2_map3;
    vector<WORKBENCH> WB3_map3;
    vector<WORKBENCH> WB6_map3;
    vector<WORKBENCH> WB9_map3;


    WORKBENCH WB;

    // 循环每帧得到判题器的输入并处理之后进行输出
    while (scanf("%d", &frameID) != EOF)
    {
        // unordered_map<WorkBench, int> work_bench_m;
        // create 7 vectors<>
        vector<WorkBench> work_bench_v1;
        vector<WorkBench> work_bench_v2;
        vector<WorkBench> work_bench_v3;
        vector<WorkBench> work_bench_v4;
        vector<WorkBench> work_bench_v5;
        vector<WorkBench> work_bench_v6;
        vector<WorkBench> work_bench_v7;
        vector<WorkBench> work_bench_v8;
        vector<WorkBench> work_bench_v9;

        vector<WorkBench> work_bench_v;
        vector<WorkBench> work_bench_v2_map3;
        vector<WorkBench> work_bench_v3_map3;
        vector<WorkBench> work_bench_v6_map3;
        vector<WorkBench> work_bench_v9_map3;
        Barrier.clear();

        scanf("%d", &currMoney);
        getchar();
        scanf("%d", &K);
        getchar();

        // 朝TXT文档中写入数据
        logFile << "==========================================[frame ID]: " << frameID << "=============================================\n"
                << "====================================================================================================================\n"
                << endl;

        logFile << "[currMoney]: " << currMoney << "\n";
        logFile << "[WorkBench number: K->]: " << K << "\n";

        logFile << "--------------------------------------------WorkBench----------------------------------------------"
                << "\n";

        int index = 0;
        // 遍历K个工作台数据
        while (index < K)
        {
            WorkBench wb;
            scanf("%d %f %f %d %d %d", &wb.id, &wb.x, &wb.y, &wb.left_time, &wb.raw_material, &wb.product);
            getchar();
            logFile << "[WorkBench]: "
                    << wb.id << ", "
                    << wb.x << ", "
                    << wb.y << ", "
                    << wb.left_time << ", "
                    << wb.raw_material << ", "
                    << wb.product << "\n";

            // 对不同的type id添加到不同的vector数组中
            if (wb.id == 1)
            {
                wb.arr_idx = index;
                work_bench_v1.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB1.push_back(WB);
                }
            }
            else if (wb.id == 2)
            {
                wb.arr_idx = index;
                work_bench_v2.push_back(wb);
                if(index == 7 || index == 16 || index == 37 )
                {
                    work_bench_v2_map3.push_back(wb);
                }
                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB2.push_back(WB);
                    if(index == 7 || index == 16 || index == 37 )
                    {
                        WB2_map3.push_back(WB);
                    }
                }
                
            }
            else if (wb.id == 3)
            {
                wb.arr_idx = index;
                work_bench_v3.push_back(wb);
                if( index ==28  || index == 39 )
                {
                    work_bench_v3_map3.push_back(wb);
                }
                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB3.push_back(WB);
                    if( index ==28  || index == 39 )
                    {
                        WB3_map3.push_back(WB);
                    }
                    
                }
            }
            else if (wb.id == 4)
            {
                wb.arr_idx = index;
                work_bench_v4.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB4.push_back(WB);
                }
            }
            else if (wb.id == 5)
            {
                wb.arr_idx = index;
                work_bench_v5.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB5.push_back(WB);
                }
            }
            else if (wb.id == 6)
            {
                wb.arr_idx = index;
                work_bench_v6.push_back(wb);
                if( index== 11 || index==12 ||index==13 ||index==14 ||index==20 || index == 22  || index==23 || index==25||index==29||index == 32 || index==33 ||index==34 )
                {
                    work_bench_v6_map3.push_back(wb);
                }

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB6.push_back(WB);
                    if( index== 11 || index==12 ||index==13 ||index==14 ||index==20 || index == 22  || index==23 || index==25||index==29||index == 32 || index==33 ||index==34)
                    {
                        
                        WB6_map3.push_back(WB);
                    }
                }

            }
            else if (wb.id == 7)
            {
                wb.arr_idx = index;
                work_bench_v7.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB7.push_back(WB);
                }
            }
            else if (wb.id == 8)
            {
                wb.arr_idx = index;
                work_bench_v8.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB8.push_back(WB);
                }
            }
            else if (wb.id == 9)
            {
                wb.arr_idx = index;
                work_bench_v9.push_back(wb);
                if(index == 24 )
                {
                    work_bench_v9_map3.push_back(wb);
                }
                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB9.push_back(WB);
                    if(index == 24 )
                    {
                        work_bench_v9_map3.push_back(wb);
                        WB9_map3.push_back(WB);
                    }
                }
                
            }
            work_bench_v.push_back(wb);
            index++;
        }

        logFile << endl;
        logFile << "-------------------------------------RoBot---------------------------------------------"
                << "\n";

        int ROBOTNUM = 4;
        for (int i = 0; i < ROBOTNUM; i++)
        {
            Robot rb;
            scanf("%d %d %f %f %f %f %f %f %f %f",
                  &rb.work_id,
                  &rb.product_type,
                  &rb.time_value,
                  &rb.collision_value,
                  &rb.angle_speed,
                  &rb.line_speed_x,
                  &rb.line_speed_y,
                  &rb.direction,
                  &rb.x,
                  &rb.y);
            // 根据每一帧的状态来更新机器人的状态
            robot_array[i].update_motion(i, rb);
            logFile << "[Robot WorkBenchId]: " << rb.work_id << ", " << rb.product_type << ", " << rb.time_value << ", " << rb.collision_value << "\n";
            logFile << "[Robot state]:" << rb.angle_speed << ", " << rb.line_speed_x << ", " << rb.line_speed_y << ", " << rb.x << ", " << rb.y << ", " << rb.direction << "\n"
                    << endl;
            getchar(); // 跳过换行符
            Barrier.push_back(rb);
        }

        // 读取最后一行OK
        char line[line_size];
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);
        
        WorkBench dst = work_bench_v[24]; // 唯一的九号台
        //只针对2，3，6，9进行购买和售卖
        // int work_bench_index_2[3]={7,16,37};
        // int work_bench_index_3[2]={28,39};
        // int work_bench_index_6[12]={11,12,13,14,20,22,23,25,29,32,33,34};
        // int work_bench_index_9[1] = {24};

       
        if(K==50)
        {   
            if(frameID==2)
            {
                // ----------------------------------------循环6数组--------------------------------------------
                // ----------------------------------------循环6数组--------------------------------------------
                
                for (int i = 0; i < WB6_map3.size(); i++)
                {
                    vector<int> tmp_v_2;
                    // key表示距离, value表示index，辅助排序
                    map<float, int> tmp_sort_map;
                    // ----------------------------------------遍历2数组--------------------------------------
                    for (int j = 0; j < WB2_map3.size(); j++)
                    {
                        if(tmp_sort_map.find(distance_between_workbench(WB6_map3[i].wb, WB2_map3[j].wb)) != tmp_sort_map.end())
                        {
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6_map3[i].wb, WB2_map3[j].wb)+0.01, j));
                        }
                        else{
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6_map3[i].wb, WB2_map3[j].wb), j));
                        } 
                    }

                    // 遍历map得到v1对应index的工作台对象
                    for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                    {
                        tmp_v_2.push_back(iter->second);
                    }
                    WB6_map3[i].need_material_map[2] = tmp_v_2;
                    // 下次遍历之前清空vector和map
                    tmp_sort_map.clear(); // 清除容器并最小化它的容量

                    // ----------------------------------------遍历3数组--------------------------------------
                    vector<int> tmp_v_3;
                    for (int j = 0; j < WB3_map3.size(); j++)
                    {
                        if(tmp_sort_map.find(distance_between_workbench(WB6_map3[i].wb, WB3_map3[j].wb)) != tmp_sort_map.end())
                        {
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6_map3[i].wb, WB3_map3[j].wb)+0.01, j));
                        }
                        else
                        {
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6_map3[i].wb, WB3_map3[j].wb), j));
                        }
                    }

                    // 遍历map得到v1对应index的工作台对象
                    for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                    {
                        tmp_v_3.push_back(iter->second);
                    }
                    WB6_map3[i].need_material_map[3] = tmp_v_3;
                    // 下次遍历之前清空vector和map
                    tmp_sort_map.clear(); // 清除容器并最小化它的容量
                }
               

            }

            if(frameID==2)
            {
                // ----------------------------------------循环9数组--------------------------------------------
                // ----------------------------------------循环9数组--------------------------------------------
                
                for (int i = 0; i < WB9_map3.size(); i++)
                {
                    vector<int> tmp_v_2;
                    // key表示距离, value表示index，辅助排序
                    map<float, int> tmp_sort_map;
                    // ----------------------------------------遍历6数组--------------------------------------
                    // for (int j = 0; j < WB6_map3.size(); j++)
                    // {
                    //     if(tmp_sort_map.find(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb)) != tmp_sort_map.end())
                    //     {
                    //         tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb)+0.01, j));
                    //     }
                    //     else{
                    //         tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb), j));
                    //     } 
                    // }

                    // 遍历map得到v1对应index的工作台对象
                    for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                    {
                        tmp_v_2.push_back(iter->second);
                        logFile << "WB9_map3[0].need_material_map[6]: index   "<< iter->second<<endl;
                    }
                    WB9_map3[i].need_material_map[6] = tmp_v_2;
                    
                    // 下次遍历之前清空vector和map
                    tmp_sort_map.clear(); // 清除容器并最小化它的容量
                    for (int j = 0; j < WB6_map3.size(); j++)
                    {
                        if(tmp_sort_map.find(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb)) != tmp_sort_map.end())
                        {
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb)+0.01, j));
                        }
                        else{
                            tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9_map3[i].wb, WB6_map3[j].wb), j));
                        } 
                    }
           
                }
               
            }
            int max_wb_num = 3;
            vector<int> target_buy_index;
            vector<int> target_sell_index;
            // logFile<< "work_bench_v2_map3.size():  "<<work_bench_v2_map3.size()<<endl;
            // logFile<< "work_bench_v3_map3.size():  "<<work_bench_v3_map3.size()<<endl;
            // logFile<< "work_bench_v6_map3.size():  "<<work_bench_v6_map3.size()<<endl;
            // logFile<< "work_bench_v9_map3.size():  "<<work_bench_v9_map3.size()<<endl;

            if (frameID > 5)
            {
                
                // 生产好工作台6号优先送6号
                for (int i = 0; i < work_bench_v6_map3.size(); i++)
                {
                    if (((work_bench_v6_map3[i].left_time <= 10) && (work_bench_v6_map3[i].left_time != -1)) || work_bench_v6_map3[i].product == 1)
                    {
                        target_buy_index.push_back(work_bench_v6_map3[i].arr_idx);
                        target_sell_index.push_back(work_bench_v9_map3[0].arr_idx);
                    }
                }
                
            // ===============================遍历工作台6需要那个材料===========================================================
            // 优先搬运缺一个的
                for (int j = 0; j < max_wb_num; j++)
                {
                    for (int i = 0; i < work_bench_v6_map3.size(); i++)
                    {
                        int wb_index;
                        // 找到所有3工作台中生产好的目的地保存下来 工作台6有2，缺3
                        if (( work_bench_v6_map3[i].raw_material == 4))
                        {
                            if (j < WB6_map3[i].need_material_map[3].size())
                            {
                                wb_index =WB6_map3[i].need_material_map[3][j];
                                if (work_bench_v3_map3[wb_index].product == 1 || (work_bench_v3_map3[wb_index].left_time <= 50 && work_bench_v3_map3[wb_index].left_time != -1))
                                {
                                    target_buy_index.push_back(work_bench_v3_map3[wb_index].arr_idx);
                                    target_sell_index.push_back(work_bench_v6_map3[i].arr_idx);
                                    // break;
                                }
                            }
                        }   
                        if ((work_bench_v6_map3[i].raw_material == 8))
                        {
                            if (j < WB6_map3[i].need_material_map[2].size())
                            {
                                wb_index = WB6_map3[i].need_material_map[2][j];
                                if (work_bench_v2_map3[wb_index].product == 1 || (work_bench_v2_map3[wb_index].left_time <= 50 && work_bench_v2_map3[wb_index].left_time != -1))
                                {
                                    target_buy_index.push_back(work_bench_v2_map3[wb_index].arr_idx);
                                    target_sell_index.push_back(work_bench_v6_map3[i].arr_idx);
                                    // break;
                                }
                            }
                        }        
                       
                        // 找到所有2工作台中生产好的目的地保存下来   工作台6有3，缺2
          

                    }
                    
                } 
                logFile << "==========================================2=============================================" << endl;
                // 6 号台缺两个
                for (int j = 0; j < max_wb_num; j++)
                {
                    for (int i = 0; i < work_bench_v6_map3.size(); i++)
                    {
                        int wb_index;
                        // 找到所有3工作台中生产好的目的地保存下来
                        if ((work_bench_v6_map3[i].raw_material == 0))
                        {
                            if (j < WB6_map3[i].need_material_map[3].size())
                            {
                                wb_index = WB6_map3[i].need_material_map[3][j];
                                if (work_bench_v3_map3[wb_index].product == 1 || (work_bench_v3_map3[wb_index].left_time <= 50 && work_bench_v3_map3[wb_index].left_time != -1))
                                {
                                    target_buy_index.push_back(work_bench_v3_map3[wb_index].arr_idx);
                                    target_sell_index.push_back(work_bench_v6_map3[i].arr_idx);
                                    // break;
                                }
                            }
                        }
                        // 找到所有2工作台中生产好的目的地保存下来
                        if ((work_bench_v6_map3[i].raw_material == 0))
                        {
                            if (j < WB6_map3[i].need_material_map[2].size())
                            {
                                wb_index = WB6_map3[i].need_material_map[2][j];
                                if (work_bench_v2_map3[wb_index].product == 1 || (work_bench_v2_map3[wb_index].left_time <= 50 && work_bench_v2_map3[wb_index].left_time != -1))
                                {
                                    target_buy_index.push_back(work_bench_v2_map3[wb_index].arr_idx);
                                    target_sell_index.push_back(work_bench_v6_map3[i].arr_idx);
                                    // break;
                                }
                            }
                        }


                    }
                    
                }
                float distance_wb_rb;
                int min_dis_wb_robot;
                int buy_sell_work_size = target_buy_index.size();


                logFile << "buy_sell_work_size: " << buy_sell_work_size << endl;
                for (int i = 0; i < buy_sell_work_size; i++)
                {
                    logFile << "target_buy_index[" << i << "]" << target_buy_index[i] << ", ";
                    logFile << "target_sell_index[" << i << "]" << target_sell_index[i] << endl;
                }




                for (int workidx = 0; workidx < buy_sell_work_size; workidx++)
                {
                    int robot_id = -1;
                    int i = 0;
                    for (int robotidx = 0; robotidx < 4; robotidx++)
                    {

                        if (robot_array[robotidx].hasDestination == 0 && (robot_array[robotidx].flag == 1 || robot_array[robotidx].flag == 0) && robot_array[robotidx].Buy_pos == -1 && robot_array[robotidx].sell_pos == -1)
                        {
                            // 得到最近的机器人
                            distance_wb_rb = dis_wb_robot(robot_array[robotidx].state, work_bench_v[target_buy_index[workidx]]);
                            if (i == 0)
                            {
                                robot_id = robotidx;
                                min_dis_wb_robot = distance_wb_rb;
                            }
                            if (min_dis_wb_robot > distance_wb_rb)
                            {
                                robot_id = robotidx;
                                min_dis_wb_robot = distance_wb_rb;
                            }
                            i++;
                        }
                        // logFile << "robot_array[" << robotidx << "]: target_buy_index:" << robot_array[robotidx].Buy_pos << ", ";
                        // logFile << "target_sell_index: " << robot_array[robotidx].sell_pos << endl;
                    }
                    
                    if (robot_id != -1)
                    {
                        // if (workidx == 0)
                        // {
                        //     robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                        //     robot_array[robot_id].sell_pos = target_sell_index[workidx];
                        // }
                        // if(work_bench_v[target_sell_index[workidx]].arr_idx == 24 && work_bench_v[target_buy_index[workidx]].left_time<=50 && robot_array[0].flag ==1 && target_buy_index[workidx] != robot_array[0].Buy_pos)
                        // {
                        //     robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                        //     robot_array[robot_id].sell_pos = target_sell_index[workidx];
                        //     logFile << "robot_array[" << robot_id << "]: Buy_pos:" << robot_array[robot_id].Buy_pos << ", ";
                        //     logFile << "sell_pos: " << robot_array[robot_id].sell_pos << endl;
                        //     logFile << "robot_id: " << robot_id << endl;
                        // }
                        // if(work_bench_v[target_sell_index[workidx]].arr_idx == 24 && work_bench_v[target_buy_index[workidx]].left_time<=50 && robot_array[1].flag ==1 && target_buy_index[workidx] != robot_array[1].Buy_pos)
                        // {
                        //     robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                        //     robot_array[robot_id].sell_pos = target_sell_index[workidx];
                        //     logFile << "robot_array[" << robot_id << "]: Buy_pos:" << robot_array[robot_id].Buy_pos << ", ";
                        //     logFile << "sell_pos: " << robot_array[robot_id].sell_pos << endl;
                        //     logFile << "robot_id: " << robot_id << endl;
                        //     continue;
                        // }
                        // if(work_bench_v[target_sell_index[workidx]].arr_idx == 24 && work_bench_v[target_buy_index[workidx]].left_time<=50 && robot_array[2].flag ==1 && target_buy_index[workidx] != robot_array[2].Buy_pos)
                        // {
                        //     robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                        //     robot_array[robot_id].sell_pos = target_sell_index[workidx];
                        //     logFile << "robot_array[" << robot_id << "]: Buy_pos:" << robot_array[robot_id].Buy_pos << ", ";
                        //     logFile << "sell_pos: " << robot_array[robot_id].sell_pos << endl;
                        //     logFile << "robot_id: " << robot_id << endl;
                        //     continue;
                        // }
                        // if(work_bench_v[target_sell_index[workidx]].arr_idx == 24 && work_bench_v[target_buy_index[workidx]].left_time<=50 && robot_array[3].flag ==1 && target_buy_index[workidx] != robot_array[3].Buy_pos)
                        // {
                        //     robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                        //     robot_array[robot_id].sell_pos = target_sell_index[workidx];
                        //     logFile << "robot_array[" << robot_id << "]: Buy_pos:" << robot_array[robot_id].Buy_pos << ", ";
                        //     logFile << "sell_pos: " << robot_array[robot_id].sell_pos << endl;
                        //     logFile << "robot_id: " << robot_id << endl;
                        //     continue;
                        // }
                        // if (((target_buy_index[workidx] == robot_array[0].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[0].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[0].sell_pos))
                        // {
                            
                        //     continue;
                        // }
                        // else if (((target_buy_index[workidx] == robot_array[1].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[1].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[1].sell_pos))
                        // {
                            
                        //     continue;
                        // }
                        // else if (((target_buy_index[workidx] == robot_array[2].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[2].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[2].sell_pos))
                        // {
                            
                        //     continue;
                        // }
                        // else if (((target_buy_index[workidx] == robot_array[3].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[3].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[3].sell_pos))
                        // {
                            
                        //     continue;
                        // }
                        if (((target_buy_index[workidx] == robot_array[0].Buy_pos) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[0].Buy_pos].id) && work_bench_v[target_buy_index[workidx]].id !=6) && (target_sell_index[workidx] == robot_array[0].sell_pos))
                        {
                            
                            continue;
                        }
                        else if (((target_buy_index[workidx] == robot_array[1].Buy_pos) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[1].Buy_pos].id)&& work_bench_v[target_buy_index[workidx]].id !=6) && (target_sell_index[workidx] == robot_array[1].sell_pos))
                        {
                            
                            continue;
                        }
                        else if (((target_buy_index[workidx] == robot_array[2].Buy_pos ) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[2].Buy_pos].id)&& work_bench_v[target_buy_index[workidx]].id !=6 ) && (target_sell_index[workidx] == robot_array[2].sell_pos))
                        {
                            
                            continue;
                        }
                        else if (((target_buy_index[workidx] == robot_array[3].Buy_pos) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[3].Buy_pos].id)&& work_bench_v[target_buy_index[workidx]].id !=6) && (target_sell_index[workidx] == robot_array[3].sell_pos))
                        {
                            
                            continue;
                        }
                        else if((target_buy_index[workidx] == robot_array[0].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3))
                        {
                            continue;
                        }
                        else if((target_buy_index[workidx] == robot_array[1].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3))
                        {
                            continue;
                        }
                        else if((target_buy_index[workidx] == robot_array[2].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3))
                        {
                            continue;
                        }
                        else if((target_buy_index[workidx] == robot_array[3].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3))
                        {
                            continue;
                        }
                        else
                        {   
                            robot_array[robot_id].Buy_pos = target_buy_index[workidx];
                            robot_array[robot_id].sell_pos = target_sell_index[workidx];
                            logFile << "robot_array[" << robot_id << "]: Buy_pos:" << robot_array[robot_id].Buy_pos << ", ";
                            logFile << "sell_pos: " << robot_array[robot_id].sell_pos << endl;
                            logFile << "robot_id: " << robot_id << endl;
                        }
                    }
                    // for (int robotidx = 0; robotidx < 4; robotidx++)
                    // {
                    //     logFile << "robot_array[" << robotidx << "]: target_buy_index:" << robot_array[robotidx].Buy_pos << ", ";
                    //     logFile << "target_sell_index: " << robot_array[robotidx].sell_pos << endl;
                    // }

                }
                 for (int i = 0; i < 4; i++)
                {
                    int buy_idx = robot_array[i].Buy_pos;
                    int sell_idx = robot_array[i].sell_pos;
                    logFile << "robot_array[" << i << "]: target_buy_index:" << buy_idx << ", ";
                    logFile << "target_sell_index: " << sell_idx << ", hasDestination: " << robot_array[i].hasDestination << ", flag: " << robot_array[i].flag << endl;
                
                    if ((robot_array[i].flag == 0 || robot_array[i].flag == 1))
                    {
                        robot_array[i].robotToBuy(work_bench_v[buy_idx]);
                        // logFile << "DW: " << robot_array[i].DW[0] << ", " << robot_array[i].DW[1] << ", " << robot_array[i].DW[2] << ", " << robot_array[i].DW[3] << endl;
                        // robot[0].point_tracking(3.25,48.25);
                        // logFile << "Barrier: " << Barrier[0].x << ", " << Barrier[0].y <<", " << Barrier[1].x << ", " << Barrier[1].y <<endl;
                        // logFile << "target_speed: " << robot_array[i].target_speed[0] << ", " << robot_array[i].target_speed[1] << endl; 
                        // logFile << "cost: " << robot_array[i].final_cost << ", " << robot_array[i].obstacle_cost << ", " << robot_array[i].goal_cost << ", " << robot_array[i].distance_cost << endl;
                    }

                    if (robot_array[i].flag == 2)
                    {

                        robot_array[i].robotToSell(work_bench_v[sell_idx], sell_idx);
                    }
                }
                // for (int j = 0; j < max_wb_num; j++)
                // {
                //     for (int i = 0; i < work_bench_v6_map3.size(); i++)
                //     {
                //         int wb_index;
                //         // 找到所有3工作台中还未生产但有缺的 缺3
                //         if ((work_bench_v6_map3[i].raw_material == 0 || work_bench_v6_map3[i].raw_material == 4 ))
                //         {
                //             if (j < WB6_map3[i].need_material_map[3].size())
                //             {
                //                 wb_index = WB6_map3[i].need_material_map[3][j];
                //                 if ((work_bench_v3_map3[wb_index].raw_material == 0))
                //                 {   

                //                     // target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                //                     // target_buy_index.push_back(work_bench_v1[WB4[wb_index].need_material_map[1][0]].arr_idx);
                //                     // target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                //                     // target_buy_index.push_back(work_bench_v2[WB4[wb_index].need_material_map[2][0]].arr_idx);
                //                     // break;
                //                 }
                //             }
                //         }
                //         // 找到所有5工作台中还未生产但有缺的
                //         if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 80))
                //         {
                //             if (j < WB7[i].need_material_map[5].size())
                //             {
                //                 wb_index = WB7[i].need_material_map[5][j];
                //                 if ((work_bench_v5[wb_index].raw_material == 0))
                //                 {
                //                     target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                //                     target_buy_index.push_back(work_bench_v1[WB5[wb_index].need_material_map[1][0]].arr_idx);
                //                     target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                //                     target_buy_index.push_back(work_bench_v3[WB5[wb_index].need_material_map[3][0]].arr_idx);
                //                     // break;
                //                 }
                //             }
                //         }
                        
                //     }

                // }



            }

        }
        //测试log
        // logFile << "WB6_map3.size(): " << WB6_map3.size() << endl;
        // logFile << "WB2_map3.size(): " << WB2_map3.size() << endl;
        // logFile << "WB6_map3[0].need_material_map.size(): " << WB6_map3[0].need_material_map.size() << endl;
        // logFile << "WB6_map3[0].need_material_map[2].size(): " <<  WB6_map3[0].need_material_map[2].size() << endl;


       


        // if(K==50 && frameID ==1){//设定好每个robot的工作台范围
        // //-------------------------Robot_0-------------------------
        //     // work_bench_buy_0={6,0};
        //     // work_bench_sell_0[5]={1,2,3,4,5};
        
        //     logFile<< " work_bench_v1_0: "<< work_bench_buy_0[0]<<endl;

        //     // work_bench_v1_0.push_back(work_bench_v[6]);
        //     // work_bench_v3_0.push_back(work_bench_v[0]);
        //     // int r0_wb5[5]={1,2,3,4,5};
        //     // for (int i = 0;i<5;i++){
        //     //     work_bench_v5_0.push_back(work_bench_v[i]);
        //     // }
        //     // logFile<< " work_bench_v1_0"<< work_bench_v1_0[0].arr_idx<<endl;
        // //-------------------------Robot_1-------------------------

        //     // work_bench_buy_1={15,31};
        //     // work_bench_sell_1={8,9,10,17,18,26,27,30};
        //     // work_bench_v1_1.push_back(work_bench_v[15]);
        //     // work_bench_v2_1.push_back(work_bench_v[31]);
        //     // int r1_wb4[8]={8,9,10,17,18,26,27,30};
        //     // for (int i = 0;i<8;i++){
        //     //     work_bench_v4_1.push_back(work_bench_v[i]);
        //     // }

        // //-------------------------Robot_2-------------------------
        //     // work_bench_buy_2={16,37,28,39};
        //     // work_bench_sell_2={11,12,13,14,20,22,23,25,29,32,33,34};

        //     // work_bench_v2_2.push_back(work_bench_v[16]);
        //     // work_bench_v2_2.push_back(work_bench_v[37]);
        //     // work_bench_v3_2.push_back(work_bench_v[28]);
        //     // work_bench_v3_2.push_back(work_bench_v[39]);
        //     // int r2_wb6[12]={11,12,13,14,20,22,23,25,29,32,33,34};
        //     // for (int i = 0;i<12;i++){
        //     //     work_bench_v6_2.push_back(work_bench_v[i]);
        //     // }

        // //-------------------------Robot_3-------------------------

        //     // work_bench_buy_3={36,49,42,43};
        //     // work_bench_sell_3={41,44,45,47,48};
            
        // }
        // if(frameID >=1 && K==50 && frameID<139){
        //     if(robot_array[0].flag==0 || robot_array[0].flag==1)
        //     {   
        //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[0]]);
        //         logFile<<"购买成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=139 && K==50 && frameID<294){
        //     if(robot_array[0].flag==2){
        //         robot_array[0].robotToSell(work_bench_v[work_bench_sell_0[3]],work_bench_v[work_bench_sell_0[3]].arr_idx);
        //         logFile<<"出售成功 "<<endl;
        //     }
        // }
        // if(frameID >=294 && K==50 && frameID<389){
        //     if(robot_array[0].flag==0 || robot_array[0].flag==1)
        //     {   
        //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[0]]);
        //         logFile<<"二次购买成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=389 && K==50 && frameID<518){
        //     if(robot_array[0].flag==2)
        //     {   
        //         robot_array[0].robotToSell(work_bench_v[work_bench_sell_0[1]],work_bench_v[work_bench_sell_0[1]].arr_idx);
        //         logFile<<"二次出售成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=518 && K==50 && frameID<587 ){
        //     if(robot_array[0].flag==0 || robot_array[0].flag==1)
        //     {   
        //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[1]]);
        //         logFile<<"三次购买成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=587 && K==50 && frameID<720){
        //     if(robot_array[0].flag==2)
        //     {   
        //         robot_array[0].robotToSell(work_bench_v[work_bench_sell_0[1]],work_bench_v[work_bench_sell_0[1]].arr_idx);
        //         logFile<<"三次出售成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=720 && K==50 && frameID<797 ){
        //     if(robot_array[0].flag==0 || robot_array[0].flag==1)
        //     {   
        //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[1]]);
        //         logFile<<"四次购买成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=797 && K==50 && frameID<943){
        //     if(robot_array[0].flag==2)
        //     {   
        //         robot_array[0].robotToSell(work_bench_v[work_bench_sell_0[3]],work_bench_v[work_bench_sell_0[3]].arr_idx);
        //         logFile<<"四次出售成功 frameID"<<endl;
        //     }
        // }
        //  if(frameID >=943 && K==50 && frameID<1013 ){
        //     if(robot_array[0].flag==0 || robot_array[0].flag==1)
        //     {   
        //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[0]]);
        //         logFile<<"五次购买成功 frameID"<<endl;
        //     }
        // }
        // if(frameID >=1013 && K==50 && frameID<1135){
        //     if(robot_array[0].flag==2)
        //     {   
        //         robot_array[0].robotToSell(work_bench_v[work_bench_sell_0[4]],work_bench_v[work_bench_sell_0[4]].arr_idx);
        //         logFile<<"五次出售成功 frameID"<<endl;
        //     }
        // }
        // if(work_bench_v[work_bench_sell_0[3]].product==1){
        //     logFile<< "FrameID 第四个五号台产品生成"<<frameID<<endl;//1442生成
        // }
        // if(work_bench_v[work_bench_sell_0[1]].product==1){
        //     logFile<< "FrameID 第2个五号台产品生成"<<frameID<<endl;//1219生成
        // }


        // robot_array[0].robotToBuy(work_bench_v[6]);
        //单独指派 robot0：
        // if(frameID > 0 && K==50)
        // {
            // robot_array[0].robotToBuy(work_bench_v1_0[0]);
        // if(robot_array[0].flag==0 || robot_array[0].flag==1)
        // {   
        //     int idx = work_bench_buy_0[0];
        //     // logFile<< "idx: "<< idx<<endl;
        //     // logFile<<work_bench_v[6].arr_idx<<endl;
        //     robot_array[0].point_tracking(work_bench_v[idx].x,work_bench_v[idx].y);
            // for(int i =0;i< work_bench_buy_0.size();i++)
            // {
            //     if((work_bench_v[work_bench_buy_0[i]].left_time<=50 &&  (work_bench_v[work_bench_buy_0[i]].left_time != -1)) || work_bench_v[work_bench_buy_0[i]].product == 1)
            //     {
            //         robot_array[0].robotToBuy(work_bench_v[work_bench_buy_0[i]]);
            //         break;
            //     }
            // }

        // }




        

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);

        logFile << endl;
    }
    logFile.close();
    return 0;
}
