#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <unordered_map>
using namespace std;
#define PI 3.1415926
#define line_size 1024

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
//---------------------------------struct---------------------------------
// 定义判题器返回数据工作台结构体
typedef struct
{
    int id;           // 工作台id
    float x;          // 工作台x坐标
    float y;          // 工作台y坐标
    int left_time;    // 剩余生产时间
    int raw_material; // 原材料格状态，二进制位表描述，110000=>拥有物品4, 5
    int product;      // 产品格状态,0=>无，1=>有
} WorkBench;

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

std::vector<Robot> Barrier; // 障碍物

//---------------------------------utils-------------------------------
class ROBOT
{
    public:
        // ROBOT();
        void update_motion(int robot_id, Robot robot);
        void planning(CarState destination);
        int point_tracking(float x, float y);
        void move(float lineSpeed, float angleSpeed);
        void Buy();
        void Sell();
        void Destroy();
        Robot state;
        int robot_ID;
        CarState destinationState;
        vector<float> DW;
        vector<float> target_speed;
        float final_cost;
        float goal_cost;
        float speed_cost = 0;
        float obstacle_cost = 0;
        float distance_cost = 0;

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
        float yaw_rate_resolution = 0.1 * PI / 180;
        float dt = 0.02;                //运动学模型预测时间
        float max_accel = 20.0;
        float predict_time = 2.0;
        float goal_cost_gain = 0.2;
        float speed_cost_gain = 1.0;
        float obstacle_cost_gain = 1.0;
        float pre_error = 0;
        float radius = 0.45;
        float Radius = 0.53;
        float max_forward_v = 6.0; //最大前进速度
        float max_back_v = -2.0; //
        float max_angleSpeed = PI;
        float max_angular_speed_rate = PI;
        float angleSpeed;
        float vel;
};

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

int ROBOT::point_tracking(float x, float y)
{
    float dx = x - state.x;
    float dy = y - state.y;

    float distance = sqrt(dy * dy + dx * dx);

    destinationState.x = x;
    destinationState.y = y;

    float target_angle = atan2(dy , dx);

    float theta_error;
    if (target_angle<0 && state.direction>0)
    {
        theta_error = 360 + target_angle - state.direction;
    }
    else if(target_angle>0 && state.direction<0)
    {
        theta_error =  target_angle - state.direction - 360;
    }
    else
    {
        theta_error = target_angle - state.direction;
    }
    
    if (distance <= 0.1)
    {
        move(0.0, 0.0);
        return 0;
    }

    if(abs(theta_error) >= PI/10)
    {
        angleSpeed = 2 * theta_error + 0.5 * (theta_error - pre_error);
        move(0 , angleSpeed);
        return -1;
    }

    if (distance > 0.1)
    {
        angleSpeed = 2 * theta_error + 0.5 * (theta_error - pre_error);
        vel = 2 * distance;
        move(vel , angleSpeed);
    }
    pre_error = theta_error;

    return -1;
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
    if(pow(currentState.x - destinationState.x, 2) + pow(currentState.y - destinationState.y, 2) <= radius * radius)
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
    dw_robot_model[2] = angleSpeed - max_angular_speed_rate * 0.02;
    dw_robot_model[3] = angleSpeed + max_angular_speed_rate * 0.02;
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
    vector<float> best_speed{0, 0};
    vector<CarState> trajectoryTmp;
    float min_cost = 10000;

    for(float i = dw[0]; i < dw[1]; i += 0.01)
    {
        for (float j = dw[2]; j < dw[3]; j += 0.1 * PI / 180)
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //计算代价
            goal_cost = goal_cost_gain * calc_goal_cost(trajectoryTmp);
            speed_cost = goal_cost_gain * (max_forward_v - trajectoryTmp.back().speed);
            obstacle_cost = obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp);
            distance_cost = 0.1 * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost;

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
    //cout << "best_speed:" << best_speed[0] << ",   " << best_speed[1] << endl;
    return best_speed;
}
// 在一段时间内预测轨迹
void ROBOT::predict_trajectory(const CarState &carstate, const float &speed, const float &angular_speed, vector<CarState> &trajectory)
{
    float time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while(time < predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        trajectory.push_back(nextState);
        time += 0.2;
    }
}
//根据动力学模型计算下一时刻状态
CarState ROBOT::motion_model(const CarState &carstate, const float &speed, const float &angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + state.line_speed_x * 0.02;
    nextState.y = carstate.y + state.line_speed_y * 0.02;
    nextState.yaw = carstate.yaw + angular_speed * 0.2;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    return nextState;
}
// 计算方位角代价
float ROBOT::calc_goal_cost(const vector<CarState> &trajectory)
{
    float error_yaw = atan2(destinationState.y - trajectory.back().y, destinationState.x - trajectory.back().x);
    float goal_cost = error_yaw - trajectory.back().yaw;

    goal_cost = atan2(sin(goal_cost), cos(goal_cost));

    if(goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}

// 计算障碍代价
float ROBOT::calc_obstacle_cost(const vector<CarState> &trajectory)
{
    //float obstacle_cost;
    float distance;
    for (int i = 0; i < Barrier.size(); i ++) {
        for (int j = 0; j < trajectory.size(); j ++) {
            if(i != robot_ID)
                distance = sqrt(pow(Barrier[i].x - trajectory[j].x, 2) + pow(Barrier[i].y - trajectory[j].y, 2));
            if(distance <= 2 * Radius)
                return 10000.0;
        }
    }
    return 0;
}

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

//-----------------------------------main-----------------------------
int main()
{
    readUntilOK();
    puts("OK");     // 初始化完毕
    fflush(stdout); // 清空标准输出中的缓存，防止输出到判题器中的数据出错
    ofstream logFile;
    logFile.open("log.txt");
    // 定义每一帧需要的变量
    int frameID;
    ROBOT robot[4];

    while (scanf("%d", &frameID) != EOF)
    {
        int currMoney = 200000;
        int K = 10;
        vector<WorkBench> work_bench_v;
        // vector<Robot> robot_v;

        scanf("%d", &currMoney);
        getchar();
        scanf("%d", &K);
        getchar();

        // 朝TXT文档中写入数据
        logFile << "[frame ID]:" << frameID << "\n" << endl;

        // 遍历K个工作台数据
        for (int i =0; i < K; i++)
        {
            WorkBench wb;
            scanf("%d %f %f %d %d %d", &wb.id, &wb.x, &wb.y, &wb.left_time, &wb.raw_material, &wb.product);
            getchar();
            work_bench_v.push_back(wb);

        }

        int ROBOTNUM = 4;
        for (int i=0; i < ROBOTNUM; i++)
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
            robot[i].update_motion(i, rb);
            logFile << "[Robot ID]:" << i << ", " << rb.product_type << ", " << rb.time_value << ", " << rb.collision_value << "\n" << endl;
            logFile << "[Robot state]:" << rb.angle_speed << ", " << rb.line_speed_x << ", " << rb.line_speed_y << ", " << robot[i].state.x << ", " << robot[i].state.y << ", " << robot[i].state.direction <<"\n" << endl;
            getchar();
            Barrier.push_back(rb);
            // robot_v.push_back(rb);
        }

        char line[line_size]; // 读取最后一行OK
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);
        CarState det;
        det.x = 0;
        det.y = 0;
        float angleSpeed;
        
        float target_angle = atan2(-robot[0].state.y , -robot[0].state.x);

        // float theta_error;
        // if (target_angle<0 && robot[0].state.direction>0)
        // {
        //     theta_error = 2*PI + target_angle - robot[0].state.direction;
        // }
        // else if(target_angle>0 && robot[0].state.direction<0)
        // {
        //     theta_error =  target_angle - robot[0].state.direction - 2*PI;
        // }
        // else
        // {
        //     theta_error = target_angle - robot[0].state.direction;
        // }

        // logFile << "theta_error: " << theta_error << endl;
        
        // if(abs(theta_error) > PI/6)
        // {
        //     angleSpeed = 2 * theta_error;
        //     robot[0].move(0 , angleSpeed);
        // }
        // else
        // {
        //     robot[0].planning(det);
        // }

        // logFile << "DW: " << robot[0].DW[0] << ", " << robot[0].DW[1] << ", " << robot[0].DW[2] << ", " << robot[0].DW[3] << endl;
        robot[0].point_tracking(3.25,48.25);
        // logFile << "target_speed: " << robot[0].target_speed[0] << ", " << robot[0].target_speed[1] << endl; 
        // logFile << "cost: " << robot[0].final_cost << ", " << robot[0].obstacle_cost << ", " << robot[0].goal_cost << ", " << robot[0].distance_cost << endl;
        logFile << "destinate_x: " << robot[0].destinationState.x << " destinate_y: " << robot[0].destinationState.y << endl;
        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);
    }
    logFile.close();
    return 0;
}
