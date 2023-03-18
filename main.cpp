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

//---------------------------------struct---------------------------------
// 定义判题器返回数据工作台结构体
typedef struct
{
    int id;           // 工作台id
    float x;          // 工作台x坐标
    float y;          // 工作台y坐标
    int left_time;    // 剩余生产时间
    int raw_material; // 原材料格状态，二进制位表描述，48(110000) => 拥有物品4, 5
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

//---------------------------------utils-------------------------------
// 定义的机器人移动类
class ROBOT
{
public:
    // ROBOT();
    void update_motion(int robot_id, Robot robot);
    int point_tracking(float x, float y);
    void move(float lineSpeed, float angleSpeed);
    void Buy();
    void Sell();
    void Destroy();
    Robot state;
    int robot_ID;
    float distance;

private:
    float pre_error = 0;
    float radius = 0.45;
    float Radius = 0.53;
    float max_forward_v = 6.0; // 最大前进速度
    float max_back_v = 2.0;    //
    float max_angleSpeed = PI;
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

int ROBOT::point_tracking(float x, float y)
{
    float dx = x - state.x;
    float dy = y - state.y;

    float distance = sqrt(dy * dy + dx * dx);

    float target_angle = atan2(dy, dx);

    float theta_error;
    if (target_angle < 0 && state.direction > 0)
    {
        theta_error = 360 + target_angle - state.direction;
    }
    else if (target_angle > 0 && state.direction < 0)
    {
        theta_error = target_angle - state.direction - 360;
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

    if (abs(theta_error) >= PI / 10)
    {
        angleSpeed = 0.5 * theta_error + 0.5 * (theta_error - pre_error);
        move(0, angleSpeed);
        return -1;
    }

    if (distance > 0.1)
    {
        angleSpeed = 2 * theta_error + 0.5 * (theta_error - pre_error);
        vel = 2 * distance;
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

// 得到对应工作台4,5,6,7的材料格状态函数
// 判断是否可以出售
int isSellNeedStatus(WorkBench wb, ROBOT robot)
{
    int wb_product_type = wb.product;
    if (wb.product == 4)
    {
        if (robot.state.product_type == 1 && (wb.raw_material == 0 || wb.raw_material == 4))
        {
            return true;
        }
        else if (robot.state.product_type == 2 && (wb.raw_material == 0 || wb.raw_material == 2))
        {
            return true;
        }
    }
    else if (wb.product == 5)
    {
        if (robot.state.product_type == 1 && (wb.raw_material == 0 || wb.raw_material == 8))
        {
            return true;
        }
        else if (robot.state.product_type == 3 && (wb.raw_material == 0 || wb.raw_material == 2))
        {
            return true;
        }
    }
    else if (wb.product == 6)
    {
        if (robot.state.product_type == 2 && (wb.raw_material == 0 || wb.raw_material == 8))
        {
            return true;
        }
        else if (robot.state.product_type == 3 && (wb.raw_material == 0 || wb.raw_material == 4))
        {
            return true;
        }
    }
    else if (wb.product == 7)
    {
        if (robot.state.product_type = 4 && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 64 || wb.raw_material == 96))
        {
            return true;
        }
        else if (robot.state.product_type == 5 && (wb.raw_material == 0 || wb.raw_material == 64 || wb.raw_material == 16 || wb.raw_material == 80))
        {
            return true;
        }
        else if (robot.state.product_type == 6 && (wb.raw_material == 0 || wb.raw_material == 32 || wb.raw_material == 16 || wb.raw_material == 48))
        {
            return true;
        }
    }
    return -1;
}

// 设置Buy函数
int isBuyDesicion(ROBOT robot, WorkBench wb, int wb_original_index)
{

    // 判断机器人在不在工作台1附近且手里面没有产品，不在的话直接奔向工作台1
    if (robot.state.work_id != wb_original_index && robot.state.product_type == 0)
    {
        return 0; // 0继续运动到指定点位
    }
    // 工作台1已经生产完毕且产品格中已满被阻塞，且机器人已经到工作台1附近，且机器人手中没有产品
    else if ((wb.left_time == 0 || wb.left_time == -1) && robot.state.work_id == wb_original_index && robot.state.product_type == 0)
    {
        return 1; // 机器人0购买产品
    }

    return -1;
}

// 售卖函数
int isSellDesicion(ROBOT robot, int pro, WorkBench wb, int wb_original_index)
{

    // 机器人已经携带物品1，可以运动到
    if (robot.state.product_type == pro && robot.state.work_id != wb_original_index)
    {
        return 0;
    }
    // 机器人到工作台
    else if (robot.state.product_type == pro && robot.state.work_id == wb_original_index)
    {
        // 工作台没有生产，且材料格1空
        if (wb.left_time == -1 && isSellNeedStatus(wb, robot))
        {
            return 1;
        }
        // 工作台生产阻塞，且材料格空
        else if (wb.left_time == 0 && isSellNeedStatus(wb, robot))
        {
            return 1;
        }
    }

    return -1;
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

    // 循环每帧得到判题器的输入并处理之后进行输出
    while (scanf("%d", &frameID) != EOF)
    {
        // unordered_map<WorkBench, int> work_bench_m;
        vector<WorkBench> work_bench_v;
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

        int index_for_wb_m = 0;
        // 遍历K个工作台数据
        while (K >= 1)
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
            work_bench_v.push_back(wb);
            K--;
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
        }

        // 读取最后一行OK
        char line[line_size];
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        //---------------------------------遍历得到第1组1,2工作台对应的位置，以及第1个4工作台对应的位置---------------------------------
        for (int i = 0; i < work_bench_v.size(); i++)
        {
            //=========================对首个工作台index: 7 -> type id: 1的处理=========================
            // 指派机器人0运送
            if (i == 7)
            {

                if (isBuyDesicion(robot_array[0], work_bench_v[i], i) == 0)
                {
                    // 机器人0到1工作台
                    logFile << "[robot 0 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    robot_array[0].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
                }
                else if (isBuyDesicion(robot_array[0], work_bench_v[i], i) == 1)
                {
                    // 机器人0购买工作台1产品
                    logFile << "[robot 0 buying]: " << work_bench_v[i].id << endl;
                    robot_array[0].Buy();
                }
            }

            //=========================对首个工作台index: 3 -> type id: 2的处理=========================
            // 指派机器人1运送
            if (i == 3)
            {
                if (isBuyDesicion(robot_array[1], work_bench_v[i], i) == 0)
                {
                    // 机器人0到1工作台
                    logFile << "[robot 1 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    robot_array[1].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
                }
                else if (isBuyDesicion(robot_array[1], work_bench_v[i], i) == 1)
                {
                    // 机器人0购买工作台1产品
                    logFile << "[robot 1 buying]: " << work_bench_v[i].id << endl;
                    robot_array[1].Buy();
                }
            }

            //==============================对首个工作台index: 11 -> type id: 4的处理==============================
            if (i == 11)
            {

                //----------------------------------------对机器人0的逻辑处理--------------------------------------------
                // 机器人0出售材料1给工作台4
                if (isSellDesicion(robot_array[0], 1, work_bench_v[i], i) == 0)
                {
                    // 机器人0到1工作台
                    logFile << "[robot 0 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    // 机器人奔向工作台
                    robot_array[0].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
                }
                else if (isSellDesicion(robot_array[0], 1, work_bench_v[i], i) == 1)
                {
                    // 机器人0出售1工作台4
                    logFile << "[robot 0 selling]: " << work_bench_v[i].id << endl;
                    // 机器人出售产品给工作台
                    robot_array[0].Sell();
                }

                //-----------------------------------------对机器人1的逻辑处理------------------------------------------
                // 如果机器人1出售材料2给工作台4
                if (isSellDesicion(robot_array[1], 2, work_bench_v[i], i) == 0)
                {
                    // 机器人奔向工作台
                    logFile << "[robot 1 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    robot_array[1].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
                }
                else if (isSellDesicion(robot_array[1], 2, work_bench_v[i], i) == 1)
                {
                    logFile << "[robot 1 selling to]: " << work_bench_v[i].id << endl;
                    // 机器人出售产品给工作台
                    robot_array[1].Sell();
                }

                //-------------------------------------对机器人2的逻辑处理--------------------------------------
                // // 机器人2购买材料4
                // if (isBuyDesicion(robot_array[2], work_bench_v[i], i) == 0)
                // {
                //     // 机器人2到4工作台
                //     logFile << "[robot 2 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                //     robot_array[2].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
                // }
                // else if (isBuyDesicion(robot_array[2], work_bench_v[i], i) == 1)
                // {
                //     // 机器人2购买工作台4产品
                //     logFile << "[robot 2 buying]: " << work_bench_v[i].id << endl;
                //     robot_array[2].Buy();
                // }
            }

            //==============================对首个工作台index: 14 -> type id: 7的处理==============================
            // 将机器人2携带的产品4运送到index: 14（type id: 7）
            // if (i == 14)
            // {
            //     if (isSellDesicion(robot_array[2], 4, work_bench_v[i], i) == 0)
            //     {
            //         // 机器人奔向工作台
            //         // 机器人奔向工作台
            //         logFile << "[robot 2 tracking to]: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
            //         robot_array[2].point_tracking(work_bench_v[i].x, work_bench_v[i].y);
            //     }
            //     else if (isSellDesicion(robot_array[2], 4, work_bench_v[i], i) == 1)
            //     {
            //         logFile << "[robot 2 selling to]: " << work_bench_v[i].id << endl;
            //         // 机器人出售产品给工作台
            //         robot_array[2].Sell();
            //     }
            // }
        }

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);

        logFile << endl;
    }
    logFile.close();
    return 0;
}