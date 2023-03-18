#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <cmath>

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

    float theta_error = target_angle - state.direction;

    if (distance <= 0.4)
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

    if (distance > 0.4)
    {
        angleSpeed = 0.5 * theta_error + 0.5 * (theta_error - pre_error);
        vel = 0.5 * distance;
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
    ROBOT robot[4];

    // 定义当前总钱数
    int currMoney = 200000;
    // 定义工作台数量
    int K;

    // 循环每帧得到判题器的输入并处理之后进行输出
    while (scanf("%d", &frameID) != EOF)
    {
        vector<WorkBench> work_bench_v;
        vector<Robot> robot_v;

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
            robot[i].update_motion(i, rb);
            logFile << "[Robot WorkBenchId]: " << rb.work_id << ", " << rb.product_type << ", " << rb.time_value << ", " << rb.collision_value << "\n";
            logFile << "[Robot state]:" << rb.angle_speed << ", " << rb.line_speed_x << ", " << rb.line_speed_y << ", " << robot[i].state.x << ", " << robot[i].state.y << ", " << robot[i].state.direction << "\n"
                    << endl;
            getchar(); // 跳过换行符
            robot_v.push_back(rb);
        }

        // 读取最后一行OK
        char line[line_size];
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        // ---------------------------------遍历得到第1组1,2工作台对应的位置，以及第1个4工作台对应的位置---------------------------------
        for (int i = 0; i < work_bench_v.size(); i++)
        {
            //=========================对首个工作台index: 0 -> type id: 1的处理=========================
            // 指派机器人0运送
            if (i == 0)
            {
                WorkBench wb = work_bench_v[i];
                // 判断机器人在不在工作台1附近且手里面没有产品，不在的话直接奔向工作台1
                if (robot[0].state.work_id != 0 && robot[0].state.product_type == 0)
                {
                    // 机器人0到1工作台
                    robot[0].point_tracking(wb.x, wb.y - 0.25);
                }
                // 工作台1已经生产完毕且产品格中已满被阻塞，且机器人已经到工作台1附近，且机器人手中没有产品
                else if ((wb.left_time == 0 || wb.left_time == -1) && robot[0].state.work_id == 0 && robot[0].state.product_type == 0)
                {
                    // 机器人0购买工作台1产品
                    robot[0].Buy();
                }
            }

            //=========================对首个工作台index: 1 -> type id: 2的处理=========================
            // 指派机器人1运送
            if (i == 1)
            {
                WorkBench wb = work_bench_v[i];
                // 机器人1中没有产品，且不在工作台2附近
                if (robot[1].state.work_id != 1 && robot[1].state.product_type == 0)
                {
                    // 机器人1向工作台2运动
                    robot[1].point_tracking(3.25, 48.75 - 0.25);
                    logFile << "[target point]: " << wb.x << ", " << wb.y << endl;
                }
                // 机器人1达到工作台2附近，且手中没有产品
                else if ((wb.left_time == 0 || wb.left_time == -1) && robot[1].state.work_id == 1 && robot[1].state.product_type == 0)
                {
                    // 机器人1购买产品2
                    robot[1].Buy();
                }
            }

            //==============================对首个工作台index: 2 -> type id: 4的处理==============================
            if (i == 2)
            {
                WorkBench wb = work_bench_v[i];
                //-----------------------------------------对机器人0的逻辑处理------------------------------------------
                // 机器人0已经携带物品1，可以运动到4
                if (robot[0].state.product_type == 1 && robot[0].state.work_id != 2)
                {
                    // 机器人0奔向工作台4
                    robot[0].point_tracking(wb.x, wb.y - 0.25);
                }
                // 机器人0到工作台4
                else if (robot[0].state.product_type == 1 && robot[0].state.work_id == 2)
                {
                    // 工作台4没有生产，且材料格1空
                    if (wb.left_time == -1 && (wb.raw_material == 0 || wb.raw_material == 4))
                    {
                        // 机器人0出售产品1给工作台4
                        robot[0].Sell();
                    }
                    // 工作台4生产阻塞，且材料格1空
                    else if (wb.left_time == 0 && (wb.raw_material == 0 || wb.raw_material == 4))
                    {
                        // 机器人0出售产品1给工作台4
                        robot[0].Sell();
                        // 机器人0购买4
                        robot[0].Buy();
                    }
                }

                //-----------------------------------------对机器人1的逻辑处理------------------------------------------
                // 如果机器人1手中持有产品2，且没有达到工作台4附近
                if (robot[1].state.product_type == 2 && robot[1].state.work_id != 2)
                {
                    // 机器人1向工作台4前进
                    robot[1].point_tracking(wb.x, wb.y - 0.25);
                } // 机器人达到工作台4，且手中有产品2
                else if (robot[1].state.product_type == 2 && robot[1].state.work_id != 2)
                {
                    // 工作台4没有生产，且材料格2空
                    if (wb.left_time == -1 && (wb.raw_material == 0 || wb.raw_material == 2))
                    {
                        // 机器人1出售产品2给工作台4
                        robot[1].Sell();
                    }
                    // 工作台4生产阻塞，且材料格2空
                    else if (wb.left_time == 0 && (wb.raw_material == 0 || wb.raw_material == 2))
                    {
                        // 机器人1出售产品2给工作台4
                        robot[1].Sell();
                    }
                }
            }

            //==============================对首个工作台index: 15 -> type id: 8的处理==============================
            // 将机器人0携带的产品4运送到8
            if (i == 15)
            {
                WorkBench wb = work_bench_v[i];
                // 机器人此时携带产品4，且没有到达8附近
                if (robot[0].state.product_type == 4 && robot[0].state.work_id != 15)
                {
                    // 机器人0向工作台8前进
                    robot[0].point_tracking(wb.x, wb.y - 0.25);
                } // 机器人携带产品4，且达到了8附近，那么出售即可
                else if (robot[0].state.product_type == 4 && robot[0].state.work_id == 15)
                {
                    // 机器人0出售产品4给工作台8
                    robot[0].Sell();
                }
            }
        }

        // robot[1].point_tracking(3.25, 48.75 - 0.25);

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);

        logFile << endl;
    }
    logFile.close();
    return 0;
}