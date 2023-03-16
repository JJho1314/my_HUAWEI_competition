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
        float max_forward_v = 6.0; //最大前进速度
        float max_back_v = 2.0; //
        float max_angleSpeed = PI;
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

    float target_angle = atan2(dy , dx);

    float theta_error = target_angle - state.direction;

    if (distance <= 0.4)
    {
        move(0.0, 0.0);
        return 0;
    }

    if(abs(theta_error) >= PI/10)
    {
        angleSpeed = 0.5 * theta_error + 0.5 * (theta_error - pre_error);
        move(0 , angleSpeed);
        return -1;
    }

    if (distance > 0.4)
    {
        angleSpeed = 0.5 * theta_error + 0.5 * (theta_error - pre_error);
        vel = 0.5 * distance;
        move(vel , angleSpeed);
    }
    pre_error = theta_error;

    return -1;
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
        vector<Robot> robot_v;

        scanf("%d", &currMoney);
        getchar();
        scanf("%d", &K);
        getchar();

        // 朝TXT文档中写入数据
        logFile << "[frame ID]:" << frameID << "\n" << endl;

        // 遍历K个工作台数据
        while (K >= 1)
        {
            WorkBench wb;
            scanf("%d %f %f %d %d %d", &wb.id, &wb.x, &wb.y, &wb.left_time, &wb.raw_material, &wb.product);
            getchar();
            work_bench_v.push_back(wb);
            K--;
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
            robot_v.push_back(rb);
        }

        char line[line_size]; // 读取最后一行OK
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        robot[0].point_tracking(0,0);

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);
    }
    logFile.close();
    return 0;
}
