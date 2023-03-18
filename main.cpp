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
    int id;           // 工作台类型
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
    int point_tracking(const float &x, const float &y);
    void move(float lineSpeed, float angleSpeed);
    void Buy();
    void Sell();
    void Destroy();
    Robot state;
    Robot destinate;
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

int ROBOT::point_tracking(const float &x, const float &y)
{

    destinate.x = x;
    destinate.y = y;

    float dx = x - state.x;
    float dy = y - state.y;

    float distance = sqrt(dy * dy + dx * dx);

    float target_angle = atan2(dy, dx);

    float theta_error;
    if (target_angle < 0 && state.direction > 0)
    {
        theta_error = 2 * PI + target_angle - state.direction;
    }
    else if (target_angle > 0 && state.direction < 0)
    {
        theta_error = target_angle - state.direction - 2 * PI;
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

// ====================================================购买逻辑================================================
int isWorkBenchCanBeBuy(WorkBench wb)
{ // 参数为工作台对象
    if (wb.left_time == 0 || wb.left_time == -1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// 机器人是否可以买
int isRobotProductNull(ROBOT robot)
{
    if (robot.state.product_type == 0)
    {
        return 1; // 表示为空
    }
    else
    {
        return 0; // 表示为不空
    }
}

// 调用哪个机器人去哪里买
void robotToBuy(ROBOT robot, WorkBench wb, int workbench_index)
{ // 参数为机器人对象和工作台对象
    if (robot.state.work_id != workbench_index && isRobotProductNull(robot))
    {
        robot.point_tracking(wb.x, wb.y);
    }
    else
    { // 机器人已经到工作台附近且已经可以买了
        robot.Buy();
    }
}

//================================================售卖逻辑==================================================

// 判断当前机器人携带的产品type作为材料的工作台是否需要
bool isWorkBenchNeedRobotProType(ROBOT robot, WorkBench wb)
{
    if (wb.id == 4)
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
    else if (wb.id == 5)
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
    else if (wb.id == 6)
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
    else if (wb.id == 7)
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

    return false;
}

// 机器人奔向对应的workbench进行出售
void robotToSell(ROBOT robot, WorkBench wb, int workbench_index)
{
    // 机器人已经携带物品1，可以运动到
    if (robot.state.work_id != workbench_index)
    {
        // 机器人向工作台跑去
        robot.point_tracking(wb.x, wb.y);
    }
    else if (robot.state.work_id == workbench_index)
    {
        robot.Sell();
    }
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
                // 判断是否生产完毕，机器人是否为空
                if (isWorkBenchCanBeBuy(work_bench_v[i]) && isRobotProductNull(robot_array[0]))
                {
                    // 机器人去买
                    robotToBuy(robot_array[0], work_bench_v[i], i);
                }
            }

            //=========================对首个工作台index: 3 -> type id: 2的处理=========================
            // 指派机器人1运送
            if (i == 3)
            {
                if (isWorkBenchCanBeBuy(work_bench_v[i]) && isRobotProductNull(robot_array[1]))
                {
                    // 机器人去买
                    robotToBuy(robot_array[1], work_bench_v[i], i);
                }
            }

            //==============================对首个工作台index: 11 -> type id: 4的处理==============================
            if (i == 11)
            {
                //----------------------------------------对机器人0的逻辑处理--------------------------------------------
                // 机器人0出售材料1给工作台4
                if (isWorkBenchNeedRobotProType(robot_array[0], work_bench_v[i]))
                {
                    logFile << "robot0 sell to: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    // 机器人0给4送材料
                    robotToSell(robot_array[0], work_bench_v[i], i);
                }

                //-----------------------------------------对机器人1的逻辑处理------------------------------------------
                // 如果机器人1出售材料2给工作台4
                if (isWorkBenchNeedRobotProType(robot_array[1], work_bench_v[i]))
                {
                    logFile << "robot1 sell to: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    // 机器人0给4送材料
                    robotToSell(robot_array[1], work_bench_v[i], i);
                }

                //-------------------------------------对机器人2的逻辑处理--------------------------------------
                if (isWorkBenchCanBeBuy(work_bench_v[i]) && isRobotProductNull(robot_array[2]))
                {
                    logFile << "robot2 buy to: " << work_bench_v[i].x << ", " << work_bench_v[i].y << endl;
                    // 机器人去买
                    robotToBuy(robot_array[2], work_bench_v[i], i);

                    logFile << "robot2 -> x, y: " << robot_array[2].destinate.x << ", " << robot_array[2].destinate.y << endl;
                }
            }

            //==================================对首个工作台index: 14 -> type id: 7的处理==============================
            // 将机器人2携带的产品4运送到index: 14（type id: 7）
            if (i == 14)
            {
                if (isWorkBenchNeedRobotProType(robot_array[2], work_bench_v[i]))
                {
                    // 机器人2给7送材料
                    robotToSell(robot_array[2], work_bench_v[i], i);
                    logFile << "robot2 -> x, y: " << robot_array[2].destinate.x << ", " << robot_array[2].destinate.y << endl;
                }
            }
        }

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);

        logFile << endl;
    }
    logFile.close();
    return 0;
}