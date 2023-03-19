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
    int arr_idx;      // 工作台在数组中的顺序
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

    int isRobotProductNull();
    void robotToBuy(WorkBench wb, int workbench_index);
    void robotToSell(WorkBench wb, int workbench_index);

    Robot state;
    Robot destinate;
    int robot_ID;
    float distance;

    int flag = 0;           // 0表示机器人空状态，1表示机器人该去买状态，2机器人该去卖状态
    int hasDestination = 0; // 0表示没有目的地, 1表示有目的地
    int curr_idx = -1;      // 表示记录当前小车去往的工作台index

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
    if (target_angle < -PI / 2 && state.direction > PI / 2)
    {
        theta_error = 2 * PI + target_angle - state.direction;
    }
    else if (target_angle > PI / 2 && state.direction < -PI / 2)
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
        angleSpeed = 5 * theta_error + 0.5 * (theta_error - pre_error);
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
void ROBOT::robotToBuy(WorkBench wb, int workbench_index)
{ // 参数为机器人对象和工作台对象
    if (state.work_id != workbench_index && isRobotProductNull())
    {
        point_tracking(wb.x, wb.y);
        hasDestination = wb.id; // 去id这个类型的目的地
    }
    else
    {                       // 机器人已经到工作台附近且已经可以买了
        hasDestination = 0; // 表示到达地方，需要下一个目的地
        Buy();
        flag = 2; // 该去卖状态
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
        flag = 1;
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
            }
            else if (wb.id == 2)
            {
                wb.arr_idx = index;
                work_bench_v2.push_back(wb);
            }
            else if (wb.id == 3)
            {
                wb.arr_idx = index;
                work_bench_v3.push_back(wb);
            }
            else if (wb.id == 4)
            {
                wb.arr_idx = index;
                work_bench_v4.push_back(wb);
            }
            else if (wb.id == 5)
            {
                wb.arr_idx = index;
                work_bench_v5.push_back(wb);
            }
            else if (wb.id == 6)
            {
                wb.arr_idx = index;
                work_bench_v6.push_back(wb);
            }
            else if (wb.id == 7)
            {
                wb.arr_idx = index;
                work_bench_v7.push_back(wb);
            }
            else if (wb.id == 8)
            {
                wb.arr_idx = index;
                work_bench_v8.push_back(wb);
            }
            else if (wb.id == 9)
            {
                wb.arr_idx = index;
                work_bench_v9.push_back(wb);
            }
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
        }

        // 读取最后一行OK
        char line[line_size];
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        // ==============================================循环1数组，找到其中准备好的工作台======================================
        for (int i = 0; i < work_bench_v1.size(); i++)
        {
            // 机器人0
            if (isWorkBenchCanBeBuy(work_bench_v1[i]) && robot_array[0].isRobotProductNull() && (robot_array[0].flag == 0 || robot_array[0].flag == 1))
            {
                if (robot_array[0].hasDestination == 0)
                {
                    robot_array[0].curr_idx = i;
                }
                logFile << "array1->robot0 to: " << work_bench_v1[robot_array[0].curr_idx].x << work_bench_v1[robot_array[0].curr_idx].y << endl;
                robot_array[0].robotToBuy(work_bench_v1[robot_array[0].curr_idx], work_bench_v1[robot_array[0].curr_idx].arr_idx);
            }
        }

        // ================================================循环2数组，找到其中准备好的工作台=========================================
        for (int i = 0; i < work_bench_v2.size(); i++)
        {
            // 机器人1
            if (isWorkBenchCanBeBuy(work_bench_v2[i]) && robot_array[1].isRobotProductNull() && (robot_array[1].flag == 0 || robot_array[1].flag == 1))
            {
                if (robot_array[1].hasDestination == 0)
                {
                    robot_array[1].curr_idx = i;
                }
                logFile << "array2->robot1 to: " << work_bench_v2[robot_array[1].curr_idx].x << work_bench_v2[robot_array[1].curr_idx].y << endl;
                robot_array[1].robotToBuy(work_bench_v2[robot_array[1].curr_idx], work_bench_v2[robot_array[1].curr_idx].arr_idx);
            }
        }

        // =============================================循环3数组，找到其中准备好的工作台======================================
        for (int i = 0; i < work_bench_v3.size(); i++)
        {
            // 机器人2
            logFile << "array3 inner robot_array[2].hasDestination: " << robot_array[2].hasDestination << endl;
            if (isWorkBenchCanBeBuy(work_bench_v3[i]) && robot_array[2].isRobotProductNull() && (robot_array[2].flag == 0 || robot_array[2].flag == 1))
            {
                if (robot_array[2].hasDestination == 0)
                {
                    robot_array[2].curr_idx = i;
                }
                logFile << "array3->robot2 to: " << work_bench_v3[robot_array[2].curr_idx].x << work_bench_v3[robot_array[2].curr_idx].y << endl;
                robot_array[2].robotToBuy(work_bench_v3[robot_array[2].curr_idx], work_bench_v3[robot_array[2].curr_idx].arr_idx);
            }
        }

        // ===================================================加工产品工作台=======================================================
        // ===================================================加工产品工作台=======================================================
        // ===================================================加工产品工作台=======================================================
        // ===================================================加工产品工作台=======================================================

        // ======================================================循环4数组=======================================================
        for (int i = 0; i < work_bench_v4.size(); i++)
        {
            // 机器人0
            if (isWorkBenchNeedRobotProType(robot_array[0], work_bench_v4[i]) && work_bench_v4[i].product == 0 && robot_array[0].flag == 2 && (robot_array[0].hasDestination == 0 || robot_array[0].hasDestination == work_bench_v4[i].id))
            {
                if (robot_array[0].hasDestination == 0)
                {
                    robot_array[0].curr_idx = i;
                }
                // 机器人0给4送材料
                logFile << "array4->robot0 to: " << work_bench_v4[robot_array[0].curr_idx].x << work_bench_v4[robot_array[0].curr_idx].y << endl;
                robot_array[0].robotToSell(work_bench_v4[robot_array[0].curr_idx], work_bench_v4[robot_array[0].curr_idx].arr_idx);
            }

            // 机器人1
            if (isWorkBenchNeedRobotProType(robot_array[1], work_bench_v4[i]) && work_bench_v4[i].product == 0 && robot_array[1].flag == 2 && (robot_array[1].hasDestination == 0 || robot_array[1].hasDestination == work_bench_v4[i].id))
            {
                if (robot_array[1].hasDestination == 0)
                {
                    robot_array[1].curr_idx = i;
                }
                // 机器人1给4送材料
                logFile << "array4->robot1 to: " << work_bench_v4[robot_array[1].curr_idx].x << work_bench_v4[robot_array[1].curr_idx].y << endl;
                robot_array[1].robotToSell(work_bench_v4[robot_array[1].curr_idx], work_bench_v4[robot_array[1].curr_idx].arr_idx);
            }

            // 机器人3去买5产品
            logFile << "array4->robot3 to: " << work_bench_v4[robot_array[3].curr_idx].x << work_bench_v4[robot_array[3].curr_idx].y << endl;
            logFile << "array4->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            // 机器人3
            if (isWorkCanBeBuyffs(work_bench_v4[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v4[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                logFile << "array4->robot3 to: " << work_bench_v4[robot_array[3].curr_idx].x << work_bench_v4[robot_array[3].curr_idx].y << endl;
                robot_array[3].robotToBuy(work_bench_v4[robot_array[3].curr_idx], work_bench_v4[robot_array[3].curr_idx].arr_idx);
            }
        }

        // ======================================================循环5数组============================================
        for (int i = 0; i < work_bench_v5.size(); i++)
        {
            logFile << "array5->robot0 to: " << work_bench_v5[robot_array[0].curr_idx].x << work_bench_v5[robot_array[0].curr_idx].y << endl;
            logFile << "array5->robot0.hasDestination: " << robot_array[0].hasDestination << endl;
            // 机器人0
            if (isWorkBenchNeedRobotProType(robot_array[0], work_bench_v5[i]) && work_bench_v5[i].product == 0 && robot_array[0].flag == 2 && (robot_array[0].hasDestination == 0 || robot_array[0].hasDestination == work_bench_v5[i].id))
            {
                if (robot_array[0].hasDestination == 0)
                {
                    robot_array[0].curr_idx = i;
                }
                // 机器人0给5送材料
                robot_array[0].robotToSell(work_bench_v5[robot_array[0].curr_idx], work_bench_v5[robot_array[0].curr_idx].arr_idx);
            }

            // 机器人2给5送材料
            logFile << "array5->robot2 to: " << work_bench_v5[robot_array[2].curr_idx].x << work_bench_v5[robot_array[2].curr_idx].y << endl;
            logFile << "array5->robot2.hasDestination: " << robot_array[2].hasDestination << endl;
            // 机器人2
            if (isWorkBenchNeedRobotProType(robot_array[2], work_bench_v5[i]) && work_bench_v5[i].product == 0 && robot_array[2].flag == 2 && (robot_array[2].hasDestination == 0 || robot_array[2].hasDestination == work_bench_v5[i].id))
            {
                if (robot_array[2].hasDestination == 0)
                {
                    robot_array[2].curr_idx = i;
                }
                robot_array[2].robotToSell(work_bench_v5[robot_array[2].curr_idx], work_bench_v5[robot_array[2].curr_idx].arr_idx);
            }

            // 机器人3去买5产品
            logFile << "array5->robot3 to: " << work_bench_v5[robot_array[3].curr_idx].x << work_bench_v5[robot_array[3].curr_idx].y << endl;
            logFile << "array5->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            // 机器人3
            if (isWorkCanBeBuyffs(work_bench_v5[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v5[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                robot_array[3].robotToBuy(work_bench_v5[robot_array[3].curr_idx], work_bench_v5[robot_array[3].curr_idx].arr_idx);
            }
        }

        // ==========================================================循环6数组======================================================
        for (int i = 0; i < work_bench_v6.size(); i++)
        {
            // 机器人1给6送材料
            logFile << "array6->robot1 to: " << work_bench_v6[robot_array[1].curr_idx].x << work_bench_v6[robot_array[1].curr_idx].y << endl;
            logFile << "array6->robot1.hasDestination: " << robot_array[1].hasDestination << endl;
            // 机器人1
            if (isWorkBenchNeedRobotProType(robot_array[1], work_bench_v6[i]) && work_bench_v6[i].product == 0 && robot_array[1].flag == 2 && (robot_array[1].hasDestination == 0 || robot_array[1].hasDestination == work_bench_v6[i].id))
            {
                if (robot_array[1].hasDestination == 0)
                {
                    robot_array[1].curr_idx = i;
                }
                robot_array[1].robotToSell(work_bench_v6[robot_array[1].curr_idx], work_bench_v6[robot_array[1].curr_idx].arr_idx);
            }

            // 机器人2给6送材料
            logFile << "array6->robot2 to: " << work_bench_v6[robot_array[2].curr_idx].x << work_bench_v6[robot_array[2].curr_idx].y << endl;
            logFile << "array6->robot2.hasDestination: " << robot_array[2].hasDestination << endl;
            if (isWorkBenchNeedRobotProType(robot_array[2], work_bench_v6[i]) && work_bench_v6[i].product == 0 && robot_array[2].flag == 2 && (robot_array[2].hasDestination == 0 || robot_array[2].hasDestination == work_bench_v6[i].id))
            {
                logFile << "robot_array[2].hasDestination: " << robot_array[2].hasDestination << endl;
                if (robot_array[2].hasDestination == 0)
                {
                    robot_array[2].curr_idx = i;
                }
                robot_array[2].robotToSell(work_bench_v6[robot_array[2].curr_idx], work_bench_v6[robot_array[2].curr_idx].arr_idx);
                logFile << "hasDestination after Sell(): " << robot_array[2].hasDestination << endl;
            }

            // 机器人3
            // 机器人3去6拿产品
            logFile << "array6->robot3 to: " << work_bench_v6[robot_array[3].curr_idx].x << work_bench_v6[robot_array[3].curr_idx].y << endl;
            logFile << "array6->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            if (isWorkCanBeBuyffs(work_bench_v6[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v6[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                robot_array[3].robotToBuy(work_bench_v6[robot_array[3].curr_idx], work_bench_v6[robot_array[3].curr_idx].arr_idx);
            }
        }

        // ========================================================循环7数组===================================================
        for (int i = 0; i < work_bench_v7.size(); i++)
        {
            // 机器人2给6送材料
            logFile << "array7->robot3 sell to: " << work_bench_v7[robot_array[3].curr_idx].x << work_bench_v7[robot_array[3].curr_idx].y << endl;
            logFile << "array7->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            if (isWorkBenchNeedRobotProType(robot_array[3], work_bench_v7[i]) && work_bench_v7[i].product == 0 && robot_array[3].flag == 2 && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v7[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                robot_array[3].robotToSell(work_bench_v7[robot_array[3].curr_idx], work_bench_v7[robot_array[3].curr_idx].arr_idx);
            }

            // 机器人3
            // 机器人3去7拿产品
            logFile << "array7->robot3 buy to: " << work_bench_v7[robot_array[3].curr_idx].x << work_bench_v7[robot_array[3].curr_idx].y << endl;
            logFile << "array7->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            if (isWorkBenchCanBeBuy(work_bench_v7[i]) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v7[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                robot_array[3].robotToBuy(work_bench_v7[robot_array[3].curr_idx], work_bench_v7[robot_array[3].curr_idx].arr_idx);
            }
        }

        // ===================================================只接收产品工作台=======================================================
        // ===================================================只接收产品工作台=======================================================
        // ===================================================只接收产品工作台=======================================================
        // ===================================================只接收产品工作台=======================================================

        // ==========================================================循环8数组==================================================
        for (int i = 0; i < work_bench_v8.size(); i++)
        {
            // 机器人2给6送材料
            logFile << "array8->robot3 sell to: " << work_bench_v8[robot_array[3].curr_idx].x << work_bench_v8[robot_array[3].curr_idx].y << endl;
            logFile << "array8->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
            // 机器人3
            if (work_bench_v8[i].product == 0 && robot_array[3].flag == 2 && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v8[i].id))
            {
                if (robot_array[3].hasDestination == 0)
                {
                    robot_array[3].curr_idx = i;
                }
                robot_array[3].robotToSell(work_bench_v8[robot_array[3].curr_idx], work_bench_v8[robot_array[3].curr_idx].arr_idx);
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