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

    Robot state;
    int robot_ID;
    float distance;
    int Buy_pos;
    int sell_pos;

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
void ROBOT::robotToBuy(WorkBench wb)
{ // 参数为机器人对象和工作台对象
    if (state.work_id != wb.arr_idx && isRobotProductNull())
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

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB2.push_back(WB);
                }
            }
            else if (wb.id == 3)
            {
                wb.arr_idx = index;
                work_bench_v3.push_back(wb);

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB3.push_back(WB);
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

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB6.push_back(WB);
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

                if (frameID == 1)
                {
                    WB.wb = wb;
                    WB9.push_back(WB);
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
        }

        // 读取最后一行OK
        char line[line_size];
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        //=================================================计算所有路径=====================================================
        //=================================================计算所有路径=====================================================
        // ----------------------------------------循环4数组--------------------------------------------
        // ----------------------------------------循环4数组--------------------------------------------
        // ----------------------------------------循环4数组--------------------------------------------
        if (frameID == 1)
        {
            for (int i = 0; i < WB4.size(); i++)
            {
                vector<int> tmp_v_1;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历1数组--------------------------------------
                for (int j = 0; j < WB1.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB1[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_1.push_back(iter->second);
                }
                WB4[i].need_material_map[1] = tmp_v_1;
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历2数组--------------------------------------
                vector<int> tmp_v_2;
                for (int j = 0; j < WB2.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB2[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_2.push_back(iter->second);
                }
                WB4[i].need_material_map[2] = tmp_v_2;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }
        }

        // ----------------------------------------循环5数组--------------------------------------------
        // ----------------------------------------循环5数组--------------------------------------------
        // ----------------------------------------循环5数组--------------------------------------------
        if (frameID == 2)
        {
            for (int i = 0; i < WB5.size(); i++)
            {
                vector<int> tmp_v_1;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历1数组--------------------------------------
                for (int j = 0; j < WB1.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB1[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_1.push_back(iter->second);
                }
                WB5[i].need_material_map[1] = tmp_v_1;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历3数组--------------------------------------
                vector<int> tmp_v_3;
                for (int j = 0; j < WB3.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB3[j].wb), j));
                }

                // 遍历map得到v3对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_3.push_back(iter->second);
                }
                WB5[i].need_material_map[3] = tmp_v_3;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }
        }

        // ----------------------------------------循环6数组--------------------------------------------
        // ----------------------------------------循环6数组--------------------------------------------
        // ----------------------------------------循环6数组--------------------------------------------
        if (frameID == 3)
        {
            for (int i = 0; i < WB6.size(); i++)
            {
                vector<int> tmp_v_2;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历2数组--------------------------------------
                for (int j = 0; j < WB2.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB2[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_2.push_back(iter->second);
                }
                WB6[i].need_material_map[2] = tmp_v_2;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历3数组--------------------------------------
                vector<int> tmp_v_3;
                for (int j = 0; j < WB3.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB3[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_3.push_back(iter->second);
                }
                WB6[i].need_material_map[3] = tmp_v_3;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }
        }

        // ----------------------------------------循环7数组--------------------------------------------
        // ----------------------------------------循环7数组--------------------------------------------
        // ----------------------------------------循环7数组--------------------------------------------
        if (frameID == 4)
        {
            for (int i = 0; i < WB7.size(); i++)
            {
                vector<int> tmp_v_4;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;

                // ----------------------------------------遍历4数组--------------------------------------
                for (int j = 0; j < WB4.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB4[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_4.push_back(iter->second);
                }
                WB7[i].need_material_map[4] = tmp_v_4;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历5数组--------------------------------------
                vector<int> tmp_v_5;
                for (int j = 0; j < WB5.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB5[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_5.push_back(iter->second);
                }
                WB7[i].need_material_map[5] = tmp_v_5;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历6数组--------------------------------------
                vector<int> tmp_v_6;
                for (int j = 0; j < WB6.size(); j++)
                {
                    tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB6[j].wb), j));
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_6.push_back(iter->second);
                }
                WB7[i].need_material_map[6] = tmp_v_6;
                //  下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }
        }

        // 测试log
        // logFile << "WB5.size(): " << WB5.size() << endl;
        // logFile << "WB5[0].need_material_map.size(): " << WB5[0].need_material_map.size() << endl;
        // logFile << "WB5[2].need_material_map[1].size(): " << WB5[2].need_material_map[1].size() << endl;

        // for (int i = 0; i < WB5.size(); i++)
        // {
        //     for (int j = 0; j < WB5[i].need_material_map[1].size(); j++)
        //     {
        //         logFile << "WB5[" << i << "].need_material_map[1][" << j << "]: " << WB5[i].need_material_map[1][j] << endl;
        //     }

        //     for (int j = 0; j < WB5[i].need_material_map[3].size(); j++)
        //     {
        //         logFile << "WB5[" << i << "].need_material_map[3][" << j << "]: " << WB5[i].need_material_map[3][j] << endl;
        //     }
        // }

        // 保存当前帧对应的任务的工作台真实index
        vector<int> target_buy_index;
        vector<int> target_sell_index;

        if (frameID > 5)
        {
            // ===============================遍历工作台7需要那个材料===========================================================
            for (int i = 0; i < work_bench_v7.size(); i++)
            {
                // 找到所有4工作台中生产好的目的地保存下来
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 96) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[4].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[4][j];
                        if (work_bench_v4[wb_index].product == 1 || work_bench_v4[wb_index].left_time <= 50)
                        {
                            target_buy_index.push_back(work_bench_v4[wb_index].arr_idx);
                            target_sell_index.push_back(work_bench_v7[i].arr_idx);
                            break;
                        }
                    }
                }
                // 找到所有5工作台中生产好的目的地保存下来
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 80) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[5].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[5][j];
                        if (work_bench_v5[wb_index].product == 1 || work_bench_v5[wb_index].left_time <= 50)
                        {
                            target_buy_index.push_back(work_bench_v5[wb_index].arr_idx);
                            target_sell_index.push_back(work_bench_v7[i].arr_idx);
                            break;
                        }
                    }
                }
                // 找到所有6工作台中生产好的目的地保存下来
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 48) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[6].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[6][j];
                        if (work_bench_v6[wb_index].product == 1 || work_bench_v6[wb_index].left_time <= 50)
                        {
                            target_buy_index.push_back(work_bench_v6[wb_index].arr_idx);
                            target_sell_index.push_back(work_bench_v7[i].arr_idx);
                            break;
                        }
                    }
                }
                // 找到所有4工作台中还未生产但有缺的
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 96) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[4].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[4][j];
                        if ((work_bench_v4[wb_index].raw_material == 4) && work_bench_v4[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                            target_buy_index.push_back(WB4[wb_index].need_material_map[wb_index][0]);
                            break;
                        }
                        else if ((work_bench_v4[wb_index].raw_material == 2) && work_bench_v4[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                            target_buy_index.push_back(WB4[wb_index].need_material_map[2][0]);
                            break;
                        }
                        else if (work_bench_v4[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v4[wb_index].arr_idx);
                            target_buy_index.push_back(WB4[wb_index].need_material_map[1][0]);
                            logFile << "work_bench_v4[wb_index].raw_material:" << work_bench_v4[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                // 找到所有5工作台中还未生产但有缺的
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 80) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[5].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[5][j];
                        if ((work_bench_v5[wb_index].raw_material == 8) && work_bench_v5[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                            target_buy_index.push_back(WB5[wb_index].need_material_map[wb_index][0]);
                            break;
                        }
                        else if ((work_bench_v5[wb_index].raw_material == 2) && work_bench_v5[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                            target_buy_index.push_back(WB5[wb_index].need_material_map[2][0]);
                            break;
                        }
                        else if (work_bench_v5[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v5[wb_index].arr_idx);
                            target_buy_index.push_back(WB5[wb_index].need_material_map[1][0]);
                            logFile << "work_bench_v5[wb_index].arr_idx:" << work_bench_v5[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                // 找到所有6工作台中还未生产但有缺的
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 48) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[6].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[6][j];
                        if ((work_bench_v6[wb_index].raw_material == 8) && work_bench_v6[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                            target_buy_index.push_back(WB6[wb_index].need_material_map[wb_index][0]);
                            break;
                        }
                        else if ((work_bench_v6[wb_index].raw_material == 4) && work_bench_v6[wb_index].left_time == -1)
                        {
                            target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                            target_buy_index.push_back(WB6[wb_index].need_material_map[2][0]);
                            break;
                        }
                        else if (work_bench_v6[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v6[wb_index].arr_idx);
                            target_buy_index.push_back(WB6[wb_index].need_material_map[1][0]);
                            logFile << "work_bench_v6[wb_index].arr_idx:" << work_bench_v6[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                // 找到所有4工作台中还未生产但有缺的
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 96) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[4].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[4][j];
                        if (work_bench_v4[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v4[wb_index].arr_idx);
                            target_buy_index.push_back(WB4[wb_index].need_material_map[1][0]);
                            target_sell_index.push_back( work_bench_v4[wb_index].arr_idx);
                            target_buy_index.push_back(WB4[wb_index].need_material_map[2][0]);
                            logFile << "work_bench_v4[wb_index].raw_material:" << work_bench_v4[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                // 找到所有5工作台中还未生产但都空
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 64 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 80) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[5].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[5][j];
                        if (work_bench_v5[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v5[wb_index].arr_idx);
                            target_buy_index.push_back(WB5[wb_index].need_material_map[1][0]);
                            target_sell_index.push_back( work_bench_v5[wb_index].arr_idx);
                            target_buy_index.push_back(WB5[wb_index].need_material_map[3][0]);
                            logFile << "work_bench_v5[wb_index].arr_idx:" << work_bench_v5[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                // 找到所有6工作台中还未生产但有空
                if ((work_bench_v7[i].raw_material == 0 || work_bench_v7[i].raw_material == 32 || work_bench_v7[i].raw_material == 16 || work_bench_v7[i].raw_material == 48) && work_bench_v7[i].left_time == -1)
                {
                    int wb_index;
                    for (int j = 0; j < WB7[i].need_material_map[6].size(); j++)
                    {
                        wb_index = WB7[i].need_material_map[6][j];
                        if (work_bench_v6[wb_index].raw_material == 0)
                        {
                            target_sell_index.push_back( work_bench_v6[wb_index].arr_idx);
                            target_buy_index.push_back(WB6[wb_index].need_material_map[2][0]);
                            target_sell_index.push_back( work_bench_v6[wb_index].arr_idx);
                            target_buy_index.push_back(WB6[wb_index].need_material_map[3][0]);
                            logFile << "work_bench_v6[wb_index].arr_idx:" << work_bench_v6[wb_index].arr_idx << endl;
                            break;
                        }
                    }
                }
                    // logFile << "target sell index: " << target_sell_index << endl;
                    // logFile << "target buy index: " << target_buy_index << endl;

                    // if (target_sell_index.size() != 0 && target_buy_index.size() != 0)
                    // {
                    //     break;
                    // }
            }
            

            int robot_id = -1;
            float distance_wb_rb;
            int min_dis_wb_robot;

            for (int i = 0; i <= 4; i++)
            {
                distance_wb_rb = dis_wb_robot(robot_array[i].state,work_bench_v[target_buy_index[0]]);
                if (i == 0)
                {
                    robot_id = i;
                    min_dis_wb_robot = distance_wb_rb;
                }
                if (min_dis_wb_robot > distance_wb_rb)
                {
                    robot_id = i;
                    min_dis_wb_robot = distance_wb_rb;
                }
            }

            // todo
        }

        // ================================================循环7数组，设置为优先级最高===================================================
        // for (int i = 0; i < work_bench_v7.size(); i++)
        // {
        //     // 机器人2给6送材料
        //     logFile << "array7->robot3 sell to: " << work_bench_v7[robot_array[3].curr_idx].x << work_bench_v7[robot_array[3].curr_idx].y << endl;
        //     logFile << "array7->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     if (isWorkBenchNeedRobotProType(robot_array[3], work_bench_v7[i]) && work_bench_v7[i].product == 0 && robot_array[3].flag == 2 && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v7[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         robot_array[3].robotToSell(work_bench_v7[robot_array[3].curr_idx], work_bench_v7[robot_array[3].curr_idx].arr_idx);
        //     }

        //     // 机器人3
        //     // 机器人3去7拿产品
        //     logFile << "array7->robot3 buy to: " << work_bench_v7[robot_array[3].curr_idx].x << work_bench_v7[robot_array[3].curr_idx].y << endl;
        //     logFile << "array7->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     if (isWorkBenchCanBeBuy(work_bench_v7[i]) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v7[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         robot_array[3].robotToBuy(work_bench_v7[robot_array[3].curr_idx], work_bench_v7[robot_array[3].curr_idx].arr_idx);
        //     }
        // }

        // // ==============================================循环1数组，找到其中准备好的工作台======================================
        // for (int i = 0; i < work_bench_v1.size(); i++)
        // {
        //     // 机器人0
        //     if (isWorkBenchCanBeBuy(work_bench_v1[i]) && robot_array[0].isRobotProductNull() && (robot_array[0].flag == 0 || robot_array[0].flag == 1))
        //     {
        //         if (robot_array[0].hasDestination == 0)
        //         {
        //             robot_array[0].curr_idx = i;
        //         }
        //         logFile << "array1->robot0 to: " << work_bench_v1[robot_array[0].curr_idx].x << work_bench_v1[robot_array[0].curr_idx].y << endl;
        //         robot_array[0].robotToBuy(work_bench_v1[robot_array[0].curr_idx], work_bench_v1[robot_array[0].curr_idx].arr_idx);
        //     }
        // }

        // // =================================================循环2数组，找到其中准备好的工作台=========================================
        // for (int i = 0; i < work_bench_v2.size(); i++)
        // {
        //     // 机器人1
        //     if (isWorkBenchCanBeBuy(work_bench_v2[i]) && robot_array[1].isRobotProductNull() && (robot_array[1].flag == 0 || robot_array[1].flag == 1))
        //     {
        //         if (robot_array[1].hasDestination == 0)
        //         {
        //             robot_array[1].curr_idx = i;
        //         }
        //         logFile << "array2->robot1 to: " << work_bench_v2[robot_array[1].curr_idx].x << work_bench_v2[robot_array[1].curr_idx].y << endl;
        //         robot_array[1].robotToBuy(work_bench_v2[robot_array[1].curr_idx], work_bench_v2[robot_array[1].curr_idx].arr_idx);
        //     }
        // }

        // // =================================================循环3数组，找到其中准备好的工作台======================================
        // for (int i = 0; i < work_bench_v3.size(); i++)
        // {
        //     // 机器人2
        //     logFile << "array3 inner robot_array[2].hasDestination: " << robot_array[2].hasDestination << endl;
        //     if (isWorkBenchCanBeBuy(work_bench_v3[i]) && robot_array[2].isRobotProductNull() && (robot_array[2].flag == 0 || robot_array[2].flag == 1))
        //     {
        //         if (robot_array[2].hasDestination == 0)
        //         {
        //             robot_array[2].curr_idx = i;
        //         }
        //         logFile << "array3->robot2 to: " << work_bench_v3[robot_array[2].curr_idx].x << work_bench_v3[robot_array[2].curr_idx].y << endl;
        //         robot_array[2].robotToBuy(work_bench_v3[robot_array[2].curr_idx], work_bench_v3[robot_array[2].curr_idx].arr_idx);
        //     }
        // }

        // // ===================================================加工产品工作台=======================================================
        // // ===================================================加工产品工作台=======================================================
        // // ===================================================加工产品工作台=======================================================
        // // ===================================================加工产品工作台=======================================================

        // // ======================================================循环4数组=======================================================
        // for (int i = 0; i < work_bench_v4.size(); i++)
        // {
        //     // 机器人0
        //     if (isWorkBenchNeedRobotProType(robot_array[0], work_bench_v4[i]) && work_bench_v4[i].product == 0 && robot_array[0].flag == 2 && (robot_array[0].hasDestination == 0 || robot_array[0].hasDestination == work_bench_v4[i].id))
        //     {
        //         if (robot_array[0].hasDestination == 0)
        //         {
        //             robot_array[0].curr_idx = i;
        //         }
        //         // 机器人0给4送材料
        //         logFile << "array4->robot0 to: " << work_bench_v4[robot_array[0].curr_idx].x << work_bench_v4[robot_array[0].curr_idx].y << endl;
        //         robot_array[0].robotToSell(work_bench_v4[robot_array[0].curr_idx], work_bench_v4[robot_array[0].curr_idx].arr_idx);
        //     }

        //     // 机器人1
        //     if (isWorkBenchNeedRobotProType(robot_array[1], work_bench_v4[i]) && work_bench_v4[i].product == 0 && robot_array[1].flag == 2 && (robot_array[1].hasDestination == 0 || robot_array[1].hasDestination == work_bench_v4[i].id))
        //     {
        //         if (robot_array[1].hasDestination == 0)
        //         {
        //             robot_array[1].curr_idx = i;
        //         }
        //         // 机器人1给4送材料
        //         logFile << "array4->robot1 to: " << work_bench_v4[robot_array[1].curr_idx].x << work_bench_v4[robot_array[1].curr_idx].y << endl;
        //         robot_array[1].robotToSell(work_bench_v4[robot_array[1].curr_idx], work_bench_v4[robot_array[1].curr_idx].arr_idx);
        //     }

        //     // 机器人3去买5产品
        //     logFile << "array4->robot3 to: " << work_bench_v4[robot_array[3].curr_idx].x << work_bench_v4[robot_array[3].curr_idx].y << endl;
        //     logFile << "array4->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     // 机器人3
        //     if (isWorkCanBeBuyffs(work_bench_v4[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v4[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         logFile << "array4->robot3 to: " << work_bench_v4[robot_array[3].curr_idx].x << work_bench_v4[robot_array[3].curr_idx].y << endl;
        //         robot_array[3].robotToBuy(work_bench_v4[robot_array[3].curr_idx], work_bench_v4[robot_array[3].curr_idx].arr_idx);
        //     }
        // }

        // // ======================================================循环5数组============================================
        // for (int i = 0; i < work_bench_v5.size(); i++)
        // {
        //     logFile << "array5->robot0 to: " << work_bench_v5[robot_array[0].curr_idx].x << work_bench_v5[robot_array[0].curr_idx].y << endl;
        //     logFile << "array5->robot0.hasDestination: " << robot_array[0].hasDestination << endl;
        //     // 机器人0
        //     if (isWorkBenchNeedRobotProType(robot_array[0], work_bench_v5[i]) && work_bench_v5[i].product == 0 && robot_array[0].flag == 2 && (robot_array[0].hasDestination == 0 || robot_array[0].hasDestination == work_bench_v5[i].id))
        //     {
        //         if (robot_array[0].hasDestination == 0)
        //         {
        //             robot_array[0].curr_idx = i;
        //         }
        //         // 机器人0给5送材料
        //         robot_array[0].robotToSell(work_bench_v5[robot_array[0].curr_idx], work_bench_v5[robot_array[0].curr_idx].arr_idx);
        //     }

        //     // 机器人2给5送材料
        //     logFile << "array5->robot2 to: " << work_bench_v5[robot_array[2].curr_idx].x << work_bench_v5[robot_array[2].curr_idx].y << endl;
        //     logFile << "array5->robot2.hasDestination: " << robot_array[2].hasDestination << endl;
        //     // 机器人2
        //     if (isWorkBenchNeedRobotProType(robot_array[2], work_bench_v5[i]) && work_bench_v5[i].product == 0 && robot_array[2].flag == 2 && (robot_array[2].hasDestination == 0 || robot_array[2].hasDestination == work_bench_v5[i].id))
        //     {
        //         if (robot_array[2].hasDestination == 0)
        //         {
        //             robot_array[2].curr_idx = i;
        //         }
        //         robot_array[2].robotToSell(work_bench_v5[robot_array[2].curr_idx], work_bench_v5[robot_array[2].curr_idx].arr_idx);
        //     }

        //     // 机器人3去买5产品
        //     logFile << "array5->robot3 to: " << work_bench_v5[robot_array[3].curr_idx].x << work_bench_v5[robot_array[3].curr_idx].y << endl;
        //     logFile << "array5->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     // 机器人3
        //     if (isWorkCanBeBuyffs(work_bench_v5[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v5[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         robot_array[3].robotToBuy(work_bench_v5[robot_array[3].curr_idx], work_bench_v5[robot_array[3].curr_idx].arr_idx);
        //     }
        // }

        // // ==========================================================循环6数组======================================================
        // for (int i = 0; i < work_bench_v6.size(); i++)
        // {
        //     // 机器人1给6送材料
        //     logFile << "array6->robot1 to: " << work_bench_v6[robot_array[1].curr_idx].x << work_bench_v6[robot_array[1].curr_idx].y << endl;
        //     logFile << "array6->robot1.hasDestination: " << robot_array[1].hasDestination << endl;
        //     // 机器人1
        //     if (isWorkBenchNeedRobotProType(robot_array[1], work_bench_v6[i]) && work_bench_v6[i].product == 0 && robot_array[1].flag == 2 && (robot_array[1].hasDestination == 0 || robot_array[1].hasDestination == work_bench_v6[i].id))
        //     {
        //         if (robot_array[1].hasDestination == 0)
        //         {
        //             robot_array[1].curr_idx = i;
        //         }
        //         robot_array[1].robotToSell(work_bench_v6[robot_array[1].curr_idx], work_bench_v6[robot_array[1].curr_idx].arr_idx);
        //     }

        //     // 机器人2给6送材料
        //     logFile << "array6->robot2 to: " << work_bench_v6[robot_array[2].curr_idx].x << work_bench_v6[robot_array[2].curr_idx].y << endl;
        //     logFile << "array6->robot2.hasDestination: " << robot_array[2].hasDestination << endl;
        //     if (isWorkBenchNeedRobotProType(robot_array[2], work_bench_v6[i]) && work_bench_v6[i].product == 0 && robot_array[2].flag == 2 && (robot_array[2].hasDestination == 0 || robot_array[2].hasDestination == work_bench_v6[i].id))
        //     {
        //         logFile << "robot_array[2].hasDestination: " << robot_array[2].hasDestination << endl;
        //         if (robot_array[2].hasDestination == 0)
        //         {
        //             robot_array[2].curr_idx = i;
        //         }
        //         robot_array[2].robotToSell(work_bench_v6[robot_array[2].curr_idx], work_bench_v6[robot_array[2].curr_idx].arr_idx);
        //         logFile << "hasDestination after Sell(): " << robot_array[2].hasDestination << endl;
        //     }

        //     // 机器人3
        //     // 机器人3去6拿产品
        //     logFile << "array6->robot3 to: " << work_bench_v6[robot_array[3].curr_idx].x << work_bench_v6[robot_array[3].curr_idx].y << endl;
        //     logFile << "array6->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     if (isWorkCanBeBuyffs(work_bench_v6[i], work_bench_v7) && robot_array[3].isRobotProductNull() && (robot_array[3].flag == 0 || robot_array[3].flag == 1) && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v6[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         robot_array[3].robotToBuy(work_bench_v6[robot_array[3].curr_idx], work_bench_v6[robot_array[3].curr_idx].arr_idx);
        //     }
        // }

        // // ===================================================只接收产品工作台=======================================================
        // // ===================================================只接收产品工作台=======================================================

        // // ==========================================================循环8数组==================================================
        // for (int i = 0; i < work_bench_v8.size(); i++)
        // {
        //     // 机器人2给6送材料
        //     logFile << "array8->robot3 sell to: " << work_bench_v8[robot_array[3].curr_idx].x << work_bench_v8[robot_array[3].curr_idx].y << endl;
        //     logFile << "array8->robot3.hasDestination: " << robot_array[3].hasDestination << endl;
        //     // 机器人3
        //     if (work_bench_v8[i].product == 0 && robot_array[3].flag == 2 && (robot_array[3].hasDestination == 0 || robot_array[3].hasDestination == work_bench_v8[i].id))
        //     {
        //         if (robot_array[3].hasDestination == 0)
        //         {
        //             robot_array[3].curr_idx = i;
        //         }
        //         robot_array[3].robotToSell(work_bench_v8[robot_array[3].curr_idx], work_bench_v8[robot_array[3].curr_idx].arr_idx);
        //     }
        // }

        // // ===========================================================处理多余的4 5 6=======================================================
        // // ===========================================================处理多余的4 5 6=======================================================
        // // ===========================================================处理多余的4 5 6=======================================================

        // // // 到最后了让小车0去买4送到9
        // for (int i = 0; i < work_bench_v4.size(); i++)
        // {
        //     // 机器人3
        //     if (isWorkBenchCanBeBuy(work_bench_v4[i]) && robot_array[0].isRobotProductNull() && (robot_array[0].flag == 0 || robot_array[0].flag == 1) && (robot_array[0].hasDestination == 0 || robot_array[0].hasDestination == work_bench_v4[i].id))
        //     {
        //         if (robot_array[0].hasDestination == 0)
        //         {
        //             robot_array[0].curr_idx = i;
        //         }
        //         logFile << "robot0 buy 4 to 9: " << work_bench_v4[robot_array[0].curr_idx].x << work_bench_v4[robot_array[0].curr_idx].y << endl;
        //         robot_array[0].robotToBuy(work_bench_v4[robot_array[0].curr_idx], work_bench_v4[robot_array[0].curr_idx].arr_idx);
        //     }
        // }

        // // 让小车1去买5送到9
        // for (int i = 0; i < work_bench_v5.size(); i++)
        // {
        //     // 机器人3
        //     if (isWorkBenchCanBeBuy(work_bench_v5[i]) && robot_array[1].isRobotProductNull() && (robot_array[1].flag == 0 || robot_array[1].flag == 1) && (robot_array[1].hasDestination == 0 || robot_array[1].hasDestination == work_bench_v4[1].id))
        //     {
        //         if (robot_array[1].hasDestination == 0)
        //         {
        //             robot_array[1].curr_idx = i;
        //         }
        //         logFile << "robot1 buy 5 to 9: " << work_bench_v5[robot_array[1].curr_idx].x << work_bench_v5[robot_array[1].curr_idx].y << endl;
        //         robot_array[1].robotToBuy(work_bench_v5[robot_array[1].curr_idx], work_bench_v5[robot_array[1].curr_idx].arr_idx);
        //     }
        // }

        // // 让小车2去买6送到9
        // for (int i = 0; i < work_bench_v6.size(); i++)
        // {
        //     // 机器人2
        //     if (isWorkBenchCanBeBuy(work_bench_v6[i]) && robot_array[2].isRobotProductNull() && (robot_array[2].flag == 0 || robot_array[2].flag == 1) && (robot_array[2].hasDestination == 0 || robot_array[2].hasDestination == work_bench_v6[i].id))
        //     {
        //         if (robot_array[2].hasDestination == 0)
        //         {
        //             robot_array[2].curr_idx = i;
        //         }
        //         logFile << "robot2 buy 6 to 9: " << work_bench_v6[robot_array[2].curr_idx].x << work_bench_v6[robot_array[2].curr_idx].y << endl;
        //         robot_array[2].robotToBuy(work_bench_v6[robot_array[2].curr_idx], work_bench_v6[robot_array[2].curr_idx].arr_idx);
        //     }
        // }

        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);

        logFile << endl;
    }
    logFile.close();
    return 0;
}