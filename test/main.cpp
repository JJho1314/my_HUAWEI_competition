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
#include "config.hpp"

using namespace std;
vector<Robot> Barrier; // 障碍物
void avoid_crash(ROBOT robot_array[4])
{
    Position pos1, pos2, des1, des2;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            if (!robot_array[i].flag || !robot_array[j].flag)
            {
                continue;
            }
            if (!robot_array[i].has_temp_destination_ || robot_array[j].has_temp_destination_)
            {
                continue;
            }

            pos1.x_ = robot_array[i].state.x;
            pos1.y_ = robot_array[i].state.y;
            pos2.x_ = robot_array[j].state.x;
            pos2.y_ = robot_array[j].state.y;

            if (calc_Collide(pos1, des1, pos2, des2))
            {
                robot_array[i].has_temp_destination_ = true;
                robot_array[j].has_temp_destination_ = true;
                calc_TempDes(pos1, pos2, robot_array[i].temp_destination_, robot_array[j].temp_destination_);
            }
        }
    }
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
            Barrier.push_back(rb);
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
                    if (tmp_sort_map.find(distance_between_workbench(WB4[i].wb, WB1[j].wb)) != tmp_sort_map.end()) {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB1[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB1[j].wb), j));
                    }
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
                    if (tmp_sort_map.find(distance_between_workbench(WB4[i].wb, WB2[j].wb)) != tmp_sort_map.end()) {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB2[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB4[i].wb, WB2[j].wb), j));
                    }
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

            for (int i = 0; i < WB3.size(); i++)
            {
                vector<int> tmp_v_5;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历1数组--------------------------------------
                for (int j = 0; j < WB5.size(); j++)
                {
                    if (tmp_sort_map.find(distance_between_workbench(WB3[i].wb, WB5[j].wb)) != tmp_sort_map.end()) {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB3[i].wb, WB5[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB3[i].wb, WB5[j].wb), j));
                    }
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_5.push_back(iter->second);
                }
                WB3[i].need_material_map[5] = tmp_v_5;
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历2数组--------------------------------------
                vector<int> tmp_v_6;
                for (int j = 0; j < WB6.size(); j++)
                {
                    if (tmp_sort_map.find(distance_between_workbench(WB3[i].wb, WB6[j].wb)) != tmp_sort_map.end()) {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB3[i].wb, WB6[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB3[i].wb, WB6[j].wb), j));
                    }
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_6.push_back(iter->second);
                }
                WB3[i].need_material_map[6] = tmp_v_6;
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
                    if(tmp_sort_map.find(distance_between_workbench(WB5[i].wb, WB1[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB1[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB1[j].wb), j));
                    }      
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
                    if(tmp_sort_map.find(distance_between_workbench(WB5[i].wb, WB3[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB3[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB5[i].wb, WB3[j].wb), j));
                    }
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

            for (int i = 0; i < WB2.size(); i++)
            {
                vector<int> tmp_v_4;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历1数组--------------------------------------
                for (int j = 0; j < WB4.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB2[i].wb, WB4[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB2[i].wb, WB4[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB2[i].wb, WB4[j].wb), j));
                    }      
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_4.push_back(iter->second);
                }
                WB2[i].need_material_map[4] = tmp_v_4;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历3数组--------------------------------------
                vector<int> tmp_v_6;
                for (int j = 0; j < WB6.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB2[i].wb, WB6[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB2[i].wb, WB6[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB2[i].wb, WB6[j].wb), j));
                    }
                }

                // 遍历map得到v3对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_6.push_back(iter->second);
                }
                WB2[i].need_material_map[6] = tmp_v_6;
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
                    if(tmp_sort_map.find(distance_between_workbench(WB6[i].wb, WB2[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB2[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB2[j].wb), j));
                    } 
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
                    if(tmp_sort_map.find(distance_between_workbench(WB6[i].wb, WB3[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB3[j].wb)+0.01, j));
                    }
                    else
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB6[i].wb, WB3[j].wb), j));
                    }
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

            for (int i = 0; i < WB1.size(); i++)
            {
                vector<int> tmp_v_4;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;
                // ----------------------------------------遍历2数组--------------------------------------
                for (int j = 0; j < WB4.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB1[i].wb, WB4[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB1[i].wb, WB4[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB1[i].wb, WB4[j].wb), j));
                    } 
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_4.push_back(iter->second);
                }
                WB1[i].need_material_map[4] = tmp_v_4;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历3数组--------------------------------------
                vector<int> tmp_v_5;
                for (int j = 0; j < WB5.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB1[i].wb, WB5[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB1[i].wb, WB5[j].wb)+0.01, j));
                    }
                    else
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB1[i].wb, WB5[j].wb), j));
                    }
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_5.push_back(iter->second);
                }
                WB1[i].need_material_map[5] = tmp_v_5;
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
                    if(tmp_sort_map.find(distance_between_workbench(WB7[i].wb, WB4[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB4[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB4[j].wb), j));
                    }
                    
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
                    if(tmp_sort_map.find(distance_between_workbench(WB7[i].wb, WB5[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB5[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB5[j].wb), j));
                    } 
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
                    if(tmp_sort_map.find(distance_between_workbench(WB7[i].wb, WB6[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB6[j].wb)+0.1, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB7[i].wb, WB6[j].wb), j));
                    } 
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

        if (frameID == 5)
        {
            for (int i = 0; i < WB8.size(); i++)
            {
                vector<int> tmp_v_7;
                map<float, int> tmp_sort_map;
                for (int j = 0; j < WB7.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB8[i].wb, WB7[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB8[i].wb, WB7[j].wb)+0.1, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB8[i].wb, WB7[j].wb), j));
                    } 
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_7.push_back(iter->second);
                }
                WB8[i].need_material_map[7] = tmp_v_7;
                //  下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }

            for (int i = 0; i < WB9.size(); i++)
            {
                vector<int> tmp_v_4;
                // key表示距离, value表示index，辅助排序
                map<float, int> tmp_sort_map;

                // ----------------------------------------遍历4数组--------------------------------------
                for (int j = 0; j < WB4.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB9[i].wb, WB4[j].wb)) != tmp_sort_map.end()){
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB4[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB4[j].wb), j));
                    }
                    
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_4.push_back(iter->second);
                }
                WB9[i].need_material_map[4] = tmp_v_4;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历5数组--------------------------------------
                vector<int> tmp_v_5;
                for (int j = 0; j < WB5.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB9[i].wb, WB5[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB5[j].wb)+0.01, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB5[j].wb), j));
                    } 
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_5.push_back(iter->second);
                }
                WB9[i].need_material_map[5] = tmp_v_5;
                // 下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                // ----------------------------------------遍历6数组--------------------------------------
                vector<int> tmp_v_6;
                for (int j = 0; j < WB6.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB9[i].wb, WB6[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB6[j].wb)+0.1, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB6[j].wb), j));
                    } 
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_6.push_back(iter->second);
                }
                WB9[i].need_material_map[6] = tmp_v_6;
                //  下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量

                vector<int> tmp_v_7;
                for (int j = 0; j < WB7.size(); j++)
                {
                    if(tmp_sort_map.find(distance_between_workbench(WB9[i].wb, WB7[j].wb)) != tmp_sort_map.end())
                    {
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB7[j].wb)+0.1, j));
                    }
                    else{
                        tmp_sort_map.insert(pair<float, int>(distance_between_workbench(WB9[i].wb, WB7[j].wb), j));
                    } 
                }

                // 遍历map得到v1对应index的工作台对象
                for (auto iter = tmp_sort_map.begin(); iter != tmp_sort_map.end(); iter++)
                {
                    tmp_v_7.push_back(iter->second);
                }
                WB9[i].need_material_map[7] = tmp_v_7;
                //  下次遍历之前清空vector和map
                tmp_sort_map.clear(); // 清除容器并最小化它的容量
            }
        }

        // 测试log
        // logFile << "WB5.size(): " << WB5.size() << endl;
        // logFile << "WB5[0].need_material_map.size(): " << WB5[0].need_material_map.size() << endl;
        // logFile << "WB5[2].need_material_map[1].size(): " << WB5[2].need_material_map[1].size() << endl;

        // for (int i = 0; i < WB3.size(); i++)
        // {
        //     for (int j = 0; j < WB3[i].need_material_map[5].size(); j++)
        //     {
        //         logFile << "WB3[" << i << "].need_material_map[5][" << j << "]: " << work_bench_v5[WB3[i].need_material_map[5][j]].arr_idx<< endl;
        //     }

        //     for (int j = 0; j < WB3[i].need_material_map[6].size(); j++)
        //     {
        //         logFile << "WB3[" << i << "].need_material_map[6][" << j << "]: " << work_bench_v6[WB3[i].need_material_map[6][j]].arr_idx << endl;
        //     }
        // }

        int work_bench_v4_size = work_bench_v4.size();
        int work_bench_v5_size = work_bench_v5.size();
        int work_bench_v6_size = work_bench_v6.size();

        logFile << "work_bench_v4_size: " << work_bench_v4_size << endl;
        logFile << "work_bench_v5_size: " << work_bench_v5_size << endl;
        logFile << "work_bench_v6_size: " << work_bench_v6_size << endl;

        int max_wb_num = work_bench_v4_size > work_bench_v5_size ? work_bench_v4_size : work_bench_v5_size;
	    max_wb_num = max_wb_num > work_bench_v6_size ? max_wb_num : work_bench_v6_size;
        
        // 保存当前帧对应的任务的工作台真实index
        vector<int> target_buy_index;
        vector<int> target_sell_index;

        if (frameID > 5)
        {
            if (work_bench_v9.size() != 0)
            {
                for(int j = 0; j < work_bench_v7.size(); j++){
                    // 生产好工作台7号优先送七号
                    for (int i = 0; i < WB9.size(); i++)
                    {
                        if (((work_bench_v7[WB9[i].need_material_map[7][j]].left_time <= 10) && (work_bench_v7[WB9[i].need_material_map[7][j]].left_time != -1)) || work_bench_v7[WB9[i].need_material_map[7][j]].product == 1)
                        {
                            target_buy_index.push_back(work_bench_v7[WB9[i].need_material_map[7][j]].arr_idx);
                            target_sell_index.push_back(work_bench_v9[i].arr_idx);
                        }
                    }
                }
            }
            // 生产好工作台7号优先送七号
            for (int i = 0; i < WB8[0].need_material_map[7].size(); i++)
            {
                if (((work_bench_v7[i].left_time <= 10) && (work_bench_v7[i].left_time != -1)) || work_bench_v7[i].product == 1)
                {
                    target_buy_index.push_back(work_bench_v7[i].arr_idx);
                    target_sell_index.push_back(work_bench_v8[0].arr_idx);
                }
            }

            for (int j = 0; j < max_wb_num; j++)
            {
                for (int i = 0; i < WB8[0].need_material_map[7].size(); i++)
                {
                    int wb_index;
                    // 找到所有4工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 96))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[4].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[4][j];
                            if (work_bench_v4[wb_index].product == 1 || (work_bench_v4[wb_index].left_time <= 50 && work_bench_v4[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v4[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }
                    // 找到所有5工作台中生产好的目的地保存下来
                    if (( work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 80))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[5].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[5][j];
                            if (work_bench_v5[wb_index].product == 1 || (work_bench_v5[wb_index].left_time <= 50 && work_bench_v5[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v5[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }
                    // 找到所有6工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 48))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[6].size())
                        {
     
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[6][j];
                            if (work_bench_v6[wb_index].product == 1 || (work_bench_v6[wb_index].left_time <= 50 && work_bench_v6[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v6[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                  
                        }
                    }
                }
                
            }
            logFile << "=======================================================" << endl;
            // ===============================遍历工作台7需要那个材料===========================================================
            // 再优先搬运缺两个的
            for (int j = 0; j < max_wb_num; j++)
            {
                for (int i = 0; i < WB8[0].need_material_map[7].size(); i++)
                {
                    int wb_index;
                    // 找到所有4工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 32 || work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 64))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[4].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[4][j];
                            if (work_bench_v4[wb_index].product == 1 || (work_bench_v4[wb_index].left_time <= 50 && work_bench_v4[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v4[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }
                    // 找到所有5工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 64 || work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 16))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[5].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[5][j];
                            if (work_bench_v5[wb_index].product == 1 || (work_bench_v5[wb_index].left_time <= 50 && work_bench_v5[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v5[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }

                    // 找到所有6工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 32 || work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 16))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[6].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[6][j];
                            if (work_bench_v6[wb_index].product == 1 || (work_bench_v6[wb_index].left_time <= 50 && work_bench_v6[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v6[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }

                }
                
            }
            logFile << "=======================================================" << endl;
            for (int j = 0; j < max_wb_num; j++)
            {
                for (int i = 0; i < WB8[0].need_material_map[7].size(); i++)
                {
                    int wb_index;
                    // 找到所有4工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 0))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[4].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[4][j];
                            if (work_bench_v4[wb_index].product == 1 || (work_bench_v4[wb_index].left_time <= 50 && work_bench_v4[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v4[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }
         
                    // 找到所有5工作台中生产好的目的地保存下来
                    if (( work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 0))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[5].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[5][j];
                            if (work_bench_v5[wb_index].product == 1 || (work_bench_v5[wb_index].left_time <= 50 && work_bench_v5[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v5[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }

                    // 找到所有6工作台中生产好的目的地保存下来
                    if ((work_bench_v7[WB8[0].need_material_map[7][i]].raw_material == 0))
                    {
                        if (j < WB7[WB8[0].need_material_map[7][i]].need_material_map[6].size())
                        {
                            wb_index = WB7[WB8[0].need_material_map[7][i]].need_material_map[6][j];
                            if (work_bench_v6[wb_index].product == 1 || (work_bench_v6[wb_index].left_time <= 50 && work_bench_v6[wb_index].left_time != -1))
                            {
                                target_buy_index.push_back(work_bench_v6[wb_index].arr_idx);
                                target_sell_index.push_back(work_bench_v7[WB8[0].need_material_map[7][i]].arr_idx);
                                // break;
                            }
                        }
                    }
                }
                
            }


            // ===============================遍历工作台1需要那个材料===========================================================
            // 优先搬运缺一个的
            for (int j = 0; j < max_wb_num; j++)
            {
                int wb_index;
                if(j < WB5.size())
                {
                    wb_index = WB1[0].need_material_map[5][j];
                    if ((work_bench_v5[wb_index].raw_material == 8) && work_bench_v5[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                        target_buy_index.push_back(WB1[0].wb.arr_idx);
                    }
                    // break;
                }
                if(j < WB4.size())
                {
                    wb_index = WB1[0].need_material_map[4][j];
                    if ((work_bench_v4[wb_index].raw_material == 4) && work_bench_v4[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                        target_buy_index.push_back(WB1[0].wb.arr_idx);
                        // break;
                    }
                }

                if(j < WB6.size())
                {
                    wb_index = WB2[0].need_material_map[6][j];
                    if ((work_bench_v6[wb_index].raw_material == 8) && work_bench_v6[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                        target_buy_index.push_back(WB2[0].wb.arr_idx);
                    }
                        // break;
                }

                if (j < WB6.size()){
                    wb_index = WB3[0].need_material_map[6][j];
                    if ((work_bench_v6[wb_index].raw_material == 4) && work_bench_v6[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                        target_buy_index.push_back(WB3[0].wb.arr_idx);
                    }
                }
    
                if(j < WB4.size())
                {
                    wb_index = WB2[0].need_material_map[4][j];
                    if ((work_bench_v4[wb_index].raw_material == 2) && work_bench_v4[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                        target_buy_index.push_back(WB2[0].wb.arr_idx);
                        // break;
                    }
                }
 
                if(j < WB5.size())
                {
                    wb_index = WB3[0].need_material_map[5][j];
                    if ((work_bench_v5[wb_index].raw_material == 2) && work_bench_v5[wb_index].left_time == -1)
                    {
                        target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                        target_buy_index.push_back(WB3[0].wb.arr_idx);
                    }
                }

            }

            // ===============================遍历工作台1需要那个材料===========================================================
            // 优先搬运缺一个的
            for (int j = 0; j < max_wb_num; j++)
            {
                int wb_index;

                if(j < WB6.size()){
                    wb_index = WB2[0].need_material_map[6][j];
                    if ((work_bench_v6[wb_index].raw_material == 0))
                    {
                        target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                        target_buy_index.push_back(WB2[0].wb.arr_idx);
                    }
                }

                if(j < WB5.size()){
                    wb_index = WB1[0].need_material_map[5][j];
                    if ((work_bench_v5[wb_index].raw_material == 0))
                    {
                        target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                        target_buy_index.push_back(WB1[0].wb.arr_idx);
                    }
                }
                if(j < WB4.size())
                {
                    wb_index = WB1[0].need_material_map[4][j];
                    if ((work_bench_v4[wb_index].raw_material == 0))
                    {
                        target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                        target_buy_index.push_back(WB1[0].wb.arr_idx);
                    }
                }


                if(j < WB4.size())
                {
                    wb_index = WB2[0].need_material_map[4][j];
                    if ((work_bench_v4[wb_index].raw_material == 0))
                    {
                        target_sell_index.push_back(work_bench_v4[wb_index].arr_idx);
                        target_buy_index.push_back(WB2[0].wb.arr_idx);
                    }
                }

                if(j < WB5.size())
                {

                    wb_index = WB3[0].need_material_map[5][j];
                    if ((work_bench_v5[wb_index].raw_material == 0) )
                    {
                        target_sell_index.push_back(work_bench_v5[wb_index].arr_idx);
                        target_buy_index.push_back(WB3[0].wb.arr_idx);
                        // break;
                    }
                }

                if(j < WB6.size()){
                    wb_index = WB3[0].need_material_map[6][j];
                    if ((work_bench_v6[wb_index].raw_material == 0) )
                    {
                        target_sell_index.push_back(work_bench_v6[wb_index].arr_idx);
                        target_buy_index.push_back(WB3[0].wb.arr_idx);
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

                    if (((target_buy_index[workidx] == robot_array[0].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[0].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[0].sell_pos))
                    {
                        
                        continue;
                    }
                    else if (((target_buy_index[workidx] == robot_array[1].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[1].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[1].sell_pos))
                    {
                        
                        continue;
                    }
                    else if (((target_buy_index[workidx] == robot_array[2].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[2].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[2].sell_pos))
                    {
                        
                        continue;
                    }
                    else if (((target_buy_index[workidx] == robot_array[3].Buy_pos && work_bench_v[target_buy_index[workidx]].id !=1 && work_bench_v[target_buy_index[workidx]].id !=2 && work_bench_v[target_buy_index[workidx]].id !=3) || (work_bench_v[target_buy_index[workidx]].id == work_bench_v[robot_array[3].Buy_pos].id)) && (target_sell_index[workidx] == robot_array[3].sell_pos))
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
                    // else if((target_sell_index[workidx] == robot_array[0].sell_pos)&&work_bench_v[target_buy_index[workidx]].id !=8)
                    // {
                    //     continue;
                    // }
                    // else if((target_sell_index[workidx] == robot_array[1].sell_pos)&&work_bench_v[target_buy_index[workidx]].id !=8)
                    // {
                    //     continue;
                    // }
                    // else if((target_sell_index[workidx] == robot_array[2].sell_pos)&&work_bench_v[target_buy_index[workidx]].id !=8)
                    // {
                    //     continue;
                    // }
                    // else if((target_sell_index[workidx] == robot_array[3].sell_pos)&&work_bench_v[target_buy_index[workidx]].id !=8)
                    // {
                    //     continue;
                    // }
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
                }
                else if (robot_array[i].flag == 2)
                {

                    robot_array[i].robotToSell(work_bench_v[sell_idx], sell_idx);
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