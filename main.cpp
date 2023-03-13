#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace std;

#define line_size 1024

//---------------------------------struct---------------------------------
// 定义工作台结构体
typedef struct
{
    int id;           // 工作台id
    float x;          // 工作台x坐标
    float y;          // 工作台y坐标
    int left_time;    // 剩余生产时间
    int raw_material; // 原材料格状态，二进制位表描述，110000=>拥有物品4, 5
    int product;      // 产品格状态,0=>无，1=>有
} WorkBench;

// 定义机器人结构体
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

    // 定义每一帧需要的变量
    int frameID;

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
        while (ROBOTNUM >= 1)
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
            getchar();
            robot_v.push_back(rb);
            ROBOTNUM--;
        }

        char line[line_size]; // 读取最后一行OK
        fgets(line, sizeof line, stdin);

        // 得到当前帧ID
        printf("%d\n", frameID);

        int lineSpeed = 3;
        double angleSpeed = 1.5;
        // 每行一个指令
        for (int robotId = 0; robotId < 4; robotId++)
        {
            printf("forward %d %d\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        // 表示当前帧输出完毕
        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;
}
