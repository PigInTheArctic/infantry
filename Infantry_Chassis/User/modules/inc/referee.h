/**
 *******************************************************************************
 * @file      : referee.h
 * @brief     : RoboMaster 2026 裁判系统串口协议（V2.0.0，2026-06-26）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-06-26      origincy        1. 按 2026 V2.0.0 协议重写
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREE_H_
#define __REFEREE_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bsp_uart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#define FALSE 0
#define TRUE 1

#define JUDGE_DATA_ERROR 0
#define JUDGE_DATA_CORRECT 1

#define LEN_HEADER 5  // 帧头长
#define LEN_CMDID 2   // 命令码长度
#define LEN_TAIL 2    // 帧尾 CRC16

// 起始字节，协议固定为 0xA5
#define JUDGE_FRAME_HEADER (0xA5)

#pragma pack(push, 1)

/***************裁判系统数据接收********************/

typedef enum {
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,
} JudgeFrameOffset;

// 5 字节帧头，偏移位置
typedef enum {
    SOF = 0,          // 起始位
    DATA_LENGTH = 1,  // 帧内数据长度（uint16，小端）
    SEQ = 3,          // 包序号
    CRC8 = 4          // CRC8
} FrameHeaderOffset;

// 命令码 ID，按 2026 V2.0.0 表 1-5
typedef enum {
    ID_game_state = 0x0001,             // 比赛状态数据
    ID_game_result = 0x0002,            // 比赛结果数据
    ID_game_robot_survivors = 0x0003,   // 机器人血量数据
    ID_event_data = 0x0101,             // 场地事件数据
    ID_referee_warning = 0x0104,        // 裁判警告数据
    ID_dart_info = 0x0105,              // 飞镖发射相关数据
    ID_game_robot_state = 0x0201,       // 机器人性能体系数据
    ID_power_heat_data = 0x0202,        // 实时底盘缓冲能量和射击热量数据
    ID_game_robot_pos = 0x0203,         // 机器人位置数据
    ID_buff_musk = 0x0204,              // 机器人增益和底盘能量数据
    ID_robot_hurt = 0x0206,             // 伤害状态数据
    ID_shoot_data = 0x0207,             // 实时射击数据
    ID_projectile_allowance = 0x0208,   // 允许发弹量与剩余金币数
    ID_rfid_status = 0x0209,            // 机器人 RFID 模块状态
    ID_dart_client_cmd = 0x020A,        // 飞镖选手端指令数据
    ID_ground_robot_position = 0x020B,  // 地面机器人位置数据
    ID_radar_mark_data = 0x020C,        // 雷达标记进度数据
    ID_sentry_info = 0x020D,            // 哨兵自主决策信息同步
    ID_radar_info = 0x020E,             // 雷达自主决策信息同步
    ID_student_interactive = 0x0301,    // 机器人交互数据
    ID_custom_robot_data = 0x0302,      // 自定义控制器与机器人交互数据
    ID_map_command = 0x0303,            // 选手端小地图交互数据
    ID_map_robot_data = 0x0305,         // 选手端小地图接收雷达数据
    ID_custom_client_data = 0x0306,     // 自定义控制器与选手端交互数据
    ID_map_data = 0x0307,               // 选手端小地图接收路径数据
    ID_custom_info = 0x0308,            // 选手端小地图接收机器人数据
    ID_robot_custom_data = 0x0309,      // 机器人发送给自定义控制器的数据
    ID_robot_custom_data_2 = 0x0310,    // 机器人发送给自定义客户端的数据
} CmdID;

// 命令码数据段长度，按 2026 V2.0.0 表 1-5
typedef enum {
    LEN_game_state = 11,                 // 0x0001
    LEN_game_result = 1,                 // 0x0002
    LEN_game_robot_HP = 20,              // 0x0003
    LEN_event_data = 4,                  // 0x0101
    LEN_referee_warning = 3,             // 0x0104
    LEN_dart_info = 3,                   // 0x0105
    LEN_game_robot_state = 17,           // 0x0201
    LEN_power_heat_data = 14,            // 0x0202
    LEN_game_robot_pos = 16,             // 0x0203（有效 12 字节 + 4 字节保留）
    LEN_buff_musk = 8,                   // 0x0204
    LEN_robot_hurt = 1,                  // 0x0206
    LEN_shoot_data = 7,                  // 0x0207
    LEN_projectile_allowance = 8,        // 0x0208
    LEN_rfid_status = 5,                 // 0x0209
    LEN_dart_client_cmd = 6,             // 0x020A
    LEN_ground_robot_position = 40,      // 0x020B
    LEN_radar_mark_data = 2,             // 0x020C
    LEN_sentry_info = 14,                // 0x020D
    LEN_radar_info = 1,                  // 0x020E
    LEN_student_interactive = 118,       // 0x0301
    LEN_custom_robot_data = 30,          // 0x0302
    LEN_map_command = 15,                // 0x0303（有效 12 字节 + 3 字节保留）
    LEN_map_robot_data = 48,             // 0x0305
    LEN_custom_client_data = 8,          // 0x0306
    LEN_map_data = 103,                  // 0x0307（表 1-37 字段偏移合计 105，表 1-5 为 103；本模块不解析）
    LEN_custom_info = 34,                // 0x0308
    LEN_robot_custom_data = 30,          // 0x0309
} JudgeDataLength;

/* 5 字节帧头 */
typedef struct {
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

/* ID: 0x0001  Byte: 11  比赛状态数据 */
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

/* ID: 0x0002  Byte: 1   比赛结果数据 */
typedef struct {
    uint8_t winner;
} game_result_t;

/* ID: 0x0003  Byte: 20  机器人血量数据 */
typedef struct {
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    int16_t damage_difference;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;
} game_robot_HP_t;

/* ID: 0x0101  Byte: 4   场地事件数据 */
typedef struct {
    uint32_t event_data;
} event_data_t;

/* ID: 0x0104  Byte: 3   裁判警告数据 */
typedef struct {
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

/* ID: 0x0105  Byte: 3   飞镖发射相关数据 */
typedef struct {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

/* ID: 0x0201  Byte: 17  机器人性能体系数据 */
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    float bullet_speed_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/* ID: 0x0202  Byte: 14  实时底盘缓冲能量和射击热量数据 */
typedef struct {
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/* ID: 0x0203  Byte: 16  机器人位置数据（12 字节有效 + 4 字节保留） */
typedef struct {
    float x;
    float y;
    float angle;
    uint32_t reserved;
} robot_pos_t;

/* ID: 0x0204  Byte: 8   机器人增益和底盘能量数据 */
typedef struct {
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

/* ID: 0x0206  Byte: 1   伤害状态数据 */
typedef struct {
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/* ID: 0x0207  Byte: 7   实时射击数据 */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

/* ID: 0x0208  Byte: 8   允许发弹量与剩余金币数 */
typedef struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* ID: 0x0209  Byte: 5   机器人 RFID 模块状态 */
typedef struct {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} rfid_status_t;

/* ID: 0x020A  Byte: 6   飞镖选手端指令数据 */
typedef struct {
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/* ID: 0x020B  Byte: 40  地面机器人位置数据 */
typedef struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
} ground_robot_position_t;

/* ID: 0x020C  Byte: 2   雷达标记进度数据 */
typedef struct {
    uint16_t mark_progress;
} radar_mark_data_t;

/* ID: 0x020D  Byte: 14  哨兵自主决策信息同步 */
typedef struct {
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3;
} sentry_info_t;

/* ID: 0x020E  Byte: 1   雷达自主决策信息同步 */
typedef struct {
    uint8_t radar_info;
} radar_info_t;

/* ID: 0x0301  Byte: 118 机器人交互数据（6 字节头 + 112 字节内容） */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t data[112];
} student_interactive_t;

/* 0x0301 子内容：0x0100 删除图层 */
typedef struct {
    uint8_t delete_type;  // 0：空操作，1：删除图层，2：删除所有
    uint8_t layer;        // 图层数 0~9
} interaction_layer_delete_t;

/* 0x0301 子内容：0x0101/0x0102/0x0103/0x0104 单个图形 */
typedef struct {
    uint8_t figure_name[3];     // 图形名，作为删除/修改的索引
    uint32_t operate_type : 3;  // 0 空操作，1 增加，2 修改，3 删除
    uint32_t figure_type : 3;   // 0 直线，1 矩形，2 正圆，3 椭圆，4 圆弧，5 浮点，6 整型，7 字符
    uint32_t layer : 4;         // 图层 0~9
    uint32_t color : 4;         // 0 红/蓝，1 黄，2 绿，3 橙，4 紫红，5 粉，6 青，7 黑，8 白
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;        // 线宽
    uint32_t start_x : 11;      // 起点/圆心 x
    uint32_t start_y : 11;      // 起点/圆心 y
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;

typedef struct {
    interaction_figure_t figure[2];
} interaction_figure_2_t;

typedef struct {
    interaction_figure_t figure[5];
} interaction_figure_5_t;

typedef struct {
    interaction_figure_t figure[7];
} interaction_figure_7_t;

/* 0x0301 子内容：0x0110 字符图形（15 字节配置 + 30 字节字符） */
typedef struct {
    interaction_figure_t figure;
    uint8_t data[30];
} interaction_character_t;

/* 0x0301 子内容：0x0120 哨兵自主决策指令 */
typedef struct {
    uint32_t confirm_revive : 1;           // bit0：确认复活
    uint32_t confirm_instant_revive : 1;   // bit1：确认兑换立即复活
    uint32_t exchange_projectile : 11;     // bit2-12：将要兑换的发弹量值
    uint32_t remote_projectile_times : 4;  // bit13-16：远程兑换发弹量请求次数
    uint32_t remote_hp_times : 4;          // bit17-20：远程兑换血量请求次数
    uint32_t posture : 3;                  // bit21-23：1 进攻，2 防御，3 移动，4/5/6 强化
    uint32_t activate_energy : 1;          // bit24：确认能量机关进入正在激活状态
    uint32_t reserved : 7;                 // bit25-31
} sentry_cmd_t;

/* 0x0301 子内容：0x0121 雷达自主决策指令 */
typedef struct {
    uint8_t radar_cmd;      // 请求触发双倍易伤的次数，单调递增
    uint8_t password_cmd;   // 1：更新己方密钥，2：验证对方密钥
    uint8_t password[6];    // 密钥值，ASCII
} radar_cmd_t;

#pragma pack(pop)

class Referee {
public:
    UartInstance* p_instance_ = nullptr;

    void Update(uint8_t* _p_data, uint16_t len);
    void Init(UART_HandleTypeDef* _p_huart);
    static void UartCallbackEntry(void* context, uint8_t* data, uint16_t len);

    xFrameHeader FrameHeader{};
    uint16_t CmdID = 0;

    game_status_t game_state_{};                            // 0x0001
    game_result_t game_result_{};                           // 0x0002
    game_robot_HP_t game_robot_HP_{};                       // 0x0003
    event_data_t event_data_{};                             // 0x0101
    referee_warning_t referee_warning_{};                   // 0x0104
    dart_info_t dart_info_{};                               // 0x0105
    robot_status_t game_robot_state_{};                     // 0x0201
    power_heat_data_t power_heat_data_{};                   // 0x0202
    robot_pos_t game_robot_pos_{};                          // 0x0203
    buff_t buff_musk_{};                                    // 0x0204
    hurt_data_t robot_hurt_{};                              // 0x0206
    shoot_data_t shoot_data_{};                             // 0x0207
    projectile_allowance_t projectile_allowance{};          // 0x0208
    rfid_status_t rfid_status_{};                           // 0x0209
    dart_client_cmd_t dart_client_cmd_{};                   // 0x020A
    ground_robot_position_t ground_robot_position_{};       // 0x020B
    radar_mark_data_t radar_mark_data_{};                   // 0x020C
    sentry_info_t sentry_info_{};                           // 0x020D
    radar_info_t radar_info_{};                             // 0x020E
    student_interactive_t student_interactive_{};           // 0x0301
};

/* Exported variables --------------------------------------------------------*/
extern Referee referee;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __REFEREE_H_ */
