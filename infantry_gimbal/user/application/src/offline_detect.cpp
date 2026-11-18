#include "offline_detect.h"
#include "bsp_dwt.h"
#include "tim.h"

namespace {
    /**
     * @brief 离线报警分组
     * @note 同组设备使用同一类音高与节奏，不同设备通过停顿长度区分
     */
    enum class AlarmGroup {
        None = 0,
        GroupA,
        GroupB,
        GroupC,
        GroupD
    };

    /* 检测任务启动成功后的标志音，用于提示离线检测模块已开始工作 */
    const BeepNote_t k_detect_init_notes[] = {
        { 784, 60 },
        { 0, 30 },
        { 1047, 80 },
        { 0, 30 },
        { 1319, 120 }
    };

    /* A组报警：板间通信、Yaw、拨盘，统一用偏高频双响 */
    const BeepNote_t k_group_a_short[] = {
        { 988, 80 },
        { 0, 80 },
        { 988, 80 },
        { 0, 420 }
    };

    const BeepNote_t k_group_a_mid[] = {
        { 988, 80 },
        { 0, 180 },
        { 988, 80 },
        { 0, 320 }
    };

    const BeepNote_t k_group_a_long[] = {
        { 988, 80 },
        { 0, 320 },
        { 988, 80 },
        { 0, 180 }
    };

    /* B组报警：Pitch、Yaw、摩擦轮，统一用中频双响 */
    const BeepNote_t k_group_b_short[] = {
        { 698, 110 },
        { 0, 80 },
        { 698, 110 },
        { 0, 420 }
    };

    const BeepNote_t k_group_b_long[] = {
        { 698, 110 },
        { 0, 260 },
        { 698, 110 },
        { 0, 240 }
    };

    const BeepNote_t k_group_b_mid[] = {
        { 698, 110 },
        { 0, 180 },
        { 698, 110 },
        { 0, 320 }
    };

    /* C组报警：遥控器、裁判系统、视觉，统一用偏低频双响 */
    const BeepNote_t k_group_c_short[] = {
        { 523, 120 },
        { 0, 120 },
        { 523, 120 },
        { 0, 360 }
    };

    const BeepNote_t k_group_c_long[] = {
        { 523, 120 },
        { 0, 280 },
        { 523, 120 },
        { 0, 200 }
    };

    /* D组报警：INS单独一类，优先级最高 */
    const BeepNote_t k_group_d_alarm[] = {
        { 440, 160 },
        { 0, 120 },
        { 440, 160 },
        { 0, 260 }
    };

    /* 底盘侧在线标志报警，统一放在C组并通过停顿区分 */
    const BeepNote_t k_group_c_mid[] = {
        { 523, 120 },
        { 0, 200 },
        { 523, 120 },
        { 0, 280 }
    };

    /**
     * @brief 根据设备类型返回报警分组
     * @note 分组只描述音效类别，不描述组内具体节奏
     */
    AlarmGroup GetAlarmGroup(DeviceId id) {
        switch (id) {
        case DeviceId::DEV_BOARD_COMM:
        case DeviceId::DEV_PITCH_FOLLOW_MOTOR:
        case DeviceId::DEV_TRIGGER:
            return AlarmGroup::GroupC;
        case DeviceId::DEV_YAW_MOTOR:
        case DeviceId::DEV_PITCH_MOTOR:
        case DeviceId::DEV_FRIC_L:
        case DeviceId::DEV_FRIC_R:
            return AlarmGroup::GroupB;
        case DeviceId::DEV_REMOTE:
            return AlarmGroup::None;
        case DeviceId::DEV_REFEREE:
        case DeviceId::DEV_VISION:
        case DeviceId::DEV_CAP:
        case DeviceId::DEV_WHEEL:
        case DeviceId::DEV_RUDDER:
        case DeviceId::DEV_CHASSIS_REFEREE:
            return AlarmGroup::GroupA;
        case DeviceId::DEV_INS:
            return AlarmGroup::GroupD;
        default:
            return AlarmGroup::None;
        }
    }

    /**
     * @brief 获取指定设备的报警音符表
     * @note 同组设备共用同一类音效，但通过停顿长度区分具体设备
     * @param id 设备id
     * @param note_num 返回音符数量
     * @retval 非空 音符表首地址
     * @retval 空指针 该设备未配置报警音效
     */
    const BeepNote_t* GetAlarmNotes(DeviceId id, uint16_t* note_num) {
        switch (id) {
        case DeviceId::DEV_BOARD_COMM:
            *note_num = sizeof(k_group_c_short) / sizeof(k_group_c_short[0]);
            return k_group_c_short;
        case DeviceId::DEV_PITCH_FOLLOW_MOTOR:
            *note_num = sizeof(k_group_c_mid) / sizeof(k_group_c_mid[0]);
            return k_group_c_mid;
        case DeviceId::DEV_TRIGGER:
            *note_num = sizeof(k_group_c_long) / sizeof(k_group_c_long[0]);
            return k_group_c_long;
        case DeviceId::DEV_PITCH_MOTOR:
            *note_num = sizeof(k_group_b_short) / sizeof(k_group_b_short[0]);
            return k_group_b_short;
        case DeviceId::DEV_YAW_MOTOR:
            *note_num = sizeof(k_group_b_mid) / sizeof(k_group_b_mid[0]);
            return k_group_b_mid;
        case DeviceId::DEV_FRIC_L:
        case DeviceId::DEV_FRIC_R:
            *note_num = sizeof(k_group_b_long) / sizeof(k_group_b_long[0]);
            return k_group_b_long;
        case DeviceId::DEV_REMOTE:
            *note_num = 0;
            return nullptr;
        case DeviceId::DEV_REFEREE:
        case DeviceId::DEV_VISION:
            *note_num = sizeof(k_group_a_long) / sizeof(k_group_a_long[0]);
            return k_group_a_long;
        case DeviceId::DEV_CAP:
            *note_num = sizeof(k_group_a_short) / sizeof(k_group_a_short[0]);
            return k_group_a_short;
        case DeviceId::DEV_WHEEL:
            *note_num = sizeof(k_group_a_mid) / sizeof(k_group_a_mid[0]);
            return k_group_a_mid;
        case DeviceId::DEV_RUDDER:
        case DeviceId::DEV_CHASSIS_REFEREE:
            *note_num = sizeof(k_group_c_long) / sizeof(k_group_c_long[0]);
            return k_group_c_long;
        case DeviceId::DEV_INS:
            *note_num = sizeof(k_group_d_alarm) / sizeof(k_group_d_alarm[0]);
            return k_group_d_alarm;
        default:
            *note_num = 0;
            return nullptr;
        }
    }

    /**
     * @brief 获取当前最高优先级的离线设备
     * @note 同时多设备离线时，只播放优先级最高设备的报警，避免蜂鸣器混响
     */
    DeviceId GetHighestPriorityOfflineDevice(const Offline_Detect& detect) {
        static const DeviceId k_priority_list[] = {
            DeviceId::DEV_INS,
            DeviceId::DEV_BOARD_COMM,
            DeviceId::DEV_CHASSIS_REFEREE,
            // DeviceId::DEV_YAW_MOTOR,
            DeviceId::DEV_PITCH_FOLLOW_MOTOR,
            DeviceId::DEV_PITCH_MOTOR,
            DeviceId::DEV_RUDDER,
            DeviceId::DEV_WHEEL,
            DeviceId::DEV_FRIC_L,
            DeviceId::DEV_FRIC_R,
            DeviceId::DEV_TRIGGER,
            DeviceId::DEV_CAP,
            DeviceId::DEV_REFEREE,
            DeviceId::DEV_VISION,
        };

        for (DeviceId id : k_priority_list) {
            if (detect.GetState(id) == offline_state::Offline) {
                return id;
            }
        }

        return static_cast<DeviceId>(0xFF);
    }
}  // namespace

Offline_Detect offline_detect;

/**
 * @brief 检测模块初始化
 * @note 这里只初始化蜂鸣器并播放一次启动提示音
 */
void Detect_Init() {
    offline_detect.GetBeep().Init(&htim4, TIM_CHANNEL_3);
    offline_detect.GetBeep().Play(k_detect_init_notes,
                                  sizeof(k_detect_init_notes) / sizeof(k_detect_init_notes[0]));
}

/**
 * @brief 检测离线任务
 * @note 执行顺序为：首次启动提示 -> 刷新状态 -> 推进当前音效 -> 根据最高优先级离线设备报警
 */
void Detect_task() {
    /* 记录上一次报警的设备与分组，便于后续扩展“首次掉线才响一次”等策略 */
    static DeviceId last_alarm_device = static_cast<DeviceId>(0xFF);
    static AlarmGroup last_alarm_group = AlarmGroup::None;

    /* 先统一刷新在线状态，再推进蜂鸣器当前播放进度 */
    offline_detect.Update();
    offline_detect.GetBeep().Task();

    /* 当前还有音效在播放时，不抢占，等待本轮音效自然播完 */
    if (offline_detect.GetBeep().IsBusy()) {
        return;
    }

    /* 无离线设备时清空报警记录并保持静默 */
    DeviceId alarm_device = GetHighestPriorityOfflineDevice(offline_detect);
    if (alarm_device == static_cast<DeviceId>(0xFF)) {
        last_alarm_device = static_cast<DeviceId>(0xFF);
        last_alarm_group = AlarmGroup::None;
        return;
    }

    uint16_t note_num = 0;
    const BeepNote_t* notes = GetAlarmNotes(alarm_device, &note_num);
    if (notes == nullptr || note_num == 0) {
        return;
    }

    /* 这里保留了设备和分组变化记录，后续可扩展更复杂的报警切换策略 */
    AlarmGroup group = GetAlarmGroup(alarm_device);
    if (alarm_device != last_alarm_device || group != last_alarm_group) {
        last_alarm_device = alarm_device;
        last_alarm_group = group;
    }

    /* 播放当前最高优先级离线设备对应的分类报警音效 */
    offline_detect.GetBeep().Play(notes, note_num);
}

/**
 * @brief 使能对应设备的检测
 * @note 初始化后该设备才会参与Update中的在线/离线判断
 * @param id 设备id，枚举类DeviceId
 * @param timeout 判离线超时时间，单位ms
 */
void Offline_Detect::Init(DeviceId id, uint16_t timeout) {
    EnableList[static_cast<uint8_t>(id)] = true;
    /* 初始化先在线，避免任务启动立即判离线 */
    offline_item[static_cast<uint8_t>(id)].state = offline_state::Online;
    offline_item[static_cast<uint8_t>(id)].last_online_ms = DWT_GetTimeline_ms();
    offline_item[static_cast<uint8_t>(id)].timeout_ms = timeout;
}

/**
 * @brief 根据当前时间统一刷新全部设备状态
 * @note 若当前时刻与上次喂狗时间的间隔大于timeout_ms，则置为Offline
 */
void Offline_Detect::Update(void) {
    float now = DWT_GetTimeline_ms();
    for (uint8_t i = 0; i < DevNum; i++) {
        if (EnableList[i] == true) {
            /* 超时未喂狗则判离线，否则判在线 */
            if ((now - offline_item[i].last_online_ms) > offline_item[i].timeout_ms) {
                offline_item[i].state = offline_state::Offline;
            }
            else {
                offline_item[i].state = offline_state::Online;
            }
        }
        else {
            /* 未启用检测的设备保持Ignore状态，不参与业务判断 */
            offline_item[i].state = offline_state::Ignore;
        }
    }
}
/**
 * @brief 喂狗函数
 * @note 应在对应模块收到“有效数据”时调用，而不是仅收到中断时调用
 * @param id 设备id
 */
void Offline_Detect::Feed(DeviceId id) {
    offline_item[static_cast<uint8_t>(id)].last_online_ms = DWT_GetTimeline_ms();
    offline_item[static_cast<uint8_t>(id)].state = offline_state::Online;
}

void Offline_Detect::UpdateFromFlag(DeviceId id, bool is_online) {
    if (!EnableList[static_cast<uint8_t>(id)]) {
        return;
    }

    if (is_online) {
        Feed(id);
        return;
    }

    offline_item[static_cast<uint8_t>(id)].state = offline_state::Offline;
    offline_item[static_cast<uint8_t>(id)].last_online_ms =
        DWT_GetTimeline_ms() - offline_item[static_cast<uint8_t>(id)].timeout_ms - 1.0f;
}
/**
 * @brief 获取设备在线状态
 * @note 对未启用检测的设备，默认按在线处理
 * @param id 设备id
 * @retval true 在线或未启用检测
 * @retval false 已启用检测且当前离线
 */
bool Offline_Detect::IsOnline(DeviceId id) {
    if (offline_item[static_cast<uint8_t>(id)].state != offline_state::Ignore)
        return offline_item[static_cast<uint8_t>(id)].state == offline_state::Online ? true : false;
    else
        return true;
}
