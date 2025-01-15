#ifndef MOTIONPLATFORM_H
#define MOTIONPLATFORM_H

/**
 * @file MotionPlatform.h
 * @brief 提供运动平台控制功能。
 *
 * 本文件定义了MotionPlatform类，该类封装了NTControl库接口，
 * 主要用于配置、打开和关闭连接到运动平台的串口，并执行各种运动控制命令，
 * 包括移动到绝对或相对位置、步进移动、连续移动、扫描移动等操作，以及获取电压和当前位置信息。
 *
 * @author 崔浩然 <204747508@qq.com>
 * @version 1.0
 * @date 2024-12-30
 * @copyright Copyright (c) 2024 Dalian University of Technology. All rights reserved.
 */
#include <QObject>
#include <QTimer>
#include <QThread>
#include "NTControl.h"

// 定义通道索引常量
#define CH1 0 ///< 第一通道索引
#define CH2 1 ///< 第二通道索引
#define CH3 2 ///< 第三通道索引

#define TIMERMODE_MOTION
/**
 * @class MotionPlatform
 * @brief 提供运动平台控制功能。
 */
class MotionPlatform : public QObject {
    Q_OBJECT

public:
    explicit MotionPlatform(const char* ID, NT_INDEX channelIndex);
    ~MotionPlatform();

public:
    bool connect();
    bool disConnect();
    bool gotoPositionAbsolute(signed int position);
    bool gotoPositionRelative(signed int diff);
    bool getPosition(int &position);
    bool setPosition();
    bool setVelocity(signed int velocity, signed int target = 5000000);
    bool stop();
    bool stepMove(signed int steps, unsigned int amplitude, unsigned int frequency);
    bool continueMove(bool dir, unsigned int amplitude, unsigned int frequency);
    bool scanMoveAbsolute(unsigned int target, unsigned int time = 100);
    bool scanMoveRelative(signed int target, unsigned int time = 100);
    bool getVoltage(unsigned int &voltage);
    bool getMotionInfo();
    bool getsta(NT_STATUS &sta);
    bool findReference();
    bool findSystem();

private slots:
    void update();

private:
    NT_STATUS error_; ///< 最近一次操作的错误状态。
    const char* deviceID_; ///< 设备ID。
    NT_INDEX ntHandle_; ///< 用于与NTControl库交互的句柄。
    NT_INDEX channelIndex_; ///< 运动平台对应的通道索引。
    bool isOpen_;///< 当前连接状态。
    unsigned int motionSta_;
    int position_;
    unsigned int voltage_;


    QTimer *updateTimer_;

};

#endif // MOTIONPLATFORM_H
