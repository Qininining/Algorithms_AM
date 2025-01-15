#ifndef SCANNER_H
#define SCANNER_H

#include "SCANControl.h"
#include <QObject>
#include <QThread>
#include "PidController.h"

class Scanner : public QObject
{
    Q_OBJECT

public:
    explicit Scanner(const char* ID);
    ~Scanner();

    enum MoveMode {
        NoMove = 0,      // 不移动
        PositionMode,    // 位置模式
        VelocityMode     // 速度模式
    };


public:
    bool connect();
    bool disConnect();

    // bool gotoPositionAbsolute(signed int position);
    // bool gotoPositionRelative(signed int diff);

    bool getPosition(double &position, unsigned int adc);
    bool getVelocity(double &velocity);

    bool scanMoveAbsolute(unsigned int target, unsigned int scanStep = 1000, unsigned int scanDelay = 10);
    bool scanMoveRelative(int diff, unsigned int scanStep = 1, unsigned int scanDelay = 10);

    bool getVoltage(unsigned int &voltage);

    bool getMotionInfo();
    bool getsta(SCAN_STATUS &sta);

    bool findSystem();

    // bool openCloseLoopControll();
    // bool closeCloseLoopControll();
    bool gotoPosition(int position);
    bool gotoPositionRelative(int diff);

    bool switchMode(MoveMode MoveMode);

    bool setVelocity(int velocity);

// signals:
//     void scanMove(int voltage);

public slots:
    void scanMove(double pidData);
    void update();


private:
    SCAN_STATUS error_; ///< 最近一次操作的错误状态。
    const char* deviceID_; ///< 设备ID。
    SCAN_INDEX ntHandle_; ///< 用于与NTControl库交互的句柄。
    SCAN_INDEX channelIndex_; ///< 运动平台对应的通道索引。
    bool isOpen_;///< 当前连接状态。
    unsigned int motionSta_;
    double position_;
    double velocity_;
    double temp_velocity_;
    unsigned int voltage_;

    QTimer *updateTimer_;

    PidController* PidPosition_;
    PidController* PidVelocity_;

    QThread* PIDupdateThread_;

    QVector<double> positionBuffer_; ///< 用于存储位置的循环缓冲区。
    int bufferIndex_; ///< 当前写入缓冲区的位置索引。
    const int bufferSize_; ///< 缓冲区大小，可以调整为适当值。
    MoveMode scanMoveMode; ///< 运动模式，1代表位置PID控制，2代表速度Pid控制


};

#endif // SCANNER_H
