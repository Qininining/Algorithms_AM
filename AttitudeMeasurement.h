#ifndef ATTITUDEMEASUREMENT_H
#define ATTITUDEMEASUREMENT_H

#include <QObject>
#include <QTimer>
#include <QMutex>
#include <QMutexLocker>
#include <Eigen/Dense>
// #include <unsupported/Eigen/FFT>
#include "MotionPlatform.h"
#include "Scanner.h"
#include "ForceSensor.h"

class AttitudeMeasurement : public QObject
{
    Q_OBJECT

public:
    AttitudeMeasurement(const char* platformID
                        , const char* scannerID_
                        , const QString &portName
                        , double sensitivityCH1 = 1
                        , double sensitivityCH2 = 1);
    ~AttitudeMeasurement();

public:
    struct MeasurementData {
        std::array<int, 3> position;
        std::array<unsigned int, 3> status;
        int force;
        int scanPositon;

        MeasurementData(
            const std::array<int, 3>& pos = {{0, 0, 0}},
            const std::array<unsigned int, 3>& stat = {{0, 0, 0}},
            int frc = 0,
            int scanpos = 0
            )
            : position(pos), status(stat), force(frc), scanPositon(scanpos){}
    };


public:
    bool initialize();           // 初始化设备
    bool shutdown();             // 关闭并释放资源

    // 运动控制
    bool moveTaskPositions(const Eigen::Vector3d& target_positions);
    bool moveTaskVelocities(const Eigen::Vector3d& target_velocities);
    bool moveJointPositions(const Eigen::Vector3d& target_positions);
    bool moveJointVelocities(const Eigen::Vector3d& target_velocities);

    bool setMicroPosition(signed int position);
    bool setMicroPositionRelative(signed int diff);
    bool setMicroVelocity(signed int velocity);
    bool setMicroVoltage(unsigned int Voltage);
    bool setMicroVoltageRelative(int diff_Voltage);

    // 获取最新的 MeasurementData
    bool getData(MeasurementData &latestData);
    bool getDataBuffer(QVector<MeasurementData> &amDataBuffer);

    bool startMeasurement(Eigen::Vector3d& position);
    bool stopMeasurement();

    // 状态设置
    bool setMoveVelocities(const Eigen::VectorXd& velocities);
    // bool setMoveMode(int flag);  // 设置运动模式：宏动、微动等
    // bool setSafetyLimits(const SafetyLimits& limits);
    bool setProbeForce(double force);
    bool setScanResolution(double resolution);

    // 状态获取
    bool getJointPositions(Eigen::VectorXd& positions);
    bool getJointVelocities(Eigen::VectorXd& velocities);
    bool getCurrentProbePosition(Eigen::Vector3d& position);

    bool getMicroVelocity(int &velocity);
    bool getMicroVoltage(unsigned int &Voltage);


    // bool openScanCloseLoop();
    // bool closeScanCloseLoop();

private slots:
    void update();

private:
    MotionPlatform *platformX_;
    MotionPlatform *platformY_;
    MotionPlatform *platformZ_;

    Scanner * scanner_;

    ForceSensor *forceSensor_;

    const char* platformID_; //移动平台控制器设备ID。
    const char* scannerID_; //移动平台控制器设备ID。
    const QString portName_; //串口名称。
    double sensitivityCH1_;     //力传感器的灵敏度。
    double sensitivityCH2_;     //力传感器的灵敏度。

    QMutex mutex_; // 线程同步用的互斥锁
    QThread *platformThread_;
    QThread *forceThread_;
    QThread *scanThread_;

    QTimer *updateTimer_;

    QVector<MeasurementData> MeasurementDataBuffer_;
    int bufferCapacity_;     //缓冲区容量。
    int currentIndex_;      //当前写入位置。
    int count_;             //已存入的数据数量。
};

#endif // ATTITUDEMEASUREMENT_H
