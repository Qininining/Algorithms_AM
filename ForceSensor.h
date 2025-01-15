#ifndef FORCESENSOR_H
#define FORCESENSOR_H

/**
 * @file ForceSensor.h
 * @brief 提供力传感器数据读取和处理的功能。
 *
 * 本文件定义了ForceSensor类，该类继承自SerialCommon，并封装了串口通信接口，
 * 主要用于配置、打开和关闭串口，以及从力传感器读取数据和进行基本的数据处理。
 *
 * @author 崔浩然 <204747508@qq.com>
 * @version 1.0
 * @date 2024-12-29
 * @copyright Copyright (c) 2024 Dalian University of Technology. All rights reserved.
 */

#include "SerialCommon.h"
#include <QThread>

class ForceSensor : public SerialCommon {

    Q_OBJECT

public:
    explicit ForceSensor(const QString &portName, double sensitivityCH1 = 1, double sensitivityCH2 = 1);
    ~ForceSensor();

public:
    bool connect(const QString &portName,
                 qint32 baudRate = 921600,
                 QSerialPort::DataBits dataBits = QSerialPort::Data8,
                 QSerialPort::Parity parity = QSerialPort::NoParity,
                 QSerialPort::StopBits stopBits = QSerialPort::OneStop);
    bool connect();
    bool disConnect();
    bool readCurrentForce(bool isRelative, int &force);
    bool readFirstChannel_A(unsigned int &force);
    bool readFirstChannel_R(int &force);
    bool readSecondChannel_A(unsigned int &force);
    bool readSecondChannel_R(int &force);
    bool setReferenceZero(int num = 0);
    bool setSensitivity(double sensitivity, int channel = 1);
    bool getSensitivity(double &sensitivity, int channel = 1);

private slots:
    void readData() override;

private:
    const QString portName_; ///< 串口名称。
    double sensitivityCH1_;     ///< 力传感器的灵敏度。
    double sensitivityCH2_;     ///< 力传感器的灵敏度。
    unsigned int referenceZeroCH1_;   ///< 力传感器的参考零点。
    unsigned int referenceZeroCH2_;   ///< 力传感器的参考零点。
    int firstChannelforce_;           ///< 最近一次读取的力值。
    int secondChannelforce_;           ///< 最近一次读取的力值。
    int getForceFromBuffer(const QByteArray data, int &channel);
    int firstDataProcess(int forcedata);
    int secondDataProcess(int forcedata);
};

#endif // FORCESENSOR_H
