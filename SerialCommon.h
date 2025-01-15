#ifndef SERIALCOMMON_H
#define SERIALCOMMON_H

/**
 * @file SerialCommon.h
 * @brief 提供串口通信的公共功能。
 *
 * 本文件定义了SerialCommon类，该类封装了Qt提供的QSerialPort接口，
 * 用于配置、打开和关闭串口，以及进行数据的读写操作。
 *
 * @author 崔浩然 <204747508@qq.com>
 * @version 1.0
 * @date 2024-12-29
 * @copyright Copyright (c) 2024 Dalian University of Technology. All rights reserved.
 */

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QIODevice>
#include <QByteArray>

class SerialCommon : public QObject {

    Q_OBJECT

public:
    explicit SerialCommon();
    virtual ~SerialCommon();
    bool setBaudRate(qint32 baudRate);
    bool setDataBits(QSerialPort::DataBits dataBits);
    bool setParity(QSerialPort::Parity parity);
    bool setStopBits(QSerialPort::StopBits stopBits);
    bool setFlowControl(QSerialPort::FlowControl flowControl);
    bool open(const QString &portName,
              qint32 baudRate,
              QSerialPort::DataBits dataBits = QSerialPort::Data8,
              QSerialPort::Parity parity = QSerialPort::NoParity,
              QSerialPort::StopBits stopBits = QSerialPort::OneStop);
    bool close();
    bool isOpen() const;
    qint64 writeData(const QByteArray &data);

signals:
    void dataReceived(const QByteArray data);

protected slots:
    virtual void readData();

protected:
    QSerialPort *serial; ///< 管理串口连接的QSerialPort实例指针。
};

#endif // SERIALCOMMON_H
