#include "SerialCommon.h"
#include <QDebug>

SerialCommon::SerialCommon()
    : QObject(), serial(new QSerialPort()) {

    connect(serial, &QSerialPort::readyRead, this, &SerialCommon::readData);
}

SerialCommon::~SerialCommon() {
    if (serial->isOpen())
        serial->close();
    delete serial;
}






bool SerialCommon::setBaudRate(qint32 baudRate) {
    return serial->setBaudRate(baudRate);
}

bool SerialCommon::setDataBits(QSerialPort::DataBits dataBits) {
    return serial->setDataBits(dataBits);
}

bool SerialCommon::setParity(QSerialPort::Parity parity) {
    return serial->setParity(parity);
}

bool SerialCommon::setStopBits(QSerialPort::StopBits stopBits) {
    return serial->setStopBits(stopBits);
}

bool SerialCommon::setFlowControl(QSerialPort::FlowControl flowControl) {
    return serial->setFlowControl(flowControl);
}





bool SerialCommon::open(const QString &portName,
          qint32 baudRate,
          QSerialPort::DataBits dataBits,
          QSerialPort::Parity parity,
          QSerialPort::StopBits stopBits)
{
    serial->setBaudRate(baudRate);
    serial->setDataBits(dataBits);
    serial->setParity(parity);
    serial->setStopBits(stopBits);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    serial->setPortName(portName);
    if (!serial->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open port" << portName << ", error:" << serial->errorString();
        return false;
    }
    qDebug() << "Serial port" << portName << "opened successfully.";
    return true;

}

bool SerialCommon::close() {
    if (serial->isOpen()) {
        serial->close();
        qDebug() << "Serial port closed.";
        return 1;
    }
    return 0;
}

bool SerialCommon::isOpen() const {
    return serial->isOpen();
}

void SerialCommon::readData() {
    QByteArray data = serial->readAll();
    // qDebug() << "data" << data;
    emit dataReceived(data);  // 发出信号，通知有新数据到达
}

qint64 SerialCommon::writeData(const QByteArray &data) {
    if (!serial->isOpen()) {
        qDebug() << "Serial port is not open for writing.";
        return -1;
    }

    // Write the data to the serial port
    qint64 bytesWritten = serial->write(data);
    if (bytesWritten == -1) {
        qDebug() << "Failed to write data to serial port.";
        return -1;
    }

    // Ensure all data is sent out before proceeding
    serial->flush();

    qDebug() << "Wrote" << bytesWritten << "bytes to serial port.";

    return bytesWritten;
}
