#include "ForceSensor.h"
#include <QDebug>
#include <QDateTime>

ForceSensor::ForceSensor(const QString &portName, double sensitivityCH1, double sensitivityCH2)
    : SerialCommon()
    , portName_(portName)
    , sensitivityCH1_(sensitivityCH1)
    , sensitivityCH2_(sensitivityCH2)
    , referenceZeroCH1_(0)
    , firstChannelforce_(0)
{

}


ForceSensor::~ForceSensor()
{

}

void ForceSensor::readData()
{
    int channel = -1;

    // quintptr currentThreadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    // qDebug() << "readData thread ID:" << currentThreadId;

    QByteArray data = serial->readAll();
    // qDebug() << "data" << data;

    int num = getForceFromBuffer(data, channel);
    // qDebug() << "num" << num;

    if(channel == 1){
        firstChannelforce_ = firstDataProcess(num);
        // qDebug() << "firstChannelforce_\t" << firstChannelforce_;
    }
    else if(channel == 2){
        secondChannelforce_ = secondDataProcess(num);
        // qDebug() << "secondChannelforce_\t" << secondChannelforce_;
    }
}

int ForceSensor::firstDataProcess(int forcedata)
{
    static int lastForce = 0;
    int currentForce = forcedata;
    if (currentForce < 0)
    {
        currentForce = lastForce;
    }
    lastForce = currentForce;
    return currentForce;
}
int ForceSensor::secondDataProcess(int forcedata)
{
    static int lastForce = 0;
    int currentForce = forcedata;
    if (currentForce < 0)
    {
        currentForce = lastForce;
    }
    lastForce = currentForce;
    return currentForce;
}



bool ForceSensor::readFirstChannel_A(unsigned int &force)
{
    int currentForce = firstChannelforce_;
    force = currentForce * sensitivityCH1_;
    return 1;
}
bool ForceSensor::readFirstChannel_R(int &force)
{
    int currentForce = firstChannelforce_;
    force = (currentForce-referenceZeroCH1_) * sensitivityCH1_;
    return 1;
}
bool ForceSensor::readSecondChannel_A(unsigned int &force)
{
    int currentForce = secondChannelforce_;
    force = currentForce * sensitivityCH2_;
    return 1;
}
bool ForceSensor::readSecondChannel_R(int &force)
{
    int currentForce = secondChannelforce_;
    force = (currentForce-referenceZeroCH2_) * sensitivityCH2_;
    return 1;
}





bool ForceSensor::readCurrentForce(bool isRelative, int &force)
{
    int currentForce = firstChannelforce_;
    // qDebug() << "currentForce" << currentForce;
    if(isRelative)
    {
        force = (currentForce-referenceZeroCH1_) * sensitivityCH1_;
        return 1;
    }
    else
    {
        force = currentForce * sensitivityCH1_;
        return 1;
    }
}


bool ForceSensor::connect(const QString &portName,
                          qint32 baudRate,
                          QSerialPort::DataBits dataBits,
                          QSerialPort::Parity parity,
                          QSerialPort::StopBits stopBits)
{
    if (!SerialCommon::open(portName, baudRate, dataBits, parity, stopBits)) {
        qDebug() << "Failed to open the serial port.";
        return 0;
    }
    return 1;
}

bool ForceSensor::connect()
{
    if (!SerialCommon::open(portName_, 921600, QSerialPort::Data8, QSerialPort::NoParity, QSerialPort::OneStop)) {
        qDebug() << "Failed to open the serial port.";
        return 0;
    }
    return 1;
}

bool ForceSensor::disConnect()
{
    SerialCommon::close();
    return 1;
}


bool ForceSensor::setReferenceZero(int num)
{
    if(num > 0)
    {
        referenceZeroCH1_ = num;
        return 1;
    }
    else
    {
        if(firstChannelforce_ > 0)
        {
            referenceZeroCH1_ = firstChannelforce_;
            return 1;
        }
        else
        {
            // setReferenceZero();
            return 0;
        }
    }
}

bool ForceSensor::setSensitivity(double sensitivity, int channel)
{
    if(sensitivity >0)
    {
        if(channel == 2)
        {
            sensitivityCH2_ = sensitivity;
        }
        else {
            sensitivityCH1_ = sensitivity;
        }
        return 1;
    }
    else
    {
        qDebug() << "Failed Set sensitivity: must bigger than 0";
        return 0;
    }
}

bool ForceSensor::getSensitivity(double &sensitivity, int channel)
{
    if(channel == 2)
    {
        sensitivity = sensitivityCH2_;
    }
    else {
        sensitivity = sensitivityCH1_;
    }
    return 1;
}

//屎山代码的开始-----2025/01/14  19:45
int ForceSensor::getForceFromBuffer(const QByteArray data, int &channel) {
    int bufSize = data.size() ;
    if (bufSize == 10) {
        if (data[6] == '0' && data[7] == 'b' && data[8] == '\r' && data[9] == '\n') {
            QByteArray result = data.left(6);
            bool ok;
            int num = result.toLongLong(&ok, 16);
            if (!ok) {
                qDebug() << "Failed to convert string to number.";
                return -1;
            }
            channel = 1;
            // qDebug() << "num" << num << "\t\t111";
            return num;
        }
        else if(data[6] == '0' && data[7] == 'd' && data[8] == '\r' && data[9] == '\n'){
            QByteArray result = data.left(6);
            bool ok;
            int num = result.toLongLong(&ok, 16);
            if (!ok) {
                qDebug() << "Failed to convert string to number.";
                return -1;
            }
            channel = 2;
            // qDebug() << "num" << num << "\t\t222";
            return num;
        }
        else {
            qDebug() << "Error: Invalid data format.";
            return -1;
        }
    }

    else if(bufSize == 20){
        // qDebug() << "E111111111111:" << bufSize << "Data:" << data;
        channel = 3;
        if (data[6] == '0' && data[8] == '\r' && data[9] == '\n' && data[16] == '0' && data[18] == '\r' && data[19] == '\n') {
            if(data[7] == 'b' && data[17] == 'd'){
                QByteArray result = data.left(6);
                bool ok;
                int num = result.toLongLong(&ok, 16);
                if (!ok) {
                    qDebug() << "Failed to convert string to number.";
                    return -1;
                }
                firstChannelforce_ = num;
                // qDebug() << "num" << num << "\t\t333333333333333333333";
                result = data.mid(10);
                result = result.left(6);
                num = result.toLongLong(&ok, 16);
                if (!ok) {
                    qDebug() << "Failed to convert string to number.";
                    return -1;
                }
                secondChannelforce_ = num;
                // qDebug() << "num" << num << "\t\t333333333333333333333";
                return 0;
            }
            else if(data[7] == 'd' && data[17] == 'b'){
                QByteArray result = data.left(6);
                bool ok;
                int num = result.toLongLong(&ok, 16);
                if (!ok) {
                    qDebug() << "Failed to convert string to number.";
                    return -1;
                }
                secondChannelforce_ = num;
                // qDebug() << "num" << num << "\t\t4444444444444444444444";
                result = data.mid(10);
                result = result.left(6);
                num = result.toLongLong(&ok, 16);
                if (!ok) {
                    qDebug() << "Failed to convert string to number.";
                    return -1;
                }
                firstChannelforce_ = num;
                // qDebug() << "num" << num << "\t\t4444444444444444444444";
                return 0;
            }else{
                return 1;
            }

        }else{
           return -1;
        }
    }else{
        // qDebug() << "Error: Invalid data length:" << bufSize << "Data:" << data;
        return -1;
    }
    return -1;
}

