#include "AttitudeMeasurement.h"
#include <QDebug>

AttitudeMeasurement::AttitudeMeasurement(const char* platformID
                                         , const char* scannerID_
                                         , const QString &portName
                                         , double sensitivityCH1
                                         , double sensitivityCH2)
    : platformX_(new MotionPlatform(platformID, CH1))
    , platformY_(new MotionPlatform(platformID, CH2))
    , platformZ_(new MotionPlatform(platformID, CH3))
    , scanner_(new Scanner(scannerID_))
    , forceSensor_(new ForceSensor(portName, sensitivityCH1, sensitivityCH2))
    , platformID_(platformID)
    , portName_(portName)
    , sensitivityCH1_(sensitivityCH1)
    , sensitivityCH2_(sensitivityCH2)
    , platformThread_(new QThread())
    , forceThread_(new QThread())
    , scanThread_(new QThread())
    , updateTimer_(new QTimer())
    , bufferCapacity_(1000) // 设置缓冲区容量
    , currentIndex_(0)      // 初始化当前写入位置为0
    , count_(0)             // 初始化已存入的数据数量为0
{
    MeasurementDataBuffer_.resize(bufferCapacity_);

    forceSensor_->moveToThread(forceThread_);
    platformX_->moveToThread(platformThread_);
    platformY_->moveToThread(platformThread_);
    platformZ_->moveToThread(platformThread_);
    scanner_->moveToThread(scanThread_);

    platformX_->findSystem();

    // quintptr currentThreadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    // qDebug() << "AttitudeMeasurement thread ID:" << currentThreadId;

    updateTimer_->setInterval(20);//Motion信息（位置、电压、状态）更新频率为20Hz，等于力传感器采样频率。

    QObject::connect(updateTimer_, &QTimer::timeout, this, &AttitudeMeasurement::update);
}

AttitudeMeasurement::~AttitudeMeasurement()
{
    shutdown();
    delete forceThread_;
    delete platformThread_;
    delete forceSensor_;
    delete platformX_;
    delete platformY_;
    delete platformZ_;
}

void AttitudeMeasurement::update()
{
    // quintptr currentThreadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    // qDebug() << "AttitudeMeasurement update thread ID:" << currentThreadId;

    // 获取最新的力传感器数据
    int  forceData = 0, pos[3] = {0, 0 , 0};
    double scanPos = 0.0;
    unsigned int secondChannelData = 0, sta[3] = {0, 0 , 0};

    platformX_->getPosition(pos[CH1]);
    platformY_->getPosition(pos[CH2]);
    platformZ_->getPosition(pos[CH3]);
    platformX_->getsta(sta[CH1]);
    platformY_->getsta(sta[CH2]);
    platformZ_->getsta(sta[CH3]);
    forceSensor_->readCurrentForce(0, forceData);
    forceSensor_->readSecondChannel_A(secondChannelData);
    //对应的ADC * Sensitivity，然后查表。也是update()中运行时长不可控的地方
    scanner_->getPosition(scanPos, secondChannelData);
    std::array<int, 3> positions = {{
        pos[CH1],
        pos[CH2],
        pos[CH3]
    }};
    std::array<unsigned int, 3> statuses = {{
        sta[CH1],
        sta[CH2],
        sta[CH3]
    }};

    // 使用互斥锁确保线程安全
    QMutexLocker locker(&mutex_);

    // 创建新的 MeasurementData 对象
    MeasurementData newData(positions, statuses, forceData, scanPos);

    // qDebug() << "X:" << newData.position[CH1]
    //      << "\tY:" << newData.position[CH2]
    //      << "\tZ:" << newData.position[CH3];
    // qDebug() << "X:" << newData.status[CH1]
    //          << "\tY:" << newData.status[CH2]
    //          << "\tZ:" << newData.status[CH3];
    // qDebug() << "Force:" << newData.force;
    // qDebug() << "scanPos:" << newData.scanPositon;

    MeasurementDataBuffer_[currentIndex_] = newData;

    if (count_ < bufferCapacity_) {
        ++count_;
    }

    // qDebug() << "currentIndex_:" << currentIndex_;
    // qDebug() << "X:" << MeasurementDataBuffer_[currentIndex_].position[CH1]
    //          << "\tY:" << MeasurementDataBuffer_[currentIndex_].position[CH2]
    //          << "\tZ:" << MeasurementDataBuffer_[currentIndex_].position[CH3];
    // qDebug() << "X:" << MeasurementDataBuffer_[currentIndex_].status[CH1]
    //          << "\tY:" << MeasurementDataBuffer_[currentIndex_].status[CH2]
    //          << "\tZ:" << MeasurementDataBuffer_[currentIndex_].status[CH3];
    // qDebug() << "Force:" << MeasurementDataBuffer_[currentIndex_].force;
    // qDebug() << "scanPos:" << MeasurementDataBuffer_[currentIndex_].scanPositon;

    currentIndex_ = (currentIndex_ + 1) % bufferCapacity_;
}


bool AttitudeMeasurement::getData(MeasurementData &latestData)
{
    QMutexLocker locker(&mutex_);

    if (count_ == 0) {
        // 如果没有数据，则返回 false 表示失败
        return false;
    }

    // 计算最新的数据项的位置
    int latestIndex = (currentIndex_ == 0) ? bufferCapacity_ - 1 : currentIndex_ - 1;

    // 将最新的数据复制到传入的对象中
    latestData = MeasurementDataBuffer_[latestIndex];

    // 成功返回 true
    return true;
}



bool AttitudeMeasurement::getDataBuffer(QVector<MeasurementData> &amDataBuffer)
{
    // 确保传入的容器有足够的空间来容纳所有的数据
    amDataBuffer.resize(bufferCapacity_);

    // 如果count_小于bufferCapacity_，意味着缓冲区不是完全填满的，
    // 只需要复制实际存储的数据数量。
    int actualCount = qMin(count_, bufferCapacity_);

    if (actualCount > 0) {
        try {
            // 使用互斥锁确保线程安全
            QMutexLocker locker(&mutex_);

            // 需要处理wrap around的情况。
            if (count_ >= bufferCapacity_) {
                // 复制从currentIndex_到末尾的数据
                int trailingCount = bufferCapacity_ - currentIndex_;
                int leadingCount = actualCount - trailingCount;

                std::copy(MeasurementDataBuffer_.begin() + currentIndex_,
                          MeasurementDataBuffer_.begin() + bufferCapacity_,
                          amDataBuffer.begin());

                // 复制从开头到currentIndex_前的数据
                if (leadingCount > 0) {
                    std::copy(MeasurementDataBuffer_.begin(),
                              MeasurementDataBuffer_.begin() + leadingCount,
                              amDataBuffer.begin() + trailingCount);
                }
            } else {
                // qDebug() << "Actuall scanPos:" << MeasurementDataBuffer_[currentIndex_-1].scanPositon;//但是currentIndex_ 可能为 0，慎用
                std::copy(MeasurementDataBuffer_.begin(),
                          MeasurementDataBuffer_.begin() + actualCount,
                          amDataBuffer.begin());
            }

            // 调整amDataBuffer的大小以匹配实际存储的数据数量
            amDataBuffer.resize(actualCount);

            return true; // 成功返回true
        } catch (...) {
            // 捕获任何可能发生的异常并返回false表示失败
            return false;
        }
    } else {
        // 如果没有数据，则清空amDataBuffer
        amDataBuffer.clear();
        return true; // 没有数据也是一种正常情况，所以返回true
    }
}


bool AttitudeMeasurement::initialize()
{
    bool success = true;

    forceThread_->start();
    success &= forceSensor_->connect();

    // 启动平台线程
    platformThread_->start();
    scanThread_->start();

    success &= platformX_->connect();
    success &= platformY_->connect();
    success &= platformZ_->connect();

    success &= scanner_->connect();

    updateTimer_->start();

    // success &= platformX_->findReference();
    // success &= platformY_->findReference();
    // success &= platformZ_->findReference();

    return success;
}

bool AttitudeMeasurement::shutdown()
{
    bool success = true;

    updateTimer_->stop();

    success &= forceSensor_->disConnect();
    success &= platformX_->disConnect();
    success &= platformY_->disConnect();
    success &= platformZ_->disConnect();
    success &= scanner_->disConnect();

    if (forceThread_) {
        forceThread_->quit();
        success &= forceThread_->wait(50); // 等待最多0.05秒
    }
    if (platformThread_) {
        platformThread_->quit();
        success &= platformThread_->wait(50); // 等待最多0.05秒
    }
    if (scanThread_) {
        scanThread_->quit();
        success &= scanThread_->wait(50); // 等待最多0.05秒
    }

    return success;
}


bool AttitudeMeasurement::moveJointPositions(const Eigen::Vector3d& target_positions)
{
    bool success = true;

    success &= platformX_->gotoPositionAbsolute(target_positions[0]);
    success &= platformY_->gotoPositionAbsolute(target_positions[1]);
    success &= platformZ_->gotoPositionAbsolute(target_positions[2]);

    return success;
}

// bool AttitudeMeasurement::openScanCloseLoop()
// {
//     scanner_->openCloseLoopControll();
//     return 1;
// }
// bool AttitudeMeasurement::closeScanCloseLoop()
// {
//     scanner_->closeCloseLoopControll();
//     return 1;
// }


bool AttitudeMeasurement::setMicroPosition(signed int position)
{
    scanner_->gotoPosition(position);
    return true;
}
bool AttitudeMeasurement::setMicroPositionRelative(signed int diff)
{
    scanner_->gotoPositionRelative(diff);
    return true;
}
bool AttitudeMeasurement::setMicroVelocity(signed int velocity)
{
    scanner_->setVelocity(velocity);
    return true;
}

bool AttitudeMeasurement::setMicroVoltage(unsigned int Voltage)
{
    scanner_->scanMoveAbsolute(Voltage);
    return true;
}

bool AttitudeMeasurement::setMicroVoltageRelative(int diff_Voltage)
{
    scanner_->scanMoveRelative(diff_Voltage);
    return true;
}


bool AttitudeMeasurement::getMicroVelocity(int &velocity)
{
    scanner_->getVelocity(velocity);
    return true;
}

bool AttitudeMeasurement::getMicroVoltage(unsigned int &Voltage)
{
    scanner_->getVoltage(Voltage);
    return true;
}

