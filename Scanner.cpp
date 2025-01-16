#include "Scanner.h"
#include <QDebug>

Scanner::Scanner(const char* ID)
    : deviceID_(ID)
    , channelIndex_(0)
    , isOpen_(0)
    , motionSta_(0)
    , position_(0)
    , velocity_(0)
    , temp_velocity_(0)
    , voltage_(0)
    , updateTimer_(new QTimer())
    , PidPosition_(new PidController(1.2, 0, 0))
    , PidVelocity_(new PidController(0.02, 0, 0))
    , PIDupdateThread_(new QThread())
    , bufferIndex_(0)
    , bufferSize_(5)
    , count(0)
    , scanMoveMode(NoMove)
{
    positionBuffer_.resize(bufferSize_);

    updateTimer_->setInterval(20);

    PidPosition_->moveToThread(PIDupdateThread_);
    PidVelocity_->moveToThread(PIDupdateThread_);
    PIDupdateThread_->start();

    QObject::connect(PidPosition_, &PidController::pidOutput, this, &Scanner::scanMove);
    QObject::connect(PidVelocity_, &PidController::pidOutput, this, &Scanner::scanMove);

    QObject::connect(updateTimer_, &QTimer::timeout, this, &Scanner::update);
}

Scanner::~Scanner()
{
    disConnect();
    delete PidPosition_;
    delete PidVelocity_;
    if (PIDupdateThread_) {
        PIDupdateThread_->quit();
        PIDupdateThread_->wait(50); // 等待最多0.05秒
    }
}

void Scanner::update()
{
    // 更新当前位置到缓冲区
    positionBuffer_[bufferIndex_] = position_;
    bufferIndex_ = (bufferIndex_ + 1) % bufferSize_; // 循环更新索引

    if(count < bufferSize_){
        count ++;
    }


    // 计算平均速度，考虑方向
    if (isOpen_ && (count >= bufferSize_)) { // 确保只有在连接状态下才进行计算
        double totalDisplacement = 0;


        // 从最新的位置向前遍历，以保持正确的顺序
        for (int i = 0; i < bufferSize_ - 1; ++i) {
            int currentIndex = (bufferIndex_ + bufferSize_ - 1 - i) % bufferSize_;
            int nextIndex = (currentIndex + bufferSize_ - 1) % bufferSize_;
            totalDisplacement += (positionBuffer_[currentIndex] - positionBuffer_[nextIndex]);
        }
        // 平均速度 = 总位移 / 时间间隔总和
        // 注意这里的时间间隔是基于updateTimer_的周期，即20ms
        velocity_ = int(totalDisplacement / (bufferSize_ * 0.02)); // 20ms转换为秒

    }

    PidVelocity_->setCurrentPoint(velocity_);
    qDebug() << "velocity_" << velocity_;
}


bool Scanner::connect()
{
    error_ = SCAN_OpenSystem(&ntHandle_, deviceID_, "sync");
    if (error_){
        qDebug() << "Open SCAN system: error_: " << error_ << "ntHandle_:" << ntHandle_;
    }
    else{
        isOpen_ = 1;
        updateTimer_->start();
        switchMode(PositionMode);
        qDebug() << "Open SCAN system  successfully!" << "ntHandle:" << ntHandle_ << "channel:" <<channelIndex_;
    }
    return error_;
}


bool Scanner::disConnect()
{
    if(isOpen_){
        if (updateTimer_->isActive()) {
            updateTimer_->stop();
        }
        error_ = SCAN_CloseSystem(ntHandle_);
        if (error_){
            qDebug() << "Close NT system: error_: " << error_ << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
        else{
            qDebug() << "Close NT system  successfully!" << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
        switchMode(NoMove);
    }
    else
        qDebug() << "Close NT system: error_: " << "Platform not Connected";
    return error_;
}


bool Scanner::getPosition(double &position, unsigned int adc)//查表，并更新position_
{
    if((adc < 13000000) && (adc > 9000000))
    {
        position = (double(adc) - 9000000.00) / 4000000.00 * 30000.00;
        position_ = position;

        // if(scanMoveMode == PositionMode){
            PidPosition_->setCurrentPoint(position_);
        // }

    }else{
        qDebug() << "Second Channel ADC Error ";
    }

    return 1;
}

bool Scanner::gotoPosition(int position)
{
    //将position传给PID做位置保持
    // scanMoveMode = PositionMode;
    switchMode(PositionMode);
    PidPosition_->setCurrentPoint(position_);
    PidPosition_->setTargetPoint(position);
    return 1;
}

bool Scanner::gotoPositionRelative(int diff)
{
    switchMode(PositionMode);
    PidPosition_->setCurrentPoint(position_);
    PidPosition_->setRelativePoint(diff);
    return 1;
}


bool Scanner::switchMode(MoveMode MoveMode)
{
    scanMoveMode = MoveMode;
    if(MoveMode == PositionMode)
    {
        PidVelocity_->closePidControll();
        PidPosition_->openPidControll();
    }
    else if(MoveMode == VelocityMode)
    {
        PidPosition_->closePidControll();
        PidVelocity_->openPidControll();
    }
    else{
        PidPosition_->closePidControll();
        PidVelocity_->closePidControll();
    }
    return 1;
}



bool Scanner::getVelocity(int &velocity)
{
    velocity = velocity_;
    return true;
}


bool Scanner::setVelocity(int velocity)
{
    switchMode(VelocityMode);
    temp_velocity_ = 0;
    PidVelocity_->setCurrentPoint(velocity);
    PidVelocity_->setTargetPoint(velocity);
    return 1;
}



bool Scanner::findSystem()
{
    char outBuffer[4096];
    unsigned int bufferSize = sizeof(outBuffer);
    SCAN_STATUS result = SCAN_FindSystems("", outBuffer, &bufferSize);
    if(result == SCAN_OK){
        qDebug() << "findSystem:" << outBuffer;
    }
    else{
        qDebug() << "find 0 System";
    }
    return true;
}


bool Scanner::getVoltage(unsigned int &voltage)
{
    error_ = SCAN_GetVoltageLevel_S(ntHandle_,channelIndex_,&voltage);
    if(error_ != SCAN_OK)
        qDebug() << "Scanner::getVoltage error:" << error_;
    return error_;
}


// ●channelIndex (unsigned 32bit)，  输入 - 通道索引，索引值从0开始。
// ●diff (signed 32bit)，            输入 – 相对目标位置。有效输入范围为-262143...262143。 如果控制器最终得到的绝对扫描目标超过-262143...262143，131072的有效输入范围，则扫描运动将在边界处停止。
// ●scanStep(unsigned 32bit)，       输入 - 扫描步数。将当前位置与目标位置分为scanStep个步数。范围2-20000。
// ●scanDelay(unsigned 32bit)，      输入 - 两步数之间的延时。US级，范围：1-65535。
bool Scanner::scanMoveAbsolute(unsigned int target, unsigned int scanStep, unsigned int scanDelay)
{
    if(isOpen_){
        error_ = SCAN_ScanMoveAbsolute_S(ntHandle_, channelIndex_, target, scanStep, scanDelay);
        if(error_ != SCAN_OK)
            qDebug() << "Scanner::ScanMoveAbsolute error_:" << error_;
        return error_;
    }
    return error_;
}

bool Scanner::scanMoveRelative(int diff, unsigned int scanStep, unsigned int scanDelay)
{
    if(isOpen_){
        error_ = SCAN_ScanMoveRelative_S(ntHandle_, channelIndex_, diff, scanStep, scanDelay);
        if(error_ != SCAN_OK)
            qDebug() << "Scanner::ScanMoveRelative error_:" << error_;
        return error_;
    }
    return error_;
}

void Scanner::scanMove(double pidData)
{
    // qDebug() << "PID Data :" << pidData;
    if(scanMoveMode == PositionMode){
        scanMoveRelative(pidData);
    }else if(scanMoveMode == VelocityMode){
        temp_velocity_ += pidData;
        // qDebug() << "velocity Mode";
        scanMoveRelative(temp_velocity_);
    }
}

// bool Scanner::openCloseLoopControll()
// {
//     //将当前位置传给PID做位置保持
//     PidPosition_->setTargetPosition(position_, position_);
//     return 1;
// }

// bool Scanner::closeCloseLoopControll()
// {
//     scanMoveMode = 0;
//     PidPosition_->closePidControll();
//     return 1;
// }
