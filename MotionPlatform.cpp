#include "MotionPlatform.h"
#include <QDebug>

MotionPlatform::MotionPlatform(const char* ID, NT_INDEX channelIndex)
    : motionSta_(0)
    , position_(0)
    , voltage_(0)
{
    deviceID_ = ID;
    isOpen_ = 0;
    channelIndex_ = channelIndex;

    updateTimer_ = new QTimer();
    updateTimer_->setInterval(20);//Motion信息（位置、电压、状态）更新频率为20Hz，等于力传感器采样频率。

    QObject::connect(updateTimer_, &QTimer::timeout, this, &MotionPlatform::update);
}

MotionPlatform::~MotionPlatform()
{
    if(isOpen_)
        disConnect();
    delete updateTimer_;
}

void MotionPlatform::update()
{
    NT_GetPosition_S(ntHandle_, channelIndex_, &position_);
    // NT_GetVoltageLevel_S(ntHandle_,channelIndex_,&voltage_);
    NT_GetStatus_S(ntHandle_,channelIndex_,&motionSta_);
    // qDebug() << "pos:" << position_ << "voltage:" << voltage_ << "sta:" << motionSta_;
    // quintptr currentThreadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    // qDebug() << "MotionPlatform thread ID:" << currentThreadId;
}

bool MotionPlatform::connect()
{
    error_ = NT_OpenSystem(&ntHandle_, deviceID_, "sync");
    if (error_)
    {
        qDebug() << "Open NT system: error_: " << error_ << "ntHandle_:" << ntHandle_;
    }
    else{
        isOpen_ = 1;
        qDebug() << "Open NT system  successfully!" << "ntHandle:" << ntHandle_ << "channel:" <<channelIndex_;

#ifdef TIMERMODE_MOTION
        updateTimer_->start();
#endif
    }
    return error_;
}

bool MotionPlatform::disConnect()
{
    if(isOpen_)
    {
        error_ = NT_CloseSystem(ntHandle_);

#ifdef TIMERMODE_MOTION
        updateTimer_->stop();
#endif
        if (error_)
        {
           // qDebug() << "Close NT system: error_: " << error_ << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
        else{
           // qDebug() << "Close NT system  successfully!" << "ntHandle_:" << ntHandle_;
            isOpen_ = 0;
        }
    }
    // else
        // qDebug() << "Close NT system: error_: " << "Platform not Connected";
    return error_;
}

bool MotionPlatform::gotoPositionAbsolute( signed int position)
{
    error_ = NT_GotoPositionAbsolute_S(ntHandle_,
    channelIndex_,
    position);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GotoPositionAbsolute error_:" << error_;
    return error_;
}

bool MotionPlatform::gotoPositionRelative( signed int diff)
{
    error_ = NT_GotoPositionRelative_S(ntHandle_,channelIndex_,diff);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GotoPositionRelative error_:" << error_;
    return error_;
}

bool MotionPlatform::getPosition(int &position)
{

#ifdef TIMERMODE_MOTION
    position = position_;
    return true;
#else
    error_ = NT_GetPosition_S(ntHandle_, channelIndex_, &position);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GetPosition error_:" << error_;
    return error_;
#endif
}

bool MotionPlatform::stepMove( signed int steps,unsigned int amplitude,unsigned int frequency)
{
    error_ = NT_StepMove_S(ntHandle_,channelIndex_,steps,amplitude,frequency);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::StepMove error_:" << error_;
    return error_;
}

bool MotionPlatform::continueMove( bool dir, unsigned int amplitude,unsigned int frequency)
{
    signed int steps = 30000;
    steps = steps * dir;
    error_ = NT_StepMove_S(ntHandle_,channelIndex_,steps,amplitude,frequency);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::StepMove error_:" << error_;
    return error_;
}


//● target (unsigned 32bit)，输入 – 绝对目标位置。有效输入范围为 0...4,095，0 对应
//0V，4,095 对于 100V。
//● scanSpeed (unsigned 32bit), 输入 - 扫描速度。有效输入范围是 1 ... 4,095,000。
//将 0-100V 等分成 4096 份，扫描速度表示每秒扫描电压变化的份数。比如，值为 1
//时，在从 0 扫描到 4,095 需要 4,095 秒，而全速扫描（值为 4,095,000）只需一毫秒。
bool MotionPlatform::scanMoveAbsolute( unsigned int target, unsigned int time)
{
    error_ = NT_ScanMoveAbsolute_S(ntHandle_, channelIndex_,target, 4095000/time);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::ScanMoveAbsolute error_:" << error_;
    return error_;
}

bool MotionPlatform::scanMoveRelative( signed int target, unsigned int time)
{
    error_ = NT_ScanMoveRelative_S(ntHandle_, channelIndex_,target, 4095000/time);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::ScanMoveRelative error_:" << error_;
    return error_;
}


//● speed (unsigned 32bit)，输入 - 设置定位台匀速运动的速度值,线性定位台的单位是
//nm/s,旋转定位台的单位是 u°/s。有效值范围是 0...5,000,000。0 默认禁用速度控制。
bool MotionPlatform::setVelocity(signed int velocity, signed int target)
{
    int dir = 0;
    unsigned int speed = 0;
    if(velocity >= 0)
    {
        dir = 1;
        speed = velocity;
    }

    else
    {
        dir = -1;
        speed = -1 * velocity;
    }
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_ENABLED, speed);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocity error_(1):" << error_;
    error_ = NT_GotoPositionRelative_S(ntHandle_, channelIndex_, target * dir);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::setVelocity error_(2):" << error_;
    return error_;
}


bool MotionPlatform::setPosition()
{
    error_ = NT_SetPosition_S(ntHandle_, channelIndex_, 0);
    if (error_ != NT_OK)
        qDebug() << "MotionPlatform::setPosition error_:" << error_;
    return error_;
}

bool MotionPlatform::stop()
{
    error_ = NT_Stop_S(ntHandle_, channelIndex_);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::closeVelocityMode error_(1):" << error_;
    error_ = NT_SetClosedLoopMoveSpeed_S(ntHandle_, channelIndex_, NT_SPEED_DISABLED, 0);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::closeVelocityMode error_(2):" << error_;
    NT_SetClosedLoopHoldEnabled_S(ntHandle_,channelIndex_,NT_CLOSELOOP_ENABLED);
    return error_;
}

bool MotionPlatform::getVoltage(unsigned int &voltage)
{
// #ifdef TIMERMODE_MOTION
//     voltage = voltage_;
//     return true;
// #else
    error_ = NT_GetVoltageLevel_S(ntHandle_,channelIndex_,&voltage);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::GetVoltage error_:" << error_;
    return error_;
// #endif
}

bool MotionPlatform::findReference()
{
    error_ = NT_FindReferenceMark_S(ntHandle_, channelIndex_, NT_FIND_FORWARD, NT_AUTO_ZERO_ENABLED);
    if(error_ != NT_OK)
        qDebug() << "MotionPlatform::findReference error_:" << error_;
    return error_;
}


bool MotionPlatform::getMotionInfo()
{
    unsigned int numOfChannels;
    char info[200];
    unsigned int size = sizeof(info);
    unsigned int known;
    unsigned int speed;

    if(isOpen_)
    {
        error_ = NT_GetNumberOfChannels(ntHandle_, &numOfChannels);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(1):" << error_;
        else
            qDebug() << "Number of Channels:" << numOfChannels;

        error_ = NT_GetVersionInfo(ntHandle_, info, &size);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(2):" << error_;
        else
            qDebug() << "VersionInfo:" << info;

        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,0,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH1 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,1,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH2 PhysicalPositionKnown:" << known;
        error_ = NT_GetPhysicalPositionKnown_S(ntHandle_,2,&known);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(3):" << error_;
        else
            qDebug() << "CH3 PhysicalPositionKnown:" << known;

        NT_GetClosedLoopMoveSpeed_S(ntHandle_,0,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH1 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_,1,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH2 ClosedLoopMoveSpeed:" << speed;
        NT_GetClosedLoopMoveSpeed_S(ntHandle_,2,&speed);
        if(error_ != NT_OK)
            qDebug() << "MotionPlatform::getMotionInfo error_(4):" << error_;
        else
            qDebug() << "CH3 ClosedLoopMoveSpeed:" << speed;
    }
    else
    {
        qDebug() << "Platform not Connected";
    }
    return error_;
}
//    // channel status codes
//    #define NT_STOPPED_STATUS                           0
//    #define NT_STEPPING_STATUS                          1
//    #define NT_SCANNING_STATUS                          2
//    #define NT_HOLDING_STATUS                           3
//    #define NT_TARGET_STATUS                            4
//    #define NT_SENSOR_CLOSED_STATUS                     5
//    #define NT_NO_HOLDING_STATUS                        6
//    #define NT_REFERENCING_STATUS                       7
//    #define NT_CALIBRATING_STATUS                       8
//    #define NT_PHY_LIMIT_STATUS                         10
//    #define NT_SOFT_LIMIT_STATUS                        11
//    #define NT_SHORT_CIRCUIT_STATUS                     13
bool MotionPlatform::getsta( NT_STATUS &sta)
{
#ifdef TIMERMODE_MOTION
    sta = motionSta_;
    return true;
#else
    error_ = NT_GetStatus_S(ntHandle_,channelIndex_,&sta);
    return error_;
#endif

}


bool MotionPlatform::findSystem()
{
    char outBuffer[4096];
    unsigned int bufferSize = sizeof(outBuffer);
    NT_STATUS result = NT_FindSystems("", outBuffer, &bufferSize);
    if(result == NT_OK){
        // outBuffer holds the locator strings, separated by '\n'
        // bufferSize holds the number of bytes written to outBuffer
        qDebug() << "findSystem:" << outBuffer;
    }
    else{
        qDebug() << "find 0 System";
    }
    return true;
}
