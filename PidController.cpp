#include "PidController.h"

PidController::PidController(double kp, double ki, double kd)
    : QObject(nullptr), m_kp(kp), m_ki(ki), m_kd(kd),
    m_setPoint(0), m_currentPoint(0), m_previousError(0), m_integral(0),
    m_integralMax(2000000.0), m_integralMin(-2000000.0),
    isReceiveData_(0)
{
    m_timer = new QTimer();
    m_timer->setInterval(20);

    connect(m_timer, &QTimer::timeout, this, &PidController::updatePidControl);
}

PidController::~PidController()
{
    if (m_timer->isActive()) {
        m_timer->stop();
        isReceiveData_ = 0;
    }
}


bool PidController::openPidControll()
{
    if (!m_timer->isActive()) {
        m_timer->start();
    }
    return 1;
}


bool PidController::closePidControll()
{
    if (m_timer->isActive()) {
        m_timer->stop();
        isReceiveData_ = 0;
    }
    return 1;
}

void PidController::setTargetPoint(double position)
{
    m_setPoint = position;
}

void PidController::setRelativePoint(double position)
{
    m_setPoint += position;
}

void PidController::updatePidControl()
{
    if(isReceiveData_){
        double output = computePidOutput();

        emit pidOutput(output);

        if (qAbs(m_setPoint - m_currentPoint) < 0.1) {
        }
    }
    else{

    }
}

bool PidController::setCurrentPoint(double position)
{
    // qDebug() << "\tposition：" <<  position<< "\tm_setPoint：" <<  m_setPoint;
    m_currentPoint = position;
    if (m_timer->isActive()) {
        if(isReceiveData_ == 0){//如果定时器开着，且接收到消息，才算接收到消息
            m_setPoint = position;
            isReceiveData_ = 1;
        }
    }
    return 1;
}

double PidController::computePidOutput()
{
    double error = m_setPoint - m_currentPoint;
    // 防止积分饱和
    if (error > m_integralMax) {
        m_integral = m_integralMax;
    } else if (error < m_integralMin) {
        m_integral = m_integralMin;
    } else {
        // m_integral += error;
    }
    double derivative = error - m_previousError;
    m_previousError = error;


    double output = m_kp * error + m_ki * m_integral + m_kd * derivative;

    // qDebug() << "\tm_setPoint：" <<  m_setPoint
    //          << "\tm_currentPoint：" <<  m_currentPoint
    //          << "error：" <<  error
    //          // << "\tm_integral：" <<  m_integral
    //          // << "\tderivative：" <<  derivative
    //          << "\t\toutput：" <<  output;


    return output;
}
