#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>
#include <QDebug>
#include <QTimer>

class PidController : public QObject
{
    Q_OBJECT
public:
    explicit PidController(double kp, double ki, double kd);
    ~PidController();

    bool openPidControll();
    bool closePidControll();

    void setTargetPoint(double position);
    void setRelativePoint(double position);

    bool setCurrentPoint(double position);

signals:
    void pidOutput(double pidData);

private slots:
    void updatePidControl();

private:
    double m_kp; // Proportional gain
    double m_ki; // Integral gain
    double m_kd; // Derivative gain
    double m_setPoint; // Target position
    double m_currentPoint; // Current position
    double m_previousError;
    double m_integral;
    double m_integralMax;
    double m_integralMin;
    QTimer *m_timer;

    bool isReceiveData_;

    double computePidOutput();
};

#endif // PIDCONTROLLER_H
