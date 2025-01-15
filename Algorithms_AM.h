#ifndef ALGORITHMS_AM_H
#define ALGORITHMS_AM_H

#include <QObject>
#include "AttitudeMeasurement.h"

class Algorithms_AM : public QObject
{
    Q_OBJECT
public:
    Algorithms_AM();
    ~Algorithms_AM();




private:
    AttitudeMeasurement* AM;
};

#endif // ALGORITHMS_AM_H
