#ifndef PPC_MAIN_H
#define PPC_MAIN_H

#define CONTROL_FREQUENCY 30
#define CONTROL_SPEED_MAX 0.5
#define PIXEL_FEEDBACK_REDUCE_SCALE 100.0

enum class ControlMode
{
    CARTESIAN = 0X00U,
    PIXEL =     0X01U,
};

class CartesianPosition
{
    public:
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        ControlMode real_mode;
};

#endif

