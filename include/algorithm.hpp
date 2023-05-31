#ifndef PPC_ALGORITHM_H
#define PPC_ALGORITHM_H

#include "main.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"

class Algorithm
{
    public:
        static const double PI = 3.14159265;

        template <typename T> static int sign(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        static geometry_msgs::Vector3 quaternion_to_euler(const geometry_msgs::Quaternion& q)
        {
            geometry_msgs::Vector3 euler;
            const double Epsilon = 0.0009765625f;
            const double Threshold = 0.5f - Epsilon;

            double pitch = q.w*q.y - q.x*q.z;

            if (pitch < -Threshold || pitch > Threshold)
            {
                int sign_pitch = sign<double>(pitch);
                euler.z = -2 * sign_pitch * (double)atan2(q.x, q.w); // yaw
                euler.y = sign_pitch * (PI / 2.0); // pitch
                euler.x = 0; // roll
            }
            else
            {
                euler.x = atan2(2 * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
                euler.y = asin(-2 * (q.x*q.z - q.w*q.y));
                euler.z = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
            }

            return euler;
        }
};


#endif
