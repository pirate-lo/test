#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H


#define car_long 0.125
#define car_wide 0.081
#define RADIUS_WHEEL 10


#pragma pack(1)

typedef struct 
{
    float chassis_vx;
    float chassis_vy;
    float chassis_vz;
}OPS_chassis;


#pragma pack()

#endif // !ROBOT_DEF_H
