/*
 * @Author: pirate-lo @caoboxun8
 * @Date: 2024-05-22 21:24:58
 * @LastEditors: pirate-lo @caoboxun8
 * @LastEditTime: 2024-08-28 11:47:09
 * @FilePath: \play\App\robot_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H


#define car_long 0.125
#define car_wide 0.081
#define RADIUS_WHEEL 100


#pragma pack(1)

typedef struct 
{
    float chassis_vx;
    float chassis_vy;
    float chassis_vz;
}OPS_chassis;


#pragma pack()

#endif // !ROBOT_DEF_H
