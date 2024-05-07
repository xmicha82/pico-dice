#ifndef DICE_TYPES_H
#define DICE_TYPES_H

#include "haw/MPU6050.h"

typedef mpu6050_vectorf_t vectorf_t;

enum DiceType {
    D6,
    D8,
};

typedef struct DiceStruct {
    enum DiceType type;
    vectorf_t* sides;
    uint8_t size;
} DiceStruct;

vectorf_t d6_sides[] = {
    { 1.0,  0.0,  0.0},
    { 0.0, -0.7, -0.7},
    { 0.0,  0.7, -0.7},
    { 0.0, -0.7,  0.7},
    { 0.0,  0.7,  0.7},
    {-1.0,  0.0,  0.0},
};

vectorf_t d8_sides[] = {
    { 0.82,     0,  0.58},
    {    0, -0.82, -0.58},
    {    0, -0.82,  0.58},
    { 0.82,     0, -0.58},
    {-0.82,     0,  0.58},
    {    0,  0.82, -0.58},
    {    0,  0.82,  0.58},
    {-0.82,     0, -0.58},
};

DiceStruct d6 = {
    D6,
    d6_sides,
    6
};

DiceStruct d8 = {
    D8,
    d8_sides,
    8
};



#endif // DICE_TYPES_H
