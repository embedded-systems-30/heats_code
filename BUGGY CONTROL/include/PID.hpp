#include "mbed.h"
#include "QEI.h"
class LineFollowingPID{
    private:
    float Kp;
    float Ki;
    float Kd;
    float deltaTime;
    float errorSum;
    float lastError;

    public:
    LineFollowingPID(float kp, float ki, float kd, float dt): Kp(kp), Ki(ki), Kd(kd), deltaTime(dt){
        errorSum = 0.0f;
        lastError = 0.0f;
    }
    float update(float *sensorValues, float setpoint){
        float averageSensorValue = 0.0f;
        for (int i=0; i<6; i++){
            averageSensorValue+=sensorValues[i];

        }
        averageSensorValue /= 6.0f;
        float error = setpoint - averageSensorValue;

        float proportional = Kp * error;
        errorSum += Ki*error*deltaTime;
        float derivative = Kd*(error-lastError) / deltaTime;

        float output = proportional + errorSum + derivative;

        lastError=error;

        return output;

    }
};
