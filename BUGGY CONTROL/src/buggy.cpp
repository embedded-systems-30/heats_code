#include "mbed.h"
//#include "PID.hpp"
#include "mbed.h"

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
        float weight[] = {-2, -1, -0.5, 0.5, 1, 2};
        float weightedSum=0.0f;
        float SumOfWeights = 0.0f;

        for (int i=0; i<6; i++){
            averageSensorValue+=sensorValues[i];
            weightedSum+= sensorValues[i]*weight[i];
            SumOfWeights += weight[i];

        }
        float estimates_line_position = weightedSum/SumOfWeights; 
        averageSensorValue /= 6.0f;
        float error = setpoint - SumOfWeights;

        float proportional = Kp * error;
        errorSum += Ki*error*deltaTime;
        float derivative = Kd*(error-lastError) / deltaTime;

        float output = proportional + errorSum + derivative;

        lastError=error;

        return output;

    }
};

AnalogIn sensor[] = {AnalogIn(A0), AnalogIn(A1), AnalogIn(A2), AnalogIn(A3), AnalogIn(A4), AnalogIn(A5)};
PwmOut motor_left(PA_8);
PwmOut motor_right(PC_7);




int main(){
    LineFollowingPID controller(1.0f, 0.1f, 0.01f, 0.1f);
    float setpoint = 0.0f;
    float sensorValues[6];
    while(1){
        for(int i=0; i<6; i++){
             sensorValues[i] = sensor[i].read();

        }
        float PIDOutput= controller.update(sensorValues, setpoint);
        float startSpeed = 0.5f;
        float maxSpeed = 1.0f;

        motor_right.write(startSpeed - PIDOutput);
        motor_left.write(startSpeed+PIDOutput);

    }
   
   
      

    
}