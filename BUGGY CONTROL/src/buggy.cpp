#include "mbed.h"
//#include "PID.hpp"
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
        float weight[] = {-0.5, -0.25, 0.1, 0.1, 0.25, 0.5};
        float weightedSum=0.0f;
        float SumOfWeights = 0.0f;

        for (int i=0; i<6; i++){
            averageSensorValue+=sensorValues[i];
            weightedSum+= sensorValues[i]*weight[i];
            SumOfWeights += weight[i];

        }
        float estimates_line_position = weightedSum/SumOfWeights; 
        averageSensorValue /= 6.0f;

        printf("Line %f\n", estimates_line_position);
        fflush(stdout);
        float error = setpoint - estimates_line_position;

        float proportional = Kp * error;
        errorSum += Ki*error*deltaTime;
        float derivative = Kd*(error-lastError) / deltaTime;

        float  output = proportional + errorSum + derivative;

        lastError=error;

        return output;

    }
};
 BufferedSerial hm10(PA_11, PA_12, 9600);
AnalogIn sensor[] = {AnalogIn(A0), AnalogIn(A1), AnalogIn(A2), AnalogIn(A3), AnalogIn(A4), AnalogIn(A5)};
PwmOut motor_left(PA_8);
PwmOut motor_right(PC_7);

FileHandle *mbed::mbed_override_console(int fd) {return &hm10;};
DigitalOut ena(PB_2, 0);
int main(){
    
    ena = 1; 
  //  QEI encoder()
    LineFollowingPID controller(0.1f, 0.0f, 0.0f, 0.1f);
    float setpoint = 0.7f;
    float sensorValues[6];
    while(1){
        float PIDOutput = 0.0f;
        for(int i=0; i<6; i++){
             sensorValues[i] = sensor[i].read();

        }
        PIDOutput= controller.update(sensorValues, setpoint);
        float startSpeed_left = 0.7f;
        float startSpeed_right = 0.3f;
        float maxSpeed = 1.0f;
        if (PIDOutput>=0.2){PIDOutput= 0.2;}
        if(PIDOutput<=-0.2){PIDOutput=0.2;}
        motor_right.write(startSpeed_right + PIDOutput);
        motor_left.write(startSpeed_left+PIDOutput);
        printf("%f\n", PIDOutput);
        fflush(stdout);
        ThisThread::sleep_for(100ms);

    }
   
   
      

    
}