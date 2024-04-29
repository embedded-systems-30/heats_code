#include "mbed.h"
#include "PID.hpp"
#include "mbed.h"
#include "QEI.h"
#include "Bluetooth.hpp"

bool noLine;
LineFollowingPID controller(0.6f, 0.2f, 0.001f, 0.001f);
Timeout stop;
// QEI encoder_left(PA_6, PA_7, NC, int 255);
Timer t;
AnalogIn sensor[] = {AnalogIn(A0), AnalogIn(A1), AnalogIn(A2), AnalogIn(A3), AnalogIn(A4), AnalogIn(A5)};
PwmOut motor_left(PA_8);
PwmOut motor_right(PC_7);

DigitalOut ena(PB_2, 0);

BufferedSerial hm10(PA_11, PA_12, 9600);
float dt;
float averageSensors;
void controlledStop()
{

    ena = 0;

    // else{ena = 1; stop.detach();}
}
void buggyTurnaround(uint8_t cmd)
{
    motor_left.write(0.7);
    motor_right.write(0.7);
    ThisThread::sleep_for(1050ms);
    motor_left.write(0.5);
    motor_right.write(0.5);
    ThisThread::sleep_for(500ms);
    controller.reset();


}
EventQueue incoming(32*EVENTS_EVENT_SIZE);
Bluetooth bluetooth(
      &hm10,
      &incoming,
      std::vector<Bluetooth::Command>{
          Bluetooth::Command('T', 0xFF, callback(&buggyTurnaround)), //  LED Test
      },
      true);
  

int main()
{

    float averageSensors;
    ena = 1;
    motor_left.period_us(30);
    motor_right.period_us(30);

    //  QEI encoder()
    
    float setpoint = 0.7f;
    float sensorValues[6];
    LineFollowingPID speedControl(0.1f, 0.0f, 0.0f, dt);
    while (1)
    {
        incoming.dispatch_once();
        // int pulses = encoder_left.read();
        // printf("pulse %d", pulses);

        float ChangeOfSpeed = 0.0f;
        float PIDOutput = 0.0f;
        float sum = 0.0f;
        // averageSensors=0.0f;

        for (int i = 0; i < 6; i++)
        {
            sensorValues[i] = sensor[i].read();
            sum += sensorValues[i];
            averageSensors = sum / 6;
        }
        PIDOutput = controller.update(sensorValues, setpoint);
        float startSpeed_left = 0.7f;
        float startSpeed_right = 0.2f;
        float maxSpeed = 1.0f;
        if (PIDOutput >= 0.2)
        {
            PIDOutput = 0.2;
        }
        if (PIDOutput <= -0.2)
        {
            PIDOutput = -0.2;
        }
        motor_right.write(startSpeed_right + PIDOutput);
        motor_left.write(startSpeed_left + PIDOutput);
        // printf("%f\n", PIDOutput);
        // fflush(stdout);
        ThisThread::sleep_for(1ms);
        /* if(hm10.checkForData()=='t'){
             motor_left.write(0.7);
             motor_right.write(0.7);
             if(sensor[2]>0.5);
             hm10.c='0';
         }*/
        bool attached = false;
        if (averageSensors < 0.1f)
        {
            ThisThread::sleep_for(50ms);
            for (int i = 0; i < 6; i++)
            {
                sensorValues[i] = sensor[i].read();
                sum += sensorValues[i];
                averageSensors = sum / 6;}
            if(averageSensors<0.1f){
             ena = 0;
            }
                
                
                // stop.attach(&controlledStop,0.5);
            }
            else
            {
                ena = 1;
                stop.detach();
                attached = false;
            }
        }
    }
