class LineFollowingPID
{
private:
    float Kp;
    float Ki;
    float Kd;
    float deltaTime;
    float errorSum;
    float lastError;
    float output; 

public:
    LineFollowingPID(float kp, float ki, float kd, float dt) : Kp(kp), Ki(ki), Kd(kd), deltaTime(dt)
    {
        errorSum = 0.0f;
        lastError = 0.0f;
    }
    float update(float *sensorValues, float setpoint)
    {
        float averageSensorValue = 0.0f;
        float weight[] = {-2, -1, 0.5, 0.5, 1, 2};
        float weightedSum = 0.0f;
        float SumOfWeights = 0.0f;

        for (int i = 0; i < 6; i++)
        {
            averageSensorValue += sensorValues[i];
            weightedSum += sensorValues[i] * weight[i];
            SumOfWeights += weight[i];
        }
        float estimates_line_position = weightedSum / SumOfWeights;
        averageSensorValue /= 6.0f;

        printf("Line %f\n", estimates_line_position);
        fflush(stdout);
        float error = setpoint - estimates_line_position;

        float proportional = Kp * error;
        errorSum += Ki * error * deltaTime;
        float derivative = Kd * (error - lastError) / deltaTime;

        float output = proportional + errorSum + derivative;

        lastError = error;

        return output;
    }
    float calculateSpeed(float DesiredSpeed, float current_speed){
        float error = DesiredSpeed - current_speed;
        float output = Kp * error + Ki*error*deltaTime + Kd * (error - lastError) / deltaTime;
        lastError = error;
        return output;

    }
    void reset(){
        output = 0.0f; 
        lastError = 0.0f;
    }
};