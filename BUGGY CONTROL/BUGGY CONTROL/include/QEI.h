


// https://os.mbed.com/users/benson516/code/QEI/file/4682b3d415bd/QEI.cpp/

#ifndef _QEI_H_
#define _QEI_H_

#include "mbed.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PREV_MASK 0x01 //Mask for the previous state in determining direction of rotation
#define CURR_MASK 0x02 //Mask for the current state in determining direction of rotation
#define INVALID 0x03   //XORing two states where both bits have changed

/**
 * Quadrature Encoder Interface.
 */
class QEI
{

public:
    typedef enum Encoding
    {
        X2_ENCODING,
        X4_ENCODING
    } Encoding;

    /**
     * Contructor
     * Read the current values on channel A and B to determine the initial state
     * 
     * Attaches the encode fuction to rise/fall interrupt edges of channels A and B to perform X4 encoding.
     * Attaches the index fuction to the rise interrupt edge of channel index(if it is used) to count revolutions.
     * 
     * @param channelA mbed pin for channel A input
     * @param channelB mbed pin for channel B input
     * @param index mbed pin for optional index channel input, (pass NC if not needed).
     * @param encoding The encoding to use. Uses X2 encoding by default.
     */
    QEI(PinName channelA, PinName channelB, PinName index, Encoding encoding = X4_ENCODING);

    /**
     * Destructor
     */
    ~QEI();

    /**
     * Reset the encoder.
     * 
     * Sets the pulses and revolutions count to zero.
     */
    void reset();

    /**
     * Read the number of pulses recorded by the encoder.
     * @return Number of pulses which have occured.
     */
    int read();

    /**
     * Sets the number of pulses
     * @param pulses Number of pulses which to set.
     */
    void write(int pulses);

    /**
     * Sets the factor for the getter-functions to convert in another unit.
     * (1.0=Hz, 1/(X*CPR)=rps, 1/(60*X*CPR)=rpm, 360/(X*CPR)=°/s)
     * Where X is encoding type [e.g. X4 encoding => X=4]
     * @param fSpeedFactor - factor to scale from Hz to user unit
     */
    void setSpeedFactor(float fSpeedFactor);

    /**
     * Gets the speed as float value.
     * @return speed The value is scales by the factor set by setSpeedFactor()
     */
    float getSpeed();

    /**
     * Sets the factor for the getter-functions to convert in another unit.
     * (1.0=Count, 1/(X*CPR)=revolution, 360/(X*CPR)=deg, (2*pi)/(X*CPR)=rad)
     * Where X is encoding type [e.g. X4 encoding => X=4]
     * @param fPositionFactor - factor to scale from counts to user unit
     */
    void setPositionFactor(float fPositionFactor);

    /**
     * Gets the position as float value.
     * @return position The value is scales by the factor set by setPositionFactor()
     */
    float getPosition();

protected:
    /**
     * Update the pulse count
     * Called on every rising/falling edge of channels A/B
     * 
     * Read the state of the channels and determines whether a pulse forward or backward has occured, update the count appropriately.
     */
    void encode();

    /**
     * Called on every rising edge of channel index to update revolution count by one
     */
    void index();

    InterruptIn _channelA;
    InterruptIn _channelB;
    InterruptIn _index;

    Encoding _encoding;
    int _prevState;
    int _currState;

    volatile int _pulses;
    volatile int _revolutions;

    float _fSpeedFactor;
    float _fPositionFactor;

    Timer _SpeedTimer;

    unsigned int _nSpeedLastTimer;
    unsigned int _nSpeedTimeoutMax;
    unsigned int _nSpeedTimeoutCount;
    int _nSpeedAvrTimeSum;
    int _nSpeedAvrTimeCount;
    float _fLastSpeed;
    float _fSpeed;
};

#endif
