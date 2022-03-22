// Hello World to sweep a servo through its full range
 
#include "mbed.h"
#include "PwmOut.h"
#include "library/inverted_pendulum.h"
#include "library/BNO055.h"

 
PwmOut motor_pwm(D5);
AnalogIn potentiometer(A0);
DigitalOut motor_brake(D2);
BNO055 imu(PB_9, PB_8);
bool control_flag = false;

// Timer
Ticker controltime;

RawSerial pc(USBTX, USBRX); // tx, rx

// Interrupt service routine for the controller.
void UpdateControlFlag() {
    control_flag = true;
}
 
int main() {
    pc.baud(9600);
    pc.printf("here we go\n");

    // Attach timer to interrup routine.
    const double control_freq = 200.0;
    controltime.attach(&UpdateControlFlag, 1 / control_freq);

    // Imu initilization.
    while (!imu.check()){
        // Let the imu initialize
        pc.printf("checking imu.\n");
    }
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_angles();
    pc.printf("%f, %f, %f\n", imu.euler.roll, imu.euler.pitch, imu.euler.yaw);

    int i = 0;
    motor_pwm = 0.0;
    motor_pwm.period_us(40);

    double roll = 0.0;
    while (true) {
        motor_brake = 1;
        motor_pwm = potentiometer.read();

        if (i % 10000) {
            pc.printf("%f\n", potentiometer.read());
            i = 0;
        }

        // This is how you read the imu.
        imu.get_angles();
        roll = imu.euler.roll;

        ++i;
    }
}