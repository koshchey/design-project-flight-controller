# Notes

## Sensors
`i2c.c` defines i2c master (which is the esp32 chip)
`sensors.c` mpu6050 data and sensor task

## Control
To-do: PID

## WebSocket
 - To-do:
    - integrate properly
    -  commands: hover, off

Note: 
 - Need to set CONFIG_HTTPD_WS_SUPPORT=y

## To Do
 - Blocked:
   - Fix the position estimate -> requires TOF 
 - High:
    - Test the low-pass filter [X]
    - Add check to see if entire system has started up before running stabiliser [X?]
 - Med:
    - Implement other tests
    - Convert fixed with string passing to binary for setpoints [X]
    - Add timestamps for proper rate estimation [in-progress]
    - Check whether the update rate is correct
    - Add something so that if the websocket doesn't reconnect the main doesn' t crash [check]
    - Check how timeout works
 - Low: 
    - Convert fixed to binary for gyro/acce 
    - Check if accelerometer scale is needed 
    - Change gyro bias calibration (make it better including variance)
    - Check if sensor is misaligned initially
    - Change estimator functions to be const
    - Consider adding calibration for the accelerometer

Original PID Constants:
#define PID_ROLL_RATE_KP  250.0
#define PID_ROLL_RATE_KI  500.0
#define PID_ROLL_RATE_KD  2.5
#define PID_ROLL_RATE_INT_LIMIT 33.3

#define PID_PITCH_RATE_KP  250.0
#define PID_PITCH_RATE_KI  500.0
#define PID_PITCH_RATE_KD  2.5
#define PID_PITCH_RATE_INT_LIMIT 33.3

#define PID_YAW_RATE_KP  120.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INT_LIMIT 166.7

#define PID_ROLL_KP  5.9
#define PID_ROLL_KI  2.9
#define PID_ROLL_KD  0.0
#define PID_ROLL_INT_LIMIT 20.0

#define PID_PITCH_KP  5.9
#define PID_PITCH_KI  2.9
#define PID_PITCH_KD  0.0
#define PID_PITCH_INT_LIMIT 20.0

#define PID_YAW_KP  6.0
#define PID_YAW_KI  1.0
#define PID_YAW_KD  0.35
#define PID_YAW_INT_LIMIT 360.0

#define DEFAULT_PID_INT_LIMIT 5000.0
#define DEFAULT_PID_OUT_LIMIT 0.0

#define ATT_LPF_FC 15.0f
#define ATT_LPF_EN false
#define ATT_RATE_LPF_FC 30.0f
#define ATT_RATE_LPF_EN false
