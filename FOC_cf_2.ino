#include <SimpleFOC.h>
#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
// esp32 2.0.17
// code for the leg-decouple hopping robot (finished)

//#define GAIN_TUNING
//#define CURRENT_SENSE

//#define DIR_FACTOR -1

#define SKIP_IDENTIFICATION
#define ROTATION_DIR CCW
#define ZERO_ELECTRIC_ANGLE 5.6642

#define SENSOR_OFFSET -4.9466

#define VOLTAGE_LIMIT 7.0
#define VOLTAGE_LIMIT_RAMPUP 0.0006f

# define VELOCITY_STREAMING


bool disable_motor_flag = false;
MagneticSensorMT6701SSI sensor(5);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);     // 电机极对数
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

#ifdef CURRENT_SENSE
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.01f, 50.0f, 34, 35, _NC);
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 34, 35, _NC);
#endif

union {
  float f_value;
  unsigned char bytes[4];
} motor_target_temp;

union {
  float f_value;
  unsigned char bytes[4];
} motor_angle;
#ifdef VELOCITY_STREAMING
union {
  float f_value;
  unsigned char bytes[4];
} motor_velocity;
#endif
union {
  float f_value;
  unsigned char bytes[4];
} motor_q;

uint8_t loop_tick = 0;

float motor_target;

#ifdef GAIN_TUNING
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar2(&motor_target, cmd);  // 串口控制指令：目标值
}
void onMotor(char* cmd) {
  command.motor(&motor, cmd);  // 串口控制指令：电机
}
#endif

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  Serial.begin(115200);
  // 编码器设置
  sensor.init();



  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
#ifdef CURRENT_SENSE
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
#endif
  motor.foc_modulation = FOCModulationType::SinePWM;
  //  Torque - MotionControlType::torque
  //  Velocity - MotionControlType::velocity
  //  Angle - MotionControlType::angle
  motor.controller = MotionControlType::torque;
  //  Voltage - TorqueControlType::voltage
  //  DC current - TorqueControlType::dc_current
  //  FOC current - TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::voltage;



  // velocity loop PID
  motor.PID_velocity.P = 4.5;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 0.0;
  motor.PID_velocity.limit = VOLTAGE_LIMIT;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 16.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.3;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 10.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.01;


  // Limits
  motor.velocity_limit = 50.0;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.current_limit = 3.0;


#ifdef GAIN_TUNING
  motor.useMonitoring(Serial);  //使用串口监视器
#endif
#ifdef CURRENT_SENSE
  current_sense.gain_a = current_sense.gain_a * -1;
  current_sense.gain_b = current_sense.gain_b * -1;
  current_sense.skip_align = true; // skip alignment procedure
#endif

#ifdef SKIP_IDENTIFICATION
  motor.zero_electric_angle  = ZERO_ELECTRIC_ANGLE; // rad  (for robot)
  motor.sensor_direction = Direction::ROTATION_DIR; // CW or CCW
#endif

  //初始化
  motor.init();
  motor.initFOC();

#ifdef GAIN_TUNING    // 添加串口命令
  command.add('T', doTarget, "target angle");
  command.add('M', onMotor, "my motor");
#endif
  _delay(1000);
  motor_target = 0.0f;
#ifdef VELOCITY_STREAMING
  Serial.println("MV"); // motor ready, with velocity streaming
#else
  Serial.println("MR"); // motor ready
#endif



//  motor.loopFOC();
//  motor.move(motor_target);
//  motor.loopFOC();
//  motor.move(motor_target);

//  if (sensor.getAngle() >= 3.14)
//    motor.sensor_offset = SENSOR_OFFSET - _2PI;
//  else
//    motor.sensor_offset = SENSOR_OFFSET;
  motor.sensor_offset = SENSOR_OFFSET;
}

void loop() {
  loop_tick++;
  motor.loopFOC();
  motor.move(motor_target);

#ifdef GAIN_TUNING
  motor.monitor();    //使用simpleFOC Studio上位机设置的时候，这句一定要打开。但是会影响程序执行速度
  command.run();
#endif

#ifndef GAIN_TUNING

  if (Serial.available()) {
    // serial receive
    char inChar = (char)Serial.read();
    if (inChar == 'T') {
      //  set motor target angle
      // T + float + E
      motor_target_temp.bytes[0] = (char)Serial.read();
      motor_target_temp.bytes[1] = (char)Serial.read();
      motor_target_temp.bytes[2] = (char)Serial.read();
      motor_target_temp.bytes[3] = (char)Serial.read();
      inChar = (char)Serial.read();
      if (inChar == 'E') {
        motor_target = motor_target_temp.f_value;
        motor.controller = MotionControlType::angle;
        disable_motor_flag = false;
      }
    }
    else if (inChar == 'Q') {
      //  disable motor
      // Q + E
      inChar = (char)Serial.read();
      inChar = (char)Serial.read();
      inChar = (char)Serial.read();
      inChar = (char)Serial.read();
      inChar = (char)Serial.read();
      if (inChar == 'E') {
        //        motor_target = 0;
        motor.controller = MotionControlType::torque;
        disable_motor_flag = true;
        motor.voltage_limit = 0.0;
      }
    }

    // serial send
    motor_angle.f_value = motor.shaft_angle;
    motor_q.f_value = - motor.voltage.q;
//    motor_q.f_value = current_sense.getQCurrent();
#ifdef VELOCITY_STREAMING
    motor_velocity.f_value = motor.shaft_velocity;
#endif
    Serial.write('A');
    Serial.write(motor_angle.bytes, 4);
    Serial.write(motor_q.bytes, 4);
#ifdef VELOCITY_STREAMING
    Serial.write(motor_velocity.bytes, 4);
#endif

    if (!disable_motor_flag && motor.voltage_limit <= VOLTAGE_LIMIT) {
      motor.voltage_limit += VOLTAGE_LIMIT_RAMPUP;
    }
  }


#endif
}
