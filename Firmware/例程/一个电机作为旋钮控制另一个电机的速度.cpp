#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(PB7, PB6);
TwoWire I2Ctwo = TwoWire(PB11, PB10);


BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2, PA3);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA6, PA7, PB0, PB1);

//定义 TROT 步态变量
void setup() {
  I2Cone.begin();   //SDA1,SCL1
  I2Ctwo.begin();
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver1.voltage_power_supply = 12;
  driver1.init();
  //连接电机和driver对象
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::velocity;

  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;
  motor1.voltage_limit = 12;

  motor1.PID_velocity.P = 0.1;
  motor1.PID_velocity.I = 5;
  motor1.PID_velocity.output_ramp = 1000.0;

  motor1.LPF_velocity.Tf = 0.02;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  //记录无刷初始位置

  
  //初始化电机
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move(5*(motor1.shaft_velocity/10 - motor.shaft_angle));
  motor1.move(10*dead_zone(motor.shaft_angle));
}

