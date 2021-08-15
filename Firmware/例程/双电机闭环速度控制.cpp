#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(PB7, PB6);
TwoWire I2Ctwo = TwoWire(PB11, PB10);


BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2, PA3);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA6, PA7, PB0, PB1);

//命令设置
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

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
  
  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  //速度PI环设置
  motor.PID_velocity.P = 0.1;
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 5;
  motor1.PID_velocity.I = 5;
  motor.PID_velocity.output_ramp = 1000.0;
  motor1.PID_velocity.output_ramp = 1000.0;
  //角度P环设置 
  motor.P_angle.P = 20.0;
  motor.P_angle.I = 0.1;
  motor.P_angle.D = 0.4;
  motor1.P_angle.P = 20.0;
  motor1.P_angle.I = 0.1;
  motor1.P_angle.D = 0.4;
  //最大电机限制电机
  motor.voltage_limit = 12;
  motor1.voltage_limit = 12;
  
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 100;
  motor1.velocity_limit = 100;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //初始化电机
  motor.init();
  motor1.init();
  //初始化 FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);

  command.run();
}