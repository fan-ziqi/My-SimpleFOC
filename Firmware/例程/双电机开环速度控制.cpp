#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(PB7, PB6);
TwoWire I2Ctwo = TwoWire(PB11, PB10);


BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2, PA3);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA6, PA7, PB0, PB1);

//目标变量
float target_velocity = 5;

//串口指令设置
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 12;   // [V]
  motor.velocity_limit = 15; // [rad/s]
  
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 12;   // [V]
  motor1.velocity_limit = 15; // [rad/s]

 
  //开环控制模式设置
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  //初始化硬件
  motor.init();
  motor1.init();


  //增加 T 指令
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);
  motor1.move(target_velocity);

  //用户通讯
  command.run();
}