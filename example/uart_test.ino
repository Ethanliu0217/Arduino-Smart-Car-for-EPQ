
#define motor_mode   1  //0代表520电机   1代表TT马达    2代表310电机
#define encoder_mode  0  //设置编码方向极性:默认0，设置1取反
#define  car_mode    1   //设置车子类型 0是麦克纳姆轮  1是四路差速   2是阿克曼   3是履带车

// ==================== 串口通信配置 ====================
#define SERIAL_BAUD_RATE 115200

// ==================== 读取电池电压 ====================
uint16_t getBatteryVoltage(void) {
  uint16_t voltage_mv = 0;
  
  // 发送读取指令
  Serial.print("$MOTOR_4CH_READ:battery!");
  
  // 等待返回值（超时100ms）
  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      String response = Serial.readStringUntil('!');
      if (response.startsWith("$MOTOR_4CH_Battery:")) {
        voltage_mv = response.substring(20).toInt();
        break;
      }
    }
  }
  
  Serial.print("Battery Voltage: ");
  Serial.print(voltage_mv);
  Serial.println(" mV");
  return voltage_mv;
}

// ==================== 读取20ms编码器增量 ====================
void getEncoder20msValue(void) {
  int16_t encoderA = 0, encoderB = 0, encoderC = 0, encoderD = 0;
  
  // 发送读取指令
  Serial.print("$MOTOR_4CH_READ:encoder_20ms!");
  
  // 等待返回值（超时100ms）
  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      String response = Serial.readStringUntil('!');
      if (response.startsWith("$MOTOR_4CH_Encoder_20ms:")) {
        String data = response.substring(24);
        int idx1 = data.indexOf(',');
        int idx2 = data.indexOf(',', idx1+1);
        int idx3 = data.indexOf(',', idx2+1);
        
        encoderA = data.substring(0, idx1).toInt();
        encoderB = data.substring(idx1+1, idx2).toInt();
        encoderC = data.substring(idx2+1, idx3).toInt();
        encoderD = data.substring(idx3+1).toInt();
        break;
      }
    }
  }

  Serial.print("Encoder 20ms: ");
  Serial.print(encoderA);
  Serial.print(" | ");
  Serial.print(encoderB);
  Serial.print(" | ");
  Serial.print(encoderC);
  Serial.print(" | ");
  Serial.println(encoderD);
}

// ==================== 读取编码器累计值 ====================
void getEncoderTotalValue(void) {
  int32_t encoderA = 0, encoderB = 0, encoderC = 0, encoderD = 0;
  
  // 发送读取指令
  Serial.print("$MOTOR_4CH_READ:encoder_total!");
  
  // 等待返回值（超时100ms）
  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      String response = Serial.readStringUntil('!');
      if (response.startsWith("$MOTOR_4CH_Encoder_Total:")) {
        String data = response.substring(25);
        int idx1 = data.indexOf(',');
        int idx2 = data.indexOf(',', idx1+1);
        int idx3 = data.indexOf(',', idx2+1);
        
        encoderA = data.substring(0, idx1).toInt();
        encoderB = data.substring(idx1+1, idx2).toInt();
        encoderC = data.substring(idx2+1, idx3).toInt();
        encoderD = data.substring(idx3+1).toInt();
        break;
      }
    }
  }

  Serial.print("Encoder Total: ");
  Serial.print(encoderA);
  Serial.print(" | ");
  Serial.print(encoderB);
  Serial.print(" | ");
  Serial.print(encoderC);
  Serial.print(" | ");
  Serial.println(encoderD);
}

// ==================== 设置电机类型====================
void setMotorType(uint8_t motorType) {
  Serial.print("$MOTOR_4CH_SET:");
  Serial.print(motorType);
  Serial.print("!");
  
  // 等待确认（可选，超时50ms）
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      Serial.readStringUntil('!'); // 丢弃确认信息
      break;
    }
  }
}

// ==================== 设置编码器极性====================
void setEncoderPolarity(uint8_t polarity) {
  Serial.print("$MOTOR_4CH_SET_ENCPDER_POLARITY:");
  Serial.print(polarity);
  Serial.print("!");
  
  // 等待确认（可选，超时50ms）
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      Serial.readStringUntil('!'); // 丢弃确认信息
      break;
    }
  }
}

// ==================== 电机开环PWM控制 ====================
void setMotorPWM(float pwmA, float pwmB, float pwmC, float pwmD) {
  // 按照要求：设置的值/100适配其他产品
  Serial.print("$Car_Pwm:");
  Serial.print(pwmA * 100.0);
  Serial.print(",");
  Serial.print(pwmB * 100.0);
  Serial.print(",");
  Serial.print(pwmC * 100.0);
  Serial.print(",");
  Serial.print(pwmD * 100.0);
  Serial.print("!");
}

// ==================== 电机闭环速度控制 ====================
void setMotorSpeed(float speedA, float speedB, float speedC, float speedD) {
  Serial.print("$Car:");
  Serial.print(speedA * 100.0);
  Serial.print(",");
  Serial.print(speedB * 100.0);
  Serial.print(",");
  Serial.print(speedC * 100.0);
  Serial.print(",");
  Serial.print(speedD * 100.0);
  Serial.print("!");
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  // 等待串口稳定
  delay(100);
  
  // 电机驱动初始化配置（和原来完全一样）
  setMotorType(motor_mode);              // 设置电机类型
  setEncoderPolarity(encoder_mode);       // 设置编码器极性
  setMotorSpeed(0.1, 0.1, 0.1, 0.1); // 设置电机速度

  Serial.println("Motor Driver Serial Init OK!");
}


  void loop() {
  // 静态变量，记录上一次执行的时间（毫秒）
  static unsigned long lastTime = 0;

  // 未到 20ms 直接返回
  if (millis() - lastTime < 20)
    return;

  // 更新执行时间
  lastTime = millis();

  // 回读并上传 20ms 编码器增量
  getEncoder20msValue();
}
