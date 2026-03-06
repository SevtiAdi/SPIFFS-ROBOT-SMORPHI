#include <Bluepad32.h>
#include "smorphi_single.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "smorphi_odometry.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <uni.h>

#define LED_PIN 2
#define AXIS_DEAD_ZONE 20    // 4 
#define AXIS_XY_THRESHOLD 80     // 30
#define MAX_POSITIONS 100
#define WAYPOINTS_FILE "/waypoints.json"


#define ENCODER_FL_A 18
#define ENCODER_FL_B 19
#define ENCODER_FR_A 25
#define ENCODER_FR_B 23
#define ENCODER_RR_A 27
#define ENCODER_RR_B 26
#define ENCODER_RL_A 4
#define ENCODER_RL_B 5

#define USE_MPU

struct Pose {
  float x;
  float y;
  float theta;
  uint8_t shape;
};

ControllerPtr myController = nullptr;
Smorphi_single my_robot;

#ifdef USE_MPU
MPU6050 mpu(0x68);
float ypr[3];
int const INTERRUPT_PIN = 36;
VectorFloat gravity;
Quaternion q;
bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
#endif

volatile bool MPUInterrupt = false;
static const char* controller_addr_string = "79:6E:2A:AC:BC:B7";

ESP32Encoder encoder_fr, encoder_fl, encoder_rr, encoder_rl;
smorphi_odometry::MotorProperty_t *motor_prop_fl, *motor_prop_fr, *motor_prop_rl, *motor_prop_rr;
smorphi_odometry::Odometry_t *odometry;

static int stop = 0, stopRotate = 0, stopDpad = 0;
int currentSpeed = 50;
int currentRotSpeed = 350;
const int minSpeed = 50;
const int maxSpeed = 100;
const int minSpeedRot = 130;
const int maxSpeedRot = 270;
const int speedIncrement = 30;

unsigned long lastJoyUpdate;
Pose currentPose = {0, 0, 0, 0};
int savedCount = 0;

bool pendingSave = false;
// bool pendingClear = false;
// unsigned long clearRequestTime = 0;
Pose poseToSave;

bool initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    return false;
  }
  Serial.println("SPIFFS Mounted Successfully");
  
  // Print SPIFFS info
  // Serial.printf("Total: %d bytes\n", SPIFFS.totalBytes());
  // Serial.printf("Used: %d bytes\n", SPIFFS.usedBytes());
  return true;
}

void savePoseToSPIFFS(Pose p) {
  static unsigned long lastSave = 0;
  if (millis() - lastSave < 500) return;
  lastSave = millis();

  pendingSave = true;
  poseToSave = p;
}


void processPendingSave() {
  if (!pendingSave) return;

  unsigned long startTime = micros();
  // Load existing data
  StaticJsonDocument<8192> doc;
  
  File file = SPIFFS.open(WAYPOINTS_FILE, "r");
  if (file) {
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
      Serial.println("JSON parse error, creating new file");
      doc.clear();
    }
  }

  // Add new waypoint
  JsonArray waypoints = doc["waypoints"].isNull() ? doc.createNestedArray("waypoints") : doc["waypoints"];
  
  JsonObject newPose = waypoints.createNestedObject();
  newPose["x"] = poseToSave.x;
  newPose["y"] = poseToSave.y;
  newPose["theta"] = poseToSave.theta;
  newPose["shape"] = poseToSave.shape;

  // Save to file
  // file = SPIFFS.open(WAYPOINTS_FILE, "w");
  // if (!file) {
  //   Serial.println("Failed to open file for writing");
  //   return;
  // }

  // if (serializeJson(doc, file) == 0) {
  //   Serial.println("Failed to write JSON");
  // } else {
  //   Serial.printf("Saved Pose[%d]: x=%.2f, y=%.2f, th=%.2f\n",
  //                 waypoints.size() - 1, p.x, p.y, p.theta);
  //   savedCount = waypoints.size();
  // }
  // file.close();

  file = SPIFFS.open(WAYPOINTS_FILE, "w");
  if (file) {
    serializeJson(doc, file);
    file.close();
    savedCount = waypoints.size();

    unsigned long saveTime = (micros() - startTime) / 1000;
    Serial.printf("S[%d] %lums\n", savedCount - 1, saveTime);
  }

  pendingSave = false;
}

void listAllWaypoints() {
  File file = SPIFFS.open(WAYPOINTS_FILE, "r");
  if (!file) {
    Serial.println("⚠️ No waypoints file found");
    return;
  }

  StaticJsonDocument<8192> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }

  JsonArray waypoints = doc["waypoints"];
  Serial.println("\n=== Saved Waypoints ===");
  for (size_t i = 0; i < waypoints.size(); i++) {
    JsonObject wp = waypoints[i];
    Serial.printf("[%d]: x=%.3f, y=%.3f, theta=%.2f, shape=%d\n",
                  i, wp["x"].as<float>(), wp["y"].as<float>(),
                  wp["theta"].as<float>(), wp["shape"].as<uint8_t>());
  }
  Serial.printf("Total: %d waypoints\n", waypoints.size());
  Serial.println("========================\n");
}

void clearAllWaypoints() {
  if (SPIFFS.remove(WAYPOINTS_FILE)) {
    Serial.println("All waypoints cleared!");
    savedCount = 0;
  } // else {
  //   Serial.println("Failed to clear waypoints");
  // }
}


// Helper Functions

int getSpeedFromAxis(int axisValue) {
  int absVal = abs(axisValue);
  if (absVal < 50) return 0;
  if (absVal < 270) return currentSpeed * 0.4;
  if (absVal < 370) return currentSpeed * 0.7;
  return currentSpeed;
}

int getSpeedRotFromAxis(int axisValue) {
  int absVal = abs(axisValue);
  if (absVal < 50) return 0;
  if (absVal < 300) return currentRotSpeed;
  if (absVal < 400) return currentRotSpeed;
  return currentRotSpeed;
}

void resetMotors() {
  my_robot.sm_reset_M1();
}

void IRAM_ATTR DMPDataReady() {
  MPUInterrupt = true;
}

void handleSpecialButtons(ControllerPtr ctl) {
  static bool lastR1 = false, lastR2 = false;
  bool currentR1 = ctl->r1();
  bool currentR2 = ctl->r2();

  if (currentR1 && !lastR1) {
    currentSpeed = min(currentSpeed + speedIncrement, maxSpeed);
    currentRotSpeed = min(currentRotSpeed + speedIncrement, maxSpeedRot);
  }

  if (currentR2 && !lastR2) {
    currentSpeed = max(currentSpeed - speedIncrement, minSpeed);
    currentRotSpeed = max(currentRotSpeed - speedIncrement, minSpeedRot);
  }

  lastR1 = currentR1;
  lastR2 = currentR2;
}

void handleJoystickMovement(ControllerPtr ctl, int& stop) {
  int leftX = ctl->axisX();
  int leftY = ctl->axisY();
  int rightX = ctl->axisRX();
  
  int speedX = getSpeedFromAxis(leftX);
  int speedY = getSpeedFromAxis(leftY);
  int rotSpeed = getSpeedRotFromAxis(rightX);

  // ROTASI (Right stick) - Bisa jalan bareng sama movement
  if (rightX < -AXIS_DEAD_ZONE) {
    my_robot.CenterPivotLeft(rotSpeed);
    stopRotate = 1;
  } else if (rightX > AXIS_DEAD_ZONE) {
    my_robot.CenterPivotRight(rotSpeed);
    stopRotate = 1;
  } else if (abs(rightX) < AXIS_DEAD_ZONE && stopRotate == 1) {
    if (abs(leftX) < AXIS_DEAD_ZONE && abs(leftY) < AXIS_DEAD_ZONE) {
      my_robot.stopSmorphi_single();
      resetMotors();
    }
    stopRotate = 0;
  }
  
  // MOVEMENT (Left stick) - Jalan terus tanpa peduli right stick
  if (leftY < -AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
    my_robot.MoveForward(speedY);
    stop = 1;
  } else if (leftY > AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
    my_robot.MoveBackward(speedY);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
    my_robot.MoveRight(speedX);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
    my_robot.MoveLeft(speedX);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagUpLeft(spd);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagUpRight(spd);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagDownLeft(spd);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagDownRight(spd);
    stop = 1;
  } else if (abs(leftX) < AXIS_DEAD_ZONE && abs(leftY) < AXIS_DEAD_ZONE) {
    // Hanya stop kalau SEMUA stick diam
    if (stop == 1 && abs(rightX) < AXIS_DEAD_ZONE) {
      resetMotors();
      my_robot.stopSmorphi_single();
      stop = 0;
    }
  }
}
  // int leftX = ctl->axisX();
  // int leftY = ctl->axisY();
  // int rightX = ctl->axisRX();

  // if (abs(leftX) < AXIS_DEAD_ZONE) leftX = 0;
  // if (abs(leftY) < AXIS_DEAD_ZONE) leftY = 0;
  // if (abs(rightX) < AXIS_DEAD_ZONE) rightX = 0;

  // int speedX = getSpeedFromAxis(leftX);
  // int speedY = getSpeedFromAxis(leftY);
  // int rotSpeed = getSpeedRotFromAxis(rightX);

  // if (abs(rightX) > AXIS_DEAD_ZONE) {
  //   if (rightX < 0) {
  //     my_robot.CenterPivotLeft(rotSpeed);
  //   } else {
  //     my_robot.CenterPivotRight(rotSpeed);
  //   }
  //   stopRotate = 1;
  //   return;
  // } else if (stopRotate == 1) {
  //   stopRotate = 0;
  //   my_robot.stopSmorphi_single();
  //   resetMotors();
  // }

  // int absX = abs(leftX);
  // int absY = abs(leftY);

  // if (absX < AXIS_XY_THRESHOLD && absY > AXIS_DEAD_ZONE) {
  //   if (leftY < 0) {
  //     my_robot.MoveForward(speedY);
  //   } else {
  //     my_robot.MoveBackward(speedY);
  //   }
  //   stop = 1;
  // } else if (absY < AXIS_XY_THRESHOLD && absX > AXIS_DEAD_ZONE) {
  //   if (leftX > 0) {
  //     my_robot.MoveRight(speedX);
  //   } else {
  //     my_robot.MoveLeft(speedX);
  //   }
  //   stop = 1;
  // } else if (absX > AXIS_XY_THRESHOLD && absY > AXIS_XY_THRESHOLD) {
  //   int spd = min(speedX, speedY);

  //   if (leftX < 0 && leftY < 0) {
  //     my_robot.MoveDiagUpLeft(spd);
  //   } else if (leftX > 0 && leftY < 0) {
  //     my_robot.MoveDiagUpRight(spd);
  //   } else if (leftX < 0 && leftY > 0) {
  //     my_robot.MoveDiagDownLeft(spd);
  //   } else if (leftX > 0 && leftY > 0) {
  //     my_robot.MoveDiagDownRight(spd);
  //   }
  //   stop = 1;
  // }

  // else if (absX <= AXIS_DEAD_ZONE && absY <= AXIS_DEAD_ZONE) {
  //   if (stop == 1) {
  //     resetMotors();
  //     my_robot.stopSmorphi_single();
  //     stop = 0;
  //   }
  // }

  // if (rightX < -AXIS_DEAD_ZONE) {
  //   my_robot.CenterPivotLeft(rotSpeed);
  //   stopRotate = 1;
  // } else if (rightX > AXIS_DEAD_ZONE) {
  //   my_robot.CenterPivotRight(rotSpeed);
  //   stopRotate = 1;
  // } else if (abs(rightX) < AXIS_DEAD_ZONE || stopRotate == 1) {
  //   stopRotate = 0;
  //   my_robot.stopSmorphi_single();
  //   resetMotors();
  // }
  
  // if (leftY < -AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
  //   my_robot.MoveForward(speedY);
  //   stop = 1;
  // } else if (leftY > AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
  //   my_robot.MoveBackward(speedY);
  //   stop = 1;
  // } else if (leftX > AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
  //   my_robot.MoveRight(speedX);
  //   stop = 1;
  // } else if (leftX < -AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
  //   my_robot.MoveLeft(speedX);
  //   stop = 1;
  // } else if (leftX < -AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
  //   int spd = min(speedX, speedY);
  //   my_robot.MoveDiagUpLeft(spd);
  //   stop = 1;
  // } else if (leftX > AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
  //   int spd = min(speedX, speedY);
  //   my_robot.MoveDiagUpRight(spd);
  //   stop = 1;
  // } else if (leftX < -AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
  //   int spd = min(speedX, speedY);
  //   my_robot.MoveDiagDownLeft(spd);
  //   stop = 1;
  // } else if (leftX > AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
  //   int spd = min(speedX, speedY);
  //   my_robot.MoveDiagDownRight(spd);
  //   stop = 1;
  // } else if (abs(leftX) < AXIS_DEAD_ZONE && abs(leftY) < AXIS_DEAD_ZONE || stop == 1) {
  //   resetMotors();
  //   my_robot.stopSmorphi_single();
  //   stop = 0;
  // }
// }

void handleDpadMovement(ControllerPtr ctl) {
  uint8_t curr_dpad = ctl->dpad();
  if (curr_dpad == 0x01) {
    my_robot.MoveForward(60);
    stopDpad = 1;
  } else if (curr_dpad == 0x02) {
    my_robot.MoveBackward(60);
    stopDpad = 1;
  } else if (curr_dpad == 0x08) {
    my_robot.MoveLeft(60);
    stopDpad = 1;
  } else if (curr_dpad == 0x04) {
    my_robot.MoveRight(60);
    stopDpad = 1;
  } else if (curr_dpad > 0 || stopDpad == 1) {
    my_robot.stopSmorphi_single();
    resetMotors();
    stopDpad = 0;
  }
}


void processGamePad(ControllerPtr ctl) {
  static uint8_t prev_buttons = 0;
  handleSpecialButtons(ctl);
  handleJoystickMovement(ctl, stop);
  handleDpadMovement(ctl);

  uint8_t curr_buttons = ctl->buttons();

  // processClearRequest(ctl);

  if (ctl->buttons() > 0 && prev_buttons != ctl->buttons()) {
    switch (curr_buttons) {
      case 0x0010:  // L1 - Save pose
        savePoseToSPIFFS(currentPose);
        currentPose.x = 0;
        currentPose.y = 0;
        odometry->setResetPose();
        break;
      
      case 0x0040:  // L2 - Reset pose
        currentPose.x = 0;
        currentPose.y = 0;
        odometry->setResetPose();
        Serial.println("Pose reset");
        break;
      
      case 0x0100:  // Select - List all waypoints
        listAllWaypoints();
        break;
      
      // case 0x0200:  // Start - Clear all waypoints
      //   Serial.println("⚠️ Clearing all waypoints in 3 seconds...");
      //   // delay(3000);
      //   // requestClearWaypoints();
      //   clearAllWaypoints();
      //   break;
      
      default:
        break;
    }
  }
  prev_buttons = curr_buttons;
}

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.printf("Controller is connected\n");
    myController = ctl;
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.printf("Controller disconnected\n");
    myController = nullptr;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize SPIFFS
  if (!initSPIFFS()) {
    Serial.println("System halted due to SPIFFS error");
    while (true) delay(1000);
  }

  Serial.println("Clearing previous waypoints...");
  clearAllWaypoints();
  Serial.println("All waypoints cleared!");
  Serial.println("Starting fresh...\n");

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  bd_addr_t controller_addr;
  sscanf_bd_addr(controller_addr_string, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  
  my_robot.BeginSmorphi_single();
  Wire.begin();

  // Setup encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_fl.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
  encoder_fr.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoder_rr.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoder_rl.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);

  encoder_fr.clearCount();
  encoder_fl.clearCount();
  encoder_rl.clearCount();
  encoder_rr.clearCount();

  motor_prop_fl = new smorphi_odometry::MotorProperty_t(&encoder_fl, my_robot.sm_wheel_radius, 540);
  motor_prop_fr = new smorphi_odometry::MotorProperty_t(&encoder_fr, my_robot.sm_wheel_radius, 540);
  motor_prop_rl = new smorphi_odometry::MotorProperty_t(&encoder_rl, my_robot.sm_wheel_radius, 540);
  motor_prop_rr = new smorphi_odometry::MotorProperty_t(&encoder_rr, my_robot.sm_wheel_radius, 540);
  
  odometry = new smorphi_odometry::Odometry_t(motor_prop_fl, motor_prop_fr, motor_prop_rl, motor_prop_rr,
                                              my_robot.sm_wheel_x, my_robot.sm_wheel_y);

#ifdef USE_MPU
  mpu.initialize();
  // Serial.printf("IMU ID: %d\n", mpu.getDeviceID());
  pinMode(INTERRUPT_PIN, INPUT);

  if (mpu.getDeviceID() == 0x52) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }

  Serial.println("MPU6050 connected");
  // Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("Active offsets:");
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.printf("DMP init failed (code %d)\n", devStatus);
  }
#endif

  // List existing waypoints
  listAllWaypoints();
  Serial.println("Ready to Position Robot");
  
  // Serial.println("\n Controls:");
  // Serial.println("  L1: Save waypoint");
  // Serial.println("  L2: Reset position");
  // Serial.println("  SELECT: List waypoints");
  // Serial.println("  START (hold 3s): Clear all\n");
}

void loop() {
  unsigned long now = millis();

  #ifdef USE_MPU
  static unsigned long lastIMURead = 0;
  static unsigned long lastIMUSuccess = 0;

  if (now - lastIMURead >= 5) {
    lastIMURead = now;

    uint8_t intStatus = mpu.getIntStatus();
    uint16_t fifoCount = mpu.getFIFOCount();
    
    // Overflow detection - lebih agresif
    if ((intStatus & 0x10) || fifoCount > 1000) {  
      mpu.resetFIFO();
      mpu.setDMPEnabled(true);
      delay(1);  // Beri waktu reset
      Serial.println("FIFO overflow!");
      lastIMUSuccess = now;
    }
    // Data ready
    else if ((intStatus & 0x02) && fifoCount >= packetSize) {
      // KOSONGKAN SEMUA PACKET YANG ADA
      while (fifoCount >= packetSize) {
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
          mpu.dmpGetQuaternion(&q, FIFOBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          
          currentPose.theta = fmod(-ypr[0] * 180 / M_PI, 360);
          if (currentPose.theta < 0) currentPose.theta += 360;

          odometry->setOrientation(currentPose.theta * M_PI / 180.0);
          lastIMUSuccess = now;
        }
        fifoCount = mpu.getFIFOCount();  // Update count
      }
    }

    // Watchdog
    if (now - lastIMUSuccess > 300) {
      mpu.resetFIFO();
      mpu.setDMPEnabled(true);
      lastIMUSuccess = now;
    }
  }
  #endif

//   #ifdef USE_MPU
//   static unsigned long lastIMUSuccess = 0;

//   if (MPUInterrupt) {
//     MPUInterrupt = false;
  
//   uint8_t intStatus = mpu.getIntStatus();
//   uint16_t fifoCount = mpu.getFIFOCount();

//   if ((intStatus & 0x10) || fifoCount >= 1024) {
//     Serial.println("Fifo overflow, resetting...");
//     mpu.resetFIFO();
//     lastIMUSuccess = millis();
//     return;
//   }

//   if (intStatus & 0x02) {
//     while (fifoCount >= packetSize) {
//       mpu.getFIFOBytes(FIFOBuffer, packetSize);
//       fifoCount -= packetSize;
//     }

//     mpu.dmpGetQuaternion(&q, FIFOBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//     currentPose.theta = fmod(-ypr[0] * 180.0 / M_PI, 360.0);
//     if (currentPose.theta < 0) currentPose.theta += 360.0;

//     odometry->setOrientation(currentPose.theta * M_PI / 180.0);
//     lastIMUSuccess = millis();
//   }
//   }

//   if (millis() - lastIMUSuccess > 500) {
//     Serial.println("IMU timeout, reinitinglizing...");
//     mpu.resetFIFO();
//     mpu.setDMPEnabled(true);
//     lastIMUSuccess = millis();
//   }
// #endif


  static unsigned long lastJoy = 0;
  if (now - lastJoy >= 10) {
    lastJoy = now;
    if (BP32.update()) {
      if (myController && myController->isConnected() && myController->isGamepad()) {
        processGamePad(myController);
      }
    }
  }

  static unsigned long lastOdom = 0;
  if (now - lastOdom >= 20) {
    lastOdom = now;
    
    motor_prop_fl->update();
    motor_prop_fr->update();
    motor_prop_rl->update();
    motor_prop_rr->update();
    odometry->update();

    currentPose.x = odometry->pos_x();
    currentPose.y = odometry->pos_y();
  }

  static unsigned long lastSaveCheck = 0;
  if (now - lastSaveCheck >= 100) {
    lastSaveCheck = now;
    processPendingSave();
  }

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.printf("x=%.3f y=%.3f th=%.3f\n", currentPose.x, currentPose.y, currentPose.theta);
  }
}