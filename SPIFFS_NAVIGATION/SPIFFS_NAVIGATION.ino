#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "smorphi_single.h"
#include "math.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "smorphi_odometry.h"

#define MAX_POSITIONS 100
#define WAYPOINTS_FILE "/waypoints.json"
#define LINEAR_TOLERANCE 0.025   
#define ANGULAR_TOLERANCE 2.0    

struct Pose {
  float x;
  float y;
  float theta;
  uint8_t shape;
};

Pose targetPose = {0, 0, 0, 0};
Pose waypoints[MAX_POSITIONS];
int totalWaypoints = 0;

bool initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    return false;
  }
  Serial.println("SPIFFS Mounted Successfully");
  Serial.printf("Total: %d bytes | Used: %d bytes\n", 
                SPIFFS.totalBytes(), SPIFFS.usedBytes());
  return true;
}

int loadWaypointsFromSPIFFS() {
  File file = SPIFFS.open(WAYPOINTS_FILE, "r");
  if (!file) {
    Serial.println("No waypoints file found!");
    return 0;
  }

  StaticJsonDocument<8192> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse JSON");
    return 0;
  }

  JsonArray waypointsArray = doc["waypoints"];
  if (waypointsArray.isNull()) {
    Serial.println(" No waypoints in file");
    return 0;
  }

  int count = 0;
  Serial.println("\n=== Loading Waypoints ===");
  for (JsonObject wp : waypointsArray) {
    if (count >= MAX_POSITIONS) break;
    
    waypoints[count].x = wp["x"] | 0.0f;
    waypoints[count].y = wp["y"] | 0.0f;
    waypoints[count].theta = wp["theta"] | 0.0f;
    waypoints[count].shape = wp["shape"] | 0;
    
    Serial.printf("[%d]: x=%.3f, y=%.3f, theta=%.2f, shape=%d\n",
                  count, waypoints[count].x, waypoints[count].y,
                  waypoints[count].theta, waypoints[count].shape);
    count++;
  }
  Serial.printf("Loaded %d waypoints\n", count);
  Serial.println("=========================\n");
  
  return count;
}

MPU6050 mpu(0x68);
float ypr[3] = {0, 0, 0};
int const INTERRUPT_PIN = 36;
VectorFloat gravity;
Quaternion q;

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
volatile bool MPUInterrupt = false;

#define ENCODER_FL_A 18
#define ENCODER_FL_B 19
#define ENCODER_FR_A 25
#define ENCODER_FR_B 23
#define ENCODER_RR_A 27
#define ENCODER_RR_B 26
#define ENCODER_RL_A 4
#define ENCODER_RL_B 5

unsigned long lastUpdate = 0;

// PID parameters - SAMA SEPERTI EEPROM VERSION
double kp_rot = 120;
double ki_rot = 20;
double kd_rot = 1.0;

double kp_lin = 120;
double ki_lin = 20;
double kd_lin = 1.0;

double integral = 0.0, integral_lin = 0.0;
double lastError = 0.0, lastError_lin = 0.0;

ESP32Encoder encoder_fr, encoder_fl, encoder_rr, encoder_rl;
smorphi_odometry::MotorProperty_t *motor_prop_fl, *motor_prop_fr, *motor_prop_rl, *motor_prop_rr;
smorphi_odometry::Odometry_t *odometry;

Smorphi_single my_robot;

const double R = 0.03;
const double lx = 0.04925;
const double ly = 0.035;
const double L = lx + ly;
const int N = 540;
const double dt = 20;

Pose currentPose = {0, 0, 0, 0};
int targetIndex = 0;
uint8_t prev_shape = 0;
bool running = true;

void DMPDataReady() {
  MPUInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Smorphi Navigation with SPIFFS");
  
  if (!initSPIFFS()) {
    Serial.println("System halted due to SPIFFS error");
    while (true) delay(1000);
  }

  totalWaypoints = loadWaypointsFromSPIFFS();
  if (totalWaypoints == 0) {
    Serial.println("No waypoints to navigate!");
    Serial.println("Please run joystick program first to record waypoints.");
    while (true) delay(1000);
  }

  my_robot.BeginSmorphi_single();

  // Setup encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_fr.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoder_fl.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
  encoder_rr.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoder_rl.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);
  
  encoder_fr.clearCount();
  encoder_fl.clearCount();
  encoder_rl.clearCount();
  encoder_rr.clearCount();

  motor_prop_fl = new smorphi_odometry::MotorProperty_t(&encoder_fl, my_robot.sm_wheel_radius, N);
  motor_prop_fr = new smorphi_odometry::MotorProperty_t(&encoder_fr, my_robot.sm_wheel_radius, N);
  motor_prop_rl = new smorphi_odometry::MotorProperty_t(&encoder_rl, my_robot.sm_wheel_radius, N);
  motor_prop_rr = new smorphi_odometry::MotorProperty_t(&encoder_rr, my_robot.sm_wheel_radius, N);
  
  odometry = new smorphi_odometry::Odometry_t(motor_prop_fl, motor_prop_fr, motor_prop_rl, motor_prop_rr,
                                              my_robot.sm_wheel_x, my_robot.sm_wheel_y);

  // Setup MPU6050
  Wire.begin();
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  if (mpu.getDeviceID() == 0x52) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  
  Serial.println("MPU6050 connected");
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

  Serial.println("\nStarting navigation...\n");
  delay(2000);
}


void loop() {
  // Update IMU
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    currentPose.theta = fmod(-ypr[0] * 180 / M_PI, 360);
    if (currentPose.theta < 0) currentPose.theta += 360;
    
    odometry->setOrientation(currentPose.theta * M_PI / 180);
  }

  // Update odometry
  static unsigned long lastOdomTime = 0;
  if (millis() - lastOdomTime >= 50) {
    lastOdomTime = millis();

    motor_prop_fl->update();
    motor_prop_fr->update();
    motor_prop_rl->update();
    motor_prop_rr->update();

    odometry->update();

    currentPose.x = odometry->pos_x();
    currentPose.y = odometry->pos_y();
  }

  // Navigation control
  if (millis() - lastUpdate >= 15) {
    lastUpdate = millis();

    if (running) {
      // Check if we've completed all waypoints
      if (targetIndex >= totalWaypoints) {
        running = false;
        my_robot.stopSmorphi_single();
        my_robot.sm_reset_M1();
        Serial.println("\n Navigation Complete!");
        Serial.printf("Visited %d waypoints\n", totalWaypoints);
        return;
      }

      targetPose = waypoints[targetIndex];

      double errorTheta = targetPose.theta - currentPose.theta;
      double errorX = targetPose.x - currentPose.x;
      double errorY = targetPose.y - currentPose.y;

      // Normalize angle error terlebih dahulu
      while (errorTheta > 180) errorTheta -= 360;
      while (errorTheta < -180) errorTheta += 360;

      double distanceToTarget = sqrt(errorX * errorX + errorY * errorY);

      Serial.printf("Target[%d]: %.2f, %.2f, %.2f | ", targetIndex,
                    targetPose.x, targetPose.y, targetPose.theta);
      Serial.printf("Current: %.2f, %.2f, %.2f | ", 
                    currentPose.x, currentPose.y, currentPose.theta);
      Serial.printf("Error: %.2f, %.2f, %.2f\n", errorX, errorY, errorTheta);
      Serial.printf("Error Distance: d=%.3f", distanceToTarget);

      // Transform error to local frame
      double errorLocalX = errorX * cos(currentPose.theta * M_PI / 180.0) + 
                          errorY * sin(currentPose.theta * M_PI / 180);
      double errorLocalY = -errorX * sin(currentPose.theta * M_PI / 180.0) + 
                           errorY * cos(currentPose.theta * M_PI / 180);

      double errorLinear = fabs(errorLocalX) > fabs(errorLocalY) ? errorLocalX : errorLocalY;

      // Normalize angle error
      // while (errorTheta > 180) errorTheta -= 360;
      // while (errorTheta < -180) errorTheta += 360;

      // Control logic: First rotate, then move
      if (fabs(errorTheta) > ANGULAR_TOLERANCE) {
        // Angular control
        integral += errorTheta * dt;
        integral = constrain(integral, -50, 50);
        double derivative = (errorTheta - lastError) / dt;
        lastError = errorTheta;

        double control = (kp_rot * errorTheta) + (ki_rot * integral) + (kd_rot * derivative);
        double speed = (control * M_PI / 180.0);
        speed = constrain(control, -255, 255); // -180, 180
        
        Serial.printf("  → Rotating: %.2f deg/s\n", speed);
        
        if (speed > 0)
          my_robot.CenterPivotLeft(abs(speed));
        else
          my_robot.CenterPivotRight(abs(speed));
          
      } else if (fabs(errorLinear) > LINEAR_TOLERANCE) {
        // Linear control
        integral_lin += errorLinear * dt;
        integral_lin = constrain(integral_lin, -50, 50);
        double derivative = (errorLinear - lastError_lin) / dt;
        lastError_lin = errorLinear;

        double speedLinear = (kp_lin * errorLinear) + (ki_lin * integral_lin) + (kd_lin * derivative);
        speedLinear = constrain(speedLinear, -50, 50);
        
        Serial.printf("  → Moving: %.2f\n", speedLinear);

        if (fabs(errorLocalX) > fabs(errorLocalY) && errorLocalX > 0) {
          my_robot.MoveForward(fabs(speedLinear));
        } else if (fabs(errorLocalX) > fabs(errorLocalY) && errorLocalX < 0) {
          my_robot.MoveBackward(fabs(speedLinear));
        } else if (fabs(errorLocalX) < fabs(errorLocalY) && errorLocalY > 0) {
          my_robot.MoveRight(fabs(speedLinear));
        } else if (fabs(errorLocalX) < fabs(errorLocalY) && errorLocalY < 0) {
          my_robot.MoveLeft(fabs(speedLinear));
        }
        
      } else {
        // Waypoint reached!
        Serial.printf(" Waypoint %d reached!\n", targetIndex);
        
        my_robot.stopSmorphi_single();
        my_robot.sm_reset_M1();
        
        // Reset PID
        integral = 0;
        lastError = 0;
        lastError_lin = 0;
        integral_lin = 0;

        // Move to next waypoint
        targetIndex++;
        
        if (targetIndex < totalWaypoints) {
          Serial.printf("→ Moving to waypoint %d\n\n", targetIndex);
        }
        
        // Reset position tracking
        currentPose.x = 0;
        currentPose.y = 0;
        odometry->setResetPose();
        
        delay(500);
      }
    }
  }
}
