//5Button, LCD display,Chemistry Robot
//Jonathan Vazquez
//Last Edit: Decemeber 18, 2024

// Include the required libraries
#include <xArmServoController.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#define RXD2 16
#define TXD2 17
#define Blue_button 19
#define Yellow_button 23
#define Black_button 25
#define Red_button 21
#define White_button 18
#define Blue_led 12
#define Red_led 2
#define Yellow_led 13
#define Black_led 4
#define White_led 14
#include <LiquidCrystal.h>

// define servo controller
xArmServoController xarm = xArmServoController(xArm, Serial2);
// Initialize the library by connecting the pins
LiquidCrystal lcd(22, 15, 32, 33, 26, 27); // RS, E, D4, D5, D6, D7
//function to move angles depending on array 


void processJointAngles(float joint_angles_degrees[][6], int numRows,int motor_delay, int time_delay) {
    bool error = false;  // Initialize error flag
  
    for (int i = 0; i < numRows; i++) {
        error = false;  // Reset error flag at the beginning of each iteration
        
        // Read and check each angle in the row
        float link_gripper = joint_angles_degrees[i][0];
        float link_hand = joint_angles_degrees[i][1];
        float link_3 = joint_angles_degrees[i][2];
        float link_2 = joint_angles_degrees[i][3];
        float link_1 = joint_angles_degrees[i][4];
        float link_Base = joint_angles_degrees[i][5];

        // Check for error conditions for each joint
        if (link_gripper > 900 || link_gripper < 100 ||
            link_hand > 900 || link_hand < 100 ||
            link_3 > 900 || link_3 < 100 ||
            link_2 > 900 || link_2 < 100 ||
            link_1 > 900 || link_1 < 100 ||
            link_Base > 900 || link_Base < 100) {
            error = true;
        }

        if (!error) {
            xArmServo movement[] = {{1, link_gripper},
                                    {2, link_hand},
                                    {3, link_3},
                                    {4, link_2},
                                    {5, link_1},
                                    {6, link_Base}};
            xarm.setPosition(movement, 6, motor_delay, true);
        }
        else {
          break;
        }

        delay(time_delay);
    }
    if(error == true){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Oh no broo ....");
      lcd.setCursor(0, 1);
      lcd.print("Invalid point");
      delay(2000);
    }
    
}
//create safe trajectory
void create_Trajectory(float new_theta1, float new_theta2, float new_theta3,int motor_delay, int numPoints){
  
  float MyTrajectory[numPoints][6];  
  float current_theta1 = xarm.getPosition(5);
  float current_theta2 = xarm.getPosition(4);
  float current_theta3 = xarm.getPosition(3);

  float step_1 = (new_theta1 - current_theta1)/numPoints;
  float step_2 = (new_theta2 - current_theta2)/numPoints;
  float step_3 = (new_theta3 - current_theta3)/numPoints;

  for (int i = 0; i < numPoints; i++){
    MyTrajectory[i][0] = 500.0;
    MyTrajectory[i][1] = 500.0;
    MyTrajectory[i][2] = current_theta3 + (step_3*i);
    MyTrajectory[i][3] = current_theta2 + (step_2*i);
    MyTrajectory[i][4] = current_theta1 + (step_1*i);
    MyTrajectory[i][5] = 500.0;
  }  

  int numRows = sizeof(MyTrajectory) / sizeof(MyTrajectory[0]);
  int time_delay = 50;
  processJointAngles(MyTrajectory, numRows,motor_delay, time_delay);
}
//inverse kinematics, does not include
void calculate_joint_angles(float xx, float yy, float theta_r, float& theta1_points, float& theta2_points, float& theta3_points) {
    float l1 = 1.00;  // size of link 1
    float l2 = 0.96;  // size of link 2
    float l3 = 1.60;  // size of link 3

    float x1 = xx - l3 * cos(radians(theta_r));
    float y1 = yy - l3 * sin(radians(theta_r));

    float R3 = sqrt(x1 * x1 + y1 * y1);
    float thetaR3 = degrees(atan2(y1, x1));

    float s = (l1 + l2 + R3) / 2;
    float SS = sqrt(s * (s - l1) * (s - l2) * (s - R3));

    float A = degrees(asin((2 * SS) / (l1 * R3)));
    if (l2 * l2 > l1 * l1 + R3 * R3) {
        A = A + 2 * (90 - A);
    }

    float B = degrees(asin((2 * SS) / (l1 * l2)));
    if (R3 * R3 > l1 * l1 + l2 * l2) {
        B = B + 2 * (90 - B);
    }

    float C = degrees(asin((2 * SS) / (R3 * l2)));
    if (l1 * l1 > l2 * l2 + R3 * R3) {
        B = B + 2 * (90 - B);
    }

    float theta1 = A + thetaR3;
    float theta2 = B - 180;
    float theta3 = theta_r - theta2 - theta1;

    theta1 = 90 - theta1;
    theta3 = -1 * theta3;

    theta1_points = 500 + (4 * theta1);
    theta2_points = 500 + (4 * theta2);
    theta3_points = 500 + (4 * theta3);
}

void clearTopRow() {
  lcd.setCursor(0, 0); // Set cursor at the beginning of the first row
  for(int i = 0; i < 16; i++) {
    lcd.print(" "); // Print a space to clear each position in the row
  }
}
float X_value = 0;
float Y_value = 0;
float Z_value = 0;

void setXYZ(){
    int motor_delay = 1000;
    delay(100);
    digitalWrite(Black_led, HIGH);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Setting X-Value: ");
    lcd.setCursor(0,1);
    lcd.print("Wht-Blu+Blk=Conf");
    delay(2000);
    clearTopRow();
    lcd.setCursor(0,0);
    lcd.print("in mm: ");
    lcd.print(X_value);
    while(digitalRead(Black_button) == HIGH){
      if(digitalRead(Blue_button) == LOW){
        X_value = X_value + 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(X_value);
        delay(500);
      }
      else if(digitalRead(White_button) == LOW){
        X_value = X_value - 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(X_value);
        delay(500);
      }
      else if(digitalRead(Yellow_button) == LOW){
        X_value = X_value + 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(X_value);
        delay(500);
      }
      else if(digitalRead(Red_button) == LOW){
        X_value = X_value - 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(X_value);
        delay(500);
      }
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("X-Value set:");
    lcd.print(X_value);
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Setting Y-Value: ");
    lcd.setCursor(0,1);
    lcd.print("Wht-Blu+Blk=Conf");
    delay(2000);
    clearTopRow();
    lcd.setCursor(0,0);
    lcd.print("in mm: ");
    lcd.print(Y_value);
    while(digitalRead(Black_button) == HIGH){
      if(digitalRead(Blue_button) == LOW){
        Y_value = Y_value + 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(Y_value);
        delay(500);
      }
      else if(digitalRead(White_button) == LOW){
        Y_value = Y_value - 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(Y_value);
        delay(500);
      }
      else if(digitalRead(Yellow_button) == LOW){
        Y_value = Y_value + 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(Y_value);
        delay(500);
      }
      else if(digitalRead(Red_button) == LOW){
        Y_value = Y_value - 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in mm: ");
        lcd.print(Y_value);
        delay(500);
      }
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Y-Value set:");
    lcd.print(Y_value);
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Setting Z-Rot: ");
    lcd.setCursor(0,1);
    lcd.print("Wht-Blu+Blk=Conf");
    delay(2000);
    clearTopRow();
    lcd.setCursor(0,0);
    lcd.print("in Degrees: ");
    lcd.print(Z_value);
    while(digitalRead(Black_button) == HIGH){
      if(digitalRead(Blue_button) == LOW){
        Z_value = Z_value + 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in Degrees: ");
        lcd.print(Z_value);
        delay(500);
      }
      else if(digitalRead(White_button) == LOW){
        Z_value = Z_value - 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in Degrees: ");
        lcd.print(Z_value);
        delay(500);
      }
      else if(digitalRead(Yellow_button) == LOW){
        Z_value = Z_value + 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in Degrees: ");
        lcd.print(Z_value);
        delay(500);
      }
      else if(digitalRead(Red_button) == LOW){
        Z_value = Z_value - 10;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("in Degrees: ");
        lcd.print(Z_value);
        delay(500);
      }
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Inter-Points ");
    lcd.setCursor(0,1);
    lcd.print("Wht-Blu+Blk=Conf");
    delay(2000);
    clearTopRow();
    lcd.setCursor(0,0);
    lcd.print("NumPts: ");
    int numPoints = 1;
    lcd.print(numPoints);
    while(digitalRead(Black_button) == HIGH){
      if(digitalRead(Blue_button) == LOW){
        numPoints = numPoints + 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("NumPts: ");
        lcd.print(numPoints);
        delay(500);
      }
      else if(digitalRead(White_button) == LOW){
        numPoints = numPoints - 1;
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("NumPts: ");
        lcd.print(numPoints);
        delay(500);
      }
    }
    float theta1, theta2, theta3;
    //(xx,yy,ThetaR)
    calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
    create_Trajectory(theta1,theta2,theta3,motor_delay,numPoints);

    digitalWrite(Black_led, LOW);
}
float Home[][6] = {
    {500.0, 500.0, 684.0, 220.0, 384.0, 500.0},  // Home position closed
};

//function for freemovement via constant updates
void freeRange(){
  int motor_delay = 1000;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Engange FreeMove");
  lcd.setCursor(0,1);
  lcd.print("x-y-x +,-");
  //start values
  X_value = 180; 
  Y_value = 175;
  Z_value = 500.0;
  float theta1, theta2, theta3;
  calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
  int numRows = sizeof(Home) / sizeof(Home[0]);
  int time_delay = 0;
  float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
  processJointAngles(Movement,numRows,motor_delay,time_delay);
  while(true){
    motor_delay = 100;
    if (digitalRead(Blue_button)==LOW){
      if(digitalRead(Black_button)==LOW){
        X_value = X_value + 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("X-Val: ");
        lcd.print(X_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
      if(digitalRead(Red_button)==LOW){
        Y_value = Y_value + 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("Y-Val: ");
        lcd.print(Y_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
      if(digitalRead(Yellow_button)==LOW){
        Z_value = Z_value + 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("Z-Val: ");
        lcd.print(Z_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
    }
    else if (digitalRead(White_button)==LOW){
      if(digitalRead(Black_button)==LOW){
        X_value = X_value - 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("X-Val: ");
        lcd.print(X_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
      if(digitalRead(Red_button)==LOW){
        Y_value = Y_value - 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("Y-Val: ");
        lcd.print(Y_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
      if(digitalRead(Yellow_button)==LOW){
        Z_value = Z_value - 1;
        calculate_joint_angles((X_value/100),(Y_value/100),0.0,theta1,theta2,theta3);
        numRows = 1;
        time_delay = 0;
        float Movement[][6] = {500.0, 500.0, theta3, theta2, theta1, Z_value};
        clearTopRow();
        lcd.setCursor(0,0);
        lcd.print("Z-Val: ");
        lcd.print(Z_value);
        processJointAngles(Movement,numRows,motor_delay,time_delay);
      }
    }
    //buttons need to be pressed to exit mode
    else if (digitalRead(Black_button)==LOW){
      if(digitalRead(Red_button)==LOW){
        if(digitalRead(Yellow_button)==LOW){
          break;
        }
      }
    }
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Exiting Mode");
  lcd.setCursor(0,1);
  lcd.print("Smell ye lateeer");
  delay(1500);
}

void grab(){
  int angle1 = xarm.getPosition(5);
  int angle2 = xarm.getPosition(4);
  int angle3 = xarm.getPosition(3);
  int Grip = xarm.getPosition(1);
  int Base = xarm.getPosition(6);

  while (digitalRead(Blue_button) == LOW){
    float Movement[][6] = {Grip + 1, 500.0,angle3,angle2,angle1,Base};
    int numRows = 1;
    int time_delay = 0;
    int motor_delay = 100;
    processJointAngles(Movement,numRows,motor_delay,time_delay);
    delay(100);
  }
}
void release(){
  int angle1 = xarm.getPosition(5);
  int angle2 = xarm.getPosition(4);
  int angle3 = xarm.getPosition(3);
  int Grip = xarm.getPosition(1);
  int Base = xarm.getPosition(6);

  while (digitalRead(White_button) == LOW){
    float Movement[][6] = {Grip - 1, 500.0,angle3,angle2,angle1,Base};
    int numRows = 1;
    int time_delay = 0;
    int motor_delay = 100;
    processJointAngles(Movement,numRows,motor_delay,time_delay);
    delay(100);
  }
}


//Tube1 
// link_1 = 90- angle
//link_3 = -(angle)

void setup() {
pinMode(Black_button, INPUT_PULLUP);
pinMode(Yellow_button, INPUT_PULLUP);
pinMode(Red_button, INPUT_PULLUP);
pinMode(Blue_button, INPUT_PULLUP);
pinMode(White_button, INPUT_PULLUP);
pinMode(Blue_led, OUTPUT);
pinMode(White_led, OUTPUT);
pinMode(Red_led, OUTPUT);
pinMode(Black_led, OUTPUT);
pinMode(Yellow_led, OUTPUT);
lcd.begin(16, 2);  // Assuming a 16x2 LCD


Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
Serial.begin(115200);
WiFi.mode(WIFI_STA);

//Init ESP-NOW
if (esp_now_init() != ESP_OK) {
Serial.println("Error initializing ESP-NOW");
return;
}

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set-Fre-Grb-Rel");
    lcd.setCursor(0,1);
    lcd.print("Select One");
    //put into home position
    processJointAngles(Home,1,1000,100);
}

void loop() {
  //to set one x,y,z point then move
  if (digitalRead(Black_button) == LOW) {
        setXYZ();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Set-Fre-Grb-Rel");
        lcd.setCursor(0,1);
        lcd.print("Select One");
    }
    //Actions for Free Movement
  else if(digitalRead(Red_button) == LOW){
    digitalWrite(Red_led, HIGH);
    freeRange();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set-Fre-Grb-Rel");
    lcd.setCursor(0,1);
    lcd.print("Select One");
    digitalWrite(Red_led, LOW);
  }
  else if(digitalRead(Blue_button) == LOW){
    lcd.clear();
    digitalWrite(Blue_led, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Closing Grip");
    grab();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set-Fre-Grb-Rel");
    lcd.setCursor(0,1);
    lcd.print("Select One");
    digitalWrite(Blue_led, LOW);
  }
  else if(digitalRead(White_button) == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Releasing Grip");
    release();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set-Fre-Grb-Rel");
    lcd.setCursor(0,1);
    lcd.print("Select One");
    digitalWrite(White_led, LOW);
  }

}