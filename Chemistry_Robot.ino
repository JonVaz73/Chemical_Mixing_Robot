//5Button, LCD display,Chemistry Robot
//Jonathan Vazquez
//Last Edit: Decemeber 18, 2024

// Include the required libraries
#include <xArmServoController.h>
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
//defining positions
float Pour_motion[][6] = {
    {1000.0, 500.0, 684.0, 220.0, 384.0, 500.0},  // Home position closed
    {1000.0, 500.0, 480.0, 84.0, 456.0, 384.0},   // Pre-pour position
    {1000.0, 1000.0, 480.0, 84.0, 456.0, 344.0},  // Pour position
    {1000.0, 500.0, 480.0, 84.0, 456.0, 384.0},   // Return to Pre-pour position
    {1000.0, 500.0, 684.0, 220.0, 384.0, 500.0}   // Return to Home position closed
};
float Home[1][6] = {
    {500.0, 500.0, 684.0, 220.0, 384.0, 500.0},  // Home position closed
};
//Tube1 
// link_1 = 90- angle
//link_3 = -(angle)
float Tube1_trajectory[][6] = {
    {500.00, 480.00, 340.00, 0.00, 500.00, 528.00},
    {500.00, 464.00, 377.47, 121.29, 603.82, 528.00},
    {1000.00, 464.00, 377.47, 121.29, 603.82, 528.00},
    {1000.00, 464.00, 390.72, 127.73, 597.02, 528.00},
    {1000.00, 464.00, 404.11, 135.06, 590.95, 528.00},
    {1000.00, 464.00, 417.63, 143.27, 585.64, 528.00},
    {1000.00, 464.00, 431.27, 152.38, 581.10, 528.00},
    {1000.00, 464.00, 445.04, 162.40, 577.36, 528.00},
    {1000.00, 464.00, 458.94, 173.37, 574.43, 528.00},
    {1000.00, 464.00, 473.00, 185.32, 572.32, 528.00},
    {1000.00, 464.00, 487.24, 198.32, 571.08, 528.00},
    {1000.00, 464.00, 501.73, 212.46, 570.73, 528.00}
};
float Tube1_putdown[][6] = {
    {1000.00, 464.00, 501.73, 212.46, 570.73, 528.00},
    {1000.00, 464.00, 487.24, 198.32, 571.08, 528.00},
    {1000.00, 464.00, 473.00, 185.32, 572.32, 528.00},
    {1000.00, 464.00, 458.94, 173.37, 574.43, 528.00},
    {1000.00, 464.00, 445.04, 162.40, 577.36, 528.00},
    {1000.00, 464.00, 431.27, 152.38, 581.10, 528.00},
    {1000.00, 464.00, 417.63, 143.27, 585.64, 528.00},
    {1000.00, 464.00, 404.11, 135.06, 590.95, 528.00},
    {1000.00, 464.00, 390.72, 127.73, 597.02, 528.00},
    {1000.00, 464.00, 377.47, 121.29, 603.82, 528.00},
    {1000.00, 464.00, 377.47, 121.29, 603.82, 528.00},
    {500.00, 480.00, 340.00, 0.00, 500.00, 528.00}
};
//Tube 2
float Tube2_trajectory[][6] = {
    {500.00, 480.00, 340.00, 0.00, 500.00, 488.00},
    {500.00, 464.00, 376.23, 117.69, 601.46, 488.00},
    {1000.00, 464.00, 376.23, 117.69, 601.46, 488.00},
    {1000.00, 464.00, 389.56, 124.14, 594.58, 488.00},
    {1000.00, 464.00, 403.02, 131.47, 588.45, 488.00},
    {1000.00, 464.00, 416.60, 139.68, 583.08, 488.00},
    {1000.00, 464.00, 430.30, 148.78, 578.49, 488.00},
    {1000.00, 464.00, 444.11, 158.80, 574.69, 488.00},
    {1000.00, 464.00, 458.04, 169.74, 571.71, 488.00},
    {1000.00, 464.00, 472.11, 181.67, 569.55, 488.00},
    {1000.00, 464.00, 486.36, 194.62, 568.26, 488.00},
    {1000.00, 464.00, 500.84, 208.69, 567.85, 488.00}
};
float Tube2_putdown[][6] = {
    {1000.00, 464.00, 500.84, 208.69, 567.85, 488.00},
    {1000.00, 464.00, 486.36, 194.62, 568.26, 488.00},
    {1000.00, 464.00, 472.11, 181.67, 569.55, 488.00},
    {1000.00, 464.00, 458.04, 169.74, 571.71, 488.00},
    {1000.00, 464.00, 444.11, 158.80, 574.69, 488.00},
    {1000.00, 464.00, 430.30, 148.78, 578.49, 488.00},
    {1000.00, 464.00, 416.60, 139.68, 583.08, 488.00},
    {1000.00, 464.00, 403.02, 131.47, 588.45, 488.00},
    {1000.00, 464.00, 389.56, 124.14, 594.58, 488.00},
    {1000.00, 464.00, 376.23, 117.69, 601.46, 488.00},
    {1000.00, 464.00, 376.23, 117.69, 601.46, 488.00},
    {500.00, 464.00, 376.23, 117.69, 601.46, 488.00},
    {500.00, 480.00, 340.00, 0.00, 500.00, 488.00}

};
//Tube3
float Tube3_trajectory[][6] = {
    {500.00, 480.00, 340.00, 0.00, 500.00, 460.00},
    {500.00, 464.00, 377.47, 121.29, 603.82, 460.00},
    {1000.00, 464.00, 377.47, 121.29, 603.82, 460.00},
    {1000.00, 464.00, 390.72, 127.73, 597.02, 460.00},
    {1000.00, 464.00, 404.11, 135.06, 590.95, 460.00},
    {1000.00, 464.00, 417.63, 143.27, 585.64, 460.00},
    {1000.00, 464.00, 431.27, 152.38, 581.10, 460.00},
    {1000.00, 464.00, 445.04, 162.40, 577.36, 460.00},
    {1000.00, 464.00, 458.94, 173.37, 574.43, 460.00},
    {1000.00, 464.00, 473.00, 185.32, 572.32, 460.00},
    {1000.00, 464.00, 487.24, 198.32, 571.08, 460.00},
    {1000.00, 464.00, 501.73, 212.46, 570.73, 460.00}
};
float Tube3_putdown[][6] = {
    {1000.00, 464.00, 501.73, 212.46, 570.73, 460.00},
    {1000.00, 464.00, 487.24, 198.32, 571.08, 460.00},
    {1000.00, 464.00, 473.00, 185.32, 572.32, 460.00},
    {1000.00, 464.00, 458.94, 173.37, 574.43, 460.00},
    {1000.00, 464.00, 445.04, 162.40, 577.36, 460.00},
    {1000.00, 464.00, 431.27, 152.38, 581.10, 460.00},
    {1000.00, 464.00, 417.63, 143.27, 585.64, 460.00},
    {1000.00, 464.00, 404.11, 135.06, 590.95, 460.00},
    {1000.00, 464.00, 390.72, 127.73, 597.02, 460.00},
    {1000.00, 464.00, 377.47, 121.29, 603.82, 460.00},
    {500.00, 464.00, 377.47, 121.29, 603.82, 460.00},
    {500.00, 480.00, 340.00, 0.00, 500.00, 460.00}
};

//function to move angles depending on array 
void processJointAngles(float joint_angles_degrees[][6], int numRows, int time_delay) {
    for (int i = 0; i < numRows; i++) {
        // Setting the joint angles from the array
        float link_Base = joint_angles_degrees[i][5];
        float link_1 =  joint_angles_degrees[i][4];
        float link_2 = joint_angles_degrees[i][3];
        float link_3 = joint_angles_degrees[i][2];
        float link_hand = joint_angles_degrees[i][1];
        float link_gripper = joint_angles_degrees[i][0];

        xArmServo movement[] = {{1, link_gripper},
                            {2, link_hand},
                            {3, link_3},
                            {4, link_2},
                            {5, link_1},
                            {6, link_Base}};
        xarm.setPosition(movement,6,1000, true);
        delay(time_delay);
        }
}

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

}

void loop() {
  
  int Yellowstate = digitalRead(Yellow_button);
  int Redstate = digitalRead(Red_button);
  int Bluestate = digitalRead(Blue_button);
  int Blackstate = digitalRead(Black_button);
  int Whitestate = digitalRead(White_button);
  if (Blackstate == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Black Button");
    lcd.setCursor(0,1);
    lcd.print("El Tubo Uno");
    int numRows = sizeof(Tube1_trajectory) / sizeof(Tube1_trajectory[0]);
    int time_delay = 0;
    digitalWrite(Black_led, HIGH);
    processJointAngles(Tube1_trajectory, numRows, time_delay);
    numRows = sizeof(Pour_motion) / sizeof(Pour_motion[0]);
    time_delay = 1000; //adding extra second delay to allow for full pour
    processJointAngles(Pour_motion, numRows, time_delay);
    numRows = sizeof(Tube1_putdown) / sizeof(Tube1_putdown[0]);
    time_delay = 0;
    processJointAngles(Tube1_putdown, numRows, time_delay);
    digitalWrite(Black_led, LOW);
    }
  else if(Redstate == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Rowho Button");
    lcd.setCursor(0,1);
    lcd.print("Tubey Doob Dos");
    int numRows = sizeof(Tube2_trajectory) / sizeof(Tube2_trajectory[0]);
    int time_delay = 0;
    digitalWrite(Red_led, HIGH);
    processJointAngles(Tube2_trajectory, numRows, time_delay);
    numRows = sizeof(Pour_motion) / sizeof(Pour_motion[0]);
    time_delay = 1000; //adding extra second delay to allow for full pour
    processJointAngles(Pour_motion, numRows, time_delay);
    numRows = sizeof(Tube2_putdown) / sizeof(Tube2_putdown[0]);
    time_delay = 0;
    processJointAngles(Tube2_putdown, numRows, time_delay);
    digitalWrite(Red_led, LOW);
  }
  else if(Yellowstate == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("YellHo Button");
    lcd.setCursor(0,1);
    lcd.print("eh prolly tres");
    int numRows = sizeof(Tube3_trajectory) / sizeof(Tube3_trajectory[0]);
    int time_delay = 0;
    digitalWrite(Yellow_led, HIGH);
    processJointAngles(Tube3_trajectory, numRows, time_delay);
    numRows = sizeof(Pour_motion) / sizeof(Pour_motion[0]);
    time_delay = 1000; //adding extra second delay to allow for full pour
    processJointAngles(Pour_motion, numRows, time_delay);
    numRows = sizeof(Tube3_putdown) / sizeof(Tube3_putdown[0]);
    time_delay = 0;
    processJointAngles(Tube1_putdown, numRows, time_delay);
    digitalWrite(Yellow_led, LOW);
  }
  else if(Bluestate == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Asuul Button");
    lcd.setCursor(0,1);
    lcd.print("Pour me UPP BABY");
    int numRows = sizeof(Pour_motion) / sizeof(Pour_motion[0]);
    int time_delay = 1000;
    digitalWrite(Blue_led, HIGH);
    processJointAngles(Pour_motion, numRows, time_delay);
    digitalWrite(Blue_led, LOW);
  }
  else if(Whitestate == LOW){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("White Button");
    lcd.setCursor(0,1);
    lcd.print("Live,Laugh,Love");
    int numRows = sizeof(Home) / sizeof(Home[0]);
    int time_delay = 0;
    digitalWrite(White_led, HIGH);
    delay(1000);
    processJointAngles(Home, numRows, time_delay);
    digitalWrite(White_led, LOW);
  }

}
