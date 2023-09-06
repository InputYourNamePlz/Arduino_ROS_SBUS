#include "sbus.h"
#include <Servo.h>

#define SBUS_PIN 19
#define THRUSTER_PIN 4
#define THRUSTER_DIR_PIN 5
#define RUDDER_PIN 6


bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusData sbus_data;

Servo Thruster;
Servo Rudder;

String input_data;
char char_array[30];
int serial_thruster_pwm = 1500;
int serial_thruster_dir = 0;
int serial_rudder_pwm = 1500;
int sbus_thruster_pwm = 1500;
int sbus_thruster_dir = 0;
int sbus_rudder_pwm = 1500;
int mode = 2;

int thruster_pwm = 1500;
int thruster_dir = 0;
int rudder_pwm = 1500;



void setup() {
  Serial.begin(115200);
  sbus_rx.Begin();
  Thruster.attach(THRUSTER_PIN);
  pinMode(THRUSTER_DIR_PIN, OUTPUT);
  Rudder.attach(RUDDER_PIN);
}

void loop() {


  if (Serial.available()) {
    input_data = Serial.readStringUntil('\n');
    if (input_data.length() == 11) {
      input_data.toCharArray(char_array, input_data.length() + 1);
      sscanf(char_array, "%d,%d,%d", &serial_thruster_pwm, &serial_thruster_dir, &serial_rudder_pwm);
    }
  }


  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();  //thruster [1] servo [3] switch[]
  }

  switch (sbus_data.ch[6]) {
    case 1809:
      mode = 1;
      break;
    case 992:
      mode = 2;
      break;
    case 172:
      mode = 2;
      break;
  }


  switch (mode) {
    case 1:
      sbus_thruster_pwm = (sbus_data.ch[2]+508-1500)/2+1500;
      sbus_rudder_pwm = (sbus_data.ch[0]+508-1500)/2+1500;

      if (sbus_thruster_pwm >= 1500) {
        thruster_pwm = sbus_thruster_pwm;
        thruster_dir = 0;
      }
      else if (sbus_thruster_pwm < 1500) {
        thruster_pwm = 3000 - sbus_thruster_pwm;
        thruster_dir = 1;
      }
      rudder_pwm = sbus_rudder_pwm;
      break;


    case 2:
      thruster_pwm = serial_thruster_pwm;
      thruster_dir = serial_thruster_dir;
      rudder_pwm = serial_rudder_pwm;
      break;
  }


  //Serial.println("mode:%d, thruster:%d, dir:%d, rudder:%d",mode, thruster_pwm, thruster_dir, rudder_pwm);
  Serial.print("mode:");
  Serial.print(mode);
  Serial.print("  thruster:");
  Serial.print(thruster_pwm);
  Serial.print("  dir:");
  Serial.print(thruster_dir);
  Serial.print("  rudder:");
  Serial.println(rudder_pwm);
  //Serial.println("haha");
  motor_control();




  delay(100);

}


void motor_control() {
  //Thruster.writeMicroseconds(thruster_pwm);
  analogWrite(THRUSTER_PIN, (thruster_pwm-1500)/2);
  digitalWrite(THRUSTER_DIR_PIN, thruster_dir); //output 0,1
  Rudder.writeMicroseconds(3000-rudder_pwm);
}
