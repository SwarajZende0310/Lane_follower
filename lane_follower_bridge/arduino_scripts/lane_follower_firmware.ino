/*
Components used:-
Motor Drive == Cytron SmartDriveDuo-30 MDDS30
Microcontroller == Arduino UNO
*/

/*
Motor Driver Setting:
SW1 = 1  ---=> PWM INPUT MODE
SW2 = 0  ---

SW3 = 1  ---=> INDEPENDENT BOTH
SW4 = 1  ---

SW5 = 0  ---=> EXPONENTIAL MODE OFF

SW6 = 0  ---=> LOCKED ANTIPHASE
*/


#define left_motor  9
#define right_motor 10

#define L 0.3  //Distance between two wheels(in meters)
#define R 0.13 //Wheel radius(in meters)

int convert_vel_to_pwm(float vel){
  /*
  Converts velocity to PWM for Locked Antiphase mode
  FOR LOCKED ANTIPHASE

  INPUT:-
  vel        Velocity calculated from kinematic equation

  OUTPUT:-
  pwm_val    PWM Value to be set to motor
  */

  //Map velocity to range -125 to 125
  int mapped_vel = map(vel,-1,1,125,-125);

  //Output PWM
  int pwm_val = 128 + mapped_vel;

  return pwm_val;
}

void setup() {
  Serial.begin(9600);
  //Set Motor pins to Output
  pinMode(left_motor,OUTPUT);
  pinMode(right_motor,OUTPUT);

  //Set motor to Zero
  analogWrite(left_motor,128);
  analogWrite(right_motor,128);
}

void loop() {
  if(Serial.available()>0){
    // Read the incoming string until a newline character is received
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');

    if (commaIndex == -1 || data.length() < 7)return;
    // Extract the substring before the comma as a linear velocity
    String linearVelocityString = data.substring(0, commaIndex);
    float v = linearVelocityString.toFloat();

    // Extract the substring after the comma as an angular velocity
    String angularVelocityString = data.substring(commaIndex + 1);
    float w = angularVelocityString.toFloat();

    float right_vel = (2*v - w*L)/(2*R);
    float left_vel = (2*v + w*L)/(2*R);

    analogWrite(left_motor,convert_vel_to_pwm(left_vel));
    analogWrite(right_motor,convert_vel_to_pwm(right_vel));
  }

}
