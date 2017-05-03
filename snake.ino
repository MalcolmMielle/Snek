#include <Servo.h>
#include <MatrixMath.h>
#define N  (4)
#include "snakeposition.h"
#include "grad.h"

#include <NewPing.h>

#define TRIGGER_PIN  26  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 40 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


Servo vservo1;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo hservo2;
Servo vservo3;
Servo hservo4;
Servo vservo5;
Servo hservo6;
Servo vservo7;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


int pos = 0;    // variable to store the servo position

int curve(int pos, int shift, int max_in, int min_in){
  int pos_servo2 = pos + shift;
  while(pos_servo2 > max_in || pos_servo2 < min_in){
    
    if(pos_servo2 < min_in){
      pos_servo2 = min_in + (min_in - pos_servo2);
    }
    if(pos_servo2 > max_in){
      pos_servo2 = max_in - (pos_servo2 - max_in);
    }

  }
}

int curve_up(int pos, int shift, int max_in){
  int pos_servo2 = pos + shift;
  if(pos_servo2 > max_in){
    pos_servo2 = max_in - (pos_servo2 - max_in);
  }
  else{
    Serial.println(" nopup ");
    Serial.println(pos_servo2);
    Serial.println(max_in);
    Serial.println(pos);
    Serial.println(shift);
  }
  return pos_servo2;
}

int curve_down(int pos, int shift, int min){
  int pos_servo2 = pos - shift;
  if(pos_servo2 < min){
    pos_servo2 = min + (min - pos_servo2);
  }
  else{
    Serial.println(" nop ");
    Serial.println(pos_servo2);
    Serial.println(min);
  }
  return pos_servo2;
}


void move(int pos, int pos_servo2, int pos_vservo3, int pos_servo4){
  vservo1.write(pos);              // tell servo to go to position in variable 'pos'
  hservo2.write(pos_servo2);              // tell servo to go to position in variable 'pos'
  vservo3.write(pos_vservo3);
  hservo4.write(pos_servo4);
  // delay(5);
}

void moveFull(int pos, int pos_servo2, int pos_vservo3, int pos_servo4, int pos_vservo5, int pos_hservo6, int pos_vservo7){
  vservo1.write(pos);              // tell servo to go to position in variable 'pos'
  hservo2.write(pos_servo2);              // tell servo to go to position in variable 'pos'
  vservo3.write(pos_vservo3);
  hservo4.write(pos_servo4);
  vservo5.write(pos_vservo5);
  hservo6.write(pos_hservo6);
  vservo7.write(pos_vservo7);
  delay(10);
}

void curver(){
  for (pos = 40; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
    int pos_servo2 = curve_up(pos, 20, 140);
    int pos_vservo3 = curve_up(pos, 40, 140);

    // in steps of 1 degree
    move(0, pos_servo2, 90, pos_vservo3);
  }
  for (pos = 140; pos >= 40; pos -= 1) { // goes from 180 degrees to 0 degrees
    int pos_servo2 = curve_down(pos, 20, 40);
    int pos_vservo3 = curve_down(pos, 40, 40);
    
    move(0, pos_servo2, 90, pos_vservo3);
  }
}

void curverFull(int shift, int max_in, int min_in){
  int pos = 0;
  int pos_vservo1 = -1;
  do{
    pos_vservo1 = pos;
    pos = pos + 1;
    int pos_vservo1 = curve(pos,   shift, max_in, min_in);
    int pos_vservo3 = curve(pos, 2*shift, max_in, min_in);
    int pos_vservo5 = curve(pos, 3*shift, max_in, min_in);
    int pos_vservo7 = curve(pos, 4*shift, max_in, min_in);

    // Serial.println("Pisition");
    // Serial.println(pos);
    // Serial.println(pos_vservo1);
    
    moveFull(pos_vservo1, 90, pos_vservo3, 90, pos_vservo5, 90, pos_vservo7);
    // Serial.println(pos_vservo1);
  } while (pos < (max_in - min_in) * 2);
    Serial.println("out");
  // for (pos = min_in; pos <= max_in; pos += 1) { // goes from 0 degrees to 180 degrees
  //   int pos_vservo1 = curve(pos,   shift, max_in, min_in);
  //   int pos_vservo3 = curve(pos, 2*shift, max_in, min_in);
  //   int pos_vservo5 = curve(pos, 3*shift, max_in, min_in);
  //   int pos_vservo7 = curve(pos, 4*shift, max_in, min_in);

  //   Serial.println("Pisition");
  //   // Serial.println(pos_vservo1);
  //   // Serial.println(pos_vservo3);
  //   // Serial.println(pos_vservo5);
  //   Serial.println(pos_vservo7);

  //   // in steps of 1 degree
  //   moveFull(pos_vservo1, 90, pos_vservo3, 90, pos_vservo5, 90, pos_vservo7);
  // }
  // for (pos = max_in; pos >= min_in; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   int pos_vservo1 = curve(pos,   shift, max_in, min_in);
  //   int pos_vservo3 = curve(pos, 2*shift, max_in, min_in);
  //   int pos_vservo5 = curve(pos, 3*shift, max_in, min_in);
  //   int pos_vservo7 = curve(pos, 4*shift, max_in, min_in);
  //   Serial.println(pos_vservo7);
  
  //   moveFull(pos_vservo1, 90, pos_vservo3, 90, pos_vservo5, 90, pos_vservo7);
  // }
}


void testKine(){
  float thetA_first = - pi/2;
  float thetA_second = 0;
  float thetA_third = 0;
  float thetA_final = 0;

  float x, y, z;
  getPose(thetA_first, thetA_second, thetA_third, thetA_final, &x, &y, &z);

  Serial.println("Now");
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);

  float ta, tb, tc, tf;
  grad(x, y, z, &ta, &tb, &tc, &tf);
  Serial.println("angles");
  Serial.println(ta);
  Serial.println(tb);
  Serial.println(tc);
  Serial.println(tf);

  float x2, y2, z2;
  getPose(ta, tb, tc, tf, &x2, &y2, &z2);

  Serial.println("Now");
  Serial.println(x2);
  Serial.println(y2);
  Serial.println(z2);
  Serial.println();
  Serial.println();
}


void setup() {
  vservo1.attach(2);  // attaches the servo on pin 9 to the servo object
  hservo2.attach(3);
  vservo3.attach(4);
  hservo4.attach(5);
  vservo5.attach(6);
  hservo6.attach(7);
  vservo7.attach(8);
  Serial.begin(9600);
}

float angleToServo(float a){
  a = a + (pi / 2);
  a = boundAngle(a);

  if (a > pi){
    if(a < 3 * pi / 2){
      a = pi;
    }
    else{
      a = 0;
    }
  }

  // Serial.println("a");
  // Serial.println(a);

  double serv = a * 180 / pi;
  return serv;
}

void moveToPose(float x_in, float y_in, float z_in){
  float ta, tb, tc, tf;
  // HACK
  grad(x_in, y_in, -z_in, &ta, &tb, &tc, &tf);

  float taserv = angleToServo(ta);
  float tbserv = angleToServo(tb); 
  float tcserv = angleToServo(tc); 
  float tfserv = angleToServo(tf);

  // Serial.println("serv ");
  // Serial.println(taserv);
  // Serial.println(tbserv);
  // Serial.println(tcserv);
  // Serial.println(tfserv);
  // have we gone past it?
  
  move(tfserv, tcserv, tbserv, taserv);  
}


void loop() {

  // testKine();
  // Serial.println("HERE");
  // Serial.println(angleToServo(-0.59));

// BUG
  // moveToPose(150, 100, 0);
  // delay(1000);
  // moveToPose(150, -100, 0);
  // delay(1000);
// curver();
int dist = sonar.ping_cm();
Serial.print("dist ");
Serial.println(dist);
if(dist < 20 && dist > 10){
  curverFull(40, 120, 20);
}
// moveFull(90, 90, 90, 90, 90, 90, 90);


  // delay(1000);
  // moveToPose(0, -240, 0);
  // delay(1000);
  // moveToPose(100, 0, 100);
  // delay(1000);
  // moveToPose(100, 0, -100);
  // delay(1000);

  // move(0, 90, 90, 90);


}
