#include <Servo.h>
#include <MatrixMath.h>
#define N  (4)
#include "snakeposition.h"
#include "grad.h"

Servo head;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo neck;
Servo body;
Servo tail;


int pos = 0;    // variable to store the servo position


int curve_up(int pos, int shift, int max){
  int pos_neck = pos + shift;
  if(pos_neck > max){
    pos_neck = max - (pos_neck - max);
  }
  return pos_neck;
}

int curve_down(int pos, int shift, int min){
  int pos_neck = pos - shift;
  if(pos_neck < min){
    pos_neck = min + (min - pos_neck);
  }
  return pos_neck;
}


void move(int pos, int pos_neck, int pos_body, int pos_tail){
  head.write(pos);              // tell servo to go to position in variable 'pos'
  neck.write(pos_neck);              // tell servo to go to position in variable 'pos'
  body.write(pos_body);
  tail.write(pos_tail);
  delay(15);
}

void curver(){
  for (pos = 40; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
    int pos_neck = curve_up(pos, 20, 140);
    int pos_body = curve_up(pos, 40, 140);

    // in steps of 1 degree
    move(pos, pos_neck, pos_body, 90);
  }
  for (pos = 140; pos >= 40; pos -= 1) { // goes from 180 degrees to 0 degrees
    int pos_neck = curve_down(pos, 20, 40);
    int pos_body = curve_down(pos, 40, 40);
    
    move(pos, pos_neck, pos_body, 90);                   // waits 15ms for the servo to reach the position
  }
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
  head.attach(2);  // attaches the servo on pin 9 to the servo object
  neck.attach(3);
  body.attach(4);
  tail.attach(5);
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

  Serial.println("a");
  Serial.println(a);

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

  Serial.println("serv ");
  Serial.println(taserv);
  Serial.println(tbserv);
  Serial.println(tcserv);
  Serial.println(tfserv);
  // have we gone past it?
  
  move(tfserv, tcserv, tbserv, taserv);  
}


void loop() {

  // testKine();
  // Serial.println("HERE");
  // Serial.println(angleToServo(-0.59));

  moveToPose(0, 240, 0);
  delay(1000);
  moveToPose(0, -240, 0);
  delay(1000);
  moveToPose(100, 0, 100);
  delay(1000);
  moveToPose(100, 0, -100);
  delay(1000);


}
