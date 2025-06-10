#include <Wire.h>//libraries for tilt sensor and servos
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Servo.h>

Servo servo; //declare objects for servos
Servo tiltServo; 
Adafruit_FXOS8700 tilt;//object for tilt sensor

//output pins for motor bridge
#define E1SPEED 11//speed of motor 1
#define M1DIRECT 13//direction of motor 1
#define E2SPEED 6//speed of motor 2
#define M2DIRECT 5//direction of motor 2

//variables for functions

float accelx,accely,accelz;//acceleration reading from tilt sensor

long startTime=0,retractTime=0;//timestamps

bool shot =false; //used to check if dart is currently being fired

int Smoothaccelx,Smoothaccely,tiltAngle=145,fireButton=3;//smoothed accel and default centred tilt angle @ 145 deg
                                                        //input pin for firing button

int indexx = 0;//variables for smoothing algorithm
int indexy = 0;
int indexz = 0;
float avgx=0;
float avgy=0;
float avgz=0;
float sumx= 0;
float sumy= 0;
float sumz= 0;
float readingsx[5];
float readingsy[5];
float readingsz[5];

float mappedx,mappedy;//for mapping function

float smoothed(float val,float reading[],float *sum,float &avg,int &index){ //smoothing function
 
  *sum = *sum - reading[index];//remove oldest value
  reading[index]=val;//replace removed value
  *sum=*sum+val;//increment sum
  index=(index+1)%5; //5 = window/buffer size
  avg=*sum/5;//calculate average
  return avg;
  delay(10);
}

void mapVoltage(int smoothAccel){ //function to map accleration to speed for motor bridge
  mappedx = map(smoothAccel,0,-9,0,255); //motor bridge accepts speeds from 0-255 (0 = stationary, 255 = max speed) 
  Serial.print(mappedx);
  Serial.print("\n");

}

void motorControl(int mappedx){//function to control driving motors through motor bridge
  if (mappedx >0){//if controller tilted up
    analogWrite(E1SPEED,mappedx);//write speed to bridge
    analogWrite(E2SPEED,mappedx);
    digitalWrite(M1DIRECT,LOW); //move forwards
    analogWrite(M2DIRECT,0); 
  }
  else if (mappedx <0){//if controller tilted down
    analogWrite(E1SPEED,(mappedx*-1));//multiply speed by -1 as you cant write negtaive speed values to motor bridge
    digitalWrite(M1DIRECT,HIGH);
    analogWrite(E2SPEED,(mappedx*-1));
    digitalWrite(M2DIRECT,HIGH); //move backwards
  }
  
  else {//if controller centred and stationary
    analogWrite(E1SPEED,0);//speed = 0
    digitalWrite(M1DIRECT,LOW);
    analogWrite(E2SPEED,0);
    digitalWrite(M2DIRECT,LOW);
    
  }
  delay(30); 
}

void tiltControl(int Smoothaccely){//function to control tilting servo
  if (Smoothaccely < -1 || Smoothaccely > 1){//if controller is tilted up or down
    
    if (Smoothaccely<-1 && tiltAngle<160){ //ensure tilt angle does not surpass 160 deg
      tiltServo.attach(10);//attach servo to pin 10
      tiltAngle+=1;//increment tilt angle
      tiltServo.write(tiltAngle);//write tilt angle to servo
    }
    else if (Smoothaccely>1 && tiltAngle>130){//ensure tilt angle doesnt go below 130 deg
      tiltServo.attach(10);
      tiltAngle-=1;//decrease tilt angle
      tiltServo.write(tiltAngle);

    }
    else if (Smoothaccely == 0 && tiltAngle >= 150){// if controller stationary and tilt angle >= 150 deg
     tiltServo.detach();//detach servo to prevent shaking while stationary
   }

  }
  Serial.print("\ntilt Angle: ");
  Serial.print(tiltAngle);
}

void fireDart(){//function to read fire button input and control micro servo to shoot dart
    

  if (startTime == 0 || (millis()-retractTime)>=500 && shot == false){//use millis instead of delay to allow other parts to still operate
  servo.write(120); //write angle of 120 deg to micro servo
  startTime = millis();//timestamp of when servo begins extending
  Serial.print("\n");
  Serial.print("Dart fired.");
  Serial.print("\n");
  shot = true;// boolean to used to check if dart is being fired
}
  
  if ((millis()-startTime)>=500 && shot == true){//approx 500 ms required to allow servo to extend and prevent jamming
  servo.write(0);  // if its been 500ms since the servo started extending, retract the servo by writing angle of 0 deg
  shot = false;//dart is finshed firing
  retractTime=millis();//timestamp of when servo begins retracting
  
  }

}

void setup() {//this function runs once at the start of the programme
 
  pinMode(fireButton,INPUT);// fire button input pin 3
  pinMode(4,OUTPUT); //pin for powering fire button
  digitalWrite(4,HIGH);//provide power to firing button

  tiltServo.write(tiltAngle);//centre tilting servo when progamme begins

  servo.attach(9);//attach micro servo to pin 9
  servo.write(0);//pull out plunger if pushed in 

  delay(1000);// delay to allow servos to centre
  Serial.begin(115200);//begin serial communication to tilt senser @ 115200 baude
  pinMode(M1DIRECT,OUTPUT);
  while (!Serial)// wait for serial connection
    delay(10);

  // start I2C connection to tilt sensor
  Wire.begin();

  // Initialize the sensor
  if (!tilt.begin(0x1E)) {//0x1E = i2c address of tilit sensor
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("sensor found");
}

void loop() {//this function runs loops infintely 
 
  sensors_event_t accel, mag;
  tilt.getEvent(&accel, &mag);//read acceleration and magnetic information from sensor

  Serial.print("accel X: ");//read acceleration in ms^-2 from the tilt sensor from all 3 axes
  accelx=accel.acceleration.x;
  Serial.print(accelx);
  Serial.print(" m/s^2, Y: ");
  accely=accel.acceleration.y;
  Serial.print(accely);
  Serial.print(" m/s^2, Z: ");
  accelz=accel.acceleration.z;
  Serial.print(accelz);
  Serial.println(" m/s^2");


  Serial.print("Smoothed accel X: ");
  Smoothaccelx=smoothed(accelx,readingsx,&sumx,avgx,indexx);//call smooth function to get smoothed acceleration of x-axis
  Serial.print(Smoothaccelx);
  
  Serial.print(" m/s^2, Smoothed Y: ");
 
  
  Smoothaccely=smoothed(accely,readingsy,&sumy,avgy,indexy);//call smooth function to get smoothed acceleration of y-axis
  Serial.print(Smoothaccely);
  Serial.print(" m/s^2, Smoothed Z: ");

  Serial.print(int(smoothed(accelz,readingsz,&sumz,avgz,indexz)));//call smooth function to get smoothed acceleration of z-axis,
  Serial.println(" m/s^2");              //no need to assign to a variable as it is only used for troubleshooting if needed and not anywhere else
 
  mapVoltage(Smoothaccelx);//map smoothed x axis accerlation between 0-255 for driving motor speed
  motorControl(mappedx);//write mapped speed value to motor bridge

  tiltControl(Smoothaccely);//control tilting servo based on movement in y-axis

  
  int fire = digitalRead(3);//read firing button 
  if ((fire==HIGH)){//if button pressed
    fireDart();//call dart firing function to fire dart
  }

  //use this if statment to automatically retract plunger (micro servo) if firing button is de pressed while plunger is pushed in             
  else if (fire == LOW && (millis()-startTime)>=500 && shot==true){
    servo.write(0);
    shot=false;
    retractTime=millis();
  }

  delay(50);
  Serial.print("\n");
}//end of programme


