#include <NewPing.h>
#include <Servo.h>

//define pins
#define motor1Control 9    //right wheel
#define motor2Control 7	 	 // left wheel
#define motor1ControlR 8   //right wheel reverse
#define motor2ControlR 6	 // left wheel reverse
#define motor1Speed 10	  	 //right wheel speed
#define motor2Speed 5			 // left wheel speed

#define trigPin  A0
#define echoPin  A1
#define servoPin 11 //pwm

#define ir1Pin	12		//right IR
#define ir2Pin	13		//left IR

#define autocon '0'
#define bluetoothCon '1'
#define lineFollow '2'

NewPing sonar (trigPin, echoPin, 200);

//define global variables
int maxSpeed = 255;
long duration;
int i, line, distance;
int safeDistance = 45;
int angle = 90;
String voice = "";
char state = bluetoothCon;


int ultraSonicDistance() {
  delay(70);
  distance = sonar.ping_cm();
  if (distance == 0) {
    distance = 250;
  }

  //Serial.print("distance:");
  Serial.println(distance);
  return distance;
}


int irScan()
{
  int ir1, ir2;
  ir1 = digitalRead(ir1Pin);
  ir2 = digitalRead(ir2Pin);
  // returns ab: a left motor, b right motor
  return (ir2Pin * 10 + ir1Pin);
}


char blueTooth() {
  char message;
  if (Serial.available() > 0) message = Serial.read();
  return message;
}


//class that controls car movement (left,right,forward,backward,stop)
class CarMove {
  private:
    int motorDir; //1 for forward, 0 for backwards

  public:
    void right() {
      analogWrite(motor1Speed, 0);
      analogWrite(motor2Speed, maxSpeed);
    }

    void left() {
      analogWrite(motor1Speed, maxSpeed);
      analogWrite(motor2Speed, 0);
    }

    void forward() {
      motorDir = 1;
      analogWrite(motor1Speed, maxSpeed);
      analogWrite(motor2Speed, maxSpeed);
      digitalWrite(motor1ControlR, LOW);
      digitalWrite(motor2ControlR, LOW);
      digitalWrite(motor1Control, HIGH);
      digitalWrite(motor2Control, HIGH);
    }

    void backward() {
      motorDir = 0;
      analogWrite(motor1Speed, maxSpeed);
      analogWrite(motor2Speed, maxSpeed);
      digitalWrite(motor1ControlR, HIGH);
      digitalWrite(motor2ControlR, HIGH);
      digitalWrite(motor1Control, LOW);
      digitalWrite(motor2Control, LOW);
      //Serial.println("backward");
    }

    void stop() { 
    //check for previous stage then go opposite direction then stops
      switch(motorDir){
      	case 1:
      		backward();
      		delay(500);
          analogWrite(motor2Speed, 0);
          analogWrite(motor1Speed, 0);
          digitalWrite(motor1ControlR, LOW);
          digitalWrite(motor2ControlR, LOW);
          digitalWrite(motor1Control, LOW);
          digitalWrite(motor2Control, LOW);
          break;
        case 0:
          forward();
          delay(500);
          analogWrite(motor2Speed,0);
          analogWrite(motor1Speed,0);
          digitalWrite(motor1ControlR,LOW);
          digitalWrite(motor2ControlR,LOW);
          digitalWrite(motor1Control,LOW);
          digitalWrite(motor2Control,LOW);
          break;
          }
    }
};

////////////////////////////////////////////////////

//define objects
Servo myservo;
CarMove carMove;



void voicecontrol () {
  //get characters to form string
  while (Serial.available()) {
    delay(10);
    char c = Serial.read ();
    if (c == '#') {
      break;
    }
    voice += c;
  }
  //do commands
  if (voice.length() > 0) {
    Serial.println(voice);
    if (voice == "*forward")
    {
      carMove.forward();
    }
    else if (voice == "*backwards")
    {
      carMove.backward ();
    }
    else if (voice == "*left") {
      carMove.left() ;
    }
    else if (voice == "*right" || voice == "*write"||voice == "*bright")
    {
      carMove.right();
    }
    else if (voice == "*stop")
    {
      carMove.stop();
    }
  }
  voice = "";
}

void scanTwiceNTurn(int degree) {
  int distance1 = 0, distance2 = 0;
  //Serial.println("start scan");
  carMove.stop();

  //scans right side
  myservo.write(degree);
  delay(1000);
  distance1 = ultraSonicDistance();
  //delay(100);
  //scans left side
  myservo.write(180 - degree);
  delay(1000);
  distance2 = ultraSonicDistance();
  //delay(100);

  //resets servo
  myservo.write(75);

  //compares the reads of each side then turns depending on it
  carMove.backward();
  if (distance1 >= distance2) {
    while (distance < safeDistance) {
      carMove.right();
      delay(150);
      distance = ultraSonicDistance();
    }
  }

  else {
    while (distance < safeDistance) {
      carMove.left();
      delay(150);
      distance = ultraSonicDistance();
    }
  }
}


void setup() {
  //init ultra sonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //init motor control pins
  pinMode(motor1Control, OUTPUT);
  pinMode(motor1ControlR, OUTPUT);
  pinMode(motor2Control, OUTPUT);
  pinMode(motor2ControlR, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);

  //init servo
  myservo.attach(servoPin);
  myservo.write(75);


  //init ir sensors
  pinMode(ir1Pin , INPUT);
  pinMode(ir2Pin , INPUT);

  //init //Serial
  Serial.begin(9600);
}

void loop() {

  //auto control
  switch (state) {
    //auto control
    case autocon:
      angle = 75;
      //Serial.println("autoMode");
      myservo.write(75);
      distance = ultraSonicDistance();
      carMove.forward();

      //obstacle avoidance
      if (distance > safeDistance) carMove.forward();
      else scanTwiceNTurn(40);

      //check for bluetooth if avaible
      if ( blueTooth() == bluetoothCon) state = bluetoothCon;
      if ( blueTooth() == lineFollow) state = lineFollow;
      break;

    //bluetooth control
    case bluetoothCon:
      voicecontrol();
      //Serial.println("bluetoothMode");
      switch (blueTooth()) {
        case 'F':
          carMove.forward();
          break;
        case 'S':
          carMove.stop();
          break;
        case 'B':
          carMove.backward();
          break;
        case 'R':
          carMove.right();
          break;
        case 'L':
          carMove.left();
          break;
        case '1': ///////////// make attention here ( O )
          state = autocon;
          break;

        case '2':
          state = lineFollow;
          break;

        //control servo for fun
        case 'X':
          angle += 10;
          myservo.write(angle);
          break;
        case 'Y':
          angle -= 10;
          myservo.write(angle);
          break;
      }
      break;

    //follow line
    case lineFollow:
      carMove.forward();
      line = irScan();
      switch ( line ) {
        case 11 : //line ok
          break;

        case 10 : //line is on left
          while (distance > safeDistance && irScan() == line) {
            carMove.left();
            distance = ultraSonicDistance();
          }
          break;
        case 01 :	//line is on right
          while (distance > safeDistance && irScan() == line) {
            carMove.right();
            distance = ultraSonicDistance();
          }
          break;

        case 00 :	//if car loses line, switch into bluetooth mode
          state = bluetoothCon;
          break;
      }

      //check for bluetooth if avaible
    if ( blueTooth() == bluetoothCon) state = bluetoothCon;
    if ( blueTooth() == lineFollow) state = lineFollow;
    break;
  }
}
