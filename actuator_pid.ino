int dir_pin  = 2;
int pwm_pin = 3;
int potentiometer = A0;
int speed = 255;

float Kp = 4;
float Kd = 1;
float Ki = 0;

int current_position = 0;
int target_position = 0;
float command;

int target_threshold = 10;
int pwm_offset = 7;

//Angular PID parameters
float pwm = 0;
unsigned long last_time = 0;
float delta_error = 0;
float total_error = 0, last_error = 0;
int delta_time = 0;

void pid(){

    unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    delta_time = current_time - last_time; //delta time interval 
    //P
    double error = target_position - current_position;

    //I
//    total_error += error; 
//    if(abs(total_error)>100)
//      total_error = 100;

    //D
    if(delta_time > 50){
      delta_error = (error - last_error); //difference of error for derivative term
      last_error = error;
      last_time = current_time;
    }

    //Output
    pwm = Kp*error + (Ki)*total_error + (Kd)*delta_error; //PID control compute
    if(abs(pwm)>254.0)
      pwm = sign(pwm)*(255 - pwm_offset);
//    Serial.print("Angular PID\t");
//    Serial.print(pwm);
//    Serial.print("\t");
//    Serial.print(target_position);
//    Serial.print("\t");
//    Serial.println(current_position);
}

int sign(float i){
  if(i>=0)
    return 1;
  else
    return -1;
}

void motor_control(float pwm){
  if(pwm>=0){
    digitalWrite(dir_pin,1);
    analogWrite(pwm_pin,pwm+pwm_offset);
  }
  else{
    pwm = abs(pwm);
    digitalWrite(dir_pin,0);
    analogWrite(pwm_pin,pwm+pwm_offset);
  }
}

void setup() {
   Serial.begin(115200);
   Serial.setTimeout(1);
   pinMode(dir_pin,OUTPUT);
   pinMode(pwm_pin,OUTPUT);
   pinMode(potentiometer,INPUT);
   current_position = analogRead(potentiometer);
   target_position = 500;
}

void loop() {
  //Get User Command //
  if (Serial.available()){
    command = Serial.readString().toFloat();
//    Serial.print(command);
    if(command>=0.0 && command<=20.0)
      {target_position = command*(921.0/20.0);
      Serial.print(target_position);}
    else if(command == 555.0){
      Serial.print(current_position*(20.0/921.0));
    }
  }
  // PID Controller //
  current_position = analogRead(potentiometer);
  pid();
  motor_control(pwm);
}
