

#define motorp1 9
#define motorp2 10
#define encoder 2
float pulso = 0;
float t = millis();
float error = 0;
float p = 0;
float velo;
float ca = 0;
float setpoint = 250; //300 era float
float aux = 0 ; //era float
float ki = 0.06;
float kp = 0.56; // 
float kd = 0;
float integral = 0; // era float
float derivative = 0;
float previousError = 0;


void setup() {
  Serial.begin(9600);
  pinMode(encoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), funcao, RISING);
  digitalWrite(8, HIGH);
  analogWrite(9, 255);
}
void loop() {
  if (millis() - t >= 1000) {
    velo = p*10;
    Serial.print("velo:");
    Serial.println(velo);
    t = millis();
    p = 0;
    error = setpoint - velo;
    integral += error;
    derivative = (error - previousError)*kd;
    previousError = error;
    
    float out = kp * error  + ki * integral + kd * derivative;
    //float out = kp * error;
   

    out = constrain(out, 0, 255);
    analogWrite(9, out);

    if (Serial.available() > 0) {
      aux = Serial.parseInt();

      if (aux > 0 && aux < 600) {
        setpoint= aux;
        Serial.println(setpoint);
        integral=0;
        //Serial.println(kp);
      }
    }
  } 
  
}

void funcao() {
  p++;
  //Serial.println(p);
}
