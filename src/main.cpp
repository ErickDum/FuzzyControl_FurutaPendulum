// FUTURA PENDULUM CONTROL
#include <Arduino.h>
#include <MsTimer2.h> 

#define PWM 9
#define IN1 10
#define IN2 11
#define ENCODER_A 2
#define ENCODER_B 4
#define POT A5
#define PWM_lectura 5

#define arm_set_point 10000
#define pendulum_set_point 758

float PWM_value, PWM_pendulum, PWM_arm;
int pendulum_position;
int arm_position = 10000;

int limit_error_pendulum = 25;
int limit_error_arm = 80;

void timerCallback();
void READ_ENCODER_A();
void READ_ENCODER_B();

void setup() {
  Serial.begin(115200);
  // Pins configuration
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(PWM_lectura, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Pin init
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Timer configuration
  delay(200);
  MsTimer2::set(5, timerCallback);
  MsTimer2::start();

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), READ_ENCODER_A, CHANGE);           
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), READ_ENCODER_B, CHANGE);
}

void loop() {
}

float rampa_function(int x, int a) {
  if (x >= a) {
    return 1;
  } else {
    // Interpolar el valor de x entre a y b
    return - (x - a) / a;
  }
}


float* fuzzy_pendulum_membership(int error) {
  float* membership = new float[2]; // Asignación dinámica de memoria
  float rampa = rampa_function(error, limit_error_pendulum);

  membership[0] = 1 - rampa;
  membership[1] = rampa;
  return membership; // Retornar el puntero al array dinámico
}

float* fuzzy_arm_membership(int error) {
  float* membership = new float[2]; // Asignación dinámica de memoria
  float rampa = rampa_function(error, limit_error_arm);

  membership[0] = 1 - rampa;
  membership[1] = rampa;
  return membership; 
}


void control_pendulum(){
  static int last_error, error, rules[4];
  static float weights[3] = {0, 50, 200};

  error = abs(pendulum_position - pendulum_set_point);
  
  // Fuzzfication
  float* membership_error = fuzzy_pendulum_membership(error);
  float* membership_error_d = fuzzy_pendulum_membership(error - last_error);


  // Fuzzy rules
  rules[0] = min(membership_error[0], membership_error_d[0]);
  rules[1] = min(membership_error[0], membership_error_d[1]);
  rules[2] = min(membership_error[1], membership_error_d[0]);
  rules[3] = min(membership_error[1], membership_error_d[1]);

  // Defuzzification
  float ponderado = rules[0]*weights[0] + max(rules[2], rules[3])*weights[2] + rules[1]*weights[1];

  PWM_pendulum = ponderado;
  last_error = error;
}

void control_arm(){
  static int last_error, error, rules[4];
  static float weights[3] = {0, 30, 80};

  error = abs(arm_position - arm_set_point);

  // Fuzzfication
  float* membership_error = fuzzy_arm_membership(error);
  float* membership_error_d = fuzzy_arm_membership(abs(error - last_error));

  // Fuzzy rules
  rules[0] = min(membership_error[0], membership_error_d[0]);
  rules[1] = min(membership_error[0], membership_error_d[1]);
  rules[2] = min(membership_error[1], membership_error_d[0]);
  rules[3] = min(membership_error[1], membership_error_d[1]);

  // Defuzzification
  float ponderado = rules[0]*weights[0] + max(rules[1], rules[2])*weights[1] + rules[3]*weights[2];  

  PWM_arm = ponderado;
  last_error = error;
}

void set_pwm(float PWM_value){
  if (pendulum_set_point - pendulum_position < 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);    
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  int PWM_value_sat = min(abs(PWM_value), 1024);
  analogWrite(PWM, PWM_value_sat);
  analogWrite(PWM_lectura, PWM_value_sat);
}

unsigned char brake(){
  if (abs(pendulum_position - pendulum_set_point) > 200){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    return 1;
  }
  return 0;
}

uint16_t get_adc_value(uint8_t ch, uint8_t times){
  uint16_t temp_val = 0;
  for (unsigned char t = 0; t < times; t++) {
    temp_val += analogRead(ch);
  }
  return temp_val / times;
}


void timerCallback(){
  pendulum_position = get_adc_value(POT, 5);

  control_pendulum();
  control_arm();

  PWM_value = PWM_pendulum - PWM_arm;
  Serial.println(pendulum_position);
  if (brake() == 0) {
    set_pwm(PWM_value);
  }
};


void READ_ENCODER_A() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    arm_position++;
  } else {
    arm_position--;
  }
}

void READ_ENCODER_B() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    arm_position--;
  } else {
    arm_position++;
  }
}