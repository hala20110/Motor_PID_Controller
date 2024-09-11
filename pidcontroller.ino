// PID Controller Class Definition
class PIDController {
  public:
    float Kp, Ki, Kd;
    float setpoint, input, output;
    float lastInput, integral;
    unsigned long lastTime;

  
    PIDController(float p, float i, float d) {
      Kp = p;
      Ki = i;
      Kd = d;
      lastInput = integral = 0;
      lastTime = millis();
    }

    // Function to compute the PID output
    void compute() {
      unsigned long now = millis();
      float timeChange = (now - lastTime) / 1000.0; 

      float error = setpoint - input;

      
      integral += error * timeChange;

     
      float derivative = (input - lastInput) / timeChange;

     
      output = Kp * error + Ki * integral - Kd * derivative;

      // Update lastInput and lastTime for the next computation
      lastInput = input;
      lastTime = now;
    }

    // Set the current input 
    void setInput(float newInput) {
      input = newInput;
    }

    // Set the desired setpoint 
    void setSetpoint(float newSetpoint) {
      setpoint = newSetpoint;
    }

    // Get the computed output (constrained between 0-255 for PWM)
    float getOutput() {
      return constrain(output, 0, 255);
    }
};


float alpha = 0.2;  // Smoothing factor 
float smoothedOutput = 0;  // Filtered output

// Function to apply the Exponential Smoothing Filter
void applySoftStart(float rawOutput) {
  smoothedOutput = alpha * rawOutput + (1 - alpha) * smoothedOutput;
  analogWrite(motorPin, constrain(smoothedOutput, 0, 255));  // Apply filtered output to the motor
}


int motorPin = 9;  // Motor control PWM pin


PIDController motorPID(2.0, 0.5, 1.0);  

void setup() {
  pinMode(motorPin, OUTPUT); 
  Serial.begin(9600);  
}

void loop() {
 
  motorPID.setInput(analogRead(A0) / 4.0);  


  motorPID.setSetpoint(100);  

  // Compute the PID output
  motorPID.compute();

  // Apply the soft start filter to smooth out the motor speed transitions
  applySoftStart(motorPID.getOutput());

  delay(100);  
}
