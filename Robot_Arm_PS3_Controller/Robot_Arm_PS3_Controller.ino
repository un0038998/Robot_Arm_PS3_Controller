#include <Ps3Controller.h>
#include <ESP32Servo.h>

#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};
std::vector<ServoPins> servoPins = 
{
  { Servo(), 27 , "Base", 90},
  { Servo(), 26 , "Shoulder", 90},
  { Servo(), 25 , "Elbow", 90},
  { Servo(), 33 , "Gripper", 90},
};

struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};
std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;
bool gripperSwitch = false;

unsigned long previousTimeInMilli = millis();

void writeServoValues(int servoIndex, int servoMoveStepSize, bool servoStepSizeIsActualServoPosition = false)
{
  int servoPosition;
  if (servoStepSizeIsActualServoPosition)
  {
    servoPosition = servoMoveStepSize; 
  }
  else
  {
    servoPosition = servoPins[servoIndex].servo.read();
    servoPosition = servoPosition + servoMoveStepSize;
  }
  if (servoPosition > 180 || servoPosition < 0)
  {
    return;
  }
  
  if (recordSteps)
  {
    recordRobotArmStep(servoIndex, servoPosition);
  }
  
  servoPins[servoIndex].servo.write(servoPosition);   
}

void recordRobotArmStep(int servoIndex, int servoPosition)
{
  RecordedStep recordedStep;       
  if (recordedSteps.size() == 0) // We will first record initial position of all servos. 
  {
    for (int i = 0; i < servoPins.size(); i++)
    {
      recordedStep.servoIndex = i; 
      recordedStep.value = servoPins[i].servo.read(); 
      recordedStep.delayInStep = 0;
      recordedSteps.push_back(recordedStep);         
    }      
  }
  unsigned long currentTime = millis();
  recordedStep.servoIndex = servoIndex; 
  recordedStep.value = servoPosition; 
  recordedStep.delayInStep = currentTime - previousTimeInMilli;
  recordedSteps.push_back(recordedStep);  
  previousTimeInMilli = currentTime;         
}

void playRecordedRobotArmSteps()
{
  if (recordedSteps.size() == 0)
  {
    return;
  }
  //This is to move servo to initial position slowly. First 4 steps are initial position    
  for (int i = 0; i < 4 && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    int currentServoPosition = servoPins[recordedStep.servoIndex].servo.read();
    while (currentServoPosition != recordedStep.value && playRecordedSteps)  
    {
      currentServoPosition = (currentServoPosition > recordedStep.value ? currentServoPosition - 1 : currentServoPosition + 1); 
      servoPins[recordedStep.servoIndex].servo.write(currentServoPosition);
      delay(50);
    }
  }
  delay(2000); // Delay before starting the actual steps.
  
  for (int i = 4; i < recordedSteps.size() && playRecordedSteps ; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    delay(recordedStep.delayInStep);
    servoPins[recordedStep.servoIndex].servo.write(recordedStep.value);
  }
}

void notify()
{
  int rx =(Ps3.data.analog.stick.rx);  //Base       =>  Right stick - x axis
  int ry =(Ps3.data.analog.stick.ry);  //Shoulder   =>  Right stick  - y axis
  int ly =(Ps3.data.analog.stick.ly);  //Elbow      =>  Left stick  - y axis 
  int lx =(Ps3.data.analog.stick.lx);  //Gripper    =>  Left stick - x axis

  if (Ps3.event.button_down.start)
  {
    playRecordedSteps = !playRecordedSteps;
    recordSteps = false;
  }  
  else if (!playRecordedSteps && Ps3.event.button_down.select)
  {
    recordSteps = !recordSteps;
    if (recordSteps)
    {
      recordedSteps.clear();
      previousTimeInMilli = millis();
    }
  }  
  else if (!playRecordedSteps)
  {
    if (rx > 50)
    {
      writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);  
    }
    else if (rx < -50)
    {
      writeServoValues(0, SERVO_FORWARD_STEP_ANGLE);  
    }
  
    if (ry > 50)
    {
      writeServoValues(1, SERVO_BACKWARD_STEP_ANGLE);  
    }
    else if (ry < -50)
    {
      writeServoValues(1, SERVO_FORWARD_STEP_ANGLE);  
    }
  
    if (ly > 50)
    {
      writeServoValues(2, SERVO_FORWARD_STEP_ANGLE);  
    }
    else if (ly < -50)
    {
      writeServoValues(2, SERVO_BACKWARD_STEP_ANGLE);  
    }

    if (lx > 50)
    {
      writeServoValues(3, SERVO_BACKWARD_STEP_ANGLE);  
    }
    else if (lx < -50)
    {
      writeServoValues(3, SERVO_FORWARD_STEP_ANGLE);  
    }

    if (Ps3.event.button_down.r2)
    {
      gripperSwitch = !gripperSwitch;  //Toggle gripper close / open
      gripperSwitch ? writeServoValues(3, 170, true) :  writeServoValues(3, 100, true) ;
    }
    
    delay(10);
  }    
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void setUpPinModes()
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }
}


void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin();
  Serial.println("Ready.");
}

void loop()
{
  if (playRecordedSteps)
  { 
    playRecordedRobotArmSteps();
  }  
}
