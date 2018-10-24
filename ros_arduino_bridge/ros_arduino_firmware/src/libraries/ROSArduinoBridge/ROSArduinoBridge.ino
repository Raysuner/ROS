
/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Include definition of serial commands */
#include "commands.h"

/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 300
long lastMotorCommand = AUTO_STOP_INTERVAL;


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[64];  // 16个参数,每个参3位数,然后加上15个冒号,再加上一个回车,最后加起来等于64,防止溢出
char argv2[64];
char argv3[64];
char argv4[64];

// The arguments converted to integers
long arg1 = 0;
long arg2 = 0;
long arg4 = 0;
long arg3 = 0;

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg4 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[16];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch (cmd) {
    case GET_BAUDRATE: // 'b'
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ: //'a'
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ: //'d'
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE: //'x'
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE: //'w'
      if (arg2 == 0) 
      {
        digitalWrite(arg1, LOW);
      }
      else if (arg2 == 1) 
      {
        digitalWrite(arg1, HIGH);
      }
      Serial.println("OK");
      break;
    case PIN_MODE: //'c'
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
  
    case READ_ENCODERS: //'e'
      Serial.print(readEncoder(A_WHEEL));
      Serial.print(" ");
      Serial.print(readEncoder(B_WHEEL));
      Serial.print(" ");
      Serial.print(readEncoder(C_WHEEL));
      Serial.print(" ");
      Serial.println(readEncoder(D_WHEEL));
      break;
    case RESET_ENCODERS: //'r'
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS: //'m'
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) 
      {
        setMotorSpeeds(0, 0, 0, 0);
        resetPID();
        moving = 0;
      }
      else 
      {
        moving = 1;
      }
      AWheelPID.TargetTicksPerFrame = arg1;
      BWheelPID.TargetTicksPerFrame = arg2;
      CWheelPID.TargetTicksPerFrame = arg3;
      DWheelPID.TargetTicksPerFrame = arg4;
      Serial.println("OK");
      break;
    case UPDATE_PID: //'u'
      while ((str = strtok_r(p, ":", &p)) != '\0') 
      {
        pid_args[i] = atoi(str);
        i++;
      }
      AWheel_Kp = pid_args[0];
      AWheel_Kd = pid_args[1];
      AWheel_Ki = pid_args[2];
      AWheel_Ko = pid_args[3];

      BWheel_Kp = pid_args[4];
      BWheel_Kd = pid_args[5];
      BWheel_Ki = pid_args[6];
      BWheel_Ko = pid_args[7];

      CWheel_Kp = pid_args[8];
      CWheel_Kd = pid_args[9];
      CWheel_Ki = pid_args[10];
      CWheel_Ko = pid_args[11];

      DWheel_Kp = pid_args[12];
      DWheel_Kd = pid_args[13];
      DWheel_Ki = pid_args[14];
      DWheel_Ko = pid_args[15];

      Serial.print(AWheel_Kp);
      Serial.print(" ");
      Serial.print(AWheel_Kd);
      Serial.print(" ");
      Serial.print(AWheel_Ki);
      Serial.print(" ");
      Serial.print(AWheel_Ko);
      Serial.print(" ");
      
      Serial.print(BWheel_Kp);
      Serial.print(" ");
      Serial.print(BWheel_Kd);
      Serial.print(" ");
      Serial.print(BWheel_Ki);
      Serial.print(" ");
      Serial.print(BWheel_Ko);
      Serial.print(" ");

      Serial.print(CWheel_Kp);
      Serial.print(" ");
      Serial.print(CWheel_Kd);
      Serial.print(" ");
      Serial.print(CWheel_Ki);
      Serial.print(" ");
      Serial.print(CWheel_Ko);
      Serial.print(" ");

      Serial.print(DWheel_Kp);
      Serial.print(" ");
      Serial.print(DWheel_Kd);
      Serial.print(" ");
      Serial.print(DWheel_Ki);
      Serial.print(" ");
      Serial.println(DWheel_Ko);
      break;

      case READ_PIDIN: //'i'
      Serial.print(readPidIn(A_WHEEL));
      Serial.print(" ");
      Serial.print(readPidIn(B_WHEEL));
      Serial.print(" ");
      Serial.print(readPidIn(C_WHEEL));
      Serial.print(" ");
      Serial.println(readPidIn(D_WHEEL));
      break;

    case READ_PIDOUT: //'o'
      Serial.print(readPidOut(A_WHEEL));
      Serial.print(" ");
      Serial.print(readPidOut(B_WHEEL));
      Serial.print(" ");
      Serial.print(readPidOut(C_WHEEL));
      Serial.print(" ");
      Serial.println(readPidOut(D_WHEEL));
      break;
      
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() 
{
  pinMode(16,OUTPUT);  //这里两个引脚给高电平是为了给编码器供电
  pinMode(17,OUTPUT);
  Serial.begin(BAUDRATE);
  initEncoders();
  // Initialize the motor controller if used */
  initMotorController();
  resetPID();
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() 
{
  digitalWrite(16,HIGH);
  digitalWrite(17,HIGH);
  while (Serial.available() > 0) 
  {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) 
    {
      if (arg == 1) 
      {
        argv1[index] = '\0';
      }
      else if (arg == 2) 
      {
        argv2[index] = '\0';
      }
      else if (arg == 3) 
      {
        argv3[index] = '\0';
      }
      else if(arg == 4) 
      {
        argv4[index] = '\0';
      }
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') 
    {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) 
      {
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }

      else if (arg == 2)  
      {
        argv2[index] = '\0';
        arg = 3;
        index = 0;
      }

      else if (arg == 3)  
      {
        argv3[index] = '\0';
        arg = 4;
        index = 0;
      }
      continue;
    }
    else 
    {
      if (arg == 0) 
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) 
      {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) 
      {
        argv2[index] = chr;
        index++;
      }

      else if (arg == 3) 
      {
        argv3[index] = chr;
        index++;
      }

      else if (arg == 4) 
      {
        argv4[index] = chr;
        index++;
      }
    }
  }

  // If we are using base control, run a PID calculation at the appropriate intervals
  if (millis() > nextPID) 
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) 
  {
    ;
    setMotorSpeeds(0, 0 ,0, 0);
    resetPID();
    moving = 0;
  }
}

