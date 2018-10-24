#include "commands.h"
/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  */
  //int Ierror;
  int ITerm;                    //integrated term
  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo AWheelPID, BWheelPID, CWheelPID, DWheelPID;

/* PID Parameters */
int AWheel_Kp = 20;
int AWheel_Kd = 12;
int AWheel_Ki = 0;
int AWheel_Ko = 50;

int BWheel_Kp = 20;
int BWheel_Kd = 12;
int BWheel_Ki = 0;
int BWheel_Ko = 50;

int CWheel_Kp = 20;
int CWheel_Kd = 12;
int CWheel_Ki = 0;
int CWheel_Ko = 50;

int DWheel_Kp = 20;
int DWheel_Kd = 12;
int DWheel_Ki = 0;
int DWheel_Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID()
{
  AWheelPID.TargetTicksPerFrame = 0.0;
  AWheelPID.Encoder = readEncoder(A_WHEEL);
  AWheelPID.PrevEnc = AWheelPID.Encoder;
  AWheelPID.output    = 0;
  AWheelPID.PrevInput = 0;
  AWheelPID.ITerm     = 0;

  BWheelPID.TargetTicksPerFrame = 0.0;
  BWheelPID.Encoder = readEncoder(B_WHEEL);
  BWheelPID.PrevEnc = BWheelPID.Encoder;
  BWheelPID.output    = 0;
  BWheelPID.PrevInput = 0;
  BWheelPID.ITerm     = 0;

  CWheelPID.TargetTicksPerFrame = 0.0;
  CWheelPID.Encoder = readEncoder(C_WHEEL);
  CWheelPID.PrevEnc = CWheelPID.Encoder;
  CWheelPID.output    = 0;
  CWheelPID.PrevInput = 0;
  CWheelPID.ITerm     = 0;

  DWheelPID.TargetTicksPerFrame = 0.0;
  DWheelPID.Encoder = readEncoder(D_WHEEL);
  DWheelPID.PrevEnc = DWheelPID.Encoder;
  DWheelPID.output    = 0;
  DWheelPID.PrevInput = 0;
  DWheelPID.ITerm     = 0;
}

/* PID routine to compute the next motor commands */
void doAWheelPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;  //计算出误差值


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */

  output = (AWheel_Kp * Perror - AWheel_Kd * (input - p->PrevInput) + p->ITerm) / AWheel_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += AWheel_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

void doBWheelPID(SetPointInfo * p)
{
  long Perror = 0;
  long output = 0;
  int input   = 0;

  p->Encoder = readEncoder(B_WHEEL);
  input  = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
    Avoid derivative kick and allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (BWheel_Kp * Perror - BWheel_Kd * (input - p->PrevInput) + p->ITerm) / BWheel_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
  {
    output = MAX_PWM;
  }
  else if (output <= -MAX_PWM)
  {
    output = -MAX_PWM;
  }
  else
  {
    /*
      allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    p->ITerm += BWheel_Ki * Perror;
  }

  p->output = output;
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands */
void doCWheelPID(SetPointInfo * p)
{
  long Perror = 0;
  long output = 0;
  int input   = 0;

  p->Encoder = readEncoder(C_WHEEL);
  input  = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
    Avoid derivative kick and allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (CWheel_Kp * Perror - CWheel_Kd * (input - p->PrevInput) + p->ITerm) / CWheel_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
  {
    output = MAX_PWM;
  }
  else if (output <= -MAX_PWM)
  {
    output = -MAX_PWM;
  }
  else
  {
    /*
      allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    p->ITerm += CWheel_Ki * Perror;
  }

  p->output    = output;
  p->PrevInput = input;
}

void doDWheelPID(SetPointInfo * p)
{
  long Perror = 0;
  long output = 0;
  int input   = 0;

  p->Encoder = readEncoder(D_WHEEL);
  input  = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
    Avoid derivative kick and allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (DWheel_Kp * Perror - DWheel_Kd * (input - p->PrevInput) + p->ITerm) / DWheel_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
  {
    output = MAX_PWM;
  }
  else if (output <= -MAX_PWM)
  {
    output = -MAX_PWM;
  }
  else
  {
    /*
      allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    p->ITerm += DWheel_Ki * Perror;
  }

  p->output    = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() 
{
  /* Read the encoders */
  AWheelPID.Encoder = readEncoder(A_WHEEL);
  BWheelPID.Encoder = readEncoder(B_WHEEL);
  CWheelPID.Encoder = readEncoder(C_WHEEL);
  DWheelPID.Encoder = readEncoder(D_WHEEL);
  
  /* If we're not moving there is nothing more to do */
  if (!moving)
  {
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (AWheelPID.PrevInput != 0 || BWheelPID.PrevInput != 0 || CWheelPID.PrevInput != 0 || DWheelPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doAWheelPID(&AWheelPID);
  doBWheelPID(&BWheelPID);
  doCWheelPID(&CWheelPID);
  doDWheelPID(&DWheelPID);
  
  /* Set the motor speeds accordingly */
  setMotorSpeeds(AWheelPID.output, BWheelPID.output, CWheelPID.output, DWheelPID.output);
}

long readPidIn(int wheel)
{
  long pidin = 0;
  if (wheel == A_WHEEL)
  {
    pidin = AWheelPID.PrevInput;
  }
  else if (wheel == B_WHEEL)
  {
    pidin = BWheelPID.PrevInput;
  }
  else if (wheel == C_WHEEL)
  {
    pidin = CWheelPID.PrevInput;
  }
  else
  {
    pidin = DWheelPID.PrevInput;
  }

  return pidin;
}

long readPidOut(int wheel)
{
  long pidout = 0;
  if (wheel == A_WHEEL)
  {
    pidout = AWheelPID.output;
  }
  else if (wheel == B_WHEEL)
  {
    pidout = BWheelPID.output;
  }
  else if (wheel == C_WHEEL)
  {
    pidout = CWheelPID.output;
  }
  else
  {
    pidout = DWheelPID.output;
  }

  return pidout;
}

