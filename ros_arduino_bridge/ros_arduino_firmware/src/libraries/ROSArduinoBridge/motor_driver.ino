/*
 * get wheel run direction
*/
boolean directionWheel(int wheel) 
{
  if (wheel == A_WHEEL) 
  {
    return direcA;
  }
  else if (wheel == B_WHEEL)
  {
    return direcB;
  }
   else if (wheel == C_WHEEL)
  {
    return direcC;
  }
  else 
  {
    return direcD;
  }
}

/* Wrap the motor driver initialization,
   set all the motor control pins to outputs **/
void initMotorController()
{
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);

  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  pinMode(M3, OUTPUT);
  pinMode(E3, OUTPUT);

  pinMode(M4, OUTPUT);
  pinMode(E4, OUTPUT);
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int wheel, int spd)
{
  if (spd > MAX_PWM) 
  {
    spd = MAX_PWM;
  }
  if (spd < -MAX_PWM) 
  {
    spd = -1 * MAX_PWM;
  }

  if (wheel == A_WHEEL)
  {
    if (spd >= 0) 
    {
      direcA = FORWARDS;
      digitalWrite(M1, HIGH);
      analogWrite(E1, spd);
    }
    else if (spd < 0) 
    {
      direcA = BACKWARDS;
      digitalWrite(M1, LOW);
      analogWrite(E1, -spd);
    }
  }
  else if (wheel == B_WHEEL)
  {
    if (spd >= 0) 
    {
      direcB = FORWARDS;
      digitalWrite(M2, LOW);
      analogWrite(E2, spd);
    }
    else if (spd < 0) 
    {
      direcB = BACKWARDS;
      digitalWrite(M2, HIGH);
      analogWrite(E2, -spd);
    }
  }
  else if (wheel == C_WHEEL)
  {
    if (spd >= 0) 
    {
      direcC = FORWARDS;
      digitalWrite(M3, LOW);
      analogWrite(E3, spd);
    }
    else if (spd < 0) 
    {
      direcC = BACKWARDS;
      digitalWrite(M3, HIGH);
      analogWrite(E3, -spd);
    }
  }
  else
  {
    if (spd >= 0) 
    {
      direcD = FORWARDS;
      digitalWrite(M4, HIGH);
      analogWrite(E4, spd);
    }
    else if (spd < 0) 
    {
      direcD = BACKWARDS;
      digitalWrite(M4, LOW);
      analogWrite(E4, -spd);
    }
  }  
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed,int DSpeed)
{
  setMotorSpeed(A_WHEEL, ASpeed);
  setMotorSpeed(B_WHEEL, BSpeed);
  setMotorSpeed(C_WHEEL, CSpeed);
  setMotorSpeed(D_WHEEL, DSpeed);
}

