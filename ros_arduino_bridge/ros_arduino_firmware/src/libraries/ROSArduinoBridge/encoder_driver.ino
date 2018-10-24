static volatile long A_enc_pos = 0L;
static volatile long B_enc_pos = 0L;
static volatile long C_enc_pos = 0L;
static volatile long D_enc_pos = 0L;

/*init encoder connect pin*/
void initEncoders()
{
  pinMode(ENC_A_PIN_A, INPUT);
  pinMode(ENC_A_PIN_B, INPUT);
  attachInterrupt(5, encoderA_ISR, CHANGE);

  pinMode(ENC_B_PIN_A, INPUT);
  pinMode(ENC_B_PIN_B, INPUT);
  attachInterrupt(4, encoderB_ISR, CHANGE);

  pinMode(ENC_C_PIN_A, INPUT);
  pinMode(ENC_C_PIN_B, INPUT);
  attachInterrupt(3, encoderC_ISR, CHANGE);

  pinMode(ENC_D_PIN_A, INPUT);
  pinMode(ENC_D_PIN_B, INPUT);
  attachInterrupt(2, encoderD_ISR, CHANGE);
}

/* Interrupt routine for A encoder, taking care of actual counting */
void encoderA_ISR ()
{
  if (directionWheel(A_WHEEL) == BACKWARDS)
  {
    A_enc_pos--;
  }
  else 
  {
    A_enc_pos++;
  }
}

/* Interrupt routine for B encoder, taking care of actual counting */
void encoderB_ISR () 
{
  if (directionWheel(B_WHEEL) == BACKWARDS)
  {
    B_enc_pos--;
  }
  else 
  {
    B_enc_pos++;
  }
}

/* Interrupt routine for C encoder, taking care of actual counting */
void encoderC_ISR () 
{
  if (directionWheel(C_WHEEL) == BACKWARDS)
  {
    C_enc_pos--;
  }
  else 
  {
    C_enc_pos++;
  }
}

void encoderD_ISR () 
{
  if (directionWheel(D_WHEEL) == BACKWARDS)
  {
    D_enc_pos--;
  }
  else 
  {
    D_enc_pos++;
  }
}

/* Wrap the encoder reading function */
long readEncoder(int i) 
{
  if (i == A_WHEEL)
  {
    return A_enc_pos;
  }
  else if (i == B_WHEEL)
  {
    return B_enc_pos;
  }
  else if (i == C_WHEEL)
  {
    return C_enc_pos;
  }
  else
  {
    return D_enc_pos;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() 
{
  A_enc_pos = 0L;
  B_enc_pos = 0L;
  C_enc_pos = 0L;
  D_enc_pos = 0L;
}

