/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

static boolean direcA = FORWARDS;
static boolean direcB = FORWARDS;
static boolean direcC = FORWARDS;
static boolean direcD = FORWARDS;


void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed,int DSpeed);
