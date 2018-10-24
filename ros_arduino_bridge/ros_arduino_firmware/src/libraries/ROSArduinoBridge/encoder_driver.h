//A wheel encode pin
#define ENC_A_PIN_A  18   
#define ENC_A_PIN_B  22   

//B wheel encode pin
#define ENC_B_PIN_A  19  
#define ENC_B_PIN_B  23  

//C wheel encode pin
#define ENC_C_PIN_A  20   
#define ENC_C_PIN_B  24   

//D wheel encode pin
#define ENC_D_PIN_A  21   
#define ENC_D_PIN_B  25   

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initEncoders();
