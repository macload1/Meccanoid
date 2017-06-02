

#define SYSCTL_MECCANO_GPIO     SYSCTL_PERIPH_GPIOE
#define GPIO_MECCANO_BASE       GPIO_PORTE_BASE
#define GPIO_MECCANO_PIN        GPIO_PIN_4


void MeccanoInit(void);
void setLEDColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t fadetime);
void setServoColor(uint8_t servoNum, uint8_t color);
void setServoPosition(uint8_t servoNum, uint8_t pos);
void setServotoLIM(uint8_t servoNum);
uint8_t getServoPosition(uint8_t servoNum);
void communicate(void);
void sendByte(uint8_t servoData);
uint8_t receiveByte(void);
uint8_t calculateCheckSum(uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4);
