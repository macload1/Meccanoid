
/* I/O Port Definitions */
#define SYSCTL_MECCANO_GPIO     SYSCTL_PERIPH_GPIOE
#define GPIO_MECCANO_BASE       GPIO_PORTE_BASE
#define GPIO_MECCANO_PIN        GPIO_PIN_4

/* Delay Definitions */
#define TIMER_LOADVALUE_417US	33360
#define TIMER_LOADVALUE_550US	44000
#define TIMER_LOADVALUE_500US	40000
#define TIMER_LOADVALUE_1500US	120000
#define TIMER_LOADVALUE_3000US	240000
#define TIMER_LOADVALUE_10MS	800000

/* Meccano Line Definitions */
#define	MECCANO_HEAD			0x00
#define	MECCANO_LEFT_ARM		0x01
#define	MECCANO_RIGHT_ARM		0x02



void MeccanoInit(void);
void setLEDColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t fadetime);
void setServoColor(uint8_t servoNum, uint8_t color);
void setServoPosition(uint8_t servoNum, uint8_t pos);
void setServotoLIM(uint8_t servoNum);
uint8_t getServoPosition(uint8_t servoNum);
uint8_t calculateCheckSum(uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4);
