/*****************************************************************************************/
/* downstairs.ino sketch v0.4                                                            */
/* Year 2020 by Jens Bundfuss                                                            */
/*                                                                                       */
/* - initial revision                                                                    */
/* - Filtering of input signals                                                          */
/* - TFT support                                                                         */
/* - detailed debug support                                                              */
/* - Documentation added                                                                 */
/* - various modes added                                                                 */
/*****************************************************************************************/
/* include I2C Driver for setting PWM, TFT, Spi, ... */
#include "FaBoPWM_PCA9685.h"
#include <SPI.h>
#include <TFT.h>

/*****************************************************************************************/
/*                                    GENERAL DEFINE                                     */
/*****************************************************************************************/
#define TRUE  1
#define FALSE 0

#define STATE_OFF           0
#define STATE_ON            1

#define LEDS_BASEMENT       0
#define LEDS_GROUNDFLOOR    1
#define LEDS_FLOOR_MAX      (LEDS_GROUNDFLOOR + 1)

/* Debug ENABLE? */
#define PRINTF_DEBUG                  STATE_OFF
#define PRINTF_DEBUG_LEVEL            2           /* 1 -> Channel Info */
                                                  /* 2 -> Channel Info + Mode Info */
                                                  /* 3 -> All Debug Info */

/*****************************************************************************************/
/*                                         DEFINE                                        */
/*****************************************************************************************/
/* TFT */
#define CS   10        /* Chipselect TFT Display. Do not change */
#define DC   9         /* Signal line TFT Display */
#define RESET  8       /* Reset line TFT Display */

/* FABO_PWM */
#define PWM_ANZAHL_PCA9685_CH   16    /* PWM Channel Number of Module */
#define PWM_ANZAHL_PCA9685      1     /* PWM Modules used */
#define PWM_FREQUENZ            1000  /* caution ISR Frequence depends on this !!! */
#define PWM_ADDRESS_DEFAULT     0x40  /* Adress of PWM Module*/

/* LED */
#define LED_0_PROZENT           0     /* [Digit] Lowest Value */
#define LED_100_PROZENT         1024  /* [Digit] Highest Value (4096) */

/* Mode Handling */
//#define LED_MODE_2_FADING_STEPS 7     /* [Digit] Fading steps */
//#define LED_MODE_3_FADING_STEPS 7     /* [Digit] Fading steps */
//#define LED_MODE_4_FADING_STEPS 254   /* [Digit] Fading steps */
//#define LED_MODE_4_ON_TIME      100   /* [10ms] On Time */

#define LED_MODE_2_FADING_STEPS 4     /* [Digit] Fading steps */
#define LED_MODE_3_FADING_STEPS 4     /* [Digit] Fading steps */
#define LED_MODE_4_FADING_STEPS 254   /* [Digit] Fading steps */
#define LED_MODE_4_ON_TIME      100   /* [10ms] On Time */

/* LCD */
#define TFT_NACHLAUF_ZEIT       100   /* equal to 20s */

/* Pin definition for External Interrup1 Pin (comes from Pwm Module) */
#define INTERRUPT_EXT1_PIN      2

/* Pin definition of PIR (digital IO) */
#define PIN_3_FREE              3
#define PIR_PIN_UG_UG           4
#define PIR_PIN_EG_UG           5
#define PIR_PIN_EG_OG           6
#define PIR_PIN_OG_OG           7

/* Pin definition for Analog Pin */
#define ANALOG_0_PIN            A0  /* Poti Brightness */
#define ANALOG_2_PIN            A1  /* Poti Afterrun */
#define ANALOG_1_PIN            A2  /* Poti Future use */
#define ANALOG_3_PIN            A3  /* Brightness sensor */  

/* ADC */
#define ADC_HYSTERESE               10    /* digit Hysterese to avoid jitter of Adc signals */
#define ADC_BRIGHTNESS_TRESHHOLD    800   /* Treshhold for Analog value (Analog Pin used as Port Pin)*/

/* Index of PIR (Photo Infrared Resistor) */
#define PIR_TIMER_INDEX_UG_UG   0
#define PIR_TIMER_INDEX_EG_UG   1
#define PIR_TIMER_INDEX_EG_OG   2
#define PIR_TIMER_INDEX_OG_OG   3
#define PIR_TIMER_INDEX_MAX     4

/* DEBUG ON */
#if (PRINTF_DEBUG == STATE_ON)
  #if (PRINTF_DEBUG_LEVEL >= 3)
    #define PRINTF_DEBUG_SENSOR_INFO_RAW    Serial.print("Adc0_Raw: "); \
                                              Serial.print(Port_AdcIn[0]); \
                                              Serial.print("\t Adc1_Raw: "); \
                                              Serial.print(Port_AdcIn[1]); \
                                              Serial.print("\t Adc2_Raw: "); \
                                              Serial.print(Port_AdcIn[2]); \
                                              Serial.print("\t Adc3_Raw: "); \
                                              Serial.print(Port_AdcIn[3]); \
                                              Serial.print("\n");
    
      #define PRINTF_DEBUG_SENSOR_INFO_HYST   Serial.print("Adc0_Hyst: "); \
                                              Serial.print(Port_AdcInHyst[0]); \
                                              Serial.print("\t Adc1_Hyst: "); \
                                              Serial.print(Port_AdcInHyst[1]); \
                                              Serial.print("\t Adc2_Hyst: "); \
                                              Serial.print(Port_AdcInHyst[2]); \
                                              Serial.print("\t Adc3_Hyst: "); \
                                              Serial.print(Port_AdcInHyst[3]); \
                                              Serial.print("\n");
  #else /* (PRINTF_DEBUG_LEVEL >= 3) */
    #define PRINTF_DEBUG_SENSOR_INFO_RAW
    #define PRINTF_DEBUG_SENSOR_INFO_HYST 
  #endif /* (PRINTF_DEBUG_LEVEL >= 3) */
  #if (PRINTF_DEBUG_LEVEL >= 2)
    #define PRINTF_DEBUG_SENSOR_INFO    Serial.print("Bright: "); \
                                        Serial.print(Led_Get_BrightnessLevel()/10); \
                                        Serial.print("%"); \
                                        Serial.print("\t Time: "); \
                                        Serial.print(Led_Get_AfterrunTime()); \
                                        Serial.print("ms"); \
                                        Serial.print("\t Mode: "); \
                                        Serial.print(Led_Get_Mode()); \
                                        Serial.print("\t BLvl1: "); \
                                        Serial.print(Brightness_Sens1); \
                                        Serial.print(" BLvl2: "); \
                                        Serial.print(Brightness_Sens2); \
                                        Serial.print("\t OGOG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Akt); \
                                        Serial.print(" EGOG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Akt); \
                                        Serial.print(" EGUG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Akt); \
                                        Serial.print(" UGUG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Akt); \
                                        Serial.print("\t UP: "); \
                                        Serial.print(Led_Upstairs); \
                                        Serial.print(" DW: "); \
                                        Serial.print(Led_Downstairs); \
                                        Serial.print(" UGUG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_ClusterState); \
                                        Serial.print(" EGUG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState); \
                                        Serial.print(" EGOG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_ClusterState); \
                                        Serial.print(" OGOG: "); \
                                        Serial.print(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState); \
                                        Serial.print("\n");
  #else /* (PRINTF_DEBUG_LEVEL >= 2) */
    #define PRINTF_DEBUG_SENSOR_INFO
  #endif /* (PRINTF_DEBUG_LEVEL >= 2) */
  #if (PRINTF_DEBUG_LEVEL >= 1)
    #define PRINTF_DEBUG_LED_CHANNEL_INFO   Serial.print("U1: "); \
                                            Serial.print(Led_Struct[0].Led_AktValue); \
                                            Serial.print("\tU2: "); \
                                            Serial.print(Led_Struct[1].Led_AktValue); \
                                            Serial.print("\tU3: "); \
                                            Serial.print(Led_Struct[2].Led_AktValue); \
                                            Serial.print("\tU4: "); \
                                            Serial.print(Led_Struct[3].Led_AktValue); \
                                            Serial.print("\tU5: "); \
                                            Serial.print(Led_Struct[4].Led_AktValue); \
                                            Serial.print("\tO1: "); \
                                            Serial.print(Led_Struct[5].Led_AktValue); \
                                            Serial.print("\tO2: "); \
                                            Serial.print(Led_Struct[6].Led_AktValue); \
                                            Serial.print("\tO3: "); \
                                            Serial.print(Led_Struct[7].Led_AktValue); \
                                            Serial.print("\tO4: "); \
                                            Serial.print(Led_Struct[8].Led_AktValue); \
                                            Serial.print("\tO5: "); \
                                            Serial.print(Led_Struct[9].Led_AktValue); \
                                            Serial.print("\t UP: "); \
                                            Serial.print(Led_Upstairs); \
                                            Serial.print(" DW: "); \
                                            Serial.print(Led_Downstairs); \
                                            Serial.print("\n"); 
  #else /* (PRINTF_DEBUG_LEVEL >= 1) */
    #define PRINTF_DEBUG_LED_CHANNEL_INFO
  #endif /* (PRINTF_DEBUG_LEVEL >= 1) */
#else /* (PRINTF_DEBUG == STATE_ON) */
  #define PRINTF_DEBUG_LED_CHANNEL_INFO
  #define PRINTF_DEBUG_SENSOR_INFO
  #define PRINTF_DEBUG_SENSOR_INFO_RAW
  #define PRINTF_DEBUG_SENSOR_INFO_HYST 
#endif /* (PRINTF_DEBUG == STATE_ON) */

/*****************************************************************************************/
/*                                     TYPEDEF ENUM                                      */
/*****************************************************************************************/
typedef enum
{
	LED_0 = 0,
	LED_1,
	LED_2,
	LED_3,
	LED_4,
	LED_5,
	LED_6,
	LED_7,
	LED_8,
	LED_9,
  LED_MAX
}LED_INDEX;

typedef enum
{
  LED_MODE_2_OFF = 0,
  LED_MODE_2_FADE_IN,
  LED_MODE_2_ON,
  LED_MODE_2_FADE_OUT
}LED_MODE_2_STATE;

typedef enum
{
  LED_MODE_3_OFF = 0,
  LED_MODE_3_SEGMENT0_ON,
  LED_MODE_3_SEGMENT1_ON,
  LED_MODE_3_SEGMENT2_ON,
  LED_MODE_3_SEGMENT3_ON,
  LED_MODE_3_SEGMENT4_ON,
}LED_MODE_3_STATE;

typedef enum
{
  LED_MODE_4_OFF = 0,
  LED_MODE_4_SEGMENT0_ON,
  LED_MODE_4_SEGMENT1_ON,
  LED_MODE_4_SEGMENT2_ON,
  LED_MODE_4_SEGMENT3_ON,
  LED_MODE_4_SEGMENT4_ON,
}LED_MODE_4_STATE;



/*****************************************************************************************/
/*                                   TYPEDEF STRUCT                                      */
/*****************************************************************************************/
typedef struct LED_STRUCT_TAG
{
	uint16_t Led_AktValue;
}LED_STRUCT;

typedef struct PIR_STRUCT_TAG
{
  uint16_t Pir_TimerValue;
  uint8_t Pir_State_Akt;
  uint8_t Pir_State_Old;
  uint8_t Pir_ClusterState;
}PIR_STRUCT;

/*****************************************************************************************/
/*                                         VARIABLES                                     */
/*****************************************************************************************/
/* ISR */
static uint32_t Isr_Ext1_counter = 0;

/* TFT */
static TFT TFTscreen = TFT(CS, DC, RESET);            /* Display TFT 1,7 Zoll ST7735 buy at AZ-Delivery */
static uint16_t TFT_OnState = FALSE;
static uint32_t TFT_OnCounter = TFT_NACHLAUF_ZEIT;
/* TFT Output Text */
char TFT_BrightnessPrintOut[4] = {"   "};
char TFT_BrightnessPrintOut_Old[4] = {"   "};
char TFT_AfterrunPrintOut[4] = {"   "};
char TFT_AfterrunPrintOut_Old[4] = {"   "};
char TFT_ModePrintOut[4] = {"   "};
char TFT_ModePrintOut_Old[4] = {"   "};

/* PWM */
static FaBoPWM faboPWM;                              /* I2C driver class */

/* PIR */
static PIR_STRUCT Pir_Struct[PIR_TIMER_INDEX_MAX];

/* Mode 2: Task State for Down and Up-/ Stairs*/
static LED_MODE_2_STATE Led_Mode_2_State[LEDS_FLOOR_MAX];
/* Mode 2: Task State for Down and Up-/ Stairs*/
static LED_MODE_3_STATE Led_Mode_3_State[LEDS_FLOOR_MAX];
/* Mode 4: Task State for Down and Up-/ Stairs*/
static LED_MODE_4_STATE Led_Mode_4_State[LEDS_FLOOR_MAX];
static uint16_t Led_Mode_4_SegmentOn_Counter[LEDS_FLOOR_MAX] = {LED_MODE_4_ON_TIME, LED_MODE_4_ON_TIME};

/* Led */
static LED_STRUCT Led_Struct[LED_MAX];
static uint8_t  Led_Downstairs = STATE_OFF;
static uint8_t  Led_Upstairs = STATE_OFF;
static uint8_t  Led_Downstairs_Old = STATE_ON;
static uint8_t  Led_Upstairs_Old = STATE_ON;
static uint16_t Led_BrightnessLevel;
static uint16_t Led_AfterrunTime;
static uint16_t Led_PotiMode;
static uint16_t Led_AdcSensorBrightness;


static uint16_t Port_AdcIn[4]= {0,0,0,0};
static uint16_t Port_AdcInHyst[4]= {0,0,0,0};

static uint16_t Brightness_Sens1;
static uint16_t Brightness_Sens2;




/*************************************************************************************************/
/**************************************************************************************************
Function: setup()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void setup(void) 
{
  uint16_t i = 0;
  uint16_t Loc_Begin = 0;
  uint16_t Loc_TftWidth = 0;
  uint16_t Loc_TftHeight = 0;

  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("################################");
  Serial.println("#### Program Downstairs v0.3 ###");
  Serial.println("################################");
  Serial.println(" ");
  Serial.println("Starting ...");
  Serial.println(" ");
  
  for (i = 0; i < PWM_ANZAHL_PCA9685; i++)
  {
  	Loc_Begin |= faboPWM.begin(PWM_ADDRESS_DEFAULT + i);
  }
  /* join I2C bus (I2Cdev library doesn't do this automatically */
  if(Loc_Begin != 0)
  {
    Serial.println("Find PCA9685 PCA9685 1,2,3");
  }
  for (i = 0; i < PWM_ANZAHL_PCA9685; i++)
  {
    faboPWM.init(PWM_ADDRESS_DEFAULT + i, 0);
    faboPWM.set_hz(PWM_ADDRESS_DEFAULT + i, PWM_FREQUENZ);
  }


  /** Init Functions **/
  Serial.println("INT Init ...");
  Interrupt_Init();
  Serial.println("PIR Init ...");
  Pir_Init();
  Serial.println("LED Init ...");
  Led_Init();
  Serial.println("PIN Init ...");
  pinMode(PIN_3_FREE, INPUT);

  /* Set LCD off */
  TFT_OnState = FALSE;
  
  Serial.println("TFT Init ...");
  TFTscreen.begin();
  Loc_TftWidth = TFTscreen.width();
  Loc_TftHeight = TFTscreen.height();
  Serial.print("TFT Screen Heigh: ");
  Serial.print(Loc_TftHeight);
  Serial.println(" Pixel");
  Serial.print("TFT Screen Width: ");
  Serial.print(Loc_TftWidth);
  Serial.println(" Pixel");
  /* Setup the default Screen */
  TFTscreen.background(0,0,0);
  TFTscreen.fill(255,255,255);
  TFTscreen.stroke(255,255,255);
  TFTscreen.setTextSize(2);
  TFTscreen.text("Helligkeit: ",0,0);
  TFTscreen.text("Nachlaufzeit: ",0,40);
  TFTscreen.text("% ",40,20);
  TFTscreen.text("sek. ",40,60);
  TFTscreen.text(TFT_BrightnessPrintOut, 0, 20);
  TFTscreen.text(TFT_AfterrunPrintOut, 0, 60);
  TFTscreen.setTextSize(3);
  TFTscreen.text("UG",0,100);
  TFTscreen.text("OG",80,100);
  TFTscreen.fill(0,0,0);
  TFTscreen.rect(45,100, 22, 22);
  TFTscreen.fill(0,0,0);
  TFTscreen.rect(125,100, 22, 22);


  Serial.println(" *** Init finished *** ");
   
  /* Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started. */
}

/*************************************************************************************************/
/**************************************************************************************************
Function: loop()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void loop()
{

  //Serial.println("loop...");

  //PRINTF_DEBUG_SENSOR_INFO              /* print detailed info */
  PRINTF_DEBUG_SENSOR_INFO_RAW          /* print detailed info */
  PRINTF_DEBUG_SENSOR_INFO_HYST         /* print detailed info */
  PRINTF_DEBUG_LED_CHANNEL_INFO         /* print detailed info */
  
  /********************************************************************************************/
  /************************      HANDLING OF LED SETTING        *******************************/ 
  /********************************************************************************************/
  /* Read Poti Brightness information */
  Port_AdcIn[0] = analogRead(ANALOG_0_PIN);
  /* Read Poti Afterrun time */
  Port_AdcIn[1] = analogRead(ANALOG_1_PIN);
    /* Read Poti Mode */
  Port_AdcIn[2] = analogRead(ANALOG_2_PIN);
  /* Read Sensor Brightness */
  Port_AdcIn[3] = analogRead(ANALOG_3_PIN);
  /* Calculate ADC Hysteresis value with 1 digit */
  Port_AdcInHyst[0] = Port_AdcFilt(Port_AdcIn[0], Port_AdcInHyst[0]);
  Port_AdcInHyst[1] = Port_AdcFilt(Port_AdcIn[1], Port_AdcInHyst[1]);
  Port_AdcInHyst[2] = Port_AdcFilt(Port_AdcIn[2], Port_AdcInHyst[2]);
  Port_AdcInHyst[3] = Port_AdcFilt(Port_AdcIn[3], Port_AdcInHyst[3]);
  Led_BrightnessLevel = Port_AdcInHyst[0];
  Led_AfterrunTime = Port_AdcInHyst[1];
  Led_PotiMode = Port_AdcInHyst[2];
  Led_AdcSensorBrightness = Port_AdcInHyst[3];

  /* calculate brightness information */
  Brightness_Sens1 = Led_Get_SensorBrightness1();
  Brightness_Sens2 = Led_Get_SensorBrightness2();

  
  /* Update the LED output driver */
  Led_UpdateDriver();
  /* Update LCD Display State*/
  faboPWM.set_channel_value(PWM_ADDRESS_DEFAULT,14, TFT_OnState);

  delay(10);
}

/**************************************************************************************************
Function: Port_AdcFilt()
Argument: Arg_AdcIn [0...4095][1] ; Actual Acd Value
          Arg_AdcHyst [0...4095][1] ; Acd Value with Hysteresis
return: void
**************************************************************************************************/
uint16_t Port_AdcFilt(uint16_t Arg_AdcIn, uint16_t Arg_AdcHyst)
{
  uint16_t Ret_AdcHyst, Loc_Adc;
  
  Loc_Adc = Arg_AdcIn;
  
  if(Loc_Adc >= Arg_AdcHyst)
  {
    Ret_AdcHyst = Loc_Adc;
  }
  else
  {
    if(Loc_Adc < (Arg_AdcHyst - ADC_HYSTERESE ))
    {
      Ret_AdcHyst = Loc_Adc + ADC_HYSTERESE;
    }
    else
    {
      Ret_AdcHyst = Arg_AdcHyst;
    }
  }
  
  return (Ret_AdcHyst);
}

/**************************************************************************************************
Function: Led_Init()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
return: void
**************************************************************************************************/
void Led_Init()
{
  uint16_t Loc_Address = PWM_ADDRESS_DEFAULT;
  uint16_t i;

  /* memset of struct */
  memset(&Led_Struct[0],0x00, LED_MAX * sizeof(&Led_Struct[0]));

  Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_OFF;
  Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_OFF;
  Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_OFF;
  Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_OFF;
  Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_OFF;
  Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_OFF;
  
  
  for (i = 0;i < LED_MAX; i++)
  {
   	Led_Struct[i].Led_AktValue = LED_0_PROZENT;

    /* update driver */
    faboPWM.set_channel_value(Loc_Address,i, Led_Struct[i].Led_AktValue);
    //Led_Struct[i].Led_AktValue = faboPWM.get_channel_value(Loc_Address, i);
  }
}


/**************************************************************************************************
Function: Led_UpdateDriver()
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_UpdateDriver(void)
{
  uint16_t Loc_Address = PWM_ADDRESS_DEFAULT;
  uint16_t i;
  
  for (i = 0;i < LED_MAX; i++)
  {
    /* update driver */
    faboPWM.set_channel_value(Loc_Address,i, Led_Struct[i].Led_AktValue);
    //Led_Struct[i].Led_AktValue = faboPWM.get_channel_value(Loc_Address, i);
  } 
}


/**************************************************************************************************
Function: Led_SetValue()
Argument: Arg_LedIndex [0...15][1] ; Index of the LED
Argument: Arg_Value [0...4095][1] ; value to set 
return: void
**************************************************************************************************/
void Led_SetValue(LED_INDEX Arg_LedIndex, uint16_t Arg_Value)
{
  if(Arg_LedIndex < LED_MAX)
  {
    Led_Struct[Arg_LedIndex].Led_AktValue = Arg_Value;
  }
}

/**************************************************************************************************
Function: Led_FadeIn()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
Argument: Arg_Step [0...4095][1] ; Step with to fade 
return: void
**************************************************************************************************/
uint16_t Led_FadeIn(LED_INDEX Arg_LedIndex, uint16_t Arg_Step)
{
  if(Arg_LedIndex < LED_MAX)
	{
		/* Fade In LED */
		if((Led_Struct[Arg_LedIndex].Led_AktValue + Arg_Step) < Led_Get_BrightnessLevel())
		{
			Led_Struct[Arg_LedIndex].Led_AktValue += Arg_Step;
		}
		else
		{
			Led_Struct[Arg_LedIndex].Led_AktValue = Led_Get_BrightnessLevel();
		}
	}
 return (Led_Struct[Arg_LedIndex].Led_AktValue);
}

/**************************************************************************************************
Function: Led_FadeOut()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
Argument: Arg_Step [0...4095][1] ; Step with to fade 
return: void
**************************************************************************************************/
uint16_t Led_FadeOut(LED_INDEX Arg_LedIndex, uint16_t Arg_Step)
{
  if(Arg_LedIndex < LED_MAX)
  {
    if(     ((Led_Struct[Arg_LedIndex].Led_AktValue - Arg_Step) < Led_Struct[Arg_LedIndex].Led_AktValue)
         && (Led_Struct[Arg_LedIndex].Led_AktValue > LED_0_PROZENT))
    {
      /* Fade Out LED */
      Led_Struct[Arg_LedIndex].Led_AktValue -= Arg_Step;
    }
    else
    {
      Led_Struct[Arg_LedIndex].Led_AktValue = LED_0_PROZENT;
    }
  }
  return Led_Struct[Arg_LedIndex].Led_AktValue;
}

/**************************************************************************************************
Function: Led_Get_BrightnessLevel()
Argument: -
Argument: - 
return: uint16_t [0...1023][1]; Level of potentiometer for brightness
**************************************************************************************************/
uint16_t Led_Get_BrightnessLevel(void)
{
  /* 10 Bit value */
  return (Led_BrightnessLevel);
}

/**************************************************************************************************
Function: Led_Get_BrightnessPercent()
Argument: -
Argument: - 
return: uint16_t [0...100][%]; Level of potentiometer for brightness
**************************************************************************************************/
uint16_t Led_Get_BrightnessPercent(void)
{
  uint16_t Loc_Brightness = Led_BrightnessLevel/10;
  if(Loc_Brightness > 100)
  {
    Loc_Brightness = 100;
  }
  /* value in % */
  return (Loc_Brightness);
}

/**************************************************************************************************
Function: Led_Get_AfterrunTime()
Argument: -
Argument: - 
return: uint16_t [0...10230][ms]; Level of potentiometer for time
**************************************************************************************************/
uint16_t Led_Get_AfterrunTime(void)
{
  /* 10 Bit value */
  return (Led_AfterrunTime * 10);
}

/**************************************************************************************************
Function: Led_Get_AfterrunTimeSec()
Argument: -
Argument: - 
return: uint16_t [0...1024][s]; Level of potentiometer for time
**************************************************************************************************/
uint16_t Led_Get_AfterrunTimeSec(void)
{
  uint16_t Loc_Afterrun = Led_AfterrunTime/10;
  if(Loc_Afterrun > 100)
  {
    Loc_Afterrun = 100;
  }
  /* Afterrun Time in Seconds */
  return (Loc_Afterrun);
}


/**************************************************************************************************
Function: Led_Get_Mode()
Argument: -
Argument: - 
return: uint16_t [0...5][]; mode of potentiometer for mode
**************************************************************************************************/
uint16_t Led_Get_Mode(void)
{
  uint16_t Loc_Mode = Led_PotiMode;
  if(Loc_Mode < 200)
  {
    Loc_Mode = 1;
  }
  else if(Loc_Mode < 400)
  {
    Loc_Mode = 2;
  }
  else if(Loc_Mode < 600)
  {
    Loc_Mode = 3;
  } 
  else if(Loc_Mode < 800)
  {
    Loc_Mode = 4;
  } 
  else
  {
    Loc_Mode = 5;
  } 
 
  /* Afterrun Time in Seconds */
  return (Loc_Mode);
}


/**************************************************************************************************
Function: Led_Get_SensorBrightness1()
Argument: -
Argument: - 
return: uint16_t [STATE_OFF...STATE_ON][1]; Sensor active brightness
**************************************************************************************************/
uint16_t Led_Get_SensorBrightness1(void)
{
  uint16_t Loc_Active;
  
  if(Led_AdcSensorBrightness > ADC_BRIGHTNESS_TRESHHOLD)
  {
    Loc_Active = STATE_OFF; 
  }
  else
  {
    Loc_Active = STATE_ON;
  }
  
  return(Loc_Active);
}

/**************************************************************************************************
Function: Led_Get_SensorBrightness2()
Argument: -
Argument: - 
return: uint16_t [STATE_OFF...STATE_ON][1]; Sensor active brightness
**************************************************************************************************/
uint16_t Led_Get_SensorBrightness2(void)
{
  uint16_t Loc_Active;

  /* Read digital Pin */
  digitalRead(PIN_3_FREE);
  
  if(digitalRead(PIN_3_FREE) != FALSE)
  {
    Loc_Active = STATE_OFF; 
  }
  else
  {
    Loc_Active = STATE_ON;
  }
  
  return(Loc_Active);
}

/**************************************************************************************************
Function: Pir_Init()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Pir_Init(void)
{
  uint16_t i;
   
  /* configure as normal pins */
  pinMode(PIR_PIN_UG_UG, INPUT); 
  pinMode(PIR_PIN_EG_UG, INPUT);
  pinMode(PIR_PIN_EG_OG, INPUT);
  pinMode(PIR_PIN_OG_OG, INPUT);

  /* memset of struct */
  memset(&Pir_Struct[0],0x00, PIR_TIMER_INDEX_MAX * sizeof(&Pir_Struct[0]));

  for (i=0; i < PIR_TIMER_INDEX_MAX; i++)
  {
    Pir_Struct[i].Pir_TimerValue = Led_Get_AfterrunTime();
    Pir_Struct[i].Pir_State_Akt = STATE_OFF;
    Pir_Struct[i].Pir_State_Old = STATE_OFF;
    Pir_Struct[i].Pir_ClusterState = STATE_OFF;
  }
}

/**************************************************************************************************
Function: ISR_Pir_UG_UG()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Pir_Read_UG_UG(void)
{
  if(digitalRead(PIR_PIN_UG_UG) == HIGH) 
  {
    if(Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Old == STATE_OFF)
    {
      Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Akt = STATE_ON;
      Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_TimerValue = Led_Get_AfterrunTime();
      Serial.println("PIR MOTION DETECTED UG UG");
      Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_ClusterState = STATE_ON;
    }
  }
  else
  {
    Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Akt = STATE_OFF;
  }

  /* save the state */
  Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Old = Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_State_Akt;
}


/**************************************************************************************************
Function: Pir_Read_OG_OG()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Pir_Read_OG_OG(void)
{
  if(digitalRead(PIR_PIN_OG_OG) == HIGH) 
  {
    if(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Old == STATE_OFF)
    {
      Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Akt = STATE_ON;
      Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_TimerValue = Led_Get_AfterrunTime();
      Serial.println("PIR MOTION DETECTED OG OG");
      Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState = STATE_ON;
    }
  }
  else
  {
    Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Akt = STATE_OFF;
  }

  /* save the state */
  Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Old = Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_State_Akt;
}


/**************************************************************************************************
Function: Pir_Read_EG_UG()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Pir_Read_EG_UG(void)
{
  if(digitalRead(PIR_PIN_EG_UG) == HIGH) 
  {
    if(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Old == STATE_OFF)
    {
      Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Akt = STATE_ON;
      Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_TimerValue = Led_Get_AfterrunTime();
      Serial.println("PIR MOTION DETECTED EG UG");
      Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState = STATE_ON;
    }
  }
  else
  {
    Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Akt = STATE_OFF;
  }

  /* save the state */
  Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Old = Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_State_Akt;
}

/**************************************************************************************************
Function: Pir_Read_EG_OG()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Pir_Read_EG_OG(void)
{
  if(digitalRead(PIR_PIN_EG_OG) == HIGH) 
  {
    if(Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Old == STATE_OFF)
    {
      Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Akt = STATE_ON;
      Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_TimerValue = Led_Get_AfterrunTime();
      Serial.println("PIR MOTION DETECTED EG OG");
      Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_ClusterState = STATE_ON;
    }
  }
  else
  {
    Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Akt = STATE_OFF;
  }

  /* save the state */
  Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Old = Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_State_Akt;
}



/**************************************************************************************************
Function: Interrupt_Init()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Interrupt_Init(void)
{
  /* Set the last channel from PWM modul as Interrupt source */
  faboPWM.set_channel_value(PWM_ADDRESS_DEFAULT,15, 2048);
  /* configure Pin as Interrupt Pin */
  pinMode(INTERRUPT_EXT1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_EXT1_PIN), ISR_Ext1, RISING);
}

/**************************************************************************************************
Function: Isr_500ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_500ms_Task0(void)
{
  //Serial.println("Isr_500ms_Task0...");
  
  /* Print Debug Info in serial Interface */
  PRINTF_DEBUG_SENSOR_INFO                /* print detailed info */
  //PRINTF_DEBUG_LED_CHANNEL_INFO           /* print detailed info */

  
}

/**************************************************************************************************
Function: Isr_200ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_200ms_Task0(void)
{
  //Serial.println("Isr_200ms_Task0...");

  /* Print Debug Info in serial Interface */
  //PRINTF_DEBUG_SENSOR_INFO                /* print detailed info */
  //PRINTF_DEBUG_LED_CHANNEL_INFO           /* print detailed info */
  

  /********************************************************************************************/
  /************************     HANDLING OF TFT Display       *******************************/
  /********************************************************************************************/

  /* Get Brightness Value in percent */
  String BrightnessValueString = String(Led_Get_BrightnessPercent());
  BrightnessValueString.toCharArray(TFT_BrightnessPrintOut, 4);
  
  /* Get Afterrun Value in seconds */
  String AfterrunValueString = String(Led_Get_AfterrunTimeSec());
  AfterrunValueString.toCharArray(TFT_AfterrunPrintOut, 4);

    /* Get Mode */
  String ModeValueString = String(Led_Get_Mode());
  ModeValueString.toCharArray(TFT_ModePrintOut, 4);

  /* Is the LCD Display currently on? the decrement the counter */
  if(TFT_OnCounter > 0)
    TFT_OnCounter--;

  /* clear the old brightness and Afterrun value if text value brightness or Afterrun is changed */
  if(    ((memcmp (TFT_BrightnessPrintOut, TFT_BrightnessPrintOut_Old, 4)) != 0)
      || ((memcmp (TFT_AfterrunPrintOut, TFT_AfterrunPrintOut_Old, 4)) != 0)
      || ((memcmp (TFT_ModePrintOut, TFT_ModePrintOut_Old, 4)) != 0))
  {
    /* clear the old values */
    TFTscreen.stroke(0,0,0);
    TFTscreen.setTextSize(2);
    TFTscreen.text(TFT_BrightnessPrintOut_Old, 0, 20);
    TFTscreen.text(TFT_AfterrunPrintOut_Old, 0, 60);
    TFTscreen.text(TFT_ModePrintOut_Old, 0, 85);

    /* Update the new values */
    TFTscreen.stroke(255,255,255);
    TFTscreen.text(TFT_BrightnessPrintOut, 0, 20);
    TFTscreen.text(TFT_AfterrunPrintOut, 0, 60);
    TFTscreen.text(TFT_ModePrintOut, 0, 85);
    /* Enable LCD Display */
    Serial.println("TFT DISPLAY ON...");
    TFT_OnState = 1024; /* DISPLAY ON */
    TFT_OnCounter = TFT_NACHLAUF_ZEIT;
    
  }

  /* Handling downstairs */
  if(    (Led_Downstairs == STATE_ON)
      && (Led_Downstairs_Old == STATE_OFF))
  {
    /* state downstairs changed to on */ 
    TFTscreen.fill(0,0,255);
    TFTscreen.rect(45,100, 22, 22);
  }
  if(    (Led_Downstairs == STATE_OFF)
      && (Led_Downstairs_Old == STATE_ON))
  {
    /* state downstairs changed to off */ 
    TFTscreen.fill(0,0,0);
    TFTscreen.rect(45,100, 22, 22);
  }


  /* Handling Upstairs */
  if(    (Led_Upstairs == STATE_ON)
      && (Led_Upstairs_Old == STATE_OFF))
  {
    /* state upstairs changed to on */ 
    TFTscreen.fill(0,0,255);
    TFTscreen.rect(125,100, 22, 22);
  }
  if(    (Led_Upstairs == STATE_OFF)
      && (Led_Upstairs_Old == STATE_ON))
  {
    /* state upstairs changed to off */ 
    TFTscreen.fill(0,0,0);
    TFTscreen.rect(125,100, 22, 22);
  }
  
  if(TFT_OnCounter == (TFT_NACHLAUF_ZEIT -1))
  {
    /* Setup the default Screen */
    TFTscreen.stroke(255,255,255);
    TFTscreen.setTextSize(2);
    TFTscreen.text("Helligkeit: ",0,0);
    TFTscreen.text("Nachlaufzeit: ",0,40);
    TFTscreen.text("% ",40,20);
    TFTscreen.text("sek. ",40,60);
    TFTscreen.text(TFT_BrightnessPrintOut, 0, 20);
    TFTscreen.text(TFT_AfterrunPrintOut, 0, 60);
    TFTscreen.text(TFT_ModePrintOut, 0, 85);
    TFTscreen.setTextSize(3);
    TFTscreen.text("UG",0,100);
    TFTscreen.text("OG",80,100);
    TFTscreen.fill(0,0,0);
    TFTscreen.rect(45,100, 22, 22);
    TFTscreen.rect(125,100, 22, 22);
    if(Led_Downstairs == STATE_ON)
    {
      TFTscreen.fill(0,0,255);
      TFTscreen.rect(45,100, 22, 22);
    }
    else
    {
      TFTscreen.fill(0,0,0);
      TFTscreen.rect(45,100, 22, 22);
    }
    if(Led_Upstairs == STATE_ON)
    {
      TFTscreen.fill(0,0,255);
      TFTscreen.rect(125,100, 22, 22); 
    }
    else
    {
      TFTscreen.fill(0,0,0);
      TFTscreen.rect(125,100, 22, 22);
    }
  }
  if(TFT_OnCounter == 1)
  {
    /* Disable LCD Display */
    TFT_OnState = FALSE;  /* DISPLAY ON */
    Serial.println("TFT DISPLAY OFF...");
  }



  /* Save the old values */
  memcpy(TFT_BrightnessPrintOut_Old, TFT_BrightnessPrintOut, 4);
  memcpy(TFT_AfterrunPrintOut_Old, TFT_AfterrunPrintOut, 4);
  memcpy(TFT_ModePrintOut_Old, TFT_ModePrintOut, 4);
  Led_Downstairs_Old = Led_Downstairs;
  Led_Upstairs_Old = Led_Upstairs;
}

/**************************************************************************************************
Function: Isr_10ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_10ms_Task0(void)
{
  uint16_t i;
  uint16_t value;
  
  //Serial.println("Isr_10ms_Task1...");


  /********************************************************************************************/
  /************************     HANDLING OF PIR DETECTION       *******************************/ 
  /********************************************************************************************/
  Pir_Read_UG_UG();
  Pir_Read_EG_UG();
  Pir_Read_EG_OG();
  Pir_Read_OG_OG();
  
  for (i=0; i<PIR_TIMER_INDEX_MAX; i++)
  {
    /* decrement the Timer value in case value is greater than zero */
    if(Pir_Struct[i].Pir_TimerValue > 0)
    {
      Pir_Struct[i].Pir_TimerValue--; 
    }
  }

  if(Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_TimerValue == 1)
  {
    Serial.println("PIR AFTERRUN OFF UG UG");
    Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_ClusterState = STATE_OFF;
  }
  if(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_TimerValue == 1)
  {
    Serial.println("PIR AFTERRUN OFF EG UG");
    Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState = STATE_OFF;
  }
  if(Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_TimerValue == 1)
  {
    Serial.println("PIR AFTERRUN OFF EG OG");
    Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_ClusterState = STATE_OFF;
  }
  if(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_TimerValue == 1)
  {
    Serial.println("PIR AFTERRUN OFF OG OG");
    Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState = STATE_OFF;
  }

  /* collect the states wich LEDS has to be set to On */
  /* ground -> first floor */
  if(    (Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState == STATE_ON)
      || (Pir_Struct[PIR_TIMER_INDEX_EG_OG].Pir_ClusterState == STATE_ON))
  {
    Led_Upstairs = STATE_ON;
  }
  else
  {
    Led_Upstairs = STATE_OFF;
  }
  /* collect the states wich LEDS has to be set to On */
  /* Basement -> ground floor */
  if(    (Pir_Struct[PIR_TIMER_INDEX_UG_UG].Pir_ClusterState == STATE_ON)
      || (Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState == STATE_ON))
  {
    Led_Downstairs = STATE_ON;
  }
  else
  {
    Led_Downstairs = STATE_OFF;
  }

  /********************************************************************************************/
  /************************      HANDLING OF LED SETTING        *******************************/ 
  /********************************************************************************************/
  switch (Led_Get_Mode())
  {
    case 1:
    {
      Led_Set_Mode_1_Statemachine();
      break;
    }
    case 2:
    {
      Led_Set_Mode_2_Statemachine();
      break;
    }
    case 3:
    {
      Led_Set_Mode_3_Statemachine();
      break;
    }
    case 4:
    {
      Led_Set_Mode_4_Statemachine(); 
      break;
    }
    case 5:
    {
      Led_Set_Mode_5_Statemachine();
      break;
    }
    default:
    {
      Led_Set_Mode_1_Statemachine();
    }
    break;
  }
}


/**************************************************************************************************
Function: ISR_Ext1()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void ISR_Ext1(void)
{
  Isr_Ext1_counter++;
  if(Isr_Ext1_counter % 10 == 0)
  {
    //Serial.println("10ms ");
    Isr_10ms_Task0(); 
  }
  if(Isr_Ext1_counter % 200 == 0)
  {
    //Serial.println("200ms ");
    Isr_200ms_Task0(); 
  }
  if(Isr_Ext1_counter %500 == 0)
  {
    //Serial.println("500ms ");
    Isr_500ms_Task0();
  }
}

/**************************************************************************************************
Function: Led_Set_Mode_1_Statemachine()
          - ALL LEDS On 
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_Set_Mode_1_Statemachine(void)
{
  uint16_t i;
  
  /****************/
  /* basement on? */
  /****************/
  if(Led_Downstairs == STATE_ON)
  { 
    for (i = LED_0;i < LED_4; i++)
    {
      Led_SetValue((LED_INDEX)i, Led_Get_BrightnessLevel());
    }
  }
  else
  {
    for (i = LED_0;i < LED_4; i++)
    {
      Led_SetValue((LED_INDEX)i, LED_0_PROZENT);
    }
  }
  /****************/
  /*  ground on?  */
  /****************/
  if(Led_Upstairs == STATE_ON)
  { 
    for (i = LED_5;i < LED_MAX; i++)
    {
      Led_SetValue((LED_INDEX)i, Led_Get_BrightnessLevel());
    }
  }
  else
  {
    for (i = LED_5;i < LED_MAX; i++)
    {
      Led_SetValue((LED_INDEX)i, LED_0_PROZENT);
    }
  }  
}

/**************************************************************************************************
Function: Led_Set_Mode_2_Statemachine()
          - Wave for LEDS
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_Set_Mode_2_Statemachine(void)
{
  uint16_t i;

  /****************/
  /* basement on? */
  /****************/
  switch (Led_Mode_2_State[LEDS_BASEMENT])
  {
    case LED_MODE_2_OFF:
      /* Check for change state */
      if(Led_Downstairs == STATE_ON)
      {
        Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_FADE_IN;
      }
      else
      {
        Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_OFF;
      }
    break;
    case LED_MODE_2_FADE_IN:
      for (i = LED_0;i < LED_4; i++)
      {
        Led_Struct[i].Led_AktValue = Led_FadeIn((LED_INDEX)i, (uint16_t)LED_MODE_2_FADING_STEPS);
      }
      if(Led_Struct[LED_3].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_ON;
      }
    break;
    case LED_MODE_2_ON:
      Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_FADE_OUT;
    break;
    case LED_MODE_2_FADE_OUT:
      for (i = LED_0;i < LED_4; i++)
      {
        Led_Struct[i].Led_AktValue = Led_FadeOut((LED_INDEX)i, (uint16_t)LED_MODE_2_FADING_STEPS);
      }
      if(Led_Struct[LED_3].Led_AktValue == LED_0_PROZENT)
      {
        Led_Mode_2_State[LEDS_BASEMENT] = LED_MODE_2_OFF;
      }
    break;
    default:
    break;
  }
  /****************/
  /*  ground on?  */
  /****************/
  switch (Led_Mode_2_State[LEDS_GROUNDFLOOR])
  {
    case LED_MODE_2_OFF:
      /* Check for change state */
      if(Led_Upstairs == STATE_ON)
      {
        Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_FADE_IN;
      }
      else
      {
        Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_OFF;
      }
    break;
    case LED_MODE_2_FADE_IN:
      for (i = LED_5;i < LED_MAX; i++)
      {
        Led_Struct[i].Led_AktValue = Led_FadeIn((LED_INDEX)i, (uint16_t)1);
      }
      if(Led_Struct[LED_9].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_ON;
      }
    break;
    case LED_MODE_2_ON:
      Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_FADE_OUT;
    break;
    case LED_MODE_2_FADE_OUT:
      for (i = LED_5;i < LED_MAX; i++)
      {
        Led_Struct[i].Led_AktValue = Led_FadeOut((LED_INDEX)i, (uint16_t)1);
      }
      if(Led_Struct[LED_9].Led_AktValue == LED_0_PROZENT)
      {
        Led_Mode_2_State[LEDS_GROUNDFLOOR] = LED_MODE_2_OFF;
      }
    break;
    default:
    break;
  }
}

/**************************************************************************************************
Function: Led_Set_Mode_3_Statemachine()
          - fade in and on LEDs
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_Set_Mode_3_Statemachine(void)
{
  uint16_t i;
  static LED_INDEX LocLedIndex_Basement[4] = {LED_0, LED_1, LED_2, LED_3};
  static LED_INDEX LocLedIndex_Groundfloor[5] = {LED_5, LED_6, LED_7, LED_8, LED_9};

  /****************/
  /* basement on? */
  /****************/
  switch (Led_Mode_3_State[LEDS_BASEMENT])
  {
    case LED_MODE_3_OFF:
      /* Check for change state */
      if(Led_Downstairs == STATE_ON)
      {
        Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
        /* change direction in case of which PIR is activated */
        if(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState == STATE_ON)
        {
          LocLedIndex_Basement[0] = LED_3;
          LocLedIndex_Basement[1] = LED_2;
          LocLedIndex_Basement[2] = LED_1;
          LocLedIndex_Basement[3] = LED_0;
        }
        else
        {
          LocLedIndex_Basement[0] = LED_0;
          LocLedIndex_Basement[1] = LED_1;
          LocLedIndex_Basement[2] = LED_2;
          LocLedIndex_Basement[3] = LED_3;
        }
        /* set state */
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_SEGMENT0_ON;
      }
      else
      {
        Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
        /* set state */
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_OFF;
      }
    break;
    case LED_MODE_3_SEGMENT0_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[0], (uint16_t)LED_MODE_3_FADING_STEPS);
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = Led_Struct[LocLedIndex_Basement[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = Led_Struct[LocLedIndex_Basement[2]].Led_AktValue; 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = Led_Struct[LocLedIndex_Basement[3]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Basement[0]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_SEGMENT1_ON;
      }
    break;
    case LED_MODE_3_SEGMENT1_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = Led_Struct[LocLedIndex_Basement[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[1], (uint16_t)LED_MODE_3_FADING_STEPS);
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = Led_Struct[LocLedIndex_Basement[2]].Led_AktValue; 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = Led_Struct[LocLedIndex_Basement[3]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Basement[1]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_SEGMENT2_ON;
      }
    break;
    case LED_MODE_3_SEGMENT2_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = Led_Struct[LocLedIndex_Basement[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = Led_Struct[LocLedIndex_Basement[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[2], (uint16_t)LED_MODE_3_FADING_STEPS); 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = Led_Struct[LocLedIndex_Basement[3]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Basement[2]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_SEGMENT3_ON;
      }
    break;
    case LED_MODE_3_SEGMENT3_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = Led_Struct[LocLedIndex_Basement[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = Led_Struct[LocLedIndex_Basement[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = Led_Struct[LocLedIndex_Basement[2]].Led_AktValue;
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[3], (uint16_t)LED_MODE_3_FADING_STEPS);
      if(Led_Struct[LocLedIndex_Basement[3]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_BASEMENT] = LED_MODE_3_OFF;
      }
    break;
    default:
    break;
  }
  /****************/
  /*  ground on?  */
  /****************/
  switch (Led_Mode_3_State[LEDS_GROUNDFLOOR])
  {
    case LED_MODE_3_OFF:
      /* Check for change state */
      if(Led_Upstairs == STATE_ON)
      {
        Led_Struct[LED_5].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_6].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_7].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LED_8].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_9].Led_AktValue = LED_0_PROZENT;
                /* change direction in case of which PIR is activated */
        if(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState == STATE_ON)
        {
          LocLedIndex_Groundfloor[0] = LED_9;
          LocLedIndex_Groundfloor[1] = LED_8;
          LocLedIndex_Groundfloor[2] = LED_7;
          LocLedIndex_Groundfloor[3] = LED_6;
          LocLedIndex_Groundfloor[4] = LED_5;
        }
        else
        {
          LocLedIndex_Groundfloor[0] = LED_5;
          LocLedIndex_Groundfloor[1] = LED_6;
          LocLedIndex_Groundfloor[2] = LED_7;
          LocLedIndex_Groundfloor[3] = LED_8;
          LocLedIndex_Groundfloor[4] = LED_9;
        }
        /* set state */
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_SEGMENT0_ON;
      }
      else
      {
        Led_Struct[LED_5].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_6].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_7].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LED_8].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_9].Led_AktValue = LED_0_PROZENT;
        /* set state */
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_OFF;
      }
    break;
    case LED_MODE_3_SEGMENT0_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[0], (uint16_t)LED_MODE_3_FADING_STEPS);
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue; 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_SEGMENT1_ON;
      }
    break;
    case LED_MODE_3_SEGMENT1_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[1], (uint16_t)LED_MODE_3_FADING_STEPS);
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue; 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_SEGMENT2_ON;
      }
    break;
    case LED_MODE_3_SEGMENT2_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[2], (uint16_t)LED_MODE_3_FADING_STEPS); 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_SEGMENT3_ON;
      }
    break;
    case LED_MODE_3_SEGMENT3_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[3], (uint16_t)LED_MODE_3_FADING_STEPS); 
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue;
      if(Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_SEGMENT4_ON;
      }
    break;
    case LED_MODE_3_SEGMENT4_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[4], (uint16_t)LED_MODE_3_FADING_STEPS);
      if(Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_3_State[LEDS_GROUNDFLOOR] = LED_MODE_3_OFF;
      }
    break;
    default:
    break;
  }
}

/**************************************************************************************************
Function: Led_Set_Mode_4_Statemachine()
          - Step by Step LEDs
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_Set_Mode_4_Statemachine(void)
{
  uint16_t i;
  static LED_INDEX LocLedIndex_Basement[4] = {LED_0, LED_1, LED_2, LED_3};
  static LED_INDEX LocLedIndex_Groundfloor[5] = {LED_5, LED_6, LED_7, LED_8, LED_9};


  /****************/
  /* basement on? */
  /****************/
  switch (Led_Mode_4_State[LEDS_BASEMENT])
  {
    case LED_MODE_4_OFF:
      /* Check for change state */
      if(Led_Downstairs == STATE_ON)
      {
        Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
        /* On Time */
        Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] = LED_MODE_4_ON_TIME;
        /* change direction in case of which PIR is activated */
        if(Pir_Struct[PIR_TIMER_INDEX_EG_UG].Pir_ClusterState == STATE_ON)
        {
          LocLedIndex_Basement[0] = LED_3;
          LocLedIndex_Basement[1] = LED_2;
          LocLedIndex_Basement[2] = LED_1;
          LocLedIndex_Basement[3] = LED_0;
        }
        else
        {
          LocLedIndex_Basement[0] = LED_0;
          LocLedIndex_Basement[1] = LED_1;
          LocLedIndex_Basement[2] = LED_2;
          LocLedIndex_Basement[3] = LED_3;
        }
        /* set state */
        Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_SEGMENT0_ON;
      }
      else
      {
        Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
        /* set state */
        Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_OFF;
      }
    break;
    case LED_MODE_4_SEGMENT0_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[0], (uint16_t)LED_MODE_4_FADING_STEPS);
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Basement[0]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_SEGMENT1_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT1_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[1], (uint16_t)LED_MODE_4_FADING_STEPS);
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT; 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Basement[1]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_SEGMENT2_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT2_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[2], (uint16_t)LED_MODE_4_FADING_STEPS); 
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Basement[2]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_SEGMENT3_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT3_ON:
      Led_Struct[LocLedIndex_Basement[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[2]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Basement[3]].Led_AktValue = Led_FadeIn(LocLedIndex_Basement[3], (uint16_t)LED_MODE_4_FADING_STEPS);
      if(Led_Struct[LocLedIndex_Basement[3]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_BASEMENT] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_BASEMENT] = LED_MODE_4_OFF;
        }
      }
    break;
    default:
    break;
  }
  /****************/
  /*  ground on?  */
  /****************/
  switch (Led_Mode_4_State[LEDS_GROUNDFLOOR])
  {
    case LED_MODE_4_OFF:
      /* Check for change state */
      if(Led_Upstairs == STATE_ON)
      {
        Led_Struct[LED_5].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_6].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_7].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LED_8].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_9].Led_AktValue = LED_0_PROZENT;
                /* change direction in case of which PIR is activated */
        if(Pir_Struct[PIR_TIMER_INDEX_OG_OG].Pir_ClusterState == STATE_ON)
        {
          LocLedIndex_Groundfloor[0] = LED_9;
          LocLedIndex_Groundfloor[1] = LED_8;
          LocLedIndex_Groundfloor[2] = LED_7;
          LocLedIndex_Groundfloor[3] = LED_6;
          LocLedIndex_Groundfloor[4] = LED_5;
        }
        else
        {
          LocLedIndex_Groundfloor[0] = LED_5;
          LocLedIndex_Groundfloor[1] = LED_6;
          LocLedIndex_Groundfloor[2] = LED_7;
          LocLedIndex_Groundfloor[3] = LED_8;
          LocLedIndex_Groundfloor[4] = LED_9;
        }
        /* set state */
        Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_SEGMENT0_ON;
      }
      else
      {
        Led_Struct[LED_5].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_6].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_7].Led_AktValue = LED_0_PROZENT; 
        Led_Struct[LED_8].Led_AktValue = LED_0_PROZENT;
        Led_Struct[LED_9].Led_AktValue = LED_0_PROZENT;
        /* set state */
        Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_OFF;
      }
    break;
    case LED_MODE_4_SEGMENT0_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[0], (uint16_t)LED_MODE_4_FADING_STEPS);
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = LED_0_PROZENT; 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_SEGMENT1_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT1_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[1], (uint16_t)LED_MODE_4_FADING_STEPS);
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = LED_0_PROZENT; 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_SEGMENT2_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT2_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[2], (uint16_t)LED_MODE_4_FADING_STEPS); 
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_SEGMENT3_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT3_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[3], (uint16_t)LED_MODE_4_FADING_STEPS); 
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = LED_0_PROZENT;
      if(Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_SEGMENT4_ON;
        }
      }
    break;
    case LED_MODE_4_SEGMENT4_ON:
      Led_Struct[LocLedIndex_Groundfloor[0]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[1]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[2]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[3]].Led_AktValue = LED_0_PROZENT;
      Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue = Led_FadeIn(LocLedIndex_Groundfloor[4], (uint16_t)LED_MODE_4_FADING_STEPS);
      if(Led_Struct[LocLedIndex_Groundfloor[4]].Led_AktValue >= Led_Get_BrightnessLevel())
      {
        Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR]--;
        /* Counter ready? */
        if(Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] == 0)
        {
          /* Reset Counter */
          Led_Mode_4_SegmentOn_Counter[LEDS_GROUNDFLOOR] = LED_MODE_4_ON_TIME;
          /* change state */
          Led_Mode_4_State[LEDS_GROUNDFLOOR] = LED_MODE_4_OFF;
        }
      }
    break;
    default:
    break;
  }
}

/**************************************************************************************************
Function: Led_Set_Mode_5_Statemachine()
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_Set_Mode_5_Statemachine(void)
{
  // Implemetation here

  /****************/
  /* basement on? */
  /****************/

  /****************/
  /*  ground on?  */
  /****************/
}
