#ifndef USER_APP_SENSOR_H__
#define USER_APP_SENSOR_H__

#define USING_APP_SENSOR

#include "user_util.h"
#include "event_driven.h"

#define MB7040_I2C_ADDR  (0x70 << 1)  // 8-bit d?a ch? (HAL d?ng d?ng n?y)

#define MLX90614_ADDR   (0x5A << 1) // HAL dùng 8-bit address
#define MLX90614_TA     0x06
#define MLX90614_TOBJ1  0x07

//#define ALARM_ON     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//#define ALARM_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

#define ADC_8CHANEL     hadc1

#if defined (STM32L433xx) 
    #define VREFINT_CAL         ((uint16_t*) ((uint32_t)  0x1FFF75AA))  
#endif

#define VDDA_VREFINT_CAL        ((uint32_t) 3000)
#define ADC_RESOLUTION          4095

#define NUM_SAMPLING_ADC        100

typedef enum
{
    _EVENT_SENSOR_ENTRY,
    _EVENT_SENSOR_GET_ADC,
    _EVENT_SENSOR_HANDLE_ADC,
    _EVENT_SENSOR_LED_STATUS,

    _EVENT_SENSOR_END,
}eKindEventSENSOR;

typedef enum
{
    _KIND_CALIB_OFFSET,
    _KIND_CALIB_POINT_1,
    _KIND_CALIB_POINT_2,
}eKindCalibTemp;

typedef struct 
{
    float TempObject_f;
    float TempAmbient_f;
    float Calib_Offset;
    
    float CalibPoint1_x_f;
    float CalibPoint1_y_f;
    
    float CalibPoint2_x_f;
    float CalibPoint2_y_f;
}struct_SensorTemp;

typedef struct
{
    uint8_t State;
    float Alarm_Lower;
    float Alarm_Upper;
}struct_TempAlarm;

typedef struct
{
    uint16_t adc_CH1;
    uint16_t adc_CH2;
    uint16_t adc_CH3;
    uint16_t adc_CH4;
    uint16_t adc_CH5;
    uint16_t adc_CH6;
    uint16_t adc_CH7;
    uint16_t adc_CH8;
    uint16_t adc_Vref;
    
    float Vol_CH1;
    float Vol_CH2;
    float Vol_CH3;
    float Vol_CH4;
    float Vol_CH5;
    float Vol_CH6;
    float Vol_CH7;
    float Vol_CH8;
    float Vol_Vref;
    
}struct_Measure_ADC;

extern sEvent_struct        sEventAppSensor[];
extern struct_SensorTemp    sSensorTemp;
extern struct_TempAlarm     sTempAlarm;
extern struct_Measure_ADC   sMeasureADC;
/*====================Function Handle====================*/

uint8_t    AppSensor_Task(void);
void       Init_AppSensor(void);

void       Save_CalibTemperature(uint8_t eKind, float var);
void       Init_CalibTemperature(void);

void       Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper);
void       Init_TempAlarm(void);

float Filter_pH(float var);
float ConvertTemperature_Calib(float var);

#endif
