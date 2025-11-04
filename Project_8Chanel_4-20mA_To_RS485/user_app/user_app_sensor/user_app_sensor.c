#include "user_app_sensor.h"
#include "user_define.h"
#include "adc.h"
#include "user_convert_variable.h"
#include "user_app_modbus_rtu.h"

/*=========================Fucntion Static=========================*/
static uint8_t fevent_sensor_entry(uint8_t event);
static uint8_t fevent_sensor_get_adc(uint8_t event);
static uint8_t fevent_sensor_handle_adc(uint8_t event);
static uint8_t fevent_sensor_led_status(uint8_t event);
/*==============================Struct=============================*/
sEvent_struct               sEventAppSensor[]=
{
  {_EVENT_SENSOR_ENTRY,              1, 5, 20,                fevent_sensor_entry},            //Doi slave khoi dong moi truyen opera
  {_EVENT_SENSOR_GET_ADC,            0, 5, 20,                fevent_sensor_get_adc}, 
  {_EVENT_SENSOR_HANDLE_ADC,         0, 5, 20,                fevent_sensor_handle_adc},
  
  {_EVENT_SENSOR_LED_STATUS,         0, 5, 5,                 fevent_sensor_led_status},
};

struct_SensorTemp   sSensorTemp = {0};
struct_TempAlarm    sTempAlarm = {0};

uint32_t ADC_Temp[9]={0};
uint32_t ADC_Avg[9]={0};

struct_Measure_ADC  sMeasureADC = {0};

/*========================Function Handle========================*/
static uint8_t fevent_sensor_entry(uint8_t event)
{
//    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  
    static uint16_t once = 0;
    once++;
    HAL_ADC_Start_DMA(&ADC_8CHANEL, (uint32_t*)ADC_Temp, 9);
    
    if(once == NUM_SAMPLING_ADC)
    {
        fevent_active(sEventAppSensor, _EVENT_SENSOR_GET_ADC);
        return 1;
    }
    fevent_enable(sEventAppSensor, event);
    return 1; 
}

static uint8_t fevent_sensor_get_adc(uint8_t event)
{
/*----------------Get ADC-----------------*/
    static uint16_t count_adc = 0;
    uint8_t status = HAL_ERROR; 
    status = HAL_ADC_Start_DMA(&ADC_8CHANEL, (uint32_t*)ADC_Temp, 9);
    if( status == HAL_OK)
    {
        for(uint8_t i = 0; i<9; i++)
        {
            ADC_Avg[i] += ADC_Temp[i];
        }
    } 
    count_adc++;
    if(count_adc >= NUM_SAMPLING_ADC)
    {
        count_adc = 0;
        fevent_active(sEventAppSensor, _EVENT_SENSOR_HANDLE_ADC);
        return 1;
    }
  
    fevent_enable(sEventAppSensor, event);
    return 1; 
}

static uint8_t fevent_sensor_handle_adc(uint8_t event)
{
    sMeasureADC.adc_CH1 = ADC_Avg[0]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH2 = ADC_Avg[1]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH3 = ADC_Avg[2]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH4 = ADC_Avg[3]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH5 = ADC_Avg[4]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH6 = ADC_Avg[5]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH7 = ADC_Avg[6]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_CH8 = ADC_Avg[7]/NUM_SAMPLING_ADC;
    sMeasureADC.adc_Vref = ADC_Avg[8]/NUM_SAMPLING_ADC;
    
    for(uint8_t i = 0; i < 9; i++)
    {
        ADC_Avg[i] = 0;
    }
    
    sMeasureADC.Vol_Vref = ((VDDA_VREFINT_CAL * (*VREFINT_CAL))/(float)sMeasureADC.adc_Vref)/1000;
//    sMeasureADC.Vol_Vref = sMeasureADC.Vol_Vref - sMeasureADC.Vol_Vref*0.022;
    
    sMeasureADC.Vol_CH1 = ((sMeasureADC.adc_CH1 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH2 = ((sMeasureADC.adc_CH2 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH3 = ((sMeasureADC.adc_CH3 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH4 = ((sMeasureADC.adc_CH4 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH5 = ((sMeasureADC.adc_CH5 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH6 = ((sMeasureADC.adc_CH6 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH7 = ((sMeasureADC.adc_CH7 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    sMeasureADC.Vol_CH8 = ((sMeasureADC.adc_CH8 * sMeasureADC.Vol_Vref)/ADC_RESOLUTION );
    
    fevent_enable(sEventAppSensor, _EVENT_SENSOR_GET_ADC);
    return 1; 
}

static uint8_t fevent_sensor_led_status(uint8_t event)
{
/*-----------------Dieu khien led Status----------------*/
    if(sEventAppSensor[_EVENT_SENSOR_LED_STATUS].e_period == 1000)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        sEventAppSensor[_EVENT_SENSOR_LED_STATUS].e_period = 40;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        sEventAppSensor[_EVENT_SENSOR_LED_STATUS].e_period = 1000;
    }
    
    fevent_enable(sEventAppSensor, event);
    return 1;
}
/*=====================Filter Temperature====================*/
float Filter_pH(float var)
{
    //Kalman Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.000000001;  // Nhieu mo hinh
    float R = 0.000001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
//    int32_t varInt32 = 0;
    
    if(var != 0)
    {
//        varFloat = Handle_int32_To_Float_Scale(var, scale);
        varFloat = var;
        
        //Thay doi nhanh du lieu
        if(x_est_last - varFloat > 5 || varFloat - x_est_last > 5)
        {
           Q *=1000; 
        }
        
        // Buoc du doan
        x_pred = x_est;
        P_pred = P + Q;

        // Tinh he so kalman
        K = P_pred / (P_pred + R);

        // Cap nhat gia tri
        x_est = x_pred + K * (varFloat - x_pred);
        P = (1 - K) * P_pred;
        
//        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
//    return varInt32;
    return x_est;
}

float ConvertTemperature_Calib(float var)
{
    float result = 0;
    // Tinh he so a và b cua phuong trinh y = a*pH + b
    float a = 0;
    float b = 0;
    
    if((sSensorTemp.CalibPoint1_x_f != sSensorTemp.CalibPoint2_x_f) && (sSensorTemp.CalibPoint1_y_f != sSensorTemp.CalibPoint2_y_f))
    {
        a = (sSensorTemp.CalibPoint2_y_f  - sSensorTemp.CalibPoint1_y_f) / (sSensorTemp.CalibPoint2_x_f - sSensorTemp.CalibPoint1_x_f);
        b = sSensorTemp.CalibPoint1_y_f - a * sSensorTemp.CalibPoint1_x_f;
        result = a * var + b;
    }
    else
    {
        result = var;
    }
    
    result = result + sSensorTemp.Calib_Offset;
    
//    if(result < TEMPERATURE_LOWER)
//      result = TEMPERATURE_LOWER;
//    
//    if(result > TEMPERATURE_UPPER)
//      result = TEMPERATURE_UPPER;
    
    return result;
}
/*===================Save and Init Calib====================*/
void Save_CalibTemperature(uint8_t eKind, float var)
{
#ifdef USING_APP_SENSOR
//    uint8_t aData[50] = {0};
//    uint8_t length = 0;
//  
//    uint32_t hexUint_Compensation   = 0;
//    uint32_t hexUint_CalibPoint1_x  = 0;
//    uint32_t hexUint_CalibPoint1_y  = 0;
//    uint32_t hexUint_CalibPoint2_x  = 0;
//    uint32_t hexUint_CalibPoint2_y  = 0;
//    
//    switch(eKind)
//    {
//        case _KIND_CALIB_OFFSET:
//          sSensorTemp.Calib_Offset = var;
//          break;
//          
//        case _KIND_CALIB_POINT_1:
//          sSensorTemp.CalibPoint1_x_f = sSensorTemp.TempObject_f;
//          sSensorTemp.CalibPoint1_y_f = var;
//          break;
//          
//        case _KIND_CALIB_POINT_2:
//          sSensorTemp.CalibPoint2_x_f = sSensorTemp.TempObject_f;
//          sSensorTemp.CalibPoint2_y_f = var;
//          break;
//        
//        default:
//          break;
//    }
//    
//    hexUint_Compensation   = Handle_Float_To_hexUint32(sSensorTemp.Calib_Offset);
//    hexUint_CalibPoint1_x  = Handle_Float_To_hexUint32(sSensorTemp.CalibPoint1_x_f);
//    hexUint_CalibPoint1_y  = Handle_Float_To_hexUint32(sSensorTemp.CalibPoint1_y_f);
//    hexUint_CalibPoint2_x  = Handle_Float_To_hexUint32(sSensorTemp.CalibPoint2_x_f);
//    hexUint_CalibPoint2_y  = Handle_Float_To_hexUint32(sSensorTemp.CalibPoint2_y_f);
//    
//    aData[length++] = hexUint_Compensation >> 24;
//    aData[length++] = hexUint_Compensation >> 16;
//    aData[length++] = hexUint_Compensation >> 8;
//    aData[length++] = hexUint_Compensation ;
//    
//    aData[length++] = hexUint_CalibPoint1_x >> 24;
//    aData[length++] = hexUint_CalibPoint1_x >> 16;
//    aData[length++] = hexUint_CalibPoint1_x >> 8;
//    aData[length++] = hexUint_CalibPoint1_x ;
//    
//    aData[length++] = hexUint_CalibPoint1_y >> 24;
//    aData[length++] = hexUint_CalibPoint1_y >> 16;
//    aData[length++] = hexUint_CalibPoint1_y >> 8;
//    aData[length++] = hexUint_CalibPoint1_y ;
//    
//    aData[length++] = hexUint_CalibPoint2_x >> 24;
//    aData[length++] = hexUint_CalibPoint2_x >> 16;
//    aData[length++] = hexUint_CalibPoint2_x >> 8;
//    aData[length++] = hexUint_CalibPoint2_x ;
//    
//    aData[length++] = hexUint_CalibPoint2_y >> 24;
//    aData[length++] = hexUint_CalibPoint2_y >> 16;
//    aData[length++] = hexUint_CalibPoint2_y >> 8;
//    aData[length++] = hexUint_CalibPoint2_y ;
//
//    Save_Array(ADDR_CALIB_TEMPERATURE, aData, length);
#endif   
}

void Init_CalibTemperature(void)
{
#ifdef USING_APP_SENSOR
//    uint32_t hexUint_Compensation   = 0;
//    uint32_t hexUint_CalibPoint1_x  = 0;
//    uint32_t hexUint_CalibPoint1_y  = 0;
//    uint32_t hexUint_CalibPoint2_x  = 0;
//    uint32_t hexUint_CalibPoint2_y  = 0;
//  
//    if(*(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE) != FLASH_BYTE_EMPTY)
//    {
//        hexUint_Compensation  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+2) << 24;
//        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+3)<< 16;
//        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+4)<< 8;
//        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+5);
//        
//        hexUint_CalibPoint1_x  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+6) << 24;
//        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+7)<< 16;
//        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+8)<< 8;
//        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+9);
//        
//        hexUint_CalibPoint1_y  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+10) << 24;
//        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+11)<< 16;
//        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+12)<< 8;
//        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+13);
//        
//        hexUint_CalibPoint2_x  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+14) << 24;
//        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+15)<< 16;
//        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+16)<< 8;
//        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+17);
//        
//        hexUint_CalibPoint2_y  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+18) << 24;
//        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+19)<< 16;
//        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+20)<< 8;
//        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+21);
//        
//        Convert_uint32Hex_To_Float(hexUint_Compensation,  &sSensorTemp.Calib_Offset );
//        Convert_uint32Hex_To_Float(hexUint_CalibPoint1_x, &sSensorTemp.CalibPoint1_x_f);
//        Convert_uint32Hex_To_Float(hexUint_CalibPoint1_y, &sSensorTemp.CalibPoint1_y_f);
//        Convert_uint32Hex_To_Float(hexUint_CalibPoint2_x, &sSensorTemp.CalibPoint2_x_f);
//        Convert_uint32Hex_To_Float(hexUint_CalibPoint2_y, &sSensorTemp.CalibPoint2_y_f);
//    }
//    else
//    {
//        sSensorTemp.Calib_Offset = 0;
//        sSensorTemp.CalibPoint1_x_f = TEMPERATURE_LOWER;
//        sSensorTemp.CalibPoint1_y_f = TEMPERATURE_LOWER;
//        sSensorTemp.CalibPoint2_x_f = TEMPERATURE_UPPER;
//        sSensorTemp.CalibPoint2_y_f = TEMPERATURE_UPPER;
//    }
#endif   
}

void Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper)
{
#ifdef USING_APP_SENSOR
//    uint8_t aData[50] = {0};
//    uint8_t length = 0;
//  
//    uint32_t hexUint_AlarmUpper  = 0;
//    uint32_t hexUint_AlarmLower  = 0;
//    
//    sTempAlarm.State = State;
//    sTempAlarm.Alarm_Upper = AlarmUpper;
//    sTempAlarm.Alarm_Lower = AlarmLower;
//    
//    hexUint_AlarmUpper  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Upper);
//    hexUint_AlarmLower  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Lower);
//    
//    aData[length++] = sTempAlarm.State;
//    
//    aData[length++] = hexUint_AlarmUpper >> 24;
//    aData[length++] = hexUint_AlarmUpper >> 16;
//    aData[length++] = hexUint_AlarmUpper >> 8;
//    aData[length++] = hexUint_AlarmUpper ;
//    
//    aData[length++] = hexUint_AlarmLower >> 24;
//    aData[length++] = hexUint_AlarmLower >> 16;
//    aData[length++] = hexUint_AlarmLower >> 8;
//    aData[length++] = hexUint_AlarmLower ;
//
//    Save_Array(ADDR_TEMPERATURE_ALARM, aData, length);
#endif   
}

void Init_TempAlarm(void)
{
#ifdef USING_APP_SENSOR
  
//    uint32_t hexUint_AlarmUpper  = 0;
//    uint32_t hexUint_AlarmLower  = 0;
//  
//    if(*(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE) != FLASH_BYTE_EMPTY)
//    {
//        sTempAlarm.State  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+2) << 24;
//      
//        hexUint_AlarmUpper  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+3) << 24;
//        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+4)<< 16;
//        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+5)<< 8;
//        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+6);
//        
//        hexUint_AlarmLower  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+7) << 24;
//        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+8)<< 16;
//        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+9)<< 8;
//        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+10);
//        
//        Convert_uint32Hex_To_Float(hexUint_AlarmUpper, &sTempAlarm.Alarm_Upper);
//        Convert_uint32Hex_To_Float(hexUint_AlarmLower, &sTempAlarm.Alarm_Lower);
//    }
//    else
//    {
//        sTempAlarm.State = 0;
//        sTempAlarm.Alarm_Upper = TEMPERATURE_UPPER;
//        sTempAlarm.Alarm_Lower = TEMPERATURE_LOWER;
//    }
#endif   
}
/*==================Handle Define AT command=================*/
#ifdef USING_AT_CONFIG
void AT_CMD_Reset_Slave(sData *str, uint16_t Pos)
{
    uint8_t aTemp[60] = {0};   
    uint16_t length = 0;
    Save_InforSlaveModbusRTU(ID_DEFAULT, BAUDRATE_DEFAULT);
      
    Insert_String_To_String(aTemp, &length, (uint8_t*)"Reset OK!\r\n",0 , 11);
//	Modem_Respond(PortConfig, aTemp, length, 0);
    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Restore_Slave(sData *str, uint16_t Pos)
{
//    uint8_t aTemp[60] = {0};   
//    uint16_t length = 0;
//    OnchipFlashPageErase(ADDR_CALIB_TEMPERATURE);
//    sSensorTemp.Calib_Offset = 0;
//    sSensorTemp.CalibPoint1_x_f = TEMPERATURE_LOWER;
//    sSensorTemp.CalibPoint1_y_f = TEMPERATURE_LOWER;
//    sSensorTemp.CalibPoint2_x_f = TEMPERATURE_UPPER;
//    sSensorTemp.CalibPoint2_y_f = TEMPERATURE_UPPER;
//    
//    Insert_String_To_String(aTemp, &length, (uint8_t*)"Restore OK!\r\n",0 , 13);
////	Modem_Respond(PortConfig, aTemp, length, 0);
//    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Get_ID_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "ID Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.ID), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_ID_Slave (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 255 )
        {
            Save_InforSlaveModbusRTU(TempU32, sSlave_ModbusRTU.Baudrate);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_BR_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "BR Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.Baudrate), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_BR_Slave (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 11 )
        {
            Save_InforSlaveModbusRTU(sSlave_ModbusRTU.ID, TempU32);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_Measure (sData *str, uint16_t Pos)
{
    uint8_t aTemp[100] = "CH1:";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 5}; 
    int temp = 0;

    temp =(int)(sMeasureADC.Vol_CH1 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH2:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH2 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH3:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH3 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH4:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH4 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH5:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH5 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH6:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH6 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH7:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH7 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" CH8:",0 , 5);
    
    temp =(int)(sMeasureADC.Vol_CH8 * 1000);
    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, temp, 0xFD);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" END",0 , 4);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}
#endif
/*==================Handle Task and Init app=================*/
void       Init_AppSensor(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    Init_CalibTemperature();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_RESET_SLAVE].CallBack = AT_CMD_Reset_Slave;
    CheckList_AT_CONFIG[_RESTORE_SLAVE].CallBack = AT_CMD_Restore_Slave;
    
    CheckList_AT_CONFIG[_GET_ID_SLAVE].CallBack = AT_CMD_Get_ID_Slave;
    CheckList_AT_CONFIG[_SET_ID_SLAVE].CallBack = AT_CMD_Set_ID_Slave;
    CheckList_AT_CONFIG[_GET_BR_SLAVE].CallBack = AT_CMD_Get_BR_Slave;
    CheckList_AT_CONFIG[_SET_BR_SLAVE].CallBack = AT_CMD_Set_BR_Slave;
    
    CheckList_AT_CONFIG[_GET_MEASURE_ADC].CallBack = AT_CMD_Get_Measure;
#endif
}

uint8_t        AppSensor_Task(void)
{
    uint8_t i = 0;
    uint8_t Result =  false;
    
    for(i = 0; i < _EVENT_SENSOR_END; i++)
    {
        if(sEventAppSensor[i].e_status == 1)
        {
            Result = true; 
            
            if((sEventAppSensor[i].e_systick == 0) ||
               ((HAL_GetTick() - sEventAppSensor[i].e_systick) >= sEventAppSensor[i].e_period))
            {
                sEventAppSensor[i].e_status = 0; //Disable event
                sEventAppSensor[i].e_systick= HAL_GetTick();
                sEventAppSensor[i].e_function_handler(i);
            }
        }
    }
    
    return Result;
}
