/*
程序目的 : 测试二通道温度传感器好坏
一、硬件材料
  1. 温度传感器 * 2 接在主板接口右侧两接口上
  2. 24v电源, 注意24V电源与USB接口不可同时供电
二、预期效果
  若程序及传感器工作正常, 则会在串口监视器打印信息为:
  Temperature value is :  CH0 xx  CH1 xx
  tempture
  其中xx的取值范围为 -250 -- 500(视温度传感器的量程而定), 保留两位小数  
*/


// the setup function runs once when you press reset or power the board
#include <Arduino.h>
#include <string>
#include <Adafruit_MAX31865.h>
#include <SPI.h>

//定义温度传感器通道数量
#define TEMP_CH 2		
//定义振动传感器通道数量
#define VIBRA_CH 4
//定义SD卡CS引脚
#define SDCARD_CS_PIN 21	
//定义max31865传感器CS引脚
#define TEMP_CH0_PIN 26		
#define TEMP_CH1_PIN 25
//定义振动传感器模拟输出引脚
#define VIBRA_CH0_PIN 34
#define VIBRA_CH1_PIN 35
#define VIBRA_CH2_PIN 32
#define VIBRA_CH3_PIN 33
//报警器引脚
#define WARNNER_LED_PIN 16
#define WARNNER_BUZZER_PIN 17
#define WARNNER_INTERRUPT_PIN 4
//状态指示灯引脚
#define STATUS_LED 2
//定义平均值滤波中温度的采样次数
//因考虑到max31865传输数字信号, 故该值可较小
#define TEMP_SAMP_COUNT 5

//定义振动信号滤波采样次数
//因该信号为模拟量传输, 故该值可较大
#define VIBRA_SAMP_COUNT 51

const char* SSID = "TEST";
const char* PWD = "20201234";

const String api_key = "ZH8lES5PNdvqTcO82bbYuQ6AHBU=";
const String device_id = "716081919";


class TempSensor {

protected:
    Adafruit_MAX31865 thermo;
    // Rref电阻阻值选择, PT100 -> 430 PT1000 -> 4300
    const float RREF = 430.0;
    // The 'nominal' 0-degrees-C resistance of the sensor
    // 100.0 for PT100, 1000.0 for PT1000
    const float RNOMINAL = 100.0;

public:

	TempSensor(int8_t spi_cs) : thermo(spi_cs) {
		//传递spi_cs值, 并且设置PT100为三线制
		thermo.begin(MAX31865_3WIRE);
	}

	float readData() {
		float temperture = thermo.temperature(RNOMINAL, RREF);
		//Serial.print("temperature = \t"); Serial.println(temperture);

        return temperture;
	}

    // 检查检测温度是否有错误
    // 错误返回true, 否则返回flase
    bool checkFault() {  
        uint8_t fault = thermo.readFault();

        if (fault) {
            Serial.print("Fault 0x"); Serial.println(fault, HEX);
            if (fault & MAX31865_FAULT_HIGHTHRESH) {
                Serial.println("RTD High Threshold");
            }
            if (fault & MAX31865_FAULT_LOWTHRESH) {
                Serial.println("RTD Low Threshold");
            }
            if (fault & MAX31865_FAULT_REFINLOW) {
                Serial.println("REFIN- > 0.85 x Bias");
            }
            if (fault & MAX31865_FAULT_REFINHIGH) {
                Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
            }
            if (fault & MAX31865_FAULT_RTDINLOW) {
                Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
            }
            if (fault & MAX31865_FAULT_OVUV) {
                Serial.println("Under/Over voltage");
            }
            thermo.clearFault();
            return true;
        } 
        else {
            return false;
        }
    }

};

class Motor {

public:
    enum SenorChEnum
    {
        CH0 = 0,
        CH1,
        CH2,
        CH3
    };

    //温度合振动烈度的报警阈值, 为方便访问, 使用public
    uint16_t tempThresholdCh[TEMP_CH] = { 100 };
    uint16_t vibraThresholdCh[VIBRA_CH] = { 10 };

    bool tempExceedFlagCh[TEMP_CH] = { false };
    bool vibraExceedFlagCh[VIBRA_CH] = { false };
protected:

    TempSensor pt100Ch[TEMP_CH] = {
        TempSensor(TEMP_CH0_PIN),
        TempSensor(TEMP_CH1_PIN)
    };
    float tempCh[TEMP_CH] = { 0.0 };		//电机温度两通道
    float vibraCh[VIBRA_CH] = { 0.0 };		//电机振动烈度四通道

public:
    bool checkSenorExceed() {

        bool exceed = false;
        //检查温度和振动的每一个通道, 如果有超过值, 则vibraExceedFlagChx设为true
        //并且方法返回true
        for (uint16_t i = 0; i < TEMP_CH; i++) {
            if (tempCh[i] >= tempThresholdCh[i]) {
                exceed = true;
                tempExceedFlagCh[i] = true;
            }
            else
                tempExceedFlagCh[i] = false;
        }
        for (uint16_t i = 0; i < VIBRA_CH; i++) {
            if (vibraCh[i] >= vibraThresholdCh[i]) {
                exceed = true;
                vibraExceedFlagCh[i] = true;
            }
            else
                vibraExceedFlagCh[i] = false;
        }
        return exceed;
    }

    //获取单通道温度值, 注意:该函数并不会更新数据, 只是单纯地返回上一次的读取值
    float getTempChx(SenorChEnum chx) {
        switch (chx)
        {
        case CH0 : return tempCh[CH0]; break;
        case CH1 : return tempCh[CH1]; break;
        default:
            Serial.println("Invalid channal ! ");
            break;
        }
    }

    //获取单通道振动值, 注意:该函数并不会更新数据, 只是单纯地返回上一次的读取值
    float getVibraChx(SenorChEnum chx) {
        switch (chx)
        {
        case CH0: return vibraCh[CH0]; break;
        case CH1: return vibraCh[CH1]; break;
        case CH2: return vibraCh[CH2]; break;
        case CH3: return vibraCh[CH3]; break;
        default:
            Serial.println("Invalid channal ! ");
            break;
        }
    }
    bool getTemp(float tempBuffer[], uint16_t array_size) {
        if (array_size < TEMP_CH) {
            Serial.print("temp buffer size so short");
            return false;
        }
        if (!updateTemp()) {
            Serial.print("get temperture error!");
            return false;
        }
        tempBuffer[0] = this->tempCh[0];
        tempBuffer[1] = this->tempCh[1];
        return true;
    }

    bool getVibra(float vibraBuffer[], uint16_t array_size) {
        if (array_size < VIBRA_CH) {
            Serial.print("vibra Buffer size so short");
            return false;
        }

        if (!updateVibra()) {
            Serial.print("get vibration error!");
            return false;
        }
        for (uint8_t i = 0; i < VIBRA_CH; i++)
            vibraBuffer[i] = vibraCh[i];

        return true;
    }

    //更新振动数据
    //
    bool updateVibra() {

        uint16_t vibra_voltage[VIBRA_CH][VIBRA_SAMP_COUNT] = { 0 };
        uint16_t vibra_vol_filter[VIBRA_CH] = { 0 };

        for (uint8_t i = 0; i < VIBRA_SAMP_COUNT; i++) {
            vibra_voltage[0][i] = analogRead(VIBRA_CH0_PIN);
            vibra_voltage[1][i] = analogRead(VIBRA_CH1_PIN);
            vibra_voltage[2][i] = analogRead(VIBRA_CH2_PIN);
            vibra_voltage[3][i] = analogRead(VIBRA_CH3_PIN);
            delay(1);
        }

        for (uint8_t i = 0; i < VIBRA_CH; i++) {
            limiterMinMax(vibra_voltage[i], VIBRA_SAMP_COUNT, 448, 2947);

            filterMedian(vibra_vol_filter[i], vibra_voltage[i], VIBRA_SAMP_COUNT);

            convertVibraFromVoltage(vibraCh[i], vibra_vol_filter[i]);
            //2.5V -> 2946.41875->2947
            //1v -> 1073.375 -> 1073
            //0.5V -> 448.2582 -> 448

        }
        return true;
    }
    //更新温度数据
    //
    bool updateTemp() {
        float temp_ch0_buffer[TEMP_SAMP_COUNT] = { 0.0 };
        float temp_ch1_buffer[TEMP_SAMP_COUNT] = { 0.0 };

        uint8_t count = 0;      //采样次数
        unsigned long startTime = millis();
        while (count < TEMP_SAMP_COUNT) {
            //超时检测
            if (millis() - startTime >= 1000) {
                Serial.print("get temperture timeout");
                return false;
            }
            //max31865最大采样间隔时间为21ms, 这里取采样间隔25ms
            if (millis() - startTime >= 25) {
                startTime = millis();
                temp_ch0_buffer[count] = pt100Ch[0].readData();
                temp_ch1_buffer[count] = pt100Ch[1].readData();

                if (pt100Ch[0].checkFault() || pt100Ch[1].checkFault())
                    continue;
                else
                    count += 1;
            }

        }
        //将temp_buffer里的数据经过滤波以后传入到tempCh[0/1];
        filterAvarge(tempCh[0], temp_ch0_buffer, TEMP_SAMP_COUNT);
        filterAvarge(tempCh[1], temp_ch1_buffer, TEMP_SAMP_COUNT);
        return true;
    }

private:
    //将振动传感器的电压信号转化为振动信号
    void convertVibraFromVoltage(float& target, const uint16_t& source) {
        //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        target = (source - 448.0) * (20.0 - 0.0) / (2947.0 - 448.0) + 0.0;

    }

    //平均值滤波, 针对温度
    void filterAvarge( float& target, const float source[], uint16_t array_size) {
        float filter_sum = 0;

        for ( uint8_t i = 0; i < array_size; i++) {
            filter_sum += source[i];
        }
        target = filter_sum / array_size;
    }

    //数值限幅器
    void limiterMinMax(uint16_t source[], uint16_t arraySize, uint16_t minVaule, uint16_t maxVaule) {

        for (uint16_t i = 0; i < arraySize; i++) {
            if (source[i] < minVaule)
                source[i] = minVaule;
            else if (source[i] > maxVaule)
                source[i] = maxVaule;
            else
                continue;
        }
    }
    
    //中位值滤波, 针对振动
    void filterMedian(uint16_t& target, uint16_t source[], uint16_t arraySize) {
        uint8_t i, j;
        uint16_t filterTemp;

        // 采样值从小到大排列（冒泡法）
        for (j = 0; j < arraySize - 1; j++) {
            for (i = 0; i < arraySize - 1 - j; i++) {
                if (source[i] > source[i + 1]) {
                    filterTemp = source[i];
                    source[i] = source[i + 1];
                    source[i + 1] = filterTemp;
                }
            }
        }
        target = source[(arraySize - 1) / 2];
    }

};

Motor motor;

void setup() {

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    Serial.begin(115200);

    //状态指示灯关闭
    digitalWrite(STATUS_LED, HIGH);
}

void loop() {
    //更新振动传感器状态
    motor.updateTemp();
    Serial.print("Temperature value is :");
    Serial.print("\tCH0 : ");Serial.print(motor.getTempChx(motor.CH0));
    Serial.print("\tCH1 : ");Serial.print(motor.getTempChx(motor.CH1));
    Serial.println();
    delay(2000);
} 
