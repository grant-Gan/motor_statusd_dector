/*
  程序目的 : 实现传感器数据读取、超阈值监测、上传数据到云平台
  一、硬件材料
  1. 振动传感器 * 4 接在主板接口左侧四路接口上
  2. 温度传感器 * 2 接在主板接口右侧两路接口上
  3. 24v电源, 注意24V电源与USB接口不可同时供电
  4. SD卡
  二、预期效果
  1. 状态指示灯会先亮起, 大约七八秒以后熄灭, 之后每十秒闪烁一次
  2. 数据会被写入到SD卡
  3. 数据会被上传到OneNet平台
  4. 会从OneNet平台读取最新的阈值
  5. 当传感器的采集到的数据超过阈值以后, 会报警(红灯亮+蜂鸣器响)

  更新信息:
    更新日期: 2021/5/12/17:13
    更新内容:
      - 增加只有在电机运行时才会自动进行SD卡及oneNet端数据上传功能
      - 增加motor::getRunningStatus()方法

    更新日期: 2021/5/14/22:56
    更新内容:
      - 在数据写入和上传之前更新一次传感器数据
      - 将阈值一起写入CSV文件

    更新日期: 2021/5/18/18:46
    更新内容:
      - 修复超过阈值时写入表格数据错误问题   
*/

#include <Arduino.h>
#include <string>
#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <Ticker.h>

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

    //温度和振动烈度的报警阈值, 为方便访问, 使用 public
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
    bool runningStatus = false;
  public:
    //检查电机的运行状态, 运行中返回false, 否则返回true;
    bool getRunningStatus() {
      if ( (vibraCh[0] + vibraCh[1] + vibraCh[2] + vibraCh[3]) <= 0.01) {
        runningStatus = false;
        return false;
      }
      else {
        runningStatus = true;
        return true;
      }
    }
    //检查温度和振动的每一个通道, 如果有超过值, 则vibraExceedFlagChx设为true
    //并且方法返回true
    bool checkSenorExceed() {

      bool exceed = false;

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
    //获取温度传感器的值 存储在tempBuffer里面
    //参数: tempBuffer数组变量, 长度必须大于温度传感器通道数
    //    array_size数组长度
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
    //获取振动传感器的值 存储在vibraBuffer里面
    //参数: vibraBuffer数组变量, 长度必须大于温度传感器通道数
    //    array_size数组长度
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

    //更新振动传感器数据
    //
    bool updateVibra() {

      //定义二位数组用于存放ADC采集到的振动传感器电压
      uint16_t vibra_voltage[VIBRA_CH][VIBRA_SAMP_COUNT] = { 0 };
      //定义单通道数组, 用于存放振动传感器经过滤波后的值
      uint16_t vibra_vol_filter[VIBRA_CH] = { 0 };

      for (uint8_t i = 0; i < VIBRA_SAMP_COUNT; i++) {
        vibra_voltage[0][i] = analogRead(VIBRA_CH0_PIN);
        vibra_voltage[1][i] = analogRead(VIBRA_CH1_PIN);
        vibra_voltage[2][i] = analogRead(VIBRA_CH2_PIN);
        vibra_voltage[3][i] = analogRead(VIBRA_CH3_PIN);
        delay(1);
      }

      for (uint8_t i = 0; i < VIBRA_CH; i++) {

        //设置数据上下限, 这里的448 和2947 为仪表测量值, 不可变动
        limiterMinMax(vibra_voltage[i], VIBRA_SAMP_COUNT, 448, 2947);
        //对采样数据进行中位值滤波
        filterMedian(vibra_vol_filter[i], vibra_voltage[i], VIBRA_SAMP_COUNT);
        //通过公式将电压信号转换为振动信号, 结果存储在vibraCh[x]数组里面
        convertVibraFromVoltage(vibraCh[i], vibra_vol_filter[i]);
        //2.5V -> 2946.41875->2947
        //1v -> 1073.375 -> 1073
        //0.5V -> 448.2582 -> 448

      }
      return true;
    }
    //更新温度数据
    //成功返回 true; 失败返回false
    bool updateTemp() {
      float temp_ch0_buffer[TEMP_SAMP_COUNT] = { 0.0 };
      float temp_ch1_buffer[TEMP_SAMP_COUNT] = { 0.0 };

      uint8_t count = 0;    //采样次数
      unsigned long startTime = millis();
      while (count < TEMP_SAMP_COUNT) {
        //while循环超时检测
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
      //将temp_buffer里的数据经过平均值滤波以后传入到tempCh[0/1];
      filterAvarge(tempCh[0], temp_ch0_buffer, TEMP_SAMP_COUNT);
      filterAvarge(tempCh[1], temp_ch1_buffer, TEMP_SAMP_COUNT);
      return true;
    }

  private:
    //将振动传感器的电压信号转化为振动信号
    void convertVibraFromVoltage(float& target, const uint16_t& source) {
      //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      //将电压信号有效值进行线性缩放至0-20mm/s的范围, 这里的数值为测量值或给定值
      //不可以进行更改, 运算过程参考arduino库的map函数
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
      //返回中间值
      target = source[(arraySize - 1) / 2];
    }

};

//定义OneNet对象, 对上传数据和 下载数据进行操作
//依赖库为<arduinoJson>
class OneNet {

  protected:
    String apiKey;
    String deviceId;
    String apiUrl = "http://api.heclouds.com/devices/" + deviceId;

  public:
    OneNet(String devId, String aKey) :
      deviceId(devId),
      apiKey(aKey) {}
    //从服务器获取数据, 并将结果更新到Motor对象里
    //主要是用户设定的传感器报警阈值
    void getDataFromServer(Motor& motor) {
      if (WiFi.status() == WL_CONNECTED) {

        HTTPClient http;
        String postUrl = apiUrl + "/datastreams";
        String response;

        http.begin(postUrl);
        http.addHeader("api-key", apiKey);
        http.addHeader("Connection", "close");
        http.addHeader("Content-Type", "application/json");

        int httpResponseCode = http.GET();
        if (httpResponseCode > 0) {

          response = http.getString();

          Serial.println(httpResponseCode);
          //Serial.println(response);
        }
        else
          Serial.println("Error occurred while sending HTTP GET");

        StaticJsonDocument<2048> doc;

        ArduinoJson::deserializeJson(doc, response);
        JsonObject obj = doc.as<JsonObject>();

        uint16_t responeJsonCode = obj["errno"];

        if (responeJsonCode == 0) {
          //此处数字根据服务器传回的Json文件更改的数组顺序, 不可修改
          motor.vibraThresholdCh[3] = obj["data"][3]["current_value"];
          motor.vibraThresholdCh[2] = obj["data"][2]["current_value"];
          motor.vibraThresholdCh[0] = obj["data"][5]["current_value"];
          motor.vibraThresholdCh[1] = obj["data"][0]["current_value"];
          //Serial.println("Vibra threshold is : ");
          //Serial.print("\tch0 "); Serial.print(motor.vibraThresholdCh[0]);
          //Serial.print("\tch1 "); Serial.print(motor.vibraThresholdCh[1]);
          //Serial.print("\tch2 "); Serial.print(motor.vibraThresholdCh[2]);
          //Serial.println("\tch3 "); Serial.print(motor.vibraThresholdCh[3]);

          motor.tempThresholdCh[0] = obj["data"][1]["current_value"];
          motor.tempThresholdCh[1] = obj["data"][4]["current_value"];
          //Serial.println("Temp threshold is : ");
          //Serial.print("\tch0 "); Serial.print(motor.tempThresholdCh[0]);
          //Serial.println("\tch1 "); Serial.print(motor.tempThresholdCh[1]);
        }
        else
          Serial.println("Json get error!");

        http.end();

      }
    }
    //传递motor对象的数据到服务器端, 主要是温度和振动传感器的数字, 传递文件为Json文件
    void postDataToServer(Motor& motor) {

      if (WiFi.status() == WL_CONNECTED) {

        HTTPClient http;

        String jsonOutput;
        String postUrl = apiUrl + "/datapoints";

        float tempBuffer[TEMP_CH] = { 0.0 };
        float vibraBuffer[VIBRA_CH] = { 0.0 };

        motor.getTemp(tempBuffer, TEMP_CH);
        motor.getVibra(vibraBuffer, VIBRA_CH);

        //创建静态Json文档, 这里的384是通过软件计算得到的缓存大小, 不可以任意更改
        StaticJsonDocument<512> doc;

        JsonArray datastreams = doc.createNestedArray("datastreams");

        JsonObject datastreams_0 = datastreams.createNestedObject();
        datastreams_0["id"] = "motorTemp";

        JsonObject datastreams_0_datapoints_0_value = datastreams_0["datapoints"][0].createNestedObject("value");
        datastreams_0_datapoints_0_value["CH0"] = tempBuffer[0];
        datastreams_0_datapoints_0_value["CH1"] = tempBuffer[1];

        JsonObject datastreams_1 = datastreams.createNestedObject();
        datastreams_1["id"] = "motorVibra";

        JsonObject datastreams_1_datapoints_0_value = datastreams_1["datapoints"][0].createNestedObject("value");
        datastreams_1_datapoints_0_value["CH0"] = vibraBuffer[0];
        datastreams_1_datapoints_0_value["CH1"] = vibraBuffer[1];
        datastreams_1_datapoints_0_value["CH2"] = vibraBuffer[2];
        datastreams_1_datapoints_0_value["CH3"] = vibraBuffer[3];

        JsonObject datastreams_2 = datastreams.createNestedObject();
        datastreams_2["id"] = "ThresholdExceed";

        JsonObject datastreams_2_datapoints_0_value = datastreams_2["datapoints"][0].createNestedObject("value");
        datastreams_2_datapoints_0_value["TCH0"] = motor.tempExceedFlagCh[0];
        datastreams_2_datapoints_0_value["TCH1"] = motor.tempExceedFlagCh[1];
        datastreams_2_datapoints_0_value["VCH0"] = motor.vibraExceedFlagCh[0];
        datastreams_2_datapoints_0_value["VCH1"] = motor.vibraExceedFlagCh[1];
        datastreams_2_datapoints_0_value["VCH2"] = motor.vibraExceedFlagCh[2];
        datastreams_2_datapoints_0_value["VCH3"] = motor.vibraExceedFlagCh[3];
        //将Json文件字符串化并存储到jsonOutput中
        serializeJson(doc, jsonOutput);
        //String json =
        //  "{\"datastreams\": [{\"id\": \"test\",\"datapoints\": [{\"value\": 68}]}]}";

        http.begin(postUrl);
        http.addHeader("api-key", apiKey);
        http.addHeader("Connection", "close");
        http.addHeader("Content-Type", "application/json");

        int httpResponseCode = http.POST(jsonOutput);
        //httpResponseCode > 0则传递成功
        if (httpResponseCode > 0) {

          String response = http.getString();

          Serial.println(httpResponseCode);
          Serial.println(response);

        }
        else {

          Serial.println("Error occurred while sending HTTP POST");

        }

        http.end();   //关闭并清理http服务端
      }

    }


};

//定义SDCard对象, 用于存储数据到本地SD卡的操作
class SDCard {

  protected:
    uint8_t csPin;

  public:
    SDCard(uint8_t cs_pin)
      : csPin(cs_pin) {}

    //SD卡初始化
    bool begin() {
      if (!SD.begin(csPin)) {
        Serial.println("Card Mount Failed");
        return false;
      }
      uint8_t cardType = SD.cardType();

      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
      }
      //判断SD卡剩余容量, 如果容量>110MB这里会不让用(对于124Mb的内存卡而言)
      //如果以后更换内存更大的卡, 这里需要更改
      //但是目前来说128Mb的内存卡已经很够了
      if (SD.usedBytes() / (1024 * 1024) > 110) {
        Serial.println("Warning : Casd size so short! ");
        return false;
      }

    }
    //添加数据到特定的文件
    void appendFile(fs::FS& fs, const char* path, const char* message) {
      Serial.printf("Appending to file: %s\n", path);

      File file = fs.open(path, FILE_APPEND);
      if (!file) {
        Serial.println("Failed to open file for appending");
        return;
      }
      if (file.print(message)) {
        Serial.println("Message appended");
      }
      else {
        Serial.println("Append failed");
      }
      file.close();
    }
    //添加传感器数据到SD卡
    void appendSenorDataToSD(Motor& motor, struct tm& tmstruct, const char* path) {

      //更新最新的写入时间
      getLocalTime(&tmstruct);
      String str = "block";

      //Temp_ch0 :,13.20,Exceed :, no,Temp_ch1 :, 13.20,Exceed :,no,Vibra_ch0 :,13.20,Exceed :,no,Vibra_ch1 :,13.20,Exceed :,no,....
      char buffer[1024] = {};
      //对数据进行组包后存放在buffer里方便写入
      sprintf(buffer,
              "Temp_ch0 :, %.2f,Exceed :, %s,Temp_ch1 :, %.2f,Exceed :, %s,"\
              "Vibra_ch0 :, %.2f,Exceed :, %s,Vibra_ch1 :, %.2f,Exceed :, %s,Vibra_ch2 :, %.2f,Exceed :, %s,Vibra_ch3 :, %.2f,Exceed :, %s,"\
              // "Threshold value,%d,%d,%d,%d,%d,%d"
              "%d-%02d-%02d %02d:%02d:%02d\n",
              motor.getTempChx(motor.CH0), motor.tempExceedFlagCh[motor.CH0] ? "yes" : "no",
              motor.getTempChx(motor.CH1), motor.tempExceedFlagCh[motor.CH1] ? "yes" : "no",
              motor.getVibraChx(motor.CH0), motor.vibraExceedFlagCh[motor.CH0] ? "yes" : "no",
              motor.getVibraChx(motor.CH1), motor.vibraExceedFlagCh[motor.CH1] ? "yes" : "no",
              motor.getVibraChx(motor.CH2), motor.vibraExceedFlagCh[motor.CH2] ? "yes" : "no",
              motor.getVibraChx(motor.CH3), motor.vibraExceedFlagCh[motor.CH3] ? "yes" : "no",
              // motor.tempThresholdCh[motor.CH0], motor.tempThresholdCh[motor.CH1], 
              // motor.vibraThresholdCh[motor.CH0], motor.vibraThresholdCh[motor.CH1], motor.vibraThresholdCh[motor.CH2], motor.vibraThresholdCh[motor.CH3], 
              (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);

      appendFile(SD, path, buffer);
      //结果写入的只有最后一行的时间 : 2021-xx-xx-xx xx:xx:xx

    }

};

//定义警报器对象用于处理传感器数值超过阈值时的警报
class Warnner {

  private:

    bool warnnerStatus = false;

  public:
    //构造函数对对象进行初始化
    Warnner() {
      pinMode(WARNNER_LED_PIN, OUTPUT);
      pinMode(WARNNER_BUZZER_PIN, OUTPUT);

      digitalWrite(WARNNER_LED_PIN, LOW);
      digitalWrite(WARNNER_BUZZER_PIN, LOW);
    }

    bool getWannerStatus() {
      return warnnerStatus;
    }
    //设置警报器是是否工作
    //参数status值为 true(打卡) 或 false(关闭)
    void setWanner(bool status) {
      //异或判断设定值与当前值是否相等
      if (warnnerStatus ^ status) {
        digitalWrite(WARNNER_LED_PIN, status);
        digitalWrite(WARNNER_BUZZER_PIN, status);
        warnnerStatus = status;
      }

    }
};

Motor motor;
Warnner wanner;
SDCard sdCard = SDCard(SDCARD_CS_PIN);
//OneNet传参为用户id和API_KEY
OneNet onenet = OneNet(device_id, api_key);
//创建两个时间任务调度器, 方便实现任务的定时执行
Ticker timCounterPer_1s, timCounterPer_10s;

//创建tm结构体, 用于保存时间数据
struct tm tmstruct;
//定时器达到标志
uint8_t TimeFlag_1s = 0;
uint8_t TimeFlag_10s = 0;
//阈值溢位标志
bool globalExceedFlag = false;
bool localExceedFlag = false;
//按键中断函数, 如果警报开启就立刻关闭警报
void IRAM_ATTR keyInterprrput() {
  // keyCount++;
  wanner.setWanner(false);
}
//定时任务, 定时标志位自加
void handleTimer_1s() {
  TimeFlag_1s++;
}
void handleTimer_10s() {
  TimeFlag_10s++;
}

void setup() {
  //配置工作状态指示灯
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  //打开串口输出
  Serial.begin(115200);
  //wifi mode设置为STA
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(SSID, PWD);
  //等待wifi连接上
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //从服务器获取当前时间戳, 并且存入tm结构体
  configTime(3600 * 8, 0, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  getLocalTime(&tmstruct, 5000);
  //从oneNet服务器获取传感器阈值数据
  onenet.getDataFromServer(motor);
  sdCard.begin();
  //设置报警停止按键中断: 下降沿中断
  pinMode(WARNNER_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WARNNER_INTERRUPT_PIN), keyInterprrput, FALLING);
  //定时器配置
  //timCounterPer_1s.attachms(500, handleTimer_1s);
  timCounterPer_1s.attach(1, handleTimer_1s);
  timCounterPer_10s.attach(10, handleTimer_10s);
  //阻塞5S 等待所有配置完成
  delay(5000);
  //状态指示灯关闭, 程序开始工作
  digitalWrite(STATUS_LED, HIGH);
}

void loop() {
  //如果1S的定时任务到时间就执行相应的功能
  //这里的功能包括更新传感器数据和超阈值报警
  if (TimeFlag_1s > 0) {
    TimeFlag_1s = 0;
    motor.updateTemp();
    motor.updateVibra();
    //如果溢位标志为true则发送一次更新数据请求
    if (globalExceedFlag = motor.checkSenorExceed()) {
      wanner.setWanner(true);
      if(globalExceedFlag != localExceedFlag){
        localExceedFlag = globalExceedFlag;
         onenet.postDataToServer(motor);
      }
    }

  }
  //如果10S的定时任务到时间就执行相应的功能
  //这里的功能包括 :
  //1. 上传和下载系统数据到onenet
  //2. 存储数据到SD卡
  if (TimeFlag_10s > 0) {
    TimeFlag_10s = 0;
    //如果电机处于启动状态则继续运行
    if (motor.getRunningStatus()) {
      digitalWrite(STATUS_LED, LOW);
      //存储数据到SD卡
      sdCard.appendSenorDataToSD(motor, tmstruct, "/SenorData.CSV");
      //发送数据到onenet服务器
      onenet.postDataToServer(motor);
      //从onenet获取最新的阈值设定
      onenet.getDataFromServer(motor);
      digitalWrite(STATUS_LED, HIGH);
    }
  }

}
