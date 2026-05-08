#include <Arduino.h>
#include <SPI.h>

// ================= 硬件引脚定义 (ESP32-S3) =================
const int PIN_ADS_CS   = 10;
const int PIN_ADS_MOSI = 11;
const int PIN_ADS_MISO = 13;
const int PIN_ADS_SCK  = 12;
const int PIN_ADS_DRDY = 9;
const int PIN_ADS_RESET= 14; 

static const SPISettings ADS_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE1);
volatile bool dataReady = false;

// ================= 极简数据包定义 (18字节) =================
// 2字节包头 (0xAA, 0x55) + 4字节时间戳 + 3个4字节肌电数据
uint8_t packet[18] = {0xAA, 0x55, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

void IRAM_ATTR drdyInterrupt() {
    dataReady = true;
}

int32_t signExtend24(uint32_t data) {
    if (data & 0x800000) return (int32_t)(data | 0xFF000000);
    return (int32_t)data;
}

void adsWriteReg(uint8_t addr, uint8_t val) {
    SPI.beginTransaction(ADS_SPI_SETTINGS);
    digitalWrite(PIN_ADS_CS, LOW);
    SPI.transfer(0x40 | (addr & 0x1F)); 
    SPI.transfer(0x00); 
    SPI.transfer(val);
    digitalWrite(PIN_ADS_CS, HIGH);
    SPI.endTransaction();
    delay(2);
}

void init_ADS1298() {
    SPI.begin(PIN_ADS_SCK, PIN_ADS_MISO, PIN_ADS_MOSI, PIN_ADS_CS); 
    
    pinMode(PIN_ADS_CS, OUTPUT); digitalWrite(PIN_ADS_CS, HIGH);
    pinMode(PIN_ADS_RESET, OUTPUT); digitalWrite(PIN_ADS_RESET, HIGH);
    pinMode(PIN_ADS_DRDY, INPUT_PULLUP);
    
    // 硬件复位
    digitalWrite(PIN_ADS_RESET, LOW); delay(20);
    digitalWrite(PIN_ADS_RESET, HIGH); delay(150);
    
    // 停止连续读取模式，进入配置状态
    SPI.beginTransaction(ADS_SPI_SETTINGS);
    digitalWrite(PIN_ADS_CS, LOW); SPI.transfer(0x11); digitalWrite(PIN_ADS_CS, HIGH);
    SPI.endTransaction(); delay(10);
    
    adsWriteReg(0x03, 0xCC); // CONFIG3: 内部 2.4V 参考
    adsWriteReg(0x01, 0x84); // CONFIG1: 2000 SPS 高分辨率模式
    
    for(int i=0; i<5; i++) adsWriteReg(0x05 + i, 0x81); // 关闭 CH1-CH5
    adsWriteReg(0x0A, 0x60); // CH6: 正常电极, 增益 12
    adsWriteReg(0x0B, 0x60); // CH7: 正常电极, 增益 12
    adsWriteReg(0x0C, 0x60); // CH8: 正常电极, 增益 12

    adsWriteReg(0x0D, 0xE0); // RLD: 开启 CH6/7/8 右腿驱动
    adsWriteReg(0x0E, 0xE0);
    adsWriteReg(0x00, 0x00);
    
    // 启动转换并恢复连续读取
    SPI.beginTransaction(ADS_SPI_SETTINGS);
    digitalWrite(PIN_ADS_CS, LOW); SPI.transfer(0x08); digitalWrite(PIN_ADS_CS, HIGH);
    SPI.endTransaction(); delay(2);
    
    SPI.beginTransaction(ADS_SPI_SETTINGS);
    digitalWrite(PIN_ADS_CS, LOW); SPI.transfer(0x10); digitalWrite(PIN_ADS_CS, HIGH);
    SPI.endTransaction();

    attachInterrupt(digitalPinToInterrupt(PIN_ADS_DRDY), drdyInterrupt, FALLING);
}

void setup() {
    Serial.begin(115200); // 开启原生 USB CDC
    delay(1000); 
    init_ADS1298();
}

void loop() {
    if (dataReady) {
        dataReady = false;
        uint32_t ts = micros(); // 抓取时间戳

        // 极速 SPI 读取 27 字节
        SPI.beginTransaction(ADS_SPI_SETTINGS);
        digitalWrite(PIN_ADS_CS, LOW);
        uint8_t rx[27];
        SPI.transferBytes(NULL, rx, 27);
        digitalWrite(PIN_ADS_CS, HIGH);
        SPI.endTransaction();

        // 提取 CH6 (18,19,20), CH7 (21,22,23), CH8 (24,25,26)
        int32_t ch6 = signExtend24((uint32_t)rx[18]<<16 | (uint32_t)rx[19]<<8 | rx[20]);
        int32_t ch7 = signExtend24((uint32_t)rx[21]<<16 | (uint32_t)rx[22]<<8 | rx[23]);
        int32_t ch8 = signExtend24((uint32_t)rx[24]<<16 | (uint32_t)rx[25]<<8 | rx[26]);

        // 小端模式封包
        memcpy(&packet[2], &ts, 4);
        memcpy(&packet[6], &ch6, 4);
        memcpy(&packet[10], &ch7, 4);
        memcpy(&packet[14], &ch8, 4);

        // USB 原生极速喷射 (18字节)
        Serial.write(packet, 18);
    }
}