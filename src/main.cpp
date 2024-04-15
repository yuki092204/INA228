#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>
#include <SPI.h>

const uint32_t I2C_Freq = 4000000UL;  // I2C 周波数
const uint32_t I2C_Normal = 100000UL; // I2C 周波数

// GPIO I2C Setting
const uint8_t I2C_SCL = 21; // GPIO21
const uint8_t I2C_SDA = 20; // GPIO20

// INA228 I2C Address　本体の設定によって変わる
const uint8_t ADDRES_INA228 = 0x40;

//	シャント抵抗値
const uint8_t ShuntR = 2; // 単位はmohm（ミリオーム）

//	以下　INA228 レジスター値の設定
//	コンフィグ設定値は16bit値らしいので2byteに揃える
// https://strawberry-linux.com/pub/ina228.pdf (INA228のデータシート）を参照

// Configuration (CONFIG) Register (Address = 0h) [reset = 0h]
const uint16_t INA228_CONFIG = 0x00U;

/*Reset Bit. Setting this bit to '1' generates a system reset that is the
same as power-on reset.
Resets all registers to default values.
0h = Normal Operation
1h = System Reset sets registers to default values
This bit self-clears.*/
const uint16_t INA228_CONFIG_RESET = 0x00U; // Reset

/*Resets the contents of accumulation registers ENERGY andCHARGE to 0
0h = Normal Operation
1h = Clears registers to default values for ENERGY and CHARGE registers*/
const uint16_t INA228_CONFIG_RSTACC = 0x00U;

/*  Sets the Delay for initial ADC conversion in steps of 2 ms.
0h = 0 s
1h = 2 ms
FFh = 510 ms*/
const uint16_t INA228_CONFIG_CONVDLY = 0x00U; // 0:140us 1:204us

/*Enables temperature compensation of an external shunt
0h = Shunt Temperature Compensation Disabled
1h = Shunt Temperature Compensation Enabled*/
const uint16_t INA228_CONFIG_TEMPCOMP = 0x00U;

// Shunt full scale range selection across IN+ and IN–.
// 0h = ±163.84 mV
// 1h = ± 40.96 mV
const uint16_t INA228_CONFIG_ADCRANCGE = 0x00U;

// Reserved. Always reads 0.
const uint16_t INA228_CONFIG_RESERVED = 0x00U;

// ADC Configuration (ADC_CONFIG) Register (Address = 1h) [reset = FB68h]
const uint16_t INA228_ADC_CONFIG = 0x01;

/*The user can set the MODE bits for continuous or triggered mode on
bus voltage, shunt voltage or temperature measurement.
0h = Shutdown
1h = Triggered bus voltage, single shot
2h = Triggered shunt voltage triggered, single shot
3h = Triggered shunt voltage and bus voltage, single shot
4h = Triggered temperature, single shot
5h = Triggered temperature and bus voltage, single shot
6h = Triggered temperature and shunt voltage, single shot
7h = Triggered bus voltage, shunt voltage and temperature, single
shot
8h = Shutdown
9h = Continuous bus voltage only
Ah = Continuous shunt voltage only
Bh = Continuous shunt and bus voltage
Ch = Continuous temperature only
Dh = Continuous bus voltage and temperature
Eh = Continuous temperature and shunt voltage
Fh = Continuous bus, shunt voltage and temperature*/
const uint16_t INA228_ADC_CONFIG_MODE = 0x000FU;

/*Sets the conversion time of the bus voltage measurement:
0h = 50 µs
1h = 84 µs
2h = 150 µs
3h = 280 µs
4h = 540 µs
5h = 1052 µs
6h = 2074 µs
7h = 4120 µs*/
const uint16_t INA228_ADC_CONFIG_VBUSCT = 0x0005U;

/*Sets the conversion time of the shunt voltage measurement:
0h = 50 µs
1h = 84 µs
2h = 150 µs
3h = 280 µs
4h = 540 µs
5h = 1052 µs
6h = 2074 µs
7h = 4120 µs*/
const uint16_t INA228_ADC_CONFIG_VSHCT = 0x0005U;

/*Sets the conversion time of the temperature measurement:
0h = 50 µs
1h = 84 µs
2h = 150 µs
3h = 280 µs
4h = 540 µs
5h = 1052 µs
6h = 2074 µs
7h = 4120 µs
*/
const uint16_t INA228_ADC_CONFIG_VTCT = 0x0005U;

/*Selects ADC sample averaging count. The averaging setting applies
to all active inputs.
When >0h, the output registers are updated after the averaging has
completed.
0h = 1
1h = 4
2h = 16
3h = 64
4h = 128
5h = 256
6h = 512
7h = 1024
*/
const uint16_t INA228_ADC_CONFIG_AVG = 0x0002U;

// Shunt Calibration (SHUNT_CAL) Register (Address = 2h) [reset = 1000h]
const uint16_t INA228_SHUNT_CAL = 0x02;

// Shunt Voltage Measurement (VSHUNT) Register (Address = 4h) [reset = 0h]
const uint32_t INA228_VSHUNT = 0x04;

// Bus Voltage Measurement (VBUS) Register (Address = 5h) [reset = 0h]
const uint8_t INA228_VBUS = 0x05;

// Temperature Measurement (DIETEMP) Register (Address = 6h) [reset = 0h]
const uint16_t INA228_DIETEMP = 0x06;

// Current Result (CURRENT) Register (Address = 7h) [reset = 0h]
const uint32_t INA228_CURRENT = 0x07;

// Power Result (POWER) Register (Address = 8h) [reset = 0h]
const uint32_t INA228_POWER = 0x08;

// Manufacturer ID (MANUFACTURER_ID) Register (Address = 3Eh) [reset = 5449h]
const uint16_t INA228_MANU_ID = 0x3E;

// Device ID (DEVICE_ID) Register (Address = 3Fh) [reset = 2280h]
const uint16_t INA228_DIE_ID = 0x3F;

// レジスタ書き込み関数
void INA228_write(uint8_t reg, uint16_t val)
{
  Wire.beginTransmission(ADDRES_INA228);
  Wire.write(reg);
  // I2Cは8bitづつ書き込みらしい
  Wire.write(val >> 8);     // 上位ビット送信
  Wire.write(val & 0x00ff); // ビットマスクして送信
  Wire.endTransmission();
}

// レジスタ読み込み関数　2byte ver
uint32_t INA228_read_2byte(uint8_t reg)
{
  uint32_t ret = 0;
  // リクエストするレジスタをコール
  Wire.beginTransmission(ADDRES_INA228);
  Wire.write(reg);
  Wire.endTransmission();
  // 2バイトリクエスト
  Wire.requestFrom((uint8_t)ADDRES_INA228, (uint8_t)2);
  // 2バイト取り込み
  while (Wire.available())
  {
    //	初回は0なので下位ビットに埋まる
    //	2回目は下位ビットを上位にずらして、新しく来たものを下位ビットに埋める
    ret = (ret << 8) | Wire.read();
  }
  return ret;
}

// レジスタ読み込み関数　3byte ver
uint32_t INA228_read_3byte(uint8_t reg)
{
  uint32_t ret = 0;
  // リクエストするレジスタをコール
  Wire.beginTransmission(ADDRES_INA228);
  Wire.write(reg);
  Wire.endTransmission();
  // 3バイトリクエスト
  Wire.requestFrom((uint8_t)ADDRES_INA228, (uint8_t)3);
  // 3バイト取り込み
  while (Wire.available())
  {
    //	初回は0なので下位ビットに埋まる
    //	2回目以降は下位ビットを上位にずらして、新しく来たものを下位ビットに埋める
    ret = (ret << 8) | Wire.read();
  }
  ret = ret >> 4; // 24bitのデータを20bitに変換(下位4bitは不要)
  return ret;
}

void setup()
{
  while (!Serial)
    ;

  //	Serial.begin(9600);
  Serial.begin(9600);
  // I2C開始
  Wire.begin();

  // 適度にディレイ
  delay(1000);

  // INA226 初期設定
  uint16_t config_ina = 0x0000U; // ベースビット 0B0000000000000000
  config_ina = config_ina | (INA228_CONFIG_RESET) << 15 | (INA228_CONFIG_RSTACC) << 14 | (INA228_CONFIG_CONVDLY) << 6 | (INA228_CONFIG_TEMPCOMP) << 5 | (INA228_CONFIG_ADCRANCGE) << 4 | (INA228_CONFIG_RESERVED);
  // config書き込み
  INA228_write(INA228_CONFIG, config_ina);
  // ADC_CONFIG書き込み
  uint16_t adc_config_ina = 0x0000U; // ベースビット 0B0000000000000000
  adc_config_ina = adc_config_ina | (INA228_ADC_CONFIG_MODE) << 12 | (INA228_ADC_CONFIG_VBUSCT) << 9 | (INA228_ADC_CONFIG_VSHCT) << 6 | (INA228_ADC_CONFIG_VTCT) << 3 | (INA228_ADC_CONFIG_AVG);
  INA228_write(INA228_ADC_CONFIG, adc_config_ina);
}

void loop()
{
  float busVoltage;
  float currentAmps;

  // 1LSB は 163.84mV / 2^19 / 0.002=0.15625mA となり読み値に 0.15625 をかけると mA の直読になります。
  currentAmps = INA228_read_3byte(INA228_CURRENT) * 0.15625; // mA
  // 。1LSB の 0.1953125 をかけると mV の直読になります
  busVoltage = INA228_read_3byte(INA228_VBUS) * 0.1953125; // mV

  Serial.print("BusVoltage: ");
  Serial.print(busVoltage);
  Serial.println(" mV");
  Serial.print("CurrentAmps: ");
  Serial.print(currentAmps);
  Serial.println(" mA");

  delay(333);
}