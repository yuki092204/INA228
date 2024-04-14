#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>
#include <SPI.h>

const uint32_t I2C_Freq = 4000000UL;  // I2C 周波数
const uint32_t I2C_Normal = 100000UL; // I2C 周波数

// GPIO I2C Setting
const uint8_t I2C_SCL = 21; // GPIO21
const uint8_t I2C_SDA = 20; // GPIO20

const uint8_t ADDRES_OLED = 0x3C;
const uint8_t ADDRES_INA228 = 0x40;

//	シャント抵抗値
const uint8_t ShuntR = 2; // 単位はmohm（ミリオーム）

//	INA228 レジスター値
//	コンフィグ設定値は16bit値らしいので2byteに揃える

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
const uint16_t INA228_CONFIG_ADCRANCGE = 0x01U;

// Reserved. Always reads 0.
const uint16_t INA228_CONFIG_RESERVED = 0x00U;

// -----------------------------------------------
// AVGBit Settings 平均値モードのサンプル数 D11-D9 << 9
// const uint16_t INA226_CONFIG_AVG = 0x0000U; // default 1@000  128@100 MAX:1024@111
// const uint16_t INA228_CONFIG_AVG = 0x0002U; // 16回計測の平均
// Bus VoltageConversionTime 電圧測定間隔 D8-D6  << 6
// const uint16_t INA226_CONFIG_VCT = 0x0004U; // default:1.1ms@100
// const uint16_t INA226_CONFIG_VCT = 0x0002U; // 16回計測するので332 μsに変更
// ShuntVoltageConversionTime シャント抵抗電圧測定間隔 D5-D3 << 3
// const uint16_t INA226_CONFIG_SVCT = 0x0004U; // default:1.1ms@100
// const uint16_t INA226_CONFIG_SVCT = 0x0002U; // 16回計測するので332 μsに変更
// ModeSettings 動作モード D2-D0 << 0
// const uint16_t INA226_CONFIG_MODE = 0x0007U; // default:Shuntand Bus,Continuous@111
// -----------------------------------------------

// ADC Configuration (ADC_CONFIG) Register (Address = 1h) [reset = FB68h]
const uint16_t INA228_ADC_CONFIG = 0x01;

const uint16_t INA228_ADC_CONFIG_MODE = 0x000FU;

const uint16_t INA228_ADC_CONFIG_VBUSCT = 0x0005U;

const uint16_t INA228_ADC_CONFIG_VSHCT = 0x0005U;

const uint16_t INA228_ADC_CONFIG_VTCT = 0x0005U;

const uint16_t INA228_ADC_CONFIG_AVG = 0x0002U;

const uint16_t INA228_SHUNT_CAL = 0x02;

const uint32_t INA228_VSHUNT = 0x04;

const uint8_t INA228_VBUS = 0x05;

const uint16_t INA228_DIETEMP = 0x06;

const uint32_t INA228_CURRENT = 0x07;

const uint32_t INA228_POWER = 0x08;

const uint16_t INA228_MANU_ID = 0x3E;

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

// 読み込み　2byte
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

// 読み込み　3byte
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
  ret = ret >> 4;
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
  // キャリブレーション書き込み
  //	5120 : VAOhm order ((0.0000025V x 2048) / 0.001A ) * 1000(to milli ohm at ShuntR unit order)
  // 　キャリブレーション書き込み
  //
  // int Max_Expected_Current = 1000; // 1A
  // float CURRENT_LSB = 0.0390625;   // 40.96mV / 2^19 / 0.002=0.0390625mA
  // SHUNT_CAL = 13107.2 * 10 ^ 6 * CURRENT_LSB * ShuntR * 4;
  // INA226_write(INA228_CALIB, (uint16_t)round(5120 / ShuntR)); // 5120 = 2.5 * 2048

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
  /*if (!display.begin(SSD1306_SWITCHCAPVCC, ADDRES_OLED))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();

  delay(5000);

  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("-- Serial monitor --");
  display.println(" display under here.");

  // レジスタ値の確認用
  display.print(INA228_read_2byte(INA228_CONFIG));
  display.print(" ");
  display.print(INA228_read_3byte(INA226_SHUNTV));
  display.print(" ");
  display.print(INA228_read_3byte(INA226_BUSV));
  display.print(" ");
  display.print(INA228_read_3byte(INA226_POWER));
  display.println("");

  display.print(INA228_read_3byte(INA228_CURRENT));
  display.print(" ");
  /*display.print(INA226_read(INA228_CALIB));
  display.print(" ");
  display.print(INA226_read(INA226_MASK));
  display.print(" ");
  display.print(INA226_read(INA226_ALERTL));
  display.println("");

  display.print(INA226_read(INA226_MANU_ID));
  display.print(" ");
  display.print(INA226_read(INA226_DIE_ID));
  display.print(" ");

  display.display();

  delay(5000);

  display.clearDisplay();
  display.display();

  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("-- Serial monitor --");
  display.println(" display under here.");

  display.display();*/
}

void loop()
{
  float busVoltage;
  float currentAmps;

  // shuntVoltage = INA228_read_3byte(INA228_CURRENT) * 0.15625; // 2.5 : ShuntLSB unit 2.5uV to cf
  currentAmps = INA228_read_3byte(INA228_CURRENT) * 0.0390625; // mA
  // shuntCurrentAmps = shuntVoltage / ShuntR;            // mA

  busVoltage = INA228_read_3byte(INA228_VBUS) * 0.1953125; // 1.25 : BusLSB unit 1.25mV to cf
  // currentAmps = INA226_read(INA226_CURRENT);    // mA

  // 下までいったら消してしまう
  /*if (display.getCursorY() >= 64)
  {
    display.setCursor(0, 16);
    display.writeFillRect(0, 15, 127, 63, BLACK);
    display.display();
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.println("");

  display.print("Shunt V: ");
  display.print(shuntVoltage); // シャント電圧
  display.println(" mV");

  display.print("Bus V: ");
  display.print(busVoltage); // バス電圧
  display.println(" mV");

  display.println("");

  display.print("ShuntLSBA: ");
  display.print(shuntCurrentAmps); // 計算値
  display.println(" mA");

  display.print("Current A: ");
  display.print(currentAmps); // INA228出力値
  display.println(" mA");

  display.display();*/
  Serial.print("BusVoltage: ");
  Serial.print(busVoltage);
  Serial.println(" mV");
  Serial.print("CurrentAmps: ");
  Serial.print(currentAmps);
  Serial.println(" mA");

  delay(333);
}