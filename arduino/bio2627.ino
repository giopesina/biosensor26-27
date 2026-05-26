#include <SPI.h>

#define CS_PIN    0
#define RESET_PIN 1

#define RTIA_OHMS 10000.0f


extern "C" {
  #include "ad5940.h"
}

// ---------- AD5940 platform glue ----------

extern "C" void AD5940_CsClr(void)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
}

extern "C" void AD5940_CsSet(void)
{
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

extern "C" void AD5940_RstClr(void)
{
  digitalWrite(RESET_PIN, LOW);
}

extern "C" void AD5940_RstSet(void)
{
  digitalWrite(RESET_PIN, HIGH);
}

extern "C" void AD5940_Delay10us(uint32_t time)
{
  delayMicroseconds(time * 10);
}

extern "C" void AD5940_ReadWriteNBytes(
  unsigned char *pSendBuffer,
  unsigned char *pRecvBuff,
  unsigned long length
)
{
  for (unsigned long i = 0; i < length; i++)
  {
    pRecvBuff[i] = SPI.transfer(pSendBuffer[i]);
  }
}

extern "C" void AD5940_MCUGpioWrite(uint32_t data) {}
extern "C" uint32_t AD5940_MCUGpioRead(uint32_t data) { return 0; }
extern "C" void AD5940_MCUGpioCtrl(uint32_t data, BoolFlag flag) {}
extern "C" uint32_t AD5940_GetMCUIntFlag(void) { return 0; }
extern "C" uint32_t AD5940_ClrMCUIntFlag(void) { return 0; }
extern "C" uint32_t AD5940_MCUResourceInit(void *pCfg) { return 0; }

// ---------- Setup helpers ----------

void hardResetAD5941()
{
  AD5940_RstClr();
  delay(10);
  AD5940_RstSet();
  delay(100);
}

void printIDs()
{
  Serial.print("ADIID  = 0x");
  Serial.println(AD5940_GetADIID(), HEX);

  Serial.print("CHIPID = 0x");
  Serial.println(AD5940_GetChipID(), HEX);
}

// ---------- Two-electrode resistor test ----------

void configureTwoLeadResistorTest(uint8_t dac6)
{
  LPLoopCfg_Type lp_loop;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;

  AD5940_StructInit(&lp_loop, sizeof(lp_loop));
  AD5940_StructInit(&adc_base, sizeof(adc_base));
  AD5940_StructInit(&adc_filter, sizeof(adc_filter));

  // Disable before reconfiguring
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);
  AD5940_ADCConvtCtrlS(bFALSE);
  delay(10);

  // ---- LPDAC ----
  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;

  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_12BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_6BIT;

  // Key change: route VZERO to TIA and VBIAS to LPAMP/PA
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA | LPDACSW_VBIAS2LPPA;

  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;

  // VZERO common-mode
  lp_loop.LpDacCfg.DacData12Bit = 0x800;

  // VBIAS test value. Try 10, 20, 32, 40, 50.
  lp_loop.LpDacCfg.DacData6Bit = dac6;

  // ---- LPTIA / PA ----
  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;

  lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_OPEN;
  lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_100R;
  lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_10K;

  // Key change: external two-lead sensor switch network
  lp_loop.LpAmpCfg.LpTiaSW = ENUM_AFE_LPTIASW0_TWOLEAD;

  AD5940_LPLoopCfgS(&lp_loop);

  // ---- ADC Filter ----
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;
  adc_filter.ADCRate = ADCRATE_800KHZ;
  adc_filter.BpNotch = bTRUE;
  adc_filter.BpSinc3 = bFALSE;
  adc_filter.Sinc2NotchEnable = bTRUE;

  AD5940_ADCFilterCfgS(&adc_filter);

  // Measure LPTIA output path
  adc_base.ADCMuxP = ADCMUXP_LPTIA0_P;
  adc_base.ADCMuxN = ADCMUXN_LPTIA0_N;
  adc_base.ADCPga = ADCPGA_1;

  AD5940_ADCBaseCfgS(&adc_base);

  AD5940_AFECtrlS(
    AFECTRL_HPREFPWR |
    AFECTRL_ADCPWR |
    AFECTRL_SINC2NOTCH,
    bTRUE
  );

  delay(20);
  AD5940_ADCConvtCtrlS(bTRUE);

  // Flush startup samples
  for (int i = 0; i < 8; i++)
  {
    AD5940_ReadAfeResult(AFERESULT_SINC2);
    delay(30);
  }

  Serial.print("Two-lead resistor test configured. DacData6Bit = ");
  Serial.println(dac6);
}

// ---------- Reading ----------

void readCurrent()
{
  uint32_t raw_adc = AD5940_ReadReg(REG_AFE_ADCDAT);
  uint32_t sinc2_u32 = AD5940_ReadAfeResult(AFERESULT_SINC2);
  uint16_t code = sinc2_u32 & 0xFFFF;

  float vtia = AD5940_ADCCode2Volt(code, ADCPGA_1, 1.82f);
  float current_uA = (vtia / RTIA_OHMS) * 1000000.0f;

  Serial.print("ADCDAT = 0x");
  Serial.print(raw_adc, HEX);

  Serial.print(" | SINC2 = ");
  Serial.print(code);

  Serial.print(" | Vtia = ");
  Serial.print(vtia, 6);
  Serial.print(" V");

  Serial.print(" | I = ");
  Serial.print(current_uA, 3);
  Serial.println(" uA");
}

void printDebugRegs()
{
  Serial.println("\n── Debug Registers ──");

  Serial.print("AFECON       = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_AFECON), HEX);

  Serial.print("LPTIACON0    = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_LPTIACON0), HEX);

  Serial.print("LPTIASW0     = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_LPTIASW0), HEX);

  Serial.print("LPDACCON0    = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_LPDACCON0), HEX);

  Serial.print("LPDACDAT0    = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_LPDACDAT0), HEX);

  Serial.print("ADCCON       = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_ADCCON), HEX);

  Serial.print("ADCFILTERCON = 0x");
  Serial.println(AD5940_ReadReg(REG_AFE_ADCFILTERCON), HEX);
}

// ---------- Arduino ----------

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("AD5941 Two-Lead Resistor Test - Arduino IDE / Seeeduino XIAO");

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  SPI.begin();

  hardResetAD5941();

  AD5940_Initialize();
  AD5940_WakeUp(10);

  printIDs();

  // Start with 40. Then test 20 and 50.
  configureTwoLeadResistorTest(40);

  printDebugRegs();

  Serial.println("\nStarting current read loop...");
}

void loop()
{
  readCurrent();
  delay(500);
}