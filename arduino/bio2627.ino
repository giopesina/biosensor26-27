#include <SPI.h>

// ============================================================
//  AD5941 Cyclic Voltammetry — Two-Electrode HS Loop
//  Hardware: test resistor / SPE validation
// ============================================================

#define CS_PIN    0
#define RESET_PIN 1

// ---------- Electrical / measurement constants ----------
#define RTIA_OHMS         10000.0f   // HSTIA feedback resistor (10k)

#define ADC_AVG_SAMPLES   32         // software-averaged samples per CV data point
#define BASELINE_SAMPLES  32         // software-averaged samples for baseline zeroing
#define ADC_SAMPLE_DELAY_MS   10     // delay between individual ADC samples
#define BASELINE_SAMPLE_DELAY_MS 30  // delay between baseline samples

// ---------- CV sweep constants ----------
#define CV_CODE_MID      2048
#define CV_CODE_START    1313
#define CV_CODE_VERTEX   3288
#define CV_CODE_END      CV_CODE_START
#define CV_STEP_CODES    50
#define CV_STEP_DELAY_MS 300
#define CV_CYCLES        5           // single cycle per run
#define CV_SETTLE_MS     500         // settling time after moving to CV_CODE_START, before sweep begins

// ---------- HSDAC constants ----------
#define HSDAC_VREF      1.82f
#define HSDAC_FULLSCALE 4095.0f

// ---------- Debug print throttling ----------
#define DEBUG_PRINT_EVERY_N_SAMPLES 50

extern "C" {
  #include "ad5940.h"
}

// ============================================================
//  AD5940 platform glue (SPI / GPIO / timing)
// ============================================================

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

extern "C" void AD5940_RstClr(void) { digitalWrite(RESET_PIN, LOW); }
extern "C" void AD5940_RstSet(void) { digitalWrite(RESET_PIN, HIGH); }

extern "C" void AD5940_Delay10us(uint32_t time)
{
  delayMicroseconds(time * 10);
}

extern "C" void AD5940_ReadWriteNBytes(
  unsigned char *pSendBuffer,
  unsigned char *pRecvBuff,
  unsigned long length)
{
  for (unsigned long i = 0; i < length; i++)
    pRecvBuff[i] = SPI.transfer(pSendBuffer[i]);
}

extern "C" void AD5940_MCUGpioWrite(uint32_t data) {}
extern "C" uint32_t AD5940_MCUGpioRead(uint32_t data) { return 0; }
extern "C" void AD5940_MCUGpioCtrl(uint32_t data, BoolFlag flag) {}
extern "C" uint32_t AD5940_GetMCUIntFlag(void) { return 0; }
extern "C" uint32_t AD5940_ClrMCUIntFlag(void) { return 0; }
extern "C" uint32_t AD5940_MCUResourceInit(void *pCfg) { return 0; }

// ============================================================
//  Globals
// ============================================================

float baseline_uA = 0.0f;

// ============================================================
//  Setup helpers
// ============================================================

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

// ============================================================
//  HS loop configuration
// ============================================================

void configureHSLoop(uint16_t dacCode)
{
  // ---- AFE reference & buffers ----
  AFERefCfg_Type ref_cfg;
  AD5940_StructInit(&ref_cfg, sizeof(ref_cfg));
  ref_cfg.HpBandgapEn = bTRUE;
  ref_cfg.Hp1V8BuffEn = bTRUE;
  ref_cfg.Hp1V1BuffEn = bTRUE;
  ref_cfg.HSDACRefEn  = bTRUE;
  AD5940_REFCfgS(&ref_cfg);

  // ---- Switch matrix ----
  // Two-electrode configuration:
  //
  //   CE0 ──── Cell ──── SE0
  //                        │
  //                       RE0  (tied externally to SE0)
  //
  // There is no dedicated reference electrode; the control amplifier
  // senses at SE0 (the return path of the cell / test resistor), and
  // RE0 is jumpered to SE0 on the board. CE0 drives excitation current
  // into the cell; DE0/SE0LOAD close the current path back through the
  // HSTIA input.
  SWMatrixCfg_Type sw;
  AD5940_StructInit(&sw, sizeof(sw));
  sw.Dswitch = SWD_CE0;
  sw.Pswitch = SWP_RE0;
  sw.Nswitch = SWN_SE0LOAD;
  sw.Tswitch = SWT_SE0LOAD | SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw);

  // ---- HSDAC (excitation source) ----
  HSDACCfg_Type dac_cfg;
  AD5940_StructInit(&dac_cfg, sizeof(dac_cfg));
  dac_cfg.ExcitBufGain    = EXCITBUFGAIN_2;   // x2 — matches dacCodeToVoltage() math
  dac_cfg.HsDacGain       = HSDACGAIN_1;
  dac_cfg.HsDacUpdateRate = 7;
  AD5940_HSDacCfgS(&dac_cfg);

  AD5940_WriteReg(REG_AFE_HSDACDAT, dacCode & 0xFFF);

  // ---- Waveform generator: direct MMR code injection ----
  WGCfg_Type wg_cfg;
  AD5940_StructInit(&wg_cfg, sizeof(wg_cfg));
  wg_cfg.WgType      = WGTYPE_MMR;
  wg_cfg.GainCalEn   = bFALSE;
  wg_cfg.OffsetCalEn = bFALSE;
  wg_cfg.WgCode      = dacCode & 0xFFF;
  AD5940_WGCfgS(&wg_cfg);

  // ---- HSTIA (current-to-voltage transimpedance amp) ----
  HSTIACfg_Type tia_cfg;
  AD5940_StructInit(&tia_cfg, sizeof(tia_cfg));
  tia_cfg.HstiaBias    = HSTIABIAS_1P1;
  tia_cfg.HstiaRtiaSel = HSTIARTIA_10K;   // must match RTIA_OHMS above
  tia_cfg.HstiaCtia    = 16;
  tia_cfg.HstiaDeRtia  = HSTIADERTIA_OPEN;
  tia_cfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  tia_cfg.DiodeClose   = bFALSE;
  AD5940_HSTIACfgS(&tia_cfg);

  // ---- ADC filter (hardware averaging/decimation) ----
  ADCFilterCfg_Type adc_filter;
  AD5940_StructInit(&adc_filter, sizeof(adc_filter));
  adc_filter.ADCSinc3Osr      = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr      = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum        = ADCAVGNUM_16;   // library max; do not increase
  adc_filter.ADCRate          = ADCRATE_800KHZ;
  adc_filter.BpNotch          = bTRUE;
  adc_filter.BpSinc3          = bFALSE;
  adc_filter.Sinc2NotchEnable = bTRUE;
  AD5940_ADCFilterCfgS(&adc_filter);

  // ---- ADC mux: measure HSTIA output directly ----
  ADCBaseCfg_Type adc_base;
  AD5940_StructInit(&adc_base, sizeof(adc_base));
  adc_base.ADCMuxP = ADCMUXP_HSTIA_P;
  adc_base.ADCMuxN = ADCMUXN_HSTIA_N;
  adc_base.ADCPga  = ADCPGA_1;
  AD5940_ADCBaseCfgS(&adc_base);

  // ---- Power up HS loop blocks ----
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);
  AD5940_AFECtrlS(
  AFECTRL_HPREFPWR  |
  AFECTRL_HSDACPWR  |
  AFECTRL_DACREFPWR |   // ADD — HSDAC reference buffer
  AFECTRL_EXTBUFPWR |
  AFECTRL_INAMPPWR  |
  AFECTRL_HSTIAPWR  |
  AFECTRL_ADCPWR    |
  AFECTRL_WG        |   // ADD — waveform generator engine
  AFECTRL_SINC2NOTCH,
  bTRUE);

  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_100KHZ);

  delay(20);
  AD5940_ADCConvtCtrlS(bTRUE);

  // Flush startup samples so the filter pipeline is settled before use
  for (int i = 0; i < 8; i++) {
    AD5940_ReadAfeResult(AFERESULT_SINC2);
    delay(30);
  }
}

// ============================================================
//  DAC / ADC helpers
// ============================================================

void setHSDACCode(uint16_t code)
{
    AD5940_WriteReg(REG_AFE_HSDACDAT, code & 0xFFF);

    uint16_t verify = AD5940_ReadReg(REG_AFE_HSDACDAT) & 0x0FFF;

    Serial.print("Requested=");
    Serial.print(code);
    Serial.print("  Register=");
    Serial.println(verify);
}
// Read a single ADC sample and convert to current (uA).
// Periodically prints a diagnostic line with ADC code, TIA voltage,
// computed current, and the live HSDAC register value.
float readCurrentRaw()
{
  static uint32_t sampleCount = 0;

  uint32_t raw  = AD5940_ReadAfeResult(AFERESULT_SINC2);
  uint16_t code = raw & 0xFFFF;
  float    vtia = AD5940_ADCCode2Volt(code, ADCPGA_1, HSDAC_VREF);
  float    i_uA = -(vtia / RTIA_OHMS) * 1e6f;

  if ((sampleCount++ % DEBUG_PRINT_EVERY_N_SAMPLES) == 0) {
    uint16_t dacCode = AD5940_ReadReg(REG_AFE_HSDACDAT) & 0xFFF;
    Serial.print("ADC=");   Serial.print(code);
    Serial.print(" VTIA="); Serial.print(vtia, 6);
    Serial.print(" I=");    Serial.print(i_uA, 4);
    Serial.print(" DAC=");  Serial.println(dacCode);
  }

  return i_uA;
}

// Take numSamples ADC readings, spaced sampleDelayMs apart, and return
// their average current (uA). Used for both baseline zeroing and CV
// data points so the averaging logic only lives in one place.
float sampleAverageCurrent(int numSamples, int sampleDelayMs)
{
  float sum = 0.0f;
  for (int s = 0; s < numSamples; s++) {
    sum += readCurrentRaw();
    delay(sampleDelayMs);
  }
  return sum / (float)numSamples;
}

// ============================================================
//  Baseline calibration
// ============================================================

void calibrateBaseline()
{
  Serial.println("Calibrating baseline...");
  baseline_uA = sampleAverageCurrent(BASELINE_SAMPLES, BASELINE_SAMPLE_DELAY_MS);
  Serial.print("Baseline = ");
  Serial.print(baseline_uA, 3);
  Serial.println(" uA");
}

// ============================================================
//  DAC code -> applied voltage (relative to midpoint)
// ============================================================

float dacCodeToVoltage(uint16_t code)
{
  float v_out = (code / HSDAC_FULLSCALE) * HSDAC_VREF;
  float v_mid = (CV_CODE_MID / HSDAC_FULLSCALE) * HSDAC_VREF;
  return (v_out - v_mid);  // x2 to match EXCITBUFGAIN_2
}

// ============================================================
//  CV sweep
// ============================================================

// Move the DAC to a code, let it settle, take an averaged current
// reading (baseline-subtracted), and emit one CSV line.
void runCVStep(int dacCode, const char *direction)
{
  setHSDACCode((uint16_t)dacCode);
  delay(CV_STEP_DELAY_MS);

  float i_uA = sampleAverageCurrent(ADC_AVG_SAMPLES, ADC_SAMPLE_DELAY_MS) - baseline_uA;
  float v    = dacCodeToVoltage((uint16_t)dacCode);

  Serial.print(v, 4);
  Serial.print(",");
  Serial.print(i_uA, 4);
  Serial.print(",");
  Serial.println(direction);
}

void runCV()
{
  Serial.println("=== CV START ===");
  Serial.println("V_applied(V),I(uA),direction");

  for (int cycle = 0; cycle < CV_CYCLES; cycle++) {

    // Forward sweep: START -> VERTEX
    for (int d = CV_CODE_START; d <= CV_CODE_VERTEX; d += CV_STEP_CODES) {
      runCVStep(d, "fwd");
    }

    // Reverse sweep: VERTEX -> START
    for (int d = CV_CODE_VERTEX; d >= CV_CODE_START; d -= CV_STEP_CODES) {
      runCVStep(d, "rev");
    }
  }

  Serial.println("=== CV END ===");
  Serial.print("Baseline used for this sweep = ");
  Serial.print(baseline_uA, 3);
  Serial.println(" uA");
}

// ============================================================
//  Debug register dump
// ============================================================

void printDebugRegs()
{
  Serial.println("\n-- Debug Registers --");
  Serial.print("AFECON       = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_AFECON),       HEX);
  Serial.print("HSDACCON     = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSDACCON),     HEX);
  Serial.print("HSDACDAT     = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSDACDAT),     HEX);
  Serial.print("HSRTIACON    = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSRTIACON),    HEX);
  Serial.print("HSTIACON     = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSTIACON),     HEX);
  Serial.print("SWCON        = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_SWCON),        HEX);
  Serial.print("ADCCON       = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_ADCCON),       HEX);
  Serial.print("ADCFILTERCON = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_ADCFILTERCON), HEX);
  Serial.print("DSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_DSWFULLCON),   HEX);
  Serial.print("PSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_PSWFULLCON),   HEX);
  Serial.print("NSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_NSWFULLCON),   HEX);
  Serial.print("TSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_TSWFULLCON),   HEX);
}


// ============================================================
//  Sweep sequence: settle -> baseline -> run CV
// ============================================================

void prepareAndRunCV()
{
  // Park at true zero (MID) before calibrating baseline — calibrating
  // while parked at CV_CODE_START would fold real current into the
  // baseline and corrupt every subsequent sweep point.
  setHSDACCode(CV_CODE_MID);
  delay(CV_SETTLE_MS);
  calibrateBaseline();

  // Now move to the actual sweep start point and let it settle
  setHSDACCode(CV_CODE_START);
  delay(CV_SETTLE_MS);

  runCV();
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("AD5941 CV - Two-Electrode HS Loop (test resistor)");

  pinMode(CS_PIN,    OUTPUT); digitalWrite(CS_PIN,    HIGH);
  pinMode(RESET_PIN, OUTPUT); digitalWrite(RESET_PIN, HIGH);

  SPI.begin();
  hardResetAD5941();
  AD5940_Initialize();
  AD5940_WakeUp(10);

  printIDs();

  configureHSLoop(CV_CODE_START);
  printDebugRegs();

  prepareAndRunCV();

  Serial.println("Done. Send 'r' to run again.");
}

void loop()
{
  while (Serial.available()) Serial.read();  // flush buffer
  Serial.println("Send 'r' to run CV...");
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'r') break;
    }
  }

  prepareAndRunCV();
  Serial.println("Done. Send 'r' to run again.");
}
