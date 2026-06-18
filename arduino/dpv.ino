#include <SPI.h>

#define CS_PIN    0
#define RESET_PIN 1

extern "C" {
  #include "ad5940.h"
}

// ─────────────────────────────────────────────
//  DPV Parameters
// ─────────────────────────────────────────────
#define DPV_START_MV      -500.0f   // Start potential  (mV vs RE0)
#define DPV_END_MV         500.0f   // End potential    (mV vs RE0)
#define DPV_STEP_MV          5.0f   // Staircase step   (mV)
#define DPV_PULSE_AMP_MV    50.0f   // Pulse amplitude  (mV)
#define DPV_PULSE_WIDTH_MS  50      // Pulse ON time    (ms)
#define DPV_STEP_PERIOD_MS 200      // Total step time  (ms)

// RTIA – 10 kΩ internal gain resistor
#define RTIA_VAL  HSTIARTIA_10K
#define RTIA_OHMS 10000.0f

// VREF used internally by AD5940 LPDAC (2.5 V)
#define VREF_MV   2500.0f

// ─────────────────────────────────────────────
//  Platform glue
// ─────────────────────────────────────────────
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
extern "C" void AD5940_Delay10us(uint32_t time) { delayMicroseconds(time * 10); }
extern "C" void AD5940_ReadWriteNBytes(
  unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
  for (unsigned long i = 0; i < length; i++)
    pRecvBuff[i] = SPI.transfer(pSendBuffer[i]);
}
extern "C" void     AD5940_MCUGpioWrite(uint32_t data) {}
extern "C" uint32_t AD5940_MCUGpioRead(uint32_t data)  { return 0; }
extern "C" void     AD5940_MCUGpioCtrl(uint32_t data, BoolFlag flag) {}
extern "C" uint32_t AD5940_GetMCUIntFlag(void)  { return 0; }
extern "C" uint32_t AD5940_ClrMCUIntFlag(void)  { return 0; }
extern "C" uint32_t AD5940_MCUResourceInit(void *pCfg) { return 0; }

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────
void hardResetAD5941()
{
  AD5940_RstClr();
  delay(10);
  AD5940_RstSet();
  delay(100);
}

void printIDs()
{
  Serial.print("ADIID  = 0x"); Serial.println(AD5940_GetADIID(),  HEX);
  Serial.print("CHIPID = 0x"); Serial.println(AD5940_GetChipID(), HEX);
}

// ─────────────────────────────────────────────
//  Voltage → 12-bit LPDAC code
//  LPDAC VZERO/VBIAS 12-bit: 0 = 0 V, 4095 = Vref (2.5 V)
//  We bias the cell mid-rail (VZERO = 1.25 V = code 0x800)
//  and express the working potential relative to that.
// ─────────────────────────────────────────────
static uint16_t voltToLpdac12(float mv)
{
  // Clamp to 0 – 2500 mV
  if (mv < 0.0f)    mv = 0.0f;
  if (mv > 2500.0f) mv = 2500.0f;
  return (uint16_t)((mv / VREF_MV) * 4095.0f + 0.5f);
}

// ─────────────────────────────────────────────
//  Configure HS loop for 3-electrode potentiostat
//  Called once; DAC value is updated per step.
// ─────────────────────────────────────────────
void configureHSLoop()
{
  // ── High-Speed DAC ──────────────────────────
  HSDACCfg_Type hsdac;
  AD5940_StructInit(&hsdac, sizeof(hsdac));
  hsdac.ExcitBufGain   = EXCITBUFGAIN_2;   // 2× output buffer
  hsdac.HsDacGain      = HSDACGAIN_1;      // 1× DAC gain
  hsdac.HsDacUpdateRate = 7;               // update rate divider
  AD5940_HSDacCfgS(&hsdac);

  // ── HSTIA (transimpedance amplifier) ────────
  HSTIACfg_Type hstia;
  AD5940_StructInit(&hstia, sizeof(hstia));
  hstia.DiodeClose      = bFALSE;
  hstia.HstiaDeRload    = HSTIADERLOAD_OPEN;
  hstia.HstiaDeRtia     = HSTIADERTIA_OPEN;
  hstia.HstiaRtiaSel    = RTIA_VAL;        // 10 kΩ
  AD5940_HSTIACfgS(&hstia);

  // ── Switch matrix: 3-electrode potentiostat ─
  SWMatrixCfg_Type sw;
  AD5940_StructInit(&sw, sizeof(sw));
  // D-switch: CE0 driven by excitation amplifier output
  sw.Dswitch = SWD_CE0;
  // P-switch: connect excitation signal path
  sw.Pswitch = SWP_RE0 | SWP_CE0;
  // N-switch: SE0 → HSTIA negative input
  sw.Nswitch = SWN_SE0LOAD;
  // T-switch: SE0 → HSTIA, node tied to RTIA
  sw.Tswitch = SWT_SE0LOAD | SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw);

  // ── Waveform generator: DC only for DPV ─────
  // We drive DC via HSDAC; no AC sine needed.
  WGCfg_Type wg;
  AD5940_StructInit(&wg, sizeof(wg));
  wg.WgType     = WGTYPE_MMR;   // Direct MMR write (manual DC)
  wg.WgCode = 0x800;   // mid-scale start
  AD5940_WGCfgS(&wg);

  // ── ADC ─────────────────────────────────────
  ADCBaseCfg_Type adc_base;
  AD5940_StructInit(&adc_base, sizeof(adc_base));
  adc_base.ADCMuxP = ADCMUXP_HSTIA_P;   // HSTIA output (+)
  adc_base.ADCMuxN = ADCMUXN_HSTIA_N;   // HSTIA output (−)
  adc_base.ADCPga  = ADCPGA_1;
  AD5940_ADCBaseCfgS(&adc_base);

  ADCFilterCfg_Type adc_filter;
  AD5940_StructInit(&adc_filter, sizeof(adc_filter));
  adc_filter.ADCSinc3Osr       = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr       = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum         = ADCAVGNUM_2;
  adc_filter.ADCRate           = ADCRATE_800KHZ;
  adc_filter.BpNotch           = bTRUE;
  adc_filter.BpSinc3           = bFALSE;
  adc_filter.Sinc2NotchEnable  = bTRUE;
  AD5940_ADCFilterCfgS(&adc_filter);

  // ── Power up excitation chain ────────────────
  AD5940_AFECtrlS(
    AFECTRL_HPREFPWR  |
    AFECTRL_HSTIAPWR  |
    AFECTRL_INAMPPWR  |
    AFECTRL_EXTBUFPWR |
    AFECTRL_ADCPWR    |
    AFECTRL_SINC2NOTCH,
    bTRUE
  );

  delay(20);  // settle time

  // ── Enable excitation buffer + ADC conversion ─
  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCCNV, bTRUE);

  Serial.println("HS loop configured for 3-electrode potentiostat.");
}

// ─────────────────────────────────────────────
//  Set HSDAC output to a DC potential (mV)
//  Potential is expressed as offset from VZERO (1250 mV mid-rail).
//  cell_mv is the desired WE potential vs RE in mV.
// ─────────────────────────────────────────────
void setDACPotential_mV(float cell_mv)
{
  // Map cell_mv into DAC code centred at mid-rail (0x800 = 1.25 V)
  // DAC full-scale = 2.5 V → 4096 codes → 0.6104 mV/code
  float abs_mv = 1250.0f + cell_mv;           // absolute mV on DAC output
  int16_t code = (int16_t)((abs_mv / VREF_MV) * 4095.0f + 0.5f);
  if (code < 0)    code = 0;
  if (code > 4095) code = 4095;

  AD5940_WGDACCodeS((uint32_t)code);
}

// ─────────────────────────────────────────────
//  SIMULATION MODE
//  Set to 1 to output synthetic DPV data instead
//  of reading from the hardware ADC.
//  Set to 0 to use real hardware.
// ─────────────────────────────────────────────
#define SIMULATE 1

// ── Simulation parameters ────────────────────
// Mimics a single faradaic peak on a sloped
// capacitive background, typical of a real DPV
// voltammogram.
#define SIM_PEAK_MV       100.0f   // Peak centre potential (mV)
#define SIM_PEAK_WIDTH_MV   60.0f   // Gaussian sigma (mV) — sharper = smaller
#define SIM_PEAK_AMP_UA      8.5f   // Peak height (µA)
#define SIM_BG_SLOPE      0.002f    // Capacitive background slope (µA/mV)
#define SIM_NOISE_UA        0.05f   // ±noise amplitude (µA)

// Simple pseudo-random noise (no stdlib rand needed)
static uint32_t _lfsr = 0xACE1u;
static float simNoise()
{
  _lfsr ^= _lfsr << 13;
  _lfsr ^= _lfsr >> 17;
  _lfsr ^= _lfsr << 5;
  // Map to -1..+1 then scale
  float n = ((float)(_lfsr & 0xFFFF) / 32767.5f) - 1.0f;
  return n * SIM_NOISE_UA;
}

// Returns simulated ΔI (µA) at a given staircase potential
static float simulateDeltaI(float e_step_mv)
{
  // Gaussian faradaic peak
  float diff   = e_step_mv - SIM_PEAK_MV;
  float gauss  = SIM_PEAK_AMP_UA * expf(-(diff * diff) /
                   (2.0f * SIM_PEAK_WIDTH_MV * SIM_PEAK_WIDTH_MV));

  // Linear capacitive background
  float bg = SIM_BG_SLOPE * (e_step_mv - DPV_START_MV);

  return gauss + bg + simNoise();
}

// ─────────────────────────────────────────────
//  Single averaged ADC read → current in µA
//  Returns real hardware value when SIMULATE=0
// ─────────────────────────────────────────────
float readCurrentUA()
{
#if SIMULATE
  // Hardware calls skipped in simulation mode;
  // timing still observed via delay() in runDPV().
  return 0.0f;   // unused in sim path — see runDPV()
#else
  // Flush one reading to clear pipeline
  AD5940_ReadAfeResult(AFERESULT_SINC2);
  delay(10);

  uint32_t sinc2 = AD5940_ReadAfeResult(AFERESULT_SINC2);
  uint16_t code  = sinc2 & 0xFFFF;

  float vtia    = AD5940_ADCCode2Volt(code, ADCPGA_1, 1.82f);
  float current = (vtia / RTIA_OHMS) * 1e6f;   // µA
  return current;
#endif
}

// ─────────────────────────────────────────────
//  Run one full DPV scan
//  Prints: potential_mV, delta_current_uA
// ─────────────────────────────────────────────
void runDPV()
{
#if SIMULATE
  Serial.println("\n── DPV Scan Start (SIMULATED) ──");
#else
  Serial.println("\n── DPV Scan Start ──");
#endif
  Serial.println("Potential(mV), DeltaI(uA)");

  int numSteps = (int)((DPV_END_MV - DPV_START_MV) / DPV_STEP_MV) + 1;

  for (int s = 0; s < numSteps; s++)
  {
    float e_step = DPV_START_MV + s * DPV_STEP_MV;   // staircase potential

#if SIMULATE
    // Observe realistic timing without real hardware reads
    delay(DPV_STEP_PERIOD_MS);
    float delta_i = simulateDeltaI(e_step);

#else
    // ── Phase 1: baseline (pre-pulse) ───────────
    setDACPotential_mV(e_step);
    delay(DPV_STEP_PERIOD_MS - DPV_PULSE_WIDTH_MS);

    float i_baseline = readCurrentUA();

    // ── Phase 2: pulse ON ────────────────────────
    setDACPotential_mV(e_step + DPV_PULSE_AMP_MV);
    delay(DPV_PULSE_WIDTH_MS);

    float i_pulse = readCurrentUA();

    // ── Phase 3: pulse OFF (return to staircase) ─
    setDACPotential_mV(e_step);

    float delta_i = i_pulse - i_baseline;
#endif

    // Output CSV line: potential at staircase base, delta current
    Serial.print(e_step, 1);
    Serial.print(", ");
    Serial.println(delta_i, 4);
  }

  Serial.println("── DPV Scan Complete ──");
}

// ─────────────────────────────────────────────
//  Debug register dump
// ─────────────────────────────────────────────
void printDebugRegs()
{
  Serial.println("\n── Debug Registers ──");
  Serial.print("AFECON       = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_AFECON),       HEX);
  Serial.print("HSDACCON     = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSDACCON),     HEX);
  Serial.print("HSRTIACON    = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_HSRTIACON),    HEX);
  Serial.print("DSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_DSWFULLCON),   HEX);
  Serial.print("PSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_PSWFULLCON),   HEX);
  Serial.print("NSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_NSWFULLCON),   HEX);
  Serial.print("TSWFULLCON   = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_TSWFULLCON),   HEX);
  Serial.print("WGCON        = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_WGCON),        HEX);
  Serial.print("ADCCON       = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_ADCCON),       HEX);
  Serial.print("ADCFILTERCON = 0x"); Serial.println(AD5940_ReadReg(REG_AFE_ADCFILTERCON), HEX);
}

// ─────────────────────────────────────────────
//  Arduino entry points
// ─────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("AD5941 DPV — Seeeduino XIAO");

  pinMode(CS_PIN,    OUTPUT); digitalWrite(CS_PIN,    HIGH);
  pinMode(RESET_PIN, OUTPUT); digitalWrite(RESET_PIN, HIGH);

  SPI.begin();

#if !SIMULATE
  hardResetAD5941();
  AD5940_Initialize();
  AD5940_WakeUp(10);

  printIDs();
  configureHSLoop();
  printDebugRegs();

  Serial.println("\nEquilibrating at start potential (2 s)...");
  setDACPotential_mV(DPV_START_MV);
  delay(2000);
#else
  Serial.println("SIMULATE=1: skipping hardware init.");
#endif

  runDPV();
}

void loop()
{
  // Single scan in setup(); add button or serial trigger here
  // to repeat. Example:
  //
  if (Serial.available() && Serial.read() == 'r') runDPV();
}
