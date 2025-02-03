#include "PCSCan.h"

bool isMux3b2Enabled = true;
bool isMux545Enabled = true;
bool isShort2B2 = false;
bool isBackup2c4Enabled = true;
bool GotDCI = false;
uint16_t pcsPowerRequest = 0;
uint16_t dcdcSetpoint = 0;
uint16_t dcdcCurrent = 0;
uint16_t acLimit = 0;
float acPower = 0;
uint16_t acVoltage = 0;
uint16_t acCurrent = 0;
uint16_t hvVoltage = 375;
float lvVoltage = 0;
float dcdcPower = 0;
int16_t localTemperature = 0;
uint16_t ambientTemperature = 0;
uint16_t temperatureA = 0;
uint16_t temperatureB = 0;
uint16_t temperatureC = 0;
uint16_t dcdcTemperature = 0;
uint16_t dcdcBusTemperature = 0;
uint8_t acCurrentLimit = 0;
uint8_t message545Count = 0;
float chargePowerAvailable = 0;
float outputCurrentPhaseA = 0;
float outputCurrentPhaseB = 0;
float outputCurrentPhaseC = 0;
float totalOutputCurrent = 0;
uint8_t pcsGridStatus = 0;
uint8_t pcsHardwareVersion = 0;
uint8_t pcsChargeStatus = 0;
uint8_t mux2C4 = 0;
uint8_t mux76C = 0;
uint8_t pcsBootId = 0;
uint8_t pcsAlertPage = 0;
uint8_t pcsAlertId = 0;
uint8_t pcsAlertSource = 0;
uint8_t pcsAlertCount = 0;
uint16_t alertCanId = 0;
uint8_t alertRxError = 0;
static uint8_t pcsAlertMatrix[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////PCS CAN Messages To Receive
///////////////////////////////////////////////////////////////////////////

void PCSCan::handle204(uint32_t data[2]) // PCS Chg status.
                                         // Power,Amps,PCS config,grid stat
{
  uint8_t *bytes = (uint8_t *)data; // byte 0 bits 4,5 = Chg state. 0=standby, 1=blocked, 2=enabled,3=faulted.
                                    // byte 0 bits 6,7 = grid config. 0=none, 1=single phase, 2=three phase, 3=three phase delta

  pcsHardwareVersion = (bytes[7] >> 3) & 0x03; // byte 7 bits 3,4 = HW type. 0=48A single phase , 1=32A single phase, 2=Three phase.
  Param::SetInt(Param::PCS_Type, pcsHardwareVersion);
  if (pcsHardwareVersion == 0)
    Param::SetInt(Param::hwaclim, 48);
  if (pcsHardwareVersion == 1)
    Param::SetInt(Param::hwaclim, 32);
  if (pcsHardwareVersion == 2)
    Param::SetInt(Param::hwaclim, 16);

  pcsChargeStatus = (bytes[0]) & 0x0f; // byte 0 bits 0-3 = charger main state. 0=init, 1=idle, 2=startup, 3=wait for line voltage
                                       // 4=qualify line config, 5=enable, 7=shutdown, 8=faulted, 9=clear faults
  Param::SetInt(Param::CHG_STAT, pcsChargeStatus);

  chargePowerAvailable = bytes[3] * 0.1f; // byte 2 = available charger power x 0.1 in kw.
  Param::SetFloat(Param::CHGPAvail, chargePowerAvailable);

  pcsGridStatus = (bytes[0] >> 6) & 0x3; // byte 0 bits 6,7 = grid config. 0=none, 1=single phase, 2=three phase, 3=three phase delta
  Param::SetInt(Param::GridCFG, pcsGridStatus);
}

void PCSCan::handle224(uint32_t data[2]) // DCDC Info

{
  uint8_t *bytes = (uint8_t *)data;

  dcdcCurrent = (((bytes[3] << 8 | bytes[2]) & 0xFFF) * 0.1); // dcdc actual current. 12 bit unsigned int in bits 16-27. scale 0.1.
  Param::SetFloat(Param::idcdc, dcdcCurrent);
  dcdcPower = dcdcCurrent * lvVoltage;
  Param::SetFloat(Param::powerdcdc, dcdcPower);
}

void PCSCan::handle264(uint32_t data[2]) // PCS Chg Line Status

{
  uint8_t *bytes = (uint8_t *)data; //

  acLimit = (((bytes[5] << 8 | bytes[4]) & 0x3ff) * 0.1);
  acPower = ((bytes[3]) * .1f);
  acVoltage = (((bytes[1] << 8 | bytes[0]) & 0x3FFF) * 0.033);
  acCurrent = (((bytes[2] << 9 | bytes[1]) >> 7) * 0.1);

  Param::SetFloat(Param::powerac, acPower);
  Param::SetFloat(Param::uac, acVoltage);
  Param::SetFloat(Param::iac, acCurrent);
  Param::SetFloat(Param::ChgACLim, acLimit);
}

void PCSCan::handle2A4(uint32_t data[2]) // PCS Temps

{
  uint8_t *bytes = (uint8_t *)data;
  // PCS_ambientTemp bits 44 to 54 x 0.1
  // PCS_chgPhtemperatureA bits 0-10 x0.1
  // PCS_chgPhtemperatureB bits 11-21 x0.1
  // PCS_chgPhtemperatureC bits 22-32 x0.1
  // DCDC Temp bits 33-43 x0.1

  temperatureA = ((bytes[1] << 8 | bytes[0]));
  temperatureB = ((bytes[2] << 8 | bytes[1]) >> 3);
  temperatureC = ((bytes[4] << 15 | bytes[3] << 7 | bytes[2] >> 1) >> 5);
  dcdcTemperature = ((bytes[5] << 8 | bytes[4]) >> 1);
  dcdcBusTemperature = (((bytes[7] << 8 | bytes[6]) >> 7) & 0x1FF) * 0.293542;
  ambientTemperature = ((bytes[6] << 8 | bytes[5]) >> 4);
  Param::SetFloat(Param::ChgATemp, ProcessTemps(temperatureA));
  Param::SetFloat(Param::ChgBTemp, ProcessTemps(temperatureB));
  Param::SetFloat(Param::ChgCTemp, ProcessTemps(temperatureC));
  Param::SetFloat(Param::DCDCTemp, ProcessTemps(dcdcTemperature));
  Param::SetFloat(Param::DCDCBTemp, dcdcBusTemperature);
  Param::SetFloat(Param::PCSAmbTemp, ProcessTemps(ambientTemperature));
}

void PCSCan::handle2C4(uint32_t data[2]) // PCS Logging

{
  uint8_t *bytes = (uint8_t *)data;
  mux2C4 = (bytes[0]);
  if ((mux2C4 == 0xE6) || (mux2C4 == 0xC6)) // if in mux 6 grab the info...
  {
    hvVoltage = (((bytes[3] << 8 | bytes[2]) & 0xFFF) * 0.146484);  // measured hv voltage. 12 bit unsigned int in bits 16-27. scale 0.146484.
    lvVoltage = ((((bytes[1] << 9 | bytes[0]) >> 6)) * 0.0390625f); // measured lv voltage. 10 bit unsigned int in bits 5-14. scale 0.0390626.
    isBackup2c4Enabled = false;
  }
  else if ((mux2C4 == 0x04) && (isBackup2c4Enabled)) // if we dont get HV volts then switch to backup.
  {
    hvVoltage = ((((bytes[7] << 8 | bytes[6]) >> 3) & 0xFFF) * 0.146484); // measured hv voltage. 12 bit unsigned int in bits 51-62. scale 0.146484.
    lvVoltage = Param::GetFloat(Param::uaux);                             // In new firmware DCDC LV is not in 0x2C4.
  }
  Param::SetFloat(Param::udc, hvVoltage);
  Param::SetFloat(Param::ulv, lvVoltage);

  mux2C4 = (bytes[0] & 0x1F);
  if (mux2C4 == 0x00) // Calculate total DC output current from all 3 charger modules.
  {
    outputCurrentPhaseA = ((bytes[4])) * 0.1f;
    GotDCI = true;
  }
  else if (mux2C4 == 0x01)
  {
    outputCurrentPhaseB = ((bytes[4])) * 0.1f;
    GotDCI = true;
  }
  else if (mux2C4 == 0x02)
  {
    outputCurrentPhaseC = ((bytes[4])) * 0.1f;
    GotDCI = true;
  }

  totalOutputCurrent = outputCurrentPhaseA + outputCurrentPhaseB + outputCurrentPhaseC;
  Param::SetFloat(Param::idc, totalOutputCurrent);
}

void PCSCan::handle3A4(uint32_t data[2]) // PCS Alert Matrix

{ // 0=None, 1=CP_MIA, 2=BMS_MIA, 3=HVP_MIA, 4=UNEX_AC, 5=CHG_VRAT, 6=DCDC_VRAT, 7=VCF_MIA, 8=CAN_RAT, 9=UI_MIA
  uint8_t *bytes = (uint8_t *)data;
  pcsAlertPage = bytes[0] & 0xf;
}

void PCSCan::handle424(uint32_t data[2]) // PCS Alert Log

{
  uint8_t *bytes = (uint8_t *)data;
  pcsAlertId = bytes[0];
  if (Param::GetBool(Param::AlertLog))
  {
    pcsAlertMatrix[pcsAlertCount] = pcsAlertId;
    pcsAlertCount++;
    if (pcsAlertCount > 10)
      pcsAlertCount = 0;
    Param::SetInt(Param::PCSAlertCnt, pcsAlertCount);
  }

  if (pcsAlertId == 0x1E) // 0x1E = Alert30= CAN rationality.
  {
    alertCanId = ((bytes[4] << 8 | bytes[3])); // Grab the CAN ID from the alert.
    alertRxError = (bytes[2] & 0x07);          // Grab the alert detail
    ProcessCANRat(alertCanId, alertRxError);   // call processing routine.
  }
}

void PCSCan::handle504(uint32_t data[2]) // PCS Boot ID

{
  uint8_t *bytes = (uint8_t *)data;
  pcsBootId = bytes[7];
  Param::SetInt(Param::PCSBoot, pcsBootId);
}

void PCSCan::handle76C(uint32_t data[2]) // PCS Debug output

{
  uint8_t *bytes = (uint8_t *)data; // Mux id in byte 0.
                                    // Mux 0x0C(12) = chg phase A outputs
                                    // Mux 0x16(22) = chg phase B outputs
                                    // Mux 0x20(32) = chg phase C outputs
                                    // IOout=bytes 2,3 14 bit unsigned scale 0.0025
  mux76C = (bytes[0]);
  if (!GotDCI)
  {
    if (mux76C == 0x0C) // Calculate total DC output current from all 3 charger modules.
    {
      outputCurrentPhaseA = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }
    else if (mux76C == 0x16)
    {
      outputCurrentPhaseB = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }
    else if (mux76C == 0x20)
    {
      outputCurrentPhaseC = ((bytes[2] << 8 | bytes[1]) & 0x3ff) * 0.0025f;
    }

    totalOutputCurrent = outputCurrentPhaseA + outputCurrentPhaseB + outputCurrentPhaseC;
    Param::SetFloat(Param::idc, totalOutputCurrent);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////PCS CAN Messages to Send
///////////////////////////////////////////////////////////////////////////////////////

void PCSCan::Msg13D() // Required by post 2020 firmwares. Mirrors some content in 0x23D.
{
  acCurrentLimit = Param::GetInt(Param::iaclim) * 2;
  uint8_t bytes[6];
  if (!DigIo::chena_out.Get())
    bytes[0] = 0x05; // Set charger enabled.
  if (DigIo::chena_out.Get())
    bytes[0] = 0x0A;         // Set charger disabled
  bytes[1] = acCurrentLimit; // charge current limit. gain 0.5. 0x40 = 64 dec =32A. Populate AC lim in here.
  bytes[2] = 0XAA;
  bytes[3] = 0X1A;
  bytes[4] = 0xFF;
  bytes[5] = 0x02;
  Can::GetInterface(0)->Send(0x13D, (uint32_t *)bytes, 6);
}

void PCSCan::Msg20A()
{ // HVP contactor state. Static msg.
  uint8_t bytes[6];
  bytes[0] = 0xF6;
  bytes[1] = 0x15;
  bytes[2] = 0x09;
  bytes[3] = 0x82;
  bytes[4] = 0x18;
  bytes[5] = 0x01;
  Can::GetInterface(0)->Send(0x20A, (uint32_t *)bytes, 6);
}

void PCSCan::Msg212()
{ // BMS status. Static msg
  uint8_t bytes[8];
  bytes[0] = 0xB9;
  bytes[1] = 0x1C;
  bytes[2] = 0x94;
  bytes[3] = 0xAD;
  bytes[4] = 0xC3;
  bytes[5] = 0x15;
  bytes[6] = 0x06;
  bytes[7] = 0x63;
  Can::GetInterface(0)->Send(0x212, (uint32_t *)bytes, 8);
}

void PCSCan::Msg21D()
{
  // CP EVSE Status. Populate with Cable lim and pilot lim? I think PCS does not care about Limits here in certain frimware.
  uint8_t bytes[8];
  bytes[0] = 0x2D;                              // 2D FOR EU TYPE 2, 5D FOR US TYPE 1 INPUTS
  bytes[1] = Param::GetInt(Param::evselim) * 2; // pilot current. 8 bits. scale 0.5.
  bytes[2] = 0x00;
  bytes[3] = Param::GetInt(Param::cablelim); // cable current limit. scale 1. 7 bits. 0x20=32Amps
  bytes[4] = 0x80;
  bytes[5] = 0x00;
  bytes[6] = 0x60;
  bytes[7] = 0x10;
  Can::GetInterface(0)->Send(0x21D, (uint32_t *)bytes, 8);
}

void PCSCan::Msg22A()
{
  // HVP PCS control.
  uint8_t activate = Param::GetInt(Param::activate);
  uint8_t bytes[4];
  bytes[0] = 0x00; // precharge request voltage. 16 bit signed int. scale 0.1. Bytes 0 and 1.
  bytes[1] = 0x00;
  if (activate == 0)
    bytes[2] = (hvVoltage & 0xF) << 4 | 0x0; // Shutdown both chg and dcdc
  if (activate == 1)
    bytes[2] = (hvVoltage & 0xF) << 4 | 0x5; // Charger en
  if (activate == 2)
    bytes[2] = (hvVoltage & 0xF) << 4 | 0x9; // DCDC en
  if (activate == 3)
    bytes[2] = (hvVoltage & 0xF) << 4 | 0xD; // Charger en and DCDC en
  bytes[3] = (hvVoltage >> 4) & 0xFF;        // 0x17;//Measured hv voltage. 0x177 = 375v.
  Can::GetInterface(0)->Send(0x22A, (uint32_t *)bytes, 4);
}

void PCSCan::Msg232()
{
  // BMS Contactor request.Static msg.
  uint8_t bytes[8];
  bytes[0] = 0x0A;
  bytes[1] = 0x02;
  bytes[2] = 0xD5;
  bytes[3] = 0x09;
  bytes[4] = 0xCB;
  bytes[5] = 0x04;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  Can::GetInterface(0)->Send(0x232, (uint32_t *)bytes, 8);
}

void PCSCan::Msg23D()
{
  // CP Charge Status
  uint8_t bytes[4];
  acCurrentLimit = Param::GetInt(Param::iaclim) * 2;
  if (!DigIo::chena_out.Get())
    bytes[0] = 0x05; // Set charger enabled.
  if (DigIo::chena_out.Get())
    bytes[0] = 0x0A;         // Set charger disabled
  bytes[1] = acCurrentLimit; // charge current limit. gain 0.5. 0x40 = 64 dec =32A. Populate AC lim in here.
  bytes[2] = 0xFF;           // Internal max current limit.
  bytes[3] = 0x0F;
  Can::GetInterface(0)->Send(0x23D, (uint32_t *)bytes, 4);
}

void PCSCan::Msg25D()
{
  // CP Status. Only byte 0 bits 0 and 1 are important to the PCS
  uint8_t bytes[8];
  bytes[0] = 0xD9; // D9 FOR EURO. D8 FOR US. 0=US Tesla , 1=Euro IEC , 2=GB, 3=IEC CCS. Must match PCS type!
  bytes[1] = 0x8C;
  bytes[2] = 0x01;
  bytes[3] = 0xB5;
  bytes[4] = 0x4A;
  bytes[5] = 0xC1;
  bytes[6] = 0x0A;
  bytes[7] = 0xE0;
  Can::GetInterface(0)->Send(0x25D, (uint32_t *)bytes, 8);
}

void PCSCan::Msg2B2(uint16_t Charger_Power)
{
  // Charge Power Request
  pcsPowerRequest = Charger_Power; // Param::GetFloat(Param::pacspnt)*1000.0f;
  if (!isShort2B2)
  {
    uint8_t bytes[5];                  // Older firmware sends this as dlc=3, newer sends as dlc=5.
                                       // A missmatch here will trigger a can rationality error.
    bytes[0] = pcsPowerRequest & 0xFF; // KW scale 0.001 16 bit unsigned in bytes 0 and 1. e.g. 0x0578 = 1400 dec = 1400Watts=1.4kW.
    bytes[1] = pcsPowerRequest >> 8;
    if (DigIo::chena_out.Get())
      bytes[2] = 0x00; // Set charger disabled.
    if (!DigIo::chena_out.Get())
      bytes[0] = 0x02; // Set charger enabled.
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    Can::GetInterface(0)->Send(0x2B2, (uint32_t *)bytes, 5);
  }
  else
  {
    uint8_t bytes[3];                  // Older firmware sends this as dlc=3, newer sends as dlc=5.
                                       // A missmatch here will trigger a can rationality error.
    bytes[0] = pcsPowerRequest & 0xFF; // KW scale 0.001 16 bit unsigned in bytes 0 and 1. e.g. 0x0578 = 1400 dec = 1400Watts=1.4kW.
    bytes[1] = pcsPowerRequest >> 8;
    if (DigIo::chena_out.Get())
      bytes[2] = 0x00; // Set charger disabled.
    if (!DigIo::chena_out.Get())
      bytes[0] = 0x02; // Set charger enabled.
    Can::GetInterface(0)->Send(0x2B2, (uint32_t *)bytes, 3);
  }
}

void PCSCan::Msg321()
{
  uint8_t bytes[8]; // VCFront sensors. Static.
  bytes[0] = 0x2C;
  bytes[1] = 0xB6;
  bytes[2] = 0xA8;
  bytes[3] = 0x7F;
  bytes[4] = 0x02;
  bytes[5] = 0x7F;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  Can::GetInterface(0)->Send(0x321, (uint32_t *)bytes, 8);
}

void PCSCan::Msg333()
{

  uint8_t bytes[4]; // UI charge request message. Can be used as an ac current limit.
  bytes[0] = 0x04;  // leaving at 48A for now.
  bytes[1] = 0x30;  // byte one. 7 bits scale 1. 0x30=48A.
  bytes[2] = 0x29;
  bytes[3] = 0x07;
  Can::GetInterface(0)->Send(0x333, (uint32_t *)bytes, 4);
}

void PCSCan::Msg3A1()
{
  dcdcSetpoint = Param::GetFloat(Param::udcdc) * 100.0f;
  uint8_t bytes[8]; // VCFront vehicle status
  bytes[0] = 0x09;  // This message contains the 12v dcdc target setpoint. bits 16-26 as an 11bit unsigned int. scale 0.01
  bytes[1] = 0x62;
  bytes[2] = dcdcSetpoint & 0xFF;          // 78 , d gives us a 14v target.
  bytes[3] = ((dcdcSetpoint >> 8) | 0x99); // 0x9D;
  bytes[4] = 0x08;
  bytes[5] = 0x2C;
  bytes[6] = 0x12;
  bytes[7] = 0x5A;
  Can::GetInterface(0)->Send(0x3A1, (uint32_t *)bytes, 8);
}

void PCSCan::Msg3B2()
{
  uint8_t bytes[8];    // This msg changed drastically between 2019 and 2020-2021 model firmwares. PCS pays close attention to these two muxes.
  if (isMux3b2Enabled) // BMS log2 message
  {
    bytes[0] = 0xE5; // mux 5=charging.
    bytes[1] = 0x0D;
    bytes[2] = 0xEB;
    bytes[3] = 0xFF;
    bytes[4] = 0x0C;
    bytes[5] = 0x66;
    bytes[6] = 0xBB;
    bytes[7] = 0x11;
    Can::GetInterface(0)->Send(0x3B2, (uint32_t *)bytes, 8);
    isMux3b2Enabled = false;
  }
  else
  {
    bytes[0] = 0xE3; // mux 3=charge termination
    bytes[1] = 0x5D;
    bytes[2] = 0xFB;
    bytes[3] = 0xFF;
    bytes[4] = 0x0C;
    bytes[5] = 0x66;
    bytes[6] = 0xBB;
    bytes[7] = 0x06;
    Can::GetInterface(0)->Send(0x3B2, (uint32_t *)bytes, 8);
    isMux3b2Enabled = true;
  }
}

void PCSCan::Msg545()
{
  uint8_t bytes[8];
  if (isMux545Enabled) // VCFront
  {
    bytes[0] = 0x14;
    bytes[1] = 0x00;
    bytes[2] = 0x3F;
    bytes[3] = 0x70;
    bytes[4] = 0x9F;
    bytes[5] = 0x01;
    bytes[6] = (message545Count << 4) | 0xA;
    bytes[7] = CalcPCSChecksum((uint8_t *)bytes, 0x545);
    Can::GetInterface(0)->Send(0x545, (uint32_t *)bytes, 8);
    isMux545Enabled = false;
  }
  else
  {
    bytes[0] = 0x03;
    bytes[1] = 0x19;
    bytes[2] = 0x64;
    bytes[3] = 0x32;
    bytes[4] = 0x19;
    bytes[5] = 0x00;
    bytes[6] = (message545Count << 4);
    bytes[7] = CalcPCSChecksum((uint8_t *)bytes, 0x545);
    Can::GetInterface(0)->Send(0x545, (uint32_t *)bytes, 8);
    isMux545Enabled = true;
  }
  message545Count++;
  if (message545Count > 0x0F)
    message545Count = 0;
}

void PCSCan::AlertHandler() // Alert handler

{
  Param::SetInt(Param::PCSAlerts, pcsAlertMatrix[Param::GetInt(Param::Alerts)]);
  if (!Param::GetBool(Param::AlertLog))
  {
    pcsAlertCount = 0;
    Param::SetInt(Param::PCSAlertCnt, pcsAlertCount);
    for (int i = 0; i < 10; i++)
      pcsAlertMatrix[i] = 0; // Clear log and counter when PCS alert logging disabled
  }
}

static uint8_t CalcPCSChecksum(uint8_t *bytes, uint16_t id)
{
  uint16_t checksum_calc = 0;
  for (int b = 0; b < 7; b++)
  {
    checksum_calc = checksum_calc + bytes[b];
  }
  checksum_calc += id + (id >> 8);
  checksum_calc &= 0xFF;

  return (uint8_t)checksum_calc;
}

static int16_t ProcessTemps(uint16_t InVal)
{
  int16_t value = InVal & 0x3ff;
  if (InVal & 0x400)
    value -= 0x3ff;
  value = value * 0.1 + 40;
  value = InVal & 0x3ff;
  if (InVal & 0x400)
    value -= 0x3ff;
  value = value * 0.1 + 40;
  return value;
}

static void ProcessCANRat(uint16_t alertCanId, uint8_t alertRxError)
{
  switch (alertCanId)
  {
  case 0x2B2:
    if (alertRxError == 0x2)
      isShort2B2 = true; // msg 0x2B2 is too short so change to long.
    if (alertRxError == 0x1)
      isShort2B2 = false; // msg 0x2B2 is too long so change to short.
    break;

  default:

    break;
  }
}
