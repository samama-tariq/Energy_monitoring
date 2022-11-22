#include <ModbusMaster.h>

#define MAX485_DE 14    // Driver Enable
#define MAX485_RE 32    // Receiver Enable

ModbusMaster node;

// ==================================================================================================================================================================================

// L1 Significant Bit Variables ..............
uint16_t V1_LSB, V1_MSB, C1_LSB, C1_MSB, TP1_LSB, TP1_MSB, AP1_LSB, AP1_MSB, F1_LSB, F1_MSB;

// L1 Data Registers .........................
uint8_t Voltage_Data_Reg, Current_Data_Reg, TR_Power_Data_Reg, AP_Power_Data_Reg, Freq_Data_Reg;

// L1 Local Holding Variables ......................
uint16_t V1, C1, P1, AP1, F1;

// L1 Global Holding Variables ......................
float L1V, L1C, L1TP, L1AP, L1RP, L1PF, L1F;

// L2 Significant Bit Variables ..............
uint16_t V2_LSB, V2_MSB, C2_LSB, C2_MSB, TP2_LSB, TP2_MSB, AP2_LSB, AP2_MSB, F2_LSB, F2_MSB;

// L2 Data Registers .........................
uint8_t Voltage_Data_Reg_L2, Current_Data_Reg_L2, TR_Power_Data_Reg_L2, AP_Power_Data_Reg_L2, Freq_Data_Reg_L2;

// L2 Local Holding Variables ......................
uint16_t V2, C2, P2, AP2, F2;

// L2 Global Holding Variables ......................
float L2V, L2C, L2TP, L2AP, L2RP, L2PF, L2F;

// L3 Significant Bit Variables ..............
uint16_t V3_LSB, V3_MSB, C3_LSB, C3_MSB, TP3_LSB, TP3_MSB, AP3_LSB, AP3_MSB, F3_LSB, F3_MSB;

// L3 Data Registers .........................
uint8_t Voltage_Data_Reg_L3, Current_Data_Reg_L3, TR_Power_Data_Reg_L3, AP_Power_Data_Reg_L3, Freq_Data_Reg_L3;

// L3 Local Holding Variables ......................
uint16_t V3, C3, P3, AP3, F3;

// L3 Global Holding Variables ......................
float L3V, L3C, L3TP, L3AP, L3RP, L3PF, L3F;

// Total Power

// TP Significant Bit Variables ..............
uint16_t TP_LSB, TP_MSB;

// TP Data Registers .........................
uint8_t Voltage_Data_Reg_TP;

// TP Local Holding Variables ......................
uint16_t TP;

// TP Global Holding Variables ......................
float TPG

// ==================================================================================================================================================================================

void setup()
{
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16, 17);  // ( Baud rate = 9600, Configuration = [8 Data bits,No parity,1 Stop bit], Rx pin, Tx pin)

  node.begin(1, Serial1);                   // Modbus slave ID 1

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

// ==================================================================================================================================================================================

void loop() {
  Serial.println("***************** LINE 1 PARAMETERS *********************");
  delay(100);
  get_voltage_val();
  delay(100);
  get_current_val();
  delay(100);
  get_True_Power();
  delay(100);
  get_Power_Derivatives();
  delay(100);
  get_frequency();
  delay(100);
  Serial.println("*********************************************************");
  delay(100);
  Serial.println("***************** LINE 2 PARAMETERS *********************");
  delay(100);
  get_voltage_val_l2();
  delay(100);
  get_current_val_l2();
  delay(100);
  get_True_Power_l2();
  delay(100);
  get_Power_Derivatives_l2();
  delay(100);
  Serial.println("*********************************************************");
  delay(100);

  Serial.println("***************** LINE 3 PARAMETERS *********************");
  delay(100);
  get_voltage_val_l3();
  delay(100);
  get_current_val_l3();
  delay(100);
  get_True_Power_l3();
  delay(100);
  get_Power_Derivatives_l3();
  delay(100);
  Serial.println("*********************************************************");
  delay(100);
}

// ==================================================================================================================================================================================

void preTransmission()
{
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

// ==================================================================================================================================================================================
// Voltages of all line 1,2,3

//line L1-N Voltages
void get_voltage_val() {

  Voltage_Data_Reg = node.readHoldingRegisters(0x0280, 2);           // This loop is to get the output of 4 bytes (Float datatype)

  if ( Voltage_Data_Reg == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    V1_LSB = node.getResponseBuffer(0x00);
    V1_MSB = node.getResponseBuffer(0x01);

    uint16_t V_Data[2] = {V1_LSB, V1_MSB};
    memcpy(&V1, V_Data, 4);

    Serial.print("L1 Line-N Voltage: ");
    L1V = V1 * 0.1;
    Serial.print(L1V);
    Serial.println(" V");
  }
}
//line L2-N Voltages
void get_voltage_val_l2() {

  Voltage_Data_Reg_L2 = node.readHoldingRegisters(0x0286, 2);           // This loop is to get the output of 4 bytes (Float datatype)

  if ( Voltage_Data_Reg_L2 == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    V2_LSB = node.getResponseBuffer(0x00);
    V2_MSB = node.getResponseBuffer(0x01);

    uint16_t V2_Data[2] = {V2_LSB, V2_MSB};
    memcpy(&V2, V2_Data, 4);

    Serial.print("L2 Line-N Voltage: ");
    L2V = V2 * 0.1;
    Serial.print(L2V);
    Serial.println(" V");
  }
}
//line L3-N Voltages
void get_voltage_val_l3() {

  Voltage_Data_Reg_L3 = node.readHoldingRegisters(0x028C, 2);           // This loop is to get the output of 4 bytes (Float datatype)

  if ( Voltage_Data_Reg_L3 == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    V3_LSB = node.getResponseBuffer(0x00);
    V3_MSB = node.getResponseBuffer(0x01);

    uint16_t V3_Data[2] = {V3_LSB, V3_MSB};
    memcpy(&V3, V3_Data, 4);

    Serial.print("L3 Line-N Voltage: ");
    L3V = V3 * 0.1;
    Serial.print(L3V);
    Serial.println(" V");
  }
}

// ==================================================================================================================================================================================

// Current of all lines 1,2,3

// current Line 1
void get_current_val() {

  Current_Data_Reg = node.readHoldingRegisters(0x0282, 2);    // This loop is to get the output of 4 bytes (Float datatype)

  if ( Current_Data_Reg == node.ku8MBSuccess )
  {
    C1_LSB = node.getResponseBuffer(0x00);
    C1_MSB = node.getResponseBuffer(0x01);

    uint16_t C_Data[2] = {C1_LSB, C1_MSB};
    memcpy(&C1, C_Data, 4);

    Serial.print("L1 Line Current: ");
    L1C = C1 * 0.001 * 7;
    Serial.print(L1C);
    Serial.println(" A");
  }
}

//Current Line 2
void get_current_val_l2() {

  Current_Data_Reg_L2 = node.readHoldingRegisters(0x0288, 2);    // This loop is to get the output of 4 bytes (Float datatype)

  if ( Current_Data_Reg_L2 == node.ku8MBSuccess )
  {
    C2_LSB = node.getResponseBuffer(0x00);
    C2_MSB = node.getResponseBuffer(0x01);

    uint16_t C2_Data[2] = {C1_LSB, C1_MSB};
    memcpy(&C2, C2_Data, 4);

    Serial.print("L2 Line Current: ");
    L2C = C2 * 0.001 * 7;
    Serial.print(L2C);
    Serial.println(" A");
  }
}

// current Line 3
void get_current_val_l3() {

  Current_Data_Reg_L3 = node.readHoldingRegisters(0x028E, 2);    // This loop is to get the output of 4 bytes (Float datatype)

  if ( Current_Data_Reg_L3 == node.ku8MBSuccess )
  {
    C3_LSB = node.getResponseBuffer(0x00);
    C3_MSB = node.getResponseBuffer(0x01);

    uint16_t C3_Data[2] = {C3_LSB, C3_MSB};
    memcpy(&C3, C3_Data, 4);

    Serial.print("L3 Line Current: ");
    L3C = C3 * 0.001 * 7;
    Serial.print(L3C);
    Serial.println(" A");
  }
}

// ==================================================================================================================================================================================

// power of all lines 1,2,3

// Line 1
void get_True_Power() {

  TR_Power_Data_Reg = node.readHoldingRegisters(0x0284, 2);      // This loop is to get the output of 4 bytes (Float datatype)

  if (TR_Power_Data_Reg == node.ku8MBSuccess)
  {
    TP1_LSB = node.getResponseBuffer(0x00);
    TP1_MSB = node.getResponseBuffer(0x01);

    uint16_t P_Data[2] = {TP1_LSB, TP1_MSB};
    memcpy(&P1, P_Data, 4);

    Serial.print("L1 Line Power: ");
    L1TP = P1 * 0.7;
    Serial.print(L1TP);
    Serial.println(" W");
  }
}

//Line 2
void get_True_Power_l2() {

  TR_Power_Data_Reg_L2 = node.readHoldingRegisters(0x028A, 2);      // This loop is to get the output of 4 bytes (Float datatype)

  if (TR_Power_Data_Reg_L2 == node.ku8MBSuccess)
  {
    TP2_LSB = node.getResponseBuffer(0x00);
    TP2_MSB = node.getResponseBuffer(0x01);

    uint16_t P2_Data[2] = {TP2_LSB, TP2_MSB};
    memcpy(&P2, P2_Data, 4);

    Serial.print("L2 Line Power: ");
    L2TP = P2 * 0.7;
    Serial.print(L2TP);
    Serial.println(" W");
  }
}

//Line 3
void get_True_Power_l3() {

  TR_Power_Data_Reg_L3 = node.readHoldingRegisters(0x0290, 2);      // This loop is to get the output of 4 bytes (Float datatype)

  if (TR_Power_Data_Reg_L3 == node.ku8MBSuccess)
  {
    TP3_LSB = node.getResponseBuffer(0x00);
    TP3_MSB = node.getResponseBuffer(0x01);

    uint16_t P3_Data[2] = {TP3_LSB, TP3_MSB};
    memcpy(&P3, P3_Data, 4);

    Serial.print("L3 Line Power: ");
    L3TP = P3 * 0.7;
    Serial.print(L3TP);
    Serial.println(" W");
  }
}

// ==================================================================================================================================================================================

// power derivaties of line 1,2,3

//line 1
void get_Power_Derivatives() {
  // Apparent power, reactive power and power factor logics
  AP_Power_Data_Reg = node.readHoldingRegisters(0x02A0, 2);         // This loop is to get the output of 4 bytes (Float datatype)

  if (AP_Power_Data_Reg == node.ku8MBSuccess)
  {
    AP1_LSB = node.getResponseBuffer(0x00);
    AP1_MSB = node.getResponseBuffer(0x01);

    uint16_t AP_Data[2] = {AP1_LSB, AP1_MSB};
    memcpy(&AP1, AP_Data, 4);

    // Apparent Power
    Serial.print("Apparent Power: ");
    L1AP = AP1 * 0.7;
    Serial.print(L1AP);
    Serial.println(" VA");

    // Reactive Power
    Serial.print("Reactive Power: ");
    L1RP = sqrt(sq(L1AP) - sq(L1TP));
    Serial.print(L1RP);
    Serial.println(" VAR");

    // Power Factor
    if (L1AP > 0) {
      Serial.print("Power Factor: ");
      L1PF = L1TP / L1AP;
      Serial.print(L1PF);
      Serial.println(" PF");
    }
    else {
      L1PF = 0.0;
      Serial.print("Power Factor L1: ");
      Serial.print(L1PF);
      Serial.println(" PF");
    }
  }
}

//Line 2
void get_Power_Derivatives_l2() {
  // Apparent power, reactive power and power factor logics
  AP_Power_Data_Reg_L2 = node.readHoldingRegisters(0x02A2, 2);         // This loop is to get the output of 4 bytes (Float datatype)

  if (AP_Power_Data_Reg_L2 == node.ku8MBSuccess)
  {
    AP2_LSB = node.getResponseBuffer(0x00);
    AP2_MSB = node.getResponseBuffer(0x01);

    uint16_t AP2_Data[2] = {AP2_LSB, AP2_MSB};
    memcpy(&AP2, AP2_Data, 4);

    // Apparent Power
    Serial.print("Apparent Power L2: ");
    L2AP = AP2 * 0.7;
    Serial.print(L2AP);
    Serial.println(" VA");

    // Reactive Power
    Serial.print("Reactive Power L2: ");
    L2RP = sqrt(sq(L2AP) - sq(L2TP));
    Serial.print(L2RP);
    Serial.println(" VAR");

    // Power Factor
    if (L2AP > 0) {
      Serial.print("Power Factor L2: ");
      L2PF = L2TP / L2AP;
      Serial.print(L2PF);
      Serial.println(" PF");
    }
    else {
      L2PF = 0.0;
      Serial.print("Power Factor L2: ");
      Serial.print(L2PF);
      Serial.println(" PF");
    }
  }
}

void get_Power_Derivatives_l3() {
  // Apparent power, reactive power and power factor logics
  AP_Power_Data_Reg_L3 = node.readHoldingRegisters(0x02A4, 2);         // This loop is to get the output of 4 bytes (Float datatype)

  if (AP_Power_Data_Reg_L3 == node.ku8MBSuccess)
  {
    AP3_LSB = node.getResponseBuffer(0x00);
    AP3_MSB = node.getResponseBuffer(0x01);

    uint16_t AP3_Data[2] = {AP3_LSB, AP3_MSB};
    memcpy(&AP3, AP3_Data, 4);

    // Apparent Power
    Serial.print("Apparent Power L3: ");
    L3AP = AP3 * 0.7;
    Serial.print(L3AP);
    Serial.println(" VA");

    // Reactive Power
    Serial.print("Reactive Power L3: ");
    L3RP = sqrt(sq(L3AP) - sq(L3TP));
    Serial.print(L3RP);
    Serial.println(" VAR");

    // Power Factor
    if (L3AP > 0) {
      Serial.print("Power Factor L2: ");
      L3PF = L3TP / L3AP;
      Serial.print(L3PF);
      Serial.println(" PF");
    }
    else {
      L3PF = 0.0;
      Serial.print("Power Factor L2: ");
      Serial.print(L3PF);
      Serial.println(" PF");
    }
  }
}

// ==================================================================================================================================================================================

void get_frequency() {

  Freq_Data_Reg = node.readHoldingRegisters(0x02B8, 2);        // This loop is to get the output of 4 bytes (Float datatype)
  if (Freq_Data_Reg == node.ku8MBSuccess)                      // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    F1_LSB = node.getResponseBuffer(0x00);
    F1_MSB = node.getResponseBuffer(0x01);

    uint16_t F_Data[2] = {F1_LSB, F1_MSB};
    memcpy(&F1, F_Data, 4);

    Serial.print("Phase A Frequency: ");
    L1F = F1 * 0.1;
    Serial.print(L1F);
    Serial.println("  Hz");
  }
}

void total_power(){

}
