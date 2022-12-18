#include <msp430.h>
#include <Wire.h>

#define RX_LED RED_LED
#define TX_LED GREEN_LED
const byte SHT40_ADDR = 0x44;
const byte TCS34725_ADDR = 0x29;

// Values
const byte TCS34725_INTEGRATIONTIME = 0x00; //614.4ms
const byte TCS34725_GAIN = 0x00; // X1 gain
const byte TCS34725_ENABLE_POWERON = 0x01; // Enable Chip (Pulls out of sleep)
const byte TCS34725_ENABLE_ADC = (byte)(TCS34725_ENABLE_POWERON | 0x02); // Enables ADC

// Commands
const byte TCS34725_CMD = 0x80;
const byte TCS34725_CMD_ID = (byte)(TCS34725_CMD | 0x12);
const byte TCS34725_CMD_ATIME = (byte)(TCS34725_CMD | 0x01); // Register
const byte TCS34725_CMD_CONTROL = (byte)(TCS34725_CMD | 0x0F); // Register
const byte TCS34725_CMD_ENABLE = (byte)(TCS34725_CMD | 0x00);
const byte TCS34725_CMD_CDATA = (byte)(TCS34725_CMD | 0x14);
const byte TCS34725_CMD_RDATA = (byte)(TCS34725_CMD | 0x16);
const byte TCS34725_CMD_GDATA = (byte)(TCS34725_CMD | 0x18);
const byte TCS34725_CMD_BDATA = (byte)(TCS34725_CMD | 0x1A);

const byte SHT40_CMD_SERIALNUM = 0x89;
const byte SHT40_CMD_HUMTEMP = 0xFD;

byte hum_and_temp_buffer[6];
byte colors_buffer[2];
uint16_t colors[4]; //r g b c
float temp_hum[2];


void setup();
void loop();
void write_8b(byte addr, byte cmd);
void write_16b(byte addr, byte *cmd);
void read_Nb(byte addr, byte cmd, byte *buffer, byte responseSize);
void initSHT40();
void initTCS34725();
void getHumTemp(float *returnArray, bool metric);
void getColors(uint16_t *returnArray);

void setup() {
  Wire.begin();            // Join I2C bus as Master (No address needed)
  pinMode(RX_LED, OUTPUT); // Setup RX LED
  pinMode(TX_LED, OUTPUT); // Setup TX LED

  Serial.begin(9600);      // Begin debug Serial
  while(!Serial);          // Wait for serial to start
  delay(100);              // Give sensors time to init
  initSHT40();             // SHT40 Setup and get info
  initTCS34725();          // TCS34725 Setup and get info
  Serial.print("Setup Complete");
}

void loop() {
  //Get data and send over UART
  
  //Get Humidity and Temp
  getHumTemp(temp_hum, false);
  Serial.print("Temp:");
  Serial.print(temp_hum[0]); // Scaled from C to F
  Serial.print(" ");

  Serial.print("Humidity:");
  Serial.println(temp_hum[1]); 
  
  //Get RGB values
  getColors(colors);
  Serial.print("R: "); Serial.print(colors[0]);
  Serial.print(" G: "); Serial.print(colors[1]);
  Serial.print(" B: "); Serial.print(colors[2]);
  Serial.print(" C: "); Serial.println(colors[3]);
  
  //Get RGB values
}


// Functions

/* Sends 8-bit command to specific address over I2C. Also
 * turns TX_LED on while transmitting message.
 */
void write_8b(byte addr, byte cmd) {
  Wire.beginTransmission(addr); // Start message queue
  Wire.write(cmd);              // Put command in queue
  digitalWrite(TX_LED, HIGH);
  Wire.endTransmission();       // Transmit message queue
  digitalWrite(TX_LED, LOW);
  delay(10);                    // Breathing room
}

/* Sends 8-bit command to specific address over I2C. Also
 * turns TX_LED on while transmitting message.
 */
void write_16b(byte addr, byte *cmd) {
  Wire.beginTransmission(addr); // Start message queue
  Wire.write(cmd, 2);              // Put command in queue
  digitalWrite(TX_LED, HIGH);
  Wire.endTransmission();       // Transmit message queue
  digitalWrite(TX_LED, LOW);
  delay(10);                    // Breathing room
}

/* Sends 8-bit command and reads a N-bit response from I2C. Also 
 * turns RX_LED on while reading message.
 */
void read_Nb(byte addr, byte cmd, byte *buffer, byte responseSize) {
  write_8b(addr, cmd);        // Transmit command
  int i = 0;
  Wire.requestFrom(addr, responseSize);
  digitalWrite(RX_LED, HIGH);
  while (Wire.available()) {
    buffer[i] = Wire.read();  // Store available bytes
    i++;
  } 
  digitalWrite(RX_LED, LOW);
}

void initSHT40() {
//  Serial.println("Sensor SHT4x current configuration: ");
//  Serial.print("Addr: 0x"); Serial.println(SHT40_ADDR, HEX);
//  Serial.print("Accuracy: "); Serial.println("HIGHEST");
  
  byte serialNum_buffer[6]; // Serial # response is 6 bytes
  read_Nb(SHT40_ADDR, SHT40_CMD_SERIALNUM, serialNum_buffer, 6);
  // Responses from SHT40 include CHC sum, so those need to be excluded (idx 2, 5)
  int trimmed = serialNum_buffer[0] + 
                serialNum_buffer[1] + 
                serialNum_buffer[3] + 
                serialNum_buffer[4];
//  Serial.print("Serial #: "); Serial.println(trimmed);
}


void initTCS34725() {
//  Serial.println("Sensor TCS34725 current configuration: ");
//  Serial.print("Addr: 0x"); Serial.println(TCS34725_ADDR, HEX);
  
  byte id_buffer[0]; //ID response is 1 byte
  read_Nb(TCS34725_ADDR, TCS34725_CMD_ID, id_buffer, 1);
//  Serial.print("ID: 0x"); Serial.println(id_buffer[0], HEX);

  byte temp_cmd[2];
  // Setup Integration time
  
  temp_cmd[0] = TCS34725_CMD_ATIME;
  temp_cmd[1] = TCS34725_INTEGRATIONTIME;
  write_16b(TCS34725_ADDR, temp_cmd);
  delay(10);
  
  // Setup Gain
  temp_cmd[0] = TCS34725_CMD_CONTROL;
  temp_cmd[1] = TCS34725_GAIN;
  write_16b(TCS34725_ADDR, temp_cmd);
  delay(10);
  
  // Enable
  temp_cmd[0] = TCS34725_CMD_ENABLE;
  temp_cmd[1] = TCS34725_ENABLE_POWERON;
  write_16b(TCS34725_ADDR, temp_cmd);
  delay(10);

  // Enable adc
  temp_cmd[0] = TCS34725_CMD_ENABLE;
  temp_cmd[1] = TCS34725_ENABLE_ADC;
  write_16b(TCS34725_ADDR, temp_cmd);

  // Allow sensor to collect data over first integration time initially
  delay((256 - TCS34725_INTEGRATIONTIME) * 12 / 5 + 1);
  
}

void getHumTemp(float *returnArray, bool metric) {
  read_Nb(SHT40_ADDR, SHT40_CMD_HUMTEMP, hum_and_temp_buffer, 6);
  
  unsigned int raw_temp = (hum_and_temp_buffer[0] * 256) + hum_and_temp_buffer[1];
  unsigned int raw_humidity = (hum_and_temp_buffer[3] * 256) + hum_and_temp_buffer[4];

  float scaled_temp = -45 + 175 * (raw_temp/65535.0);        // C
  float scaled_humidity = -6 + 125 * (raw_humidity/65535.0); // %

  if (metric) {
    returnArray[0] = scaled_temp;
  } else {
    returnArray[0] = scaled_temp*(9.0/5.0)+32;
  }

  returnArray[1] = scaled_humidity;
}

void getColors(uint16_t *returnArray) {
  // Red
  read_Nb(TCS34725_ADDR, TCS34725_CMD_RDATA, colors_buffer, 2);
  returnArray[0] = colors_buffer[0] + colors_buffer[1];

  // Green
  read_Nb(TCS34725_ADDR, TCS34725_CMD_GDATA, colors_buffer, 2);
  returnArray[1] = colors_buffer[0] + colors_buffer[1];

  // Blue
  read_Nb(TCS34725_ADDR, TCS34725_CMD_BDATA, colors_buffer, 2);
  returnArray[2] = colors_buffer[0] + colors_buffer[1];

  // Yes
  read_Nb(TCS34725_ADDR, TCS34725_CMD_CDATA, colors_buffer, 2);
  returnArray[3] = colors_buffer[0] + colors_buffer[1];

  delay((256 - TCS34725_INTEGRATIONTIME) * 12 / 5 + 1);
}
