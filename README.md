NKModbusTCP Library

The NKModbusTCP library allows communication with PLC devices using the Modbus TCP/IP protocol.
It is designed to be used with the ESP8266 WiFi module. It includes functions to read and write 16-bit and 32-bit (float) registers, as well as to manipulate coils and discrete inputs.

Included functions:
Reading holding and input registers (16 and 32 bits)
Writing holding registers (16 and 32 bits)
Reading and writing coils and discrete inputs
Reading and writing multiple registers

Author: Rafael Valdés Silva
Contact: rafael_andrew@yahoo.es
https://github.com/NeiKid

Use example:

 * #include <NKModbusTCP.h>
 *
 * const char* ssid = "ssid";
 * const char* password = "password";
 * const char* plc_ip = "192.168.1.100";
 * const uint16_t plc_port = 502;
 *
 * NKModbusTCP modbus(plc_ip, plc_port);
 *
 * void setup() {
 *     Serial.begin(115200);
 *     WiFi.begin(ssid, password);
 *     
 *     while (WiFi.status() != WL_CONNECTED) {
 *         delay(1000);
 *         Serial.println("connecting...");
 *     }
 *
 *     Serial.println("connected");
 *     
 *     // read float register example
 *     float buffer[2];
 *     if (modbus.readMultipleHoldingRegistersFloat(0x0001, 2, buffer)) {
 *         for (int i = 0; i < 2; i++) {
 *             Serial.println(buffer[i]);
 *         }
 *     }
 * }
 *
 * void loop() {
 *     // your program here
 * }

Functions by header file:
    NKModbusTCP(const char* plc_ip, uint16_t plc_port);
    bool sendModbusRequest(byte* request, byte requestLength, byte* response, byte responseLength);
    float readHoldingRegisterFloat(int address);
    float readInputRegisterFloat(int address);
    bool writeSingleRegisterFloat(int address, float value);
    bool readMultipleHoldingRegistersFloat(int startAddress, int quantity, float* buffer);
    bool writeMultipleRegistersFloat(int startAddress, int quantity, float* values);
    int readCoils(int address, int quantity);
    int readDiscreteInputs(int address, int quantity);
    int readHoldingRegister(int address, int quantity);
    int readInputRegister(int address, int quantity);
    bool writeSingleCoil(int address, bool state);
    bool writeSingleRegister(int address, int value);
    bool writeMultipleCoils(int address, int quantity, byte* values);
    bool writeMultipleRegisters(int address, int quantity, int* values);
 */
