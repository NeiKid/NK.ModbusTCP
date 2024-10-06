/*
 * NKModbusTCP Library
 * 
 * Descripción:
 * La biblioteca NKModbusTCP permite la comunicación con dispositivos PLC utilizando el protocolo Modbus TCP/IP.
 * Está diseñada para ser utilizada con el módulo ESP8266 WiFi. Incluye funciones para leer y escribir 
 * registros de 16 y 32 bits (float), así como para manipular bobinas y entradas discretas.
 *
 * Funciones incluidas:
 * - Lectura de registros holding y de entrada (16 y 32 bits)
 * - Escritura de registros holding (16 y 32 bits)
 * - Lectura y escritura de bobinas y entradas discretas
 * - Lectura y escritura de múltiples registros
 *
 * Autor: Rafael Valdés Silva
 * Contacto: rafael_andrew@yahoo.es
 * https://github.com/NeiKid
 *
 * Ejemplo de uso:
 * #include <NKModbusTCP.h>
 *
 * const char* ssid = "tu_ssid";
 * const char* password = "tu_password";
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
 *         Serial.println("Conectando a WiFi...");
 *     }
 *
 *     Serial.println("Conectado a WiFi");
 *     
 *     // Ejemplo de lectura de registros float
 *     float buffer[2];
 *     if (modbus.readMultipleHoldingRegistersFloat(0x0001, 2, buffer)) {
 *         for (int i = 0; i < 2; i++) {
 *             Serial.println(buffer[i]);
 *         }
 *     }
 * }
 *
 * void loop() {
 *     // Tu lógica principal aquí
 * }
 */

#ifndef NKMODBUSTCP_H
#define NKMODBUSTCP_H

#include <Arduino.h>
#include <ESP8266WiFi.h>

class NKModbusTCP {
public:
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

private:
    const char* plc_ip;
    uint16_t plc_port;
    WiFiClient client;
};

#endif