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

#include "NKModbusTCP.h"

NKModbusTCP::NKModbusTCP(const char* plc_ip, uint16_t plc_port) : plc_ip(plc_ip), plc_port(plc_port) {}

bool NKModbusTCP::sendModbusRequest(byte* request, byte requestLength, byte* response, byte responseLength) {
    if (client.connect(plc_ip, plc_port)) {
        client.write(request, requestLength);
        delay(100); // Esperar respuesta

        if (client.available()) {
            client.read(response, responseLength);
            return true;
        } else {
            Serial.println("No hay respuesta del PLC");
            return false;
        }
    } else {
        Serial.println("Fallo en la conexión");
        return false;
    }
}

float NKModbusTCP::readHoldingRegisterFloat(int address) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x03,             // Function Code (Read Holding Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        0x00, 0x02        // Quantity of Registers (2 for 32-bit float)
    };

    byte response[13];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x03) { // Confirmar que es una respuesta de lectura de registros
            union {
                byte b[4];
                float f;
            } data;

            data.b[0] = response[10];
            data.b[1] = response[9];
            data.b[2] = response[12];
            data.b[3] = response[11];

            return data.f;
        }
    }
    return -1; // Error
}

float NKModbusTCP::readInputRegisterFloat(int address) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x04,             // Function Code (Read Input Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        0x00, 0x02        // Quantity of Registers (2 for 32-bit float)
    };

    byte response[13];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x04) { // Confirmar que es una respuesta de lectura de registros de entrada
            union {
                byte b[4];
                float f;
            } data;

            data.b[0] = response[10];
            data.b[1] = response[9];
            data.b[2] = response[12];
            data.b[3] = response[11];

            return data.f;
        }
    }
    return -1; // Error
}

bool NKModbusTCP::writeSingleRegisterFloat(int address, float value) {
    union {
        byte b[4];
        float f;
    } data;

    data.f = value;

    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x09,       // Length
        0x01,             // Unit Identifier
        0x10,             // Function Code (Write Multiple Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        0x00, 0x02,       // Quantity of Registers
        0x04,             // Byte Count
        data.b[1], data.b[0], // Value
        data.b[3], data.b[2]
    };

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}

bool NKModbusTCP::readMultipleHoldingRegistersFloat(int startAddress, int quantity, float* buffer) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        (6 + quantity * 2) >> 8, (6 + quantity * 2) & 0xFF, // Length
        0x01,             // Unit Identifier
        0x03,             // Function Code (Read Holding Registers)
        (startAddress >> 8), (startAddress & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF)  // Quantity of Registers
    };

    byte response[9 + quantity * 4];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x03) { // Confirmar que es una respuesta de lectura de registros
            for (int i = 0; i < quantity; i++) {
                union {
                    byte b[4];
                    float f;
                } data;

                data.b[0] = response[9 + i * 4 + 1];
                data.b[1] = response[9 + i * 4];
                data.b[2] = response[9 + i * 4 + 3];
                data.b[3] = response[9 + i * 4 + 2];

                buffer[i] = data.f;
            }
            return true;
        }
    }
    return false; // Error
}

bool NKModbusTCP::writeMultipleRegistersFloat(int startAddress, int quantity, float* values) {
    int byteCount = 2 * quantity;
    byte request[13 + byteCount * 2] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        (7 + byteCount * 2) >> 8, (7 + byteCount * 2) & 0xFF, // Length
        0x01,             // Unit Identifier
        0x10,             // Function Code (Write Multiple Registers)
        (startAddress >> 8), (startAddress & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF), // Quantity of Registers
        byteCount * 2         // Byte Count
    };

    for (int i = 0; i < quantity; i++) {
        union {
            byte b[4];
            float f;
        } data;

        data.f = values[i];

        request[13 + i * 4] = data.b[1];
        request[14 + i * 4] = data.b[0];
        request[15 + i * 4] = data.b[3];
        request[16 + i * 4] = data.b[2];
    }

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}

int NKModbusTCP::readCoils(int address, int quantity) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x01,             // Function Code (Read Coils)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF)  // Quantity of Coils
    };

    byte response[9 + (quantity + 7) / 8];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x01) { // Confirmar que es una respuesta de lectura de bobinas
            return response[9]; // Devuelve el primer byte de los datos leídos
        }
    }
    return -1; // Error
}

int NKModbusTCP::readDiscreteInputs(int address, int quantity) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x02,             // Function Code (Read Discrete Inputs)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF)  // Quantity of Inputs
    };

    byte response[9 + (quantity + 7) / 8];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x02) { // Confirmar que es una respuesta de lectura de entradas
            return response[9]; // Devuelve el primer byte de los datos leídos
        }
    }
    return -1; // Error
}

int NKModbusTCP::readHoldingRegister(int address, int quantity) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x03,             // Function Code (Read Holding Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF)  // Quantity of Registers
    };

    byte response[9 + 2 * quantity];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x03) { // Confirmar que es una respuesta de lectura de registros
            int value = (response[9] << 8) | response[10];
            // Convertir el valor a un signed int de 16 bits
            if (value > 32767) value -= 65536;
            return value;
        }
    }
    return -1; // Error
}

int NKModbusTCP::readInputRegister(int address, int quantity) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x04,             // Function Code (Read Input Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF)  // Quantity of Registers
    };

    byte response[9 + 2 * quantity];
    if (sendModbusRequest(request, sizeof(request), response, sizeof(response))) {
        if (response[7] == 0x04) { // Confirmar que es una respuesta de lectura de registros de entrada
            int value = (response[9] << 8) | response[10];
            return value;
        }
    }
    return -1; // Error
}

bool NKModbusTCP::writeSingleCoil(int address, bool state) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x05,             // Function Code (Write Single Coil)
        (address >> 8), (address & 0xFF), // Address
        state ? 0xFF : 0x00, 0x00  // Value (0xFF00 to turn on, 0x0000 to turn off)
    };

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}

bool NKModbusTCP::writeSingleRegister(int address, int value) {
    byte request[] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        0x00, 0x06,       // Length
        0x01,             // Unit Identifier
        0x06,             // Function Code (Write Single Register)
        (address >> 8), (address & 0xFF), // Address
        (value >> 8), (value & 0xFF)  // Value
    };

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}

bool NKModbusTCP::writeMultipleCoils(int address, int quantity, byte* values) {
    int byteCount = (quantity + 7) / 8;
    byte request[13 + byteCount] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        (7 + byteCount) >> 8, (7 + byteCount) & 0xFF, // Length
        0x01,             // Unit Identifier
        0x0F,             // Function Code (Write Multiple Coils)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF), // Quantity of Coils
        byteCount         // Byte Count
    };

    for (int i = 0; i < byteCount; i++) {
        request[13 + i] = values[i];
    }

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}

bool NKModbusTCP::writeMultipleRegisters(int address, int quantity, int* values) {
    int byteCount = 2 * quantity;
    byte request[13 + byteCount] = {
        0x00, 0x01,       // Transaction Identifier
        0x00, 0x00,       // Protocol Identifier
        (7 + byteCount) >> 8, (7 + byteCount) & 0xFF, // Length
        0x01,             // Unit Identifier
        0x10,             // Function Code (Write Multiple Registers)
        (address >> 8), (address & 0xFF), // Starting Address
        (quantity >> 8), (quantity & 0xFF), // Quantity of Registers
        byteCount         // Byte Count
    };

    for (int i = 0; i < quantity; i++) {
        request[13 + 2 * i] = (values[i] >> 8) & 0xFF;
        request[14 + 2 * i] = values[i] & 0xFF;
    }

    byte response[12];
    return sendModbusRequest(request, sizeof(request), response, sizeof(response));
}
