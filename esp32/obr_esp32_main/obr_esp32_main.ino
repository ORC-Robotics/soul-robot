// Firmware principal da ESP32 usado junto com a Raspberry Pi.
// Este modo exclui Wi-Fi, servidor HTTP e dashboard durante a compilação.
#define OBR_ESP32_RASPBERRY_MODE

// O núcleo de hardware é compartilhado com o firmware de bancada para que
// pinos, filtros, sensores e proteções de motor permaneçam idênticos.
#include "../obr_esp32_bridge/obr_esp32_bridge.ino"
