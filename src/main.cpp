/**
 * Elegoo Canvas RS485 Monitor + Full Decoder
 * Target : Arduino Mega 2560  (PlatformIO / avr framework)
 * Libraries: HardwareSerial (built-in) — nothing to add to lib_deps.
 *
 * Wiring
 * ──────
 *  Waveshare RS485 module
 *    TXD → Pin 19 (RX1)
 *    VCC → 5V
 *    GND → GND
 *    A/B → RS485 bus
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// Config
// ═══════════════════════════════════════════════════════════════════════════
static constexpr uint32_t RS485_BAUD = 115200;
static constexpr uint32_t DEBUG_BAUD = 115200;
static constexpr uint32_t CHAR_US    = (10UL * 1000000UL) / RS485_BAUD;
static constexpr uint32_t IFG_US     = (uint32_t)(3.5f * CHAR_US);

// ═══════════════════════════════════════════════════════════════════════════
// Protocol constants  (canvas_dev.h)
// ═══════════════════════════════════════════════════════════════════════════

// Frame header / flags
static constexpr uint8_t FRAME_HEAD  = 0x3D;
static constexpr uint8_t FRAME_SHORT = 0x80;
static constexpr uint8_t FRAME_LONG  = 0x00;

// Device ID
static constexpr uint8_t CANVAS_LITE_DEVICE_ID = 0x00;

// Commands
static constexpr uint8_t CMD_CONNECT_STATUS      = 0x00;
static constexpr uint8_t CMD_HARDWARE_VERSION    = 0x01;
static constexpr uint8_t CMD_SOFTWARE_VERSION    = 0x02;
static constexpr uint8_t CMD_FILAMENT_PLUG_IN    = 0x04;
static constexpr uint8_t CMD_ODO_VALUE           = 0x08;
static constexpr uint8_t CMD_ENCODER_STATUS      = 0x10;
static constexpr uint8_t CMD_MOTOR_STATUS        = 0x20;
static constexpr uint8_t CMD_RFID_DATA           = 0x40;
static constexpr uint8_t CMD_ALL_STATUS          = 0x7F;
static constexpr uint8_t CMD_LED_CONTROL         = 0x81;
static constexpr uint8_t CMD_BEEPER_CONTROL      = 0x82;
static constexpr uint8_t CMD_FEEDER_STOP         = 0x90;
static constexpr uint8_t CMD_FILA_CTRL           = 0x91;
static constexpr uint8_t CMD_FILA_CTRL_SINGLE    = 0x92;
static constexpr uint8_t CMD_FILA_CTRL_REACH_ACK = 0x93;
static constexpr uint8_t CMD_FILA_CTRL_BOTH_ACK  = 0x94;

// Respond codes
static constexpr uint8_t RESPOND_OK          = 0x00;
static constexpr uint8_t RESPOND_POS_REACHED = 0x01;
static constexpr uint8_t RESPOND_ERROR       = 0x80;

// Channel count
static constexpr uint8_t CHANNEL_NUM = 4;

// ── Filament material types (canvas_dev.h — uint32 ASCII codes) ────────────
static constexpr uint32_t FTYPE_PLA  = 0x00807665;
static constexpr uint32_t FTYPE_PETG = 0x80698471;
static constexpr uint32_t FTYPE_ABS  = 0x00656683;
static constexpr uint32_t FTYPE_TPU  = 0x00848085;
static constexpr uint32_t FTYPE_PA   = 0x00008065;
static constexpr uint32_t FTYPE_CPE  = 0x00678069;
static constexpr uint32_t FTYPE_PC   = 0x00008067;
static constexpr uint32_t FTYPE_PVA  = 0x00808665;
static constexpr uint32_t FTYPE_ASA  = 0x00658365;
static constexpr uint32_t FTYPE_BVOH = 0x42564F48;
static constexpr uint32_t FTYPE_EVA  = 0x00455641;
static constexpr uint32_t FTYPE_HIPS = 0x48495053;
static constexpr uint32_t FTYPE_PP   = 0x00005050;
static constexpr uint32_t FTYPE_PPA  = 0x00505041;

// ── Filament detail/name codes (canvas_dev.h — uint16) ────────────────────
static constexpr uint16_t FNAME_PLA         = 0x0000;
static constexpr uint16_t FNAME_PLA_Plus    = 0x0001;
static constexpr uint16_t FNAME_PLA_Hyper   = 0x0002;
static constexpr uint16_t FNAME_PLA_Silk    = 0x0003;
static constexpr uint16_t FNAME_PLA_CF      = 0x0004;
static constexpr uint16_t FNAME_PLA_Carbon  = 0x0005;
static constexpr uint16_t FNAME_PLA_Matte   = 0x0006;
static constexpr uint16_t FNAME_PLA_Fluo    = 0x0007;
static constexpr uint16_t FNAME_PLA_Wood    = 0x0008;
static constexpr uint16_t FNAME_PETG        = 0x0100;
static constexpr uint16_t FNAME_PETG_CF     = 0x0101;
static constexpr uint16_t FNAME_PETG_Hyper  = 0x0102;
static constexpr uint16_t FNAME_ABS         = 0x0200;
static constexpr uint16_t FNAME_ABS_Hyper   = 0x0201;
static constexpr uint16_t FNAME_TPU         = 0x0300;
static constexpr uint16_t FNAME_TPU_Hyper   = 0x0301;
static constexpr uint16_t FNAME_PA          = 0x0400;
static constexpr uint16_t FNAME_PA_CF       = 0x0401;
static constexpr uint16_t FNAME_PA_Hyper    = 0x0402;
static constexpr uint16_t FNAME_CPE         = 0x0500;
static constexpr uint16_t FNAME_CPE_Hyper   = 0x0501;
static constexpr uint16_t FNAME_PC          = 0x0600;
static constexpr uint16_t FNAME_PC_PCTG     = 0x0601;
static constexpr uint16_t FNAME_PC_Hyper    = 0x0602;
static constexpr uint16_t FNAME_PVA         = 0x0700;
static constexpr uint16_t FNAME_PVA_Hyper   = 0x0701;
static constexpr uint16_t FNAME_ASA         = 0x0800;
static constexpr uint16_t FNAME_ASA_Hyper   = 0x0801;
static constexpr uint16_t FNAME_BVOH        = 0x0900;
static constexpr uint16_t FNAME_EVA         = 0x0A00;
static constexpr uint16_t FNAME_HIPS        = 0x0B00;
static constexpr uint16_t FNAME_PP          = 0x0C00;
static constexpr uint16_t FNAME_PP_CF       = 0x0C01;
static constexpr uint16_t FNAME_PP_GF       = 0x0C02;
static constexpr uint16_t FNAME_PPA         = 0x0D00;
static constexpr uint16_t FNAME_PPA_CF      = 0x0D01;
static constexpr uint16_t FNAME_PPA_GF      = 0x0D02;

// ── Known brand code ────────────────────────────────────────────────────────
static constexpr uint32_t BRAND_ELEGOO = 0xEEEEEEEE;

// ── Filament transparency / fan constants (canvas_dev.h) ───────────────────
static constexpr uint8_t TRANSPARENCY_TRANSPARENT = 0x00;
static constexpr uint8_t TRANSPARENCY_OPAQUE       = 0xFF;
static constexpr uint8_t FAN_OFF = 0x00;
static constexpr uint8_t FAN_ON  = 0xFF;

// ── RFID / NTAG sizes (canvas_dev.h) ───────────────────────────────────────
static constexpr uint8_t NTAG_BYTES_PER_PAGE = 4;
static constexpr uint8_t NTAG_END_PAGES      = 0x2C;
static constexpr uint8_t NTAG_MAX_PAGES      = NTAG_END_PAGES + 1;
// sizeof(RFID) = 8 (uid) + NTAG_BYTES_PER_PAGE * NTAG_MAX_PAGES
static constexpr uint16_t RFID_DATA_SIZE = 8 + (uint16_t)NTAG_BYTES_PER_PAGE * NTAG_MAX_PAGES;

// ── Filament struct layout (canvas_dev.h, packed, big-endian on wire) ──────
// Offset  Size  Field
//  0       1    header      (0x36 = EPC-256 identifier)
//  1       4    brand       (0xEEEEEEEE = ELEGOO)
//  5       2    code        (internal filament code)
//  7       4    type        (ASCII material e.g. PLA)
// 11       2    name        (detail type code)
// 13       2    date        (YYMM BCD, high byte = YY, low byte = MM)
// 15       3    rgb[3]      (RGB888 colour)
// 18       1    transparency
// 19       2    low_tmp     (min nozzle temp °C)
// 21       2    hig_tmp     (max nozzle temp °C)
// 23       1    low_bed_tmp
// 24       1    hig_bed_tmp
// 25       1    hig_dry_tmp
// 26       1    max_dry_time (hours)
// 27       2    diameter    (÷100 → mm, e.g. 175 = 1.75mm)
// 29       2    weight      (grams)
// 31       1    fan_ctrl
// Total = 32 bytes

// ═══════════════════════════════════════════════════════════════════════════
// CRC  (canvas_dev.cpp)
// ═══════════════════════════════════════════════════════════════════════════
static constexpr uint8_t  CRC8_POLY  = 0x39;
static constexpr uint8_t  CRC8_INIT  = 0x66;
static constexpr uint16_t CRC16_POLY = 0x1021;
static constexpr uint16_t CRC16_INIT = 0x913D;

static uint8_t crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = CRC8_INIT;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ CRC8_POLY : crc << 1;
    }
    return crc;
}

static uint16_t crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = CRC16_INIT;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ CRC16_POLY : crc << 1;
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════
// Lookup helpers
// ═══════════════════════════════════════════════════════════════════════════
static const __FlashStringHelper* filamentTypeName(uint32_t t)
{
    switch (t) {
        case FTYPE_PLA:  return F("PLA");
        case FTYPE_PETG: return F("PETG");
        case FTYPE_ABS:  return F("ABS");
        case FTYPE_TPU:  return F("TPU");
        case FTYPE_PA:   return F("PA");
        case FTYPE_CPE:  return F("CPE");
        case FTYPE_PC:   return F("PC");
        case FTYPE_PVA:  return F("PVA");
        case FTYPE_ASA:  return F("ASA");
        case FTYPE_BVOH: return F("BVOH");
        case FTYPE_EVA:  return F("EVA");
        case FTYPE_HIPS: return F("HIPS");
        case FTYPE_PP:   return F("PP");
        case FTYPE_PPA:  return F("PPA");
        default:         return F("UNKNOWN");
    }
}

static const __FlashStringHelper* filamentDetailName(uint16_t n)
{
    switch (n) {
        case FNAME_PLA:        return F("PLA");
        case FNAME_PLA_Plus:   return F("PLA+");
        case FNAME_PLA_Hyper:  return F("PLA Hyper");
        case FNAME_PLA_Silk:   return F("PLA Silk");
        case FNAME_PLA_CF:     return F("PLA CF");
        case FNAME_PLA_Carbon: return F("PLA Carbon");
        case FNAME_PLA_Matte:  return F("PLA Matte");
        case FNAME_PLA_Fluo:   return F("PLA Fluo");
        case FNAME_PLA_Wood:   return F("PLA Wood");
        case FNAME_PETG:       return F("PETG");
        case FNAME_PETG_CF:    return F("PETG CF");
        case FNAME_PETG_Hyper: return F("PETG Hyper");
        case FNAME_ABS:        return F("ABS");
        case FNAME_ABS_Hyper:  return F("ABS Hyper");
        case FNAME_TPU:        return F("TPU");
        case FNAME_TPU_Hyper:  return F("TPU Hyper");
        case FNAME_PA:         return F("PA");
        case FNAME_PA_CF:      return F("PA CF");
        case FNAME_PA_Hyper:   return F("PA Hyper");
        case FNAME_CPE:        return F("CPE");
        case FNAME_CPE_Hyper:  return F("CPE Hyper");
        case FNAME_PC:         return F("PC");
        case FNAME_PC_PCTG:    return F("PC PCTG");
        case FNAME_PC_Hyper:   return F("PC Hyper");
        case FNAME_PVA:        return F("PVA");
        case FNAME_PVA_Hyper:  return F("PVA Hyper");
        case FNAME_ASA:        return F("ASA");
        case FNAME_ASA_Hyper:  return F("ASA Hyper");
        case FNAME_BVOH:       return F("BVOH");
        case FNAME_EVA:        return F("EVA");
        case FNAME_HIPS:       return F("HIPS");
        case FNAME_PP:         return F("PP");
        case FNAME_PP_CF:      return F("PP CF");
        case FNAME_PP_GF:      return F("PP GF");
        case FNAME_PPA:        return F("PPA");
        case FNAME_PPA_CF:     return F("PPA CF");
        case FNAME_PPA_GF:     return F("PPA GF");
        default:               return F("UNKNOWN");
    }
}

static const __FlashStringHelper* cmdName(uint8_t cmd)
{
    switch (cmd) {
        case CMD_CONNECT_STATUS:      return F("CONNECT_STATUS");
        case CMD_HARDWARE_VERSION:    return F("HARDWARE_VERSION");
        case CMD_SOFTWARE_VERSION:    return F("SOFTWARE_VERSION");
        case CMD_FILAMENT_PLUG_IN:    return F("FILAMENT_PLUG_IN");
        case CMD_ODO_VALUE:           return F("ODO_VALUE");
        case CMD_ENCODER_STATUS:      return F("ENCODER_STATUS");
        case CMD_MOTOR_STATUS:        return F("MOTOR_STATUS");
        case CMD_RFID_DATA:           return F("RFID_DATA");
        case CMD_ALL_STATUS:          return F("ALL_STATUS");
        case CMD_LED_CONTROL:         return F("LED_CONTROL");
        case CMD_BEEPER_CONTROL:      return F("BEEPER_CONTROL");
        case CMD_FEEDER_STOP:         return F("FEEDER_STOP");
        case CMD_FILA_CTRL:           return F("FILA_CTRL");
        case CMD_FILA_CTRL_SINGLE:    return F("FILA_CTRL_SINGLE");
        case CMD_FILA_CTRL_REACH_ACK: return F("FILA_CTRL_REACH_ACK");
        case CMD_FILA_CTRL_BOTH_ACK:  return F("FILA_CTRL_BOTH_ACK");
        default:                      return F("UNKNOWN");
    }
}

static const __FlashStringHelper* respondName(uint8_t r)
{
    switch (r) {
        case RESPOND_OK:          return F("OK");
        case RESPOND_POS_REACHED: return F("POS_REACHED");
        case RESPOND_ERROR:       return F("ERROR");
        default:                  return F("?");
    }
}

static const char* ledStateName(uint8_t s)
{
    switch (s) {
        case 0: return "OFF";
        case 1: return "ON";
        case 2: return "BLINK_1Hz";
        case 3: return "BREATHE";
        case 4: return "BLINK_2Hz";
        default: return "?";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Hex print helper
// ═══════════════════════════════════════════════════════════════════════════
static void printHex(const uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        if (i < len - 1) Serial.print(' ');
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Filament struct decoder
// Parses the 32-byte packed Filament struct from RFID tag data
// (canvas_dev.h struct Filament, canvas_dev.cpp get_cavans_rfid_info())
// ═══════════════════════════════════════════════════════════════════════════
static void decodeFilamentStruct(const uint8_t* f)
{
    // [0] header
    Serial.print(F("\n    header=0x"));
    if (f[0] < 0x10) Serial.print('0');
    Serial.print(f[0], HEX);
    Serial.print(f[0] == 0x36 ? F(" (EPC-256)") : F(" (unknown)"));

    // [1..4] brand (big-endian uint32)
    uint32_t brand = ((uint32_t)f[1] << 24) | ((uint32_t)f[2] << 16) |
                     ((uint32_t)f[3] <<  8) |  (uint32_t)f[4];
    Serial.print(F("\n    brand=0x"));
    Serial.print(brand, HEX);
    if (brand == BRAND_ELEGOO) Serial.print(F(" (ELEGOO)"));

    // [5..6] code (big-endian uint16)
    uint16_t code = ((uint16_t)f[5] << 8) | f[6];
    Serial.print(F("\n    code=0x"));
    if (code < 0x10) Serial.print('0');
    Serial.print(code, HEX);

    // [7..10] material type (big-endian uint32)
    uint32_t type = ((uint32_t)f[7] << 24) | ((uint32_t)f[8] << 16) |
                    ((uint32_t)f[9] <<  8) |  (uint32_t)f[10];
    Serial.print(F("\n    material="));
    Serial.print(filamentTypeName(type));
    Serial.print(F(" (0x")); Serial.print(type, HEX); Serial.print(')');

    // [11..12] detail name (big-endian uint16)
    uint16_t name = ((uint16_t)f[11] << 8) | f[12];
    Serial.print(F("\n    detail="));
    Serial.print(filamentDetailName(name));
    Serial.print(F(" (0x"));
    if (name < 0x10) Serial.print('0');
    Serial.print(name, HEX); Serial.print(')');

    // [13..14] date YYMM (high=YY, low=MM)
    uint8_t yy = f[13], mm = f[14];
    Serial.print(F("\n    date=20"));
    if (yy < 10) Serial.print('0'); Serial.print(yy);
    Serial.print('/');
    if (mm < 10) Serial.print('0'); Serial.print(mm);

    // [15..17] RGB colour
    Serial.print(F("\n    color=#"));
    if (f[15] < 0x10) Serial.print('0'); Serial.print(f[15], HEX);
    if (f[16] < 0x10) Serial.print('0'); Serial.print(f[16], HEX);
    if (f[17] < 0x10) Serial.print('0'); Serial.print(f[17], HEX);

    // [18] transparency
    Serial.print(F("\n    transparency=0x"));
    if (f[18] < 0x10) Serial.print('0');
    Serial.print(f[18], HEX);
    if      (f[18] == TRANSPARENCY_TRANSPARENT) Serial.print(F(" (transparent)"));
    else if (f[18] == TRANSPARENCY_OPAQUE)       Serial.print(F(" (opaque)"));

    // [19..20] min nozzle temp
    uint16_t low_tmp = ((uint16_t)f[19] << 8) | f[20];
    Serial.print(F("\n    nozzle_min="));
    Serial.print(low_tmp); Serial.print(F("°C"));

    // [21..22] max nozzle temp
    uint16_t hig_tmp = ((uint16_t)f[21] << 8) | f[22];
    Serial.print(F("  nozzle_max="));
    Serial.print(hig_tmp); Serial.print(F("°C"));

    // [23] min bed temp
    Serial.print(F("\n    bed_min="));
    Serial.print(f[23]); Serial.print(F("°C"));

    // [24] max bed temp
    Serial.print(F("  bed_max="));
    Serial.print(f[24]); Serial.print(F("°C"));

    // [25] max dry temp
    Serial.print(F("\n    dry_max="));
    Serial.print(f[25]); Serial.print(F("°C"));

    // [26] max dry time (hours)
    Serial.print(F("  dry_time="));
    Serial.print(f[26]); Serial.print(F("h"));

    // [27..28] diameter (÷100 mm)
    uint16_t diam = ((uint16_t)f[27] << 8) | f[28];
    Serial.print(F("\n    diameter="));
    Serial.print(diam / 100); Serial.print('.');
    uint8_t dec = diam % 100;
    if (dec < 10) Serial.print('0');
    Serial.print(dec); Serial.print(F("mm"));

    // [29..30] weight (grams)
    uint16_t weight = ((uint16_t)f[29] << 8) | f[30];
    Serial.print(F("  weight="));
    Serial.print(weight); Serial.print(F("g"));

    // [31] fan control
    Serial.print(F("\n    fan="));
    if      (f[31] == FAN_OFF) Serial.print(F("OFF"));
    else if (f[31] == FAN_ON)  Serial.print(F("ON"));
    else { Serial.print(F("0x")); Serial.print(f[31], HEX); }
}

// ═══════════════════════════════════════════════════════════════════════════
// Payload decoder
// payload[] = bytes after [device_id, cmd]
// payLen    = number of those bytes (0 = query frame with no data)
// ═══════════════════════════════════════════════════════════════════════════
static void decodePayload(uint8_t cmd, const uint8_t* p, uint8_t pLen, bool isQuery)
{
    if (isQuery) {
        Serial.print(F("  [query — no payload]"));
        return;
    }

    switch (cmd) {

    // ── 0x00  CONNECT_STATUS ─────────────────────────────────────────────
    case CMD_CONNECT_STATUS:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        break;

    // ── 0x01  HARDWARE_VERSION ───────────────────────────────────────────
    // ── 0x02  SOFTWARE_VERSION ───────────────────────────────────────────
    // Response: [respond] [11 ASCII version bytes]
    case CMD_HARDWARE_VERSION:
    case CMD_SOFTWARE_VERSION:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 2) {
            Serial.print(F("  version=\""));
            for (uint8_t i = 1; i < pLen; i++) {
                if (p[i] >= 0x20 && p[i] < 0x7F) Serial.print((char)p[i]);
            }
            Serial.print('"');
        }
        break;

    // ── 0x04  FILAMENT_PLUG_IN ───────────────────────────────────────────
    // Response: [respond] [plug_status bitmask ch4..ch1]
    case CMD_FILAMENT_PLUG_IN:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 2) {
            Serial.print(F("  plug_status=0x"));
            if (p[1] < 0x10) Serial.print('0');
            Serial.print(p[1], HEX);
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                Serial.print(F("  ch")); Serial.print(ch + 1); Serial.print('=');
                Serial.print((p[1] >> ch) & 0x01 ? F("IN") : F("OUT"));
            }
        }
        break;

    // ── 0x08  ODO_VALUE ──────────────────────────────────────────────────
    // Response: [respond] then per-channel int32 odo values
    case CMD_ODO_VALUE:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 1 + CHANNEL_NUM * 4) {
            uint8_t idx = 1;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int32_t val = ((int32_t)p[idx] << 24) | ((int32_t)p[idx+1] << 16) |
                              ((int32_t)p[idx+2] << 8) | p[idx+3];
                idx += 4;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(" odo=")); Serial.print(val);
            }
        }
        break;

    // ── 0x10  ENCODER_STATUS ─────────────────────────────────────────────
    // Response: [respond] then per-channel int32 encoder values
    case CMD_ENCODER_STATUS:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 1 + CHANNEL_NUM * 4) {
            uint8_t idx = 1;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int32_t val = ((int32_t)p[idx] << 24) | ((int32_t)p[idx+1] << 16) |
                              ((int32_t)p[idx+2] << 8) | p[idx+3];
                idx += 4;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(" encoder=")); Serial.print(val);
            }
        }
        break;

    // ── 0x20  MOTOR_STATUS ───────────────────────────────────────────────
    case CMD_MOTOR_STATUS:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 2) {
            Serial.print(F("  motor_status=0x"));
            if (p[1] < 0x10) Serial.print('0');
            Serial.print(p[1], HEX);
        }
        break;

    // ── 0x40  RFID_DATA ──────────────────────────────────────────────────
    // Response: [respond] [has_data] [uid 8 bytes] [tag data pages]
    // Filament struct lives at offset 16 of the tag data (rfid.data[16])
    case CMD_RFID_DATA:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 2) {
            bool hasData = p[1] != 0;
            Serial.print(F("  has_data="));
            Serial.print(hasData ? F("YES") : F("NO"));
        }
        if (pLen >= 3) {
            // UID: 8 bytes big-endian
            Serial.print(F("\n    uid=0x"));
            for (uint8_t i = 2; i < 10 && i < pLen; i++) {
                if (p[i] < 0x10) Serial.print('0');
                Serial.print(p[i], HEX);
            }
        }
        // Filament data starts at p[3 + 16] = p[19]
        // (respond + has_data + 8 uid bytes = offset 10, then data[0..15] = 16 padding = p[10+16]=p[26])
        // From canvas_dev.cpp: memcpy(&rfid_, ptr, sizeof(RFID))
        // ptr = rx_data.data() + 3  (after id, cmd, has_data)
        // RFID = { uint64_t uid; uint8_t data[NTAG_BYTES_PER_PAGE * NTAG_MAX_PAGES]; }
        // filament is at rfid.data[16], so offset from p[0]:
        //   p[0]=respond, p[1]=has_data, p[2..9]=uid(8), p[10..]=tag data pages
        //   filament at p[10 + 16] = p[26]
        if (pLen >= 26 + 32) {
            Serial.print(F("\n    --- Filament Tag ---"));
            decodeFilamentStruct(p + 26);
        }
        break;

    // ── 0x7F  ALL_STATUS ─────────────────────────────────────────────────
    // Response layout (canvas_dev.cpp read_feeders_status()):
    //   rx_data[0] = device_id  (stripped by our unpack)
    //   rx_data[1] = cmd        (stripped)
    //   rx_data[2] = overall status byte       → p[0]
    //   rx_data[3 + i*9 + 0] = feeder[i].status
    //   rx_data[3 + i*9 + 1..4] = odo_value32 (big-endian)
    //   rx_data[3 + i*9 + 5..8] = encoder_value32 (big-endian)
    case CMD_ALL_STATUS:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        if (pLen >= 2) {
            Serial.print(F("  overall_status=0x"));
            if (p[1] < 0x10) Serial.print('0');
            Serial.print(p[1], HEX);
        }
        if (pLen >= 2 + (uint8_t)(CHANNEL_NUM * 9)) {
            uint8_t idx = 2;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                uint8_t  st  = p[idx];
                int32_t odo  = ((int32_t)p[idx+1] << 24) | ((int32_t)p[idx+2] << 16) |
                               ((int32_t)p[idx+3] <<  8) |  (int32_t)p[idx+4];
                int32_t enc  = ((int32_t)p[idx+5] << 24) | ((int32_t)p[idx+6] << 16) |
                               ((int32_t)p[idx+7] <<  8) |  (int32_t)p[idx+8];
                idx += 9;
                Serial.print(F("\n    ch")); Serial.print(ch + 1); Serial.print(F(": "));
                Serial.print(F("fila_in="));          Serial.print((st >> 1) & 1);
                Serial.print(F(" dragging="));         Serial.print((st >> 6) & 1);
                Serial.print(F(" stalled="));          Serial.print((st >> 3) & 1);
                Serial.print(F(" abnormal="));         Serial.print((st >> 7) & 1);
                Serial.print(F(" compensate_err="));   Serial.print((st >> 4) & 1);
                Serial.print(F(" motor_ctrl_err="));   Serial.print((st >> 2) & 1);
                Serial.print(F(" motor_fault="));      Serial.print((st >> 0) & 1);
                Serial.print(F(" odo="));              Serial.print(odo);
                Serial.print(F(" enc="));              Serial.print(enc);
            }
        }
        break;

    // ── 0x81  LED_CONTROL ────────────────────────────────────────────────
    // TX: [enable_mask] [ch0_red] [ch0_blue] [ch1_red] [ch1_blue] ...
    // RX: [respond]
    // We decode both directions — if pLen==1 it's a response, else command
    case CMD_LED_CONTROL:
        if (pLen == 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        } else if (pLen >= 1 + CHANNEL_NUM * 2) {
            uint8_t enable = p[0];
            Serial.print(F("  enable_mask=0b"));
            for (int8_t b = 3; b >= 0; b--)
                Serial.print((enable >> b) & 1);
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                uint8_t red  = p[1 + ch * 2];
                uint8_t blue = p[2 + ch * 2];
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(": red=")); Serial.print(ledStateName(red));
                Serial.print(F(" blue="));  Serial.print(ledStateName(blue));
            }
        }
        break;

    // ── 0x82  BEEPER_CONTROL ─────────────────────────────────────────────
    // TX: [times] [duration_ms_H] [duration_ms_L]
    // RX: [respond]
    case CMD_BEEPER_CONTROL:
        if (pLen == 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        } else if (pLen >= 3) {
            uint16_t dur = ((uint16_t)p[1] << 8) | p[2];
            Serial.print(F("  times=")); Serial.print(p[0]);
            Serial.print(F("  duration_ms=")); Serial.print(dur);
        }
        break;

    // ── 0x90  FEEDER_STOP ────────────────────────────────────────────────
    // TX: [channels_bitmask]  bit3..0 = ch4..ch1
    // RX: [respond]
    case CMD_FEEDER_STOP:
        if (pLen == 1 && (p[0] == RESPOND_OK || p[0] == RESPOND_ERROR)) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        } else if (pLen >= 1) {
            Serial.print(F("  stop_channels=0b"));
            for (int8_t b = 3; b >= 0; b--)
                Serial.print((p[0] >> b) & 1);
            Serial.print(F(" ["));
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++)
                if ((p[0] >> ch) & 1) { Serial.print(F("ch")); Serial.print(ch+1); Serial.print(' '); }
            Serial.print(']');
        }
        break;

    // ── 0x91  FILA_CTRL  (all 4 channels) ────────────────────────────────
    // TX: 4 × [mm_H mm_L mm_s_H mm_s_L mm_ss_H mm_ss_L]
    // RX: [respond]
    case CMD_FILA_CTRL:
        if (pLen == 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        } else if (pLen >= CHANNEL_NUM * 6) {
            uint8_t idx = 0;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int16_t mm    = ((int16_t)p[idx+0] << 8) | p[idx+1];
                int16_t mm_s  = ((int16_t)p[idx+2] << 8) | p[idx+3];
                int16_t mm_ss = ((int16_t)p[idx+4] << 8) | p[idx+5];
                idx += 6;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(": mm=")); Serial.print(mm);
                Serial.print(F(" mm/s=")); Serial.print(mm_s);
                Serial.print(F(" mm/s²=")); Serial.print(mm_ss);
            }
        }
        break;

    // ── 0x92  FILA_CTRL_SINGLE  (single channel) ─────────────────────────
    // TX: [ch] [mm_H mm_L] [mm_s_H mm_s_L] [mm_ss_H mm_ss_L]
    // RX: [respond]
    case CMD_FILA_CTRL_SINGLE:
        if (pLen == 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        } else if (pLen >= 7) {
            int16_t mm    = ((int16_t)p[1] << 8) | p[2];
            int16_t mm_s  = ((int16_t)p[3] << 8) | p[4];
            int16_t mm_ss = ((int16_t)p[5] << 8) | p[6];
            Serial.print(F("  ch=")); Serial.print(p[0]);
            Serial.print(F("  mm=")); Serial.print(mm);
            Serial.print(F("  mm/s=")); Serial.print(mm_s);
            Serial.print(F("  mm/s²=")); Serial.print(mm_ss);
        }
        break;

    // ── 0x93  FILA_CTRL_REACH_ACK ────────────────────────────────────────
    // ── 0x94  FILA_CTRL_BOTH_ACK  ────────────────────────────────────────
    // RX: [respond]
    case CMD_FILA_CTRL_REACH_ACK:
    case CMD_FILA_CTRL_BOTH_ACK:
        if (pLen >= 1) {
            Serial.print(F("  respond="));
            Serial.print(respondName(p[0]));
        }
        break;

    default:
        Serial.print(F("  [unknown cmd — raw payload] "));
        printHex(p, pLen);
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Frame decoder
// ═══════════════════════════════════════════════════════════════════════════
static void decodeFrame(const uint8_t* frame, uint16_t len)
{
    Serial.println(F("────────────────────────────────────────"));
    Serial.print(F("RAW (")); Serial.print(len); Serial.print(F("): "));
    printHex(frame, len);
    Serial.println();

    if (len < 6 || frame[0] != FRAME_HEAD) {
        Serial.println(F("  [bad/short frame]"));
        return;
    }

    uint8_t flag = frame[1];

    // ── SHORT FRAME ──────────────────────────────────────────────────────
    if (flag == FRAME_SHORT) {
        uint8_t flen = frame[2];
        if (len < flen) { Serial.println(F("  [incomplete]")); return; }
        if (crc8(frame, 3) != frame[3])         { Serial.println(F("  [CRC8 FAIL]")); return; }
        uint16_t c16r = ((uint16_t)frame[flen-2] << 8) | frame[flen-1];
        if (crc16(frame, flen-2) != c16r)       { Serial.println(F("  [CRC16 FAIL]")); return; }

        uint8_t devId  = frame[4];
        uint8_t cmd    = frame[5];
        // payload = frame[6 .. flen-3],  2 bytes CRC16 at end
        const uint8_t* payload = frame + 6;
        uint8_t payLen = (flen >= 8) ? flen - 8 : 0;
        bool isQuery   = (payLen == 0);

        Serial.print(F("TYPE: SHORT  devId=0x"));
        if (devId < 0x10) Serial.print('0'); Serial.print(devId, HEX);
        Serial.print(devId == CANVAS_LITE_DEVICE_ID ? F(" (Canvas Lite)") : F(""));
        Serial.print(F("  cmd=0x"));
        if (cmd < 0x10) Serial.print('0'); Serial.print(cmd, HEX);
        Serial.print(F(" (")); Serial.print(cmdName(cmd)); Serial.print(F(")  "));
        Serial.println(isQuery ? F("QUERY") : F("RESPONSE"));

        decodePayload(cmd, payload, payLen, isQuery);
        Serial.println();
    }

    // ── LONG FRAME ───────────────────────────────────────────────────────
    else if (flag == FRAME_LONG) {
        if (len < 9) { Serial.println(F("  [too short]")); return; }
        uint16_t seq   = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t flen  = ((uint16_t)frame[4] << 8) | frame[5];
        if (len < flen) { Serial.println(F("  [incomplete]")); return; }
        if (crc8(frame, 6) != frame[6])              { Serial.println(F("  [CRC8 FAIL]")); return; }
        uint16_t c16r = ((uint16_t)frame[flen-2] << 8) | frame[flen-1];
        if (crc16(frame, flen-2) != c16r)            { Serial.println(F("  [CRC16 FAIL]")); return; }

        const uint8_t* payload = frame + 7;
        uint8_t payLen = (flen >= 9) ? flen - 9 : 0;

        Serial.print(F("TYPE: LONG  seq=")); Serial.print(seq);
        Serial.print(F("  payLen=")); Serial.println(payLen);
        if (payLen > 0) {
            Serial.print(F("  payload: "));
            printHex(payload, payLen);
            Serial.println();
        }
    }

    else {
        Serial.println(F("  [unknown frame type]"));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RX state
// ═══════════════════════════════════════════════════════════════════════════
static uint8_t  rxBuf[256];
static uint16_t rxLen      = 0;
static uint32_t lastByteUs = 0;
static bool     active     = false;

// ═══════════════════════════════════════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(DEBUG_BAUD);
    Serial1.begin(RS485_BAUD);
    Serial.println(F("Elegoo Canvas RS485 Decoder — full protocol"));
    Serial.print(F("Baud: ")); Serial.print(RS485_BAUD);
    Serial.print(F("  IFG: ")); Serial.print(IFG_US); Serial.println(F("µs"));
}

void loop()
{
    while (Serial1.available()) {
        uint8_t b  = (uint8_t)Serial1.read();
        lastByteUs = micros();
        active     = true;

        if (rxLen < sizeof(rxBuf)) {
            rxBuf[rxLen++] = b;
        } else {
            Serial.println(F("[overflow — flushing]"));
            decodeFrame(rxBuf, rxLen);
            rxLen = 0;
            rxBuf[rxLen++] = b;
        }
    }

    if (active && (micros() - lastByteUs) >= IFG_US) {
        decodeFrame(rxBuf, rxLen);
        rxLen  = 0;
        active = false;
    }
}