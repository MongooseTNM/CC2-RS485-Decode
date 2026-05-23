/**
 * Elegoo Canvas RS485 Monitor + Decoder + Limit Switch Injector
 * Target : Arduino Mega 2560  (PlatformIO / avr framework)
 * Libraries: HardwareSerial (built-in) — nothing to add to lib_deps.
 *
 * Wiring (MAX485)
 * ──────
 *  MAX485 pin 1 (RO)  → Pin 19 (RX1)
 *  MAX485 pin 2 (/RE) → Pin 3  (separate RE control)
 *  MAX485 pin 3 (DE)  → Pin 2
 *  MAX485 pin 4 (DI)  → Pin 18 (TX1)
 *  MAX485 pin 5 (GND) → GND
 *  MAX485 pin 6 (B)   → RS485 B
 *  MAX485 pin 7 (A)   → RS485 A
 *  MAX485 pin 8 (VCC) → 5V
 *
 *  Limit switch       → Pin 4 (other leg to GND, uses internal pull-up)
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// Config
// ═══════════════════════════════════════════════════════════════════════════
static constexpr uint32_t RS485_BAUD = 115200;
static constexpr uint32_t DEBUG_BAUD = 115200;

static constexpr uint8_t DE_PIN     = 2;   // MAX485 DE  (HIGH = transmit)
static constexpr uint8_t RE_PIN     = 3;   // MAX485 /RE (LOW  = receive)
static constexpr uint8_t SW_PIN     = 4;   // Limit switch (HIGH when pressed)

// Feed command parameters for limit switch
static constexpr uint8_t  FEED_CHANNEL  = 4;      // Canvas channel 4
static constexpr int16_t  FEED_MM       = 9;       // 1 mm forward
static constexpr int16_t  FEED_MM_S     = 100;      // 35 mm/s
static constexpr int16_t  FEED_MM_SS    = 3000;    // 2500 mm/s²

// Debounce time for limit switch
static constexpr uint32_t DEBOUNCE_MS   = 50;

static constexpr uint32_t CHAR_US = (10UL * 1000000UL) / RS485_BAUD;
static constexpr uint32_t IFG_US  = (uint32_t)(3.5f * CHAR_US);

// ═══════════════════════════════════════════════════════════════════════════
// Protocol constants
// ═══════════════════════════════════════════════════════════════════════════
static constexpr uint8_t FRAME_HEAD  = 0x3D;
static constexpr uint8_t FRAME_SHORT = 0x80;
static constexpr uint8_t FRAME_LONG  = 0x00;

static constexpr uint8_t CANVAS_LITE_DEVICE_ID = 0x00;

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

static constexpr uint8_t RESPOND_OK          = 0x00;
static constexpr uint8_t RESPOND_POS_REACHED = 0x01;
static constexpr uint8_t RESPOND_ERROR       = 0x80;

static constexpr uint8_t CHANNEL_NUM = 4;

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

static constexpr uint32_t BRAND_ELEGOO            = 0xEEEEEEEE;
static constexpr uint8_t  TRANSPARENCY_TRANSPARENT = 0x00;
static constexpr uint8_t  TRANSPARENCY_OPAQUE      = 0xFF;
static constexpr uint8_t  FAN_OFF = 0x00;
static constexpr uint8_t  FAN_ON  = 0xFF;

static constexpr uint8_t  NTAG_BYTES_PER_PAGE = 4;
static constexpr uint8_t  NTAG_END_PAGES      = 0x2C;
static constexpr uint8_t  NTAG_MAX_PAGES      = NTAG_END_PAGES + 1;
static constexpr uint16_t RFID_DATA_SIZE      = 8 + (uint16_t)NTAG_BYTES_PER_PAGE * NTAG_MAX_PAGES;

// ═══════════════════════════════════════════════════════════════════════════
// CRC
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
// Frame builder + transmit
// ═══════════════════════════════════════════════════════════════════════════

// Tracks last RX time for IFG enforcement before TX
static uint32_t lastRxUs = 0;

// Build a short frame from a payload (starting with device_id, cmd, ...)
// Returns total frame length written into outBuf.
static uint8_t buildShortFrame(const uint8_t* payload, uint8_t payLen, uint8_t* outBuf)
{
    // Layout:
    //  [0]      FRAME_HEAD  (0x3D)
    //  [1]      FRAME_SHORT (0x80)
    //  [2]      totalLen    = 4 header + payLen + 2 CRC16
    //  [3]      CRC8 of [0..2]
    //  [4..4+payLen-1]  payload (device_id, cmd, data...)
    //  [totalLen-2..totalLen-1]  CRC16 of [0..totalLen-3]

    uint8_t totalLen = 4 + payLen + 2;

    outBuf[0] = FRAME_HEAD;
    outBuf[1] = FRAME_SHORT;
    outBuf[2] = totalLen;
    outBuf[3] = crc8(outBuf, 3);          // CRC8 over header bytes [0..2]

    memcpy(outBuf + 4, payload, payLen);

    uint16_t c16 = crc16(outBuf, totalLen - 2);  // CRC16 over everything except the last 2 bytes
    outBuf[totalLen - 2] = (c16 >> 8) & 0xFF;
    outBuf[totalLen - 1] = (c16 >> 0) & 0xFF;

    return totalLen;
}

static void setTx()
{
    digitalWrite(RE_PIN, HIGH);   // disable receiver
    digitalWrite(DE_PIN, HIGH);   // enable driver
    delayMicroseconds(2);
}

static void setRx()
{
    Serial1.flush();                       // wait for shift register to empty
    delayMicroseconds(CHAR_US + 10);       // one extra char time for last stop bit
    digitalWrite(DE_PIN, LOW);             // disable driver
    digitalWrite(RE_PIN, LOW);             // enable receiver
}

static void transmitFrame(const uint8_t* frame, uint8_t len)
{
    // Wait for bus idle (IFG) before driving
    while ((micros() - lastRxUs) < IFG_US);

    Serial.print(F("[TX] "));
    for (uint8_t i = 0; i < len; i++) {
        if (frame[i] < 0x10) Serial.print('0');
        Serial.print(frame[i], HEX);
        if (i < len - 1) Serial.print(' ');
    }
    Serial.println();

    setTx();
    Serial1.write(frame, len);
    setRx();
}

static void sendFeedSingle(uint8_t ch, int16_t mm, int16_t mm_s, int16_t mm_ss)
{
    // Payload: [device_id] [cmd] [ch] [mm_H] [mm_L] [mm_s_H] [mm_s_L] [mm_ss_H] [mm_ss_L]
    uint8_t payload[9];
    payload[0] = CANVAS_LITE_DEVICE_ID;
    payload[1] = CMD_FILA_CTRL_SINGLE;
    payload[2] = ch;
    payload[3] = (mm    >> 8) & 0xFF;  payload[4] = mm    & 0xFF;
    payload[5] = (mm_s  >> 8) & 0xFF;  payload[6] = mm_s  & 0xFF;
    payload[7] = (mm_ss >> 8) & 0xFF;  payload[8] = mm_ss & 0xFF;

    uint8_t frame[64];
    uint8_t flen = buildShortFrame(payload, sizeof(payload), frame);
    transmitFrame(frame, flen);
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
// ═══════════════════════════════════════════════════════════════════════════
static void decodeFilamentStruct(const uint8_t* f)
{
    Serial.print(F("\n    header=0x"));
    if (f[0] < 0x10) Serial.print('0'); Serial.print(f[0], HEX);
    Serial.print(f[0] == 0x36 ? F(" (EPC-256)") : F(" (unknown)"));

    uint32_t brand = ((uint32_t)f[1]<<24)|((uint32_t)f[2]<<16)|((uint32_t)f[3]<<8)|(uint32_t)f[4];
    Serial.print(F("\n    brand=0x")); Serial.print(brand, HEX);
    if (brand == BRAND_ELEGOO) Serial.print(F(" (ELEGOO)"));

    uint16_t code = ((uint16_t)f[5]<<8)|f[6];
    Serial.print(F("\n    code=0x"));
    if (code < 0x10) Serial.print('0'); Serial.print(code, HEX);

    uint32_t type = ((uint32_t)f[7]<<24)|((uint32_t)f[8]<<16)|((uint32_t)f[9]<<8)|(uint32_t)f[10];
    Serial.print(F("\n    material=")); Serial.print(filamentTypeName(type));
    Serial.print(F(" (0x")); Serial.print(type, HEX); Serial.print(')');

    uint16_t name = ((uint16_t)f[11]<<8)|f[12];
    Serial.print(F("\n    detail=")); Serial.print(filamentDetailName(name));
    Serial.print(F(" (0x")); if (name < 0x10) Serial.print('0');
    Serial.print(name, HEX); Serial.print(')');

    Serial.print(F("\n    date=20"));
    if (f[13]<10) Serial.print('0'); Serial.print(f[13]);
    Serial.print('/');
    if (f[14]<10) Serial.print('0'); Serial.print(f[14]);

    Serial.print(F("\n    color=#"));
    if (f[15]<0x10) Serial.print('0'); Serial.print(f[15], HEX);
    if (f[16]<0x10) Serial.print('0'); Serial.print(f[16], HEX);
    if (f[17]<0x10) Serial.print('0'); Serial.print(f[17], HEX);

    Serial.print(F("\n    transparency=0x"));
    if (f[18]<0x10) Serial.print('0'); Serial.print(f[18], HEX);
    if      (f[18] == TRANSPARENCY_TRANSPARENT) Serial.print(F(" (transparent)"));
    else if (f[18] == TRANSPARENCY_OPAQUE)       Serial.print(F(" (opaque)"));

    uint16_t low_tmp = ((uint16_t)f[19]<<8)|f[20];
    uint16_t hig_tmp = ((uint16_t)f[21]<<8)|f[22];
    Serial.print(F("\n    nozzle_min=")); Serial.print(low_tmp); Serial.print(F("°C"));
    Serial.print(F("  nozzle_max=")); Serial.print(hig_tmp); Serial.print(F("°C"));
    Serial.print(F("\n    bed_min=")); Serial.print(f[23]); Serial.print(F("°C"));
    Serial.print(F("  bed_max=")); Serial.print(f[24]); Serial.print(F("°C"));
    Serial.print(F("\n    dry_max=")); Serial.print(f[25]); Serial.print(F("°C"));
    Serial.print(F("  dry_time=")); Serial.print(f[26]); Serial.print(F("h"));

    uint16_t diam   = ((uint16_t)f[27]<<8)|f[28];
    uint16_t weight = ((uint16_t)f[29]<<8)|f[30];
    Serial.print(F("\n    diameter="));
    Serial.print(diam/100); Serial.print('.');
    uint8_t dec = diam%100; if (dec<10) Serial.print('0'); Serial.print(dec);
    Serial.print(F("mm  weight=")); Serial.print(weight); Serial.print(F("g"));

    Serial.print(F("\n    fan="));
    if      (f[31] == FAN_OFF) Serial.print(F("OFF"));
    else if (f[31] == FAN_ON)  Serial.print(F("ON"));
    else { Serial.print(F("0x")); Serial.print(f[31], HEX); }
}

// ═══════════════════════════════════════════════════════════════════════════
// Change-detection state
// Each field we want to suppress duplicates for is stored here.
// We only print when the value differs from last time.
// ═══════════════════════════════════════════════════════════════════════════
struct ChannelState {
    uint8_t  status      = 0xFF;   // 0xFF = "never seen"
    int32_t  odo         = 0x7FFFFFFF;
    int32_t  enc         = 0x7FFFFFFF;
};

static ChannelState lastCh[CHANNEL_NUM];
static uint8_t      lastOverall    = 0xFF;
static uint8_t      lastPlugStatus = 0xFF;

// Returns true if the ALL_STATUS content differs from last time and updates state.
static bool allStatusChanged(uint8_t overall, const uint8_t* p, uint8_t pLen)
{
    bool changed = false;

    if (overall != lastOverall) {
        lastOverall = overall;
        changed = true;
    }

    if (pLen >= 2 + (uint8_t)(CHANNEL_NUM * 9)) {
        uint8_t idx = 2;
        for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
            uint8_t  st  = p[idx];
            int32_t odo  = ((int32_t)p[idx+1]<<24)|((int32_t)p[idx+2]<<16)|
                           ((int32_t)p[idx+3]<<8)|(int32_t)p[idx+4];
            int32_t enc  = ((int32_t)p[idx+5]<<24)|((int32_t)p[idx+6]<<16)|
                           ((int32_t)p[idx+7]<<8)|(int32_t)p[idx+8];
            idx += 9;

            if (st != lastCh[ch].status || odo != lastCh[ch].odo || enc != lastCh[ch].enc) {
                lastCh[ch].status = st;
                lastCh[ch].odo    = odo;
                lastCh[ch].enc    = enc;
                changed = true;
            }
        }
    }
    return changed;
}

// ═══════════════════════════════════════════════════════════════════════════
// Payload decoder
// ═══════════════════════════════════════════════════════════════════════════
// Returns false if the frame should be suppressed (no change).
static bool decodePayload(uint8_t cmd, const uint8_t* p, uint8_t pLen, bool isQuery)
{
    if (isQuery) { Serial.print(F("  [query]")); return true; }

    switch (cmd) {
    case CMD_CONNECT_STATUS:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        break;

    case CMD_HARDWARE_VERSION:
    case CMD_SOFTWARE_VERSION:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 2) {
            Serial.print(F("  version=\""));
            for (uint8_t i = 1; i < pLen; i++)
                if (p[i] >= 0x20 && p[i] < 0x7F) Serial.print((char)p[i]);
            Serial.print('"');
        }
        break;

    case CMD_FILAMENT_PLUG_IN:
        if (pLen >= 2) {
            if (p[1] == lastPlugStatus) return false;   // no change
            lastPlugStatus = p[1];
        }
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 2) {
            Serial.print(F("  plug_status=0x"));
            if (p[1]<0x10) Serial.print('0'); Serial.print(p[1], HEX);
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                Serial.print(F("  ch")); Serial.print(ch+1); Serial.print('=');
                Serial.print((p[1]>>ch)&0x01 ? F("IN") : F("OUT"));
            }
        }
        break;

    case CMD_ODO_VALUE:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 1 + CHANNEL_NUM * 4) {
            uint8_t idx = 1;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int32_t val = ((int32_t)p[idx]<<24)|((int32_t)p[idx+1]<<16)|
                              ((int32_t)p[idx+2]<<8)|p[idx+3];
                idx += 4;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(" odo=")); Serial.print(val);
            }
        }
        break;

    case CMD_ENCODER_STATUS:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 1 + CHANNEL_NUM * 4) {
            uint8_t idx = 1;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int32_t val = ((int32_t)p[idx]<<24)|((int32_t)p[idx+1]<<16)|
                              ((int32_t)p[idx+2]<<8)|p[idx+3];
                idx += 4;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(" encoder=")); Serial.print(val);
            }
        }
        break;

    case CMD_MOTOR_STATUS:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 2) {
            Serial.print(F("  motor_status=0x"));
            if (p[1]<0x10) Serial.print('0'); Serial.print(p[1], HEX);
        }
        break;

    case CMD_RFID_DATA:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 2) { Serial.print(F("  has_data=")); Serial.print(p[1] ? F("YES") : F("NO")); }
        if (pLen >= 10) {
            Serial.print(F("\n    uid=0x"));
            for (uint8_t i = 2; i < 10 && i < pLen; i++) {
                if (p[i]<0x10) Serial.print('0'); Serial.print(p[i], HEX);
            }
        }
        if (pLen >= 26 + 32) {
            Serial.print(F("\n    --- Filament Tag ---"));
            decodeFilamentStruct(p + 26);
        }
        break;

    case CMD_ALL_STATUS: {
        uint8_t overall = (pLen >= 2) ? p[1] : 0;
        if (!allStatusChanged(overall, p, pLen)) return false;   // suppress duplicate

        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        if (pLen >= 2) {
            Serial.print(F("  overall=0x"));
            if (p[1]<0x10) Serial.print('0'); Serial.print(p[1], HEX);
        }
        if (pLen >= 2 + (uint8_t)(CHANNEL_NUM * 9)) {
            uint8_t idx = 2;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                uint8_t  st  = p[idx];
                int32_t odo  = ((int32_t)p[idx+1]<<24)|((int32_t)p[idx+2]<<16)|
                               ((int32_t)p[idx+3]<<8)|(int32_t)p[idx+4];
                int32_t enc  = ((int32_t)p[idx+5]<<24)|((int32_t)p[idx+6]<<16)|
                               ((int32_t)p[idx+7]<<8)|(int32_t)p[idx+8];
                idx += 9;
                Serial.print(F("\n    ch")); Serial.print(ch+1); Serial.print(F(": "));
                Serial.print(F("fila_in="));        Serial.print((st>>1)&1);
                Serial.print(F(" dragging="));       Serial.print((st>>6)&1);
                Serial.print(F(" stalled="));        Serial.print((st>>3)&1);
                Serial.print(F(" abnormal="));       Serial.print((st>>7)&1);
                Serial.print(F(" comp_err="));       Serial.print((st>>4)&1);
                Serial.print(F(" motor_ctrl_err=")); Serial.print((st>>2)&1);
                Serial.print(F(" motor_fault="));    Serial.print((st>>0)&1);
                Serial.print(F(" odo="));            Serial.print(odo);
                Serial.print(F(" enc="));            Serial.print(enc);
            }
        }
        break;
    }

    case CMD_LED_CONTROL:
        if (pLen == 1) {
            Serial.print(F("  respond=")); Serial.print(respondName(p[0]));
        } else if (pLen >= 1 + CHANNEL_NUM * 2) {
            Serial.print(F("  enable_mask=0b"));
            for (int8_t b = 3; b >= 0; b--) Serial.print((p[0]>>b)&1);
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(": red=")); Serial.print(ledStateName(p[1+ch*2]));
                Serial.print(F(" blue="));  Serial.print(ledStateName(p[2+ch*2]));
            }
        }
        break;

    case CMD_BEEPER_CONTROL:
        if (pLen == 1) {
            Serial.print(F("  respond=")); Serial.print(respondName(p[0]));
        } else if (pLen >= 3) {
            uint16_t dur = ((uint16_t)p[1]<<8)|p[2];
            Serial.print(F("  times=")); Serial.print(p[0]);
            Serial.print(F("  duration_ms=")); Serial.print(dur);
        }
        break;

    case CMD_FEEDER_STOP:
        if (pLen == 1 && (p[0]==RESPOND_OK||p[0]==RESPOND_ERROR)) {
            Serial.print(F("  respond=")); Serial.print(respondName(p[0]));
        } else if (pLen >= 1) {
            Serial.print(F("  stop_channels=0b"));
            for (int8_t b = 3; b >= 0; b--) Serial.print((p[0]>>b)&1);
            Serial.print(F(" ["));
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++)
                if ((p[0]>>ch)&1) { Serial.print(F("ch")); Serial.print(ch+1); Serial.print(' '); }
            Serial.print(']');
        }
        break;

    case CMD_FILA_CTRL:
        if (pLen == 1) {
            Serial.print(F("  respond=")); Serial.print(respondName(p[0]));
        } else if (pLen >= CHANNEL_NUM * 6) {
            uint8_t idx = 0;
            for (uint8_t ch = 0; ch < CHANNEL_NUM; ch++) {
                int16_t mm    = ((int16_t)p[idx+0]<<8)|p[idx+1];
                int16_t mm_s  = ((int16_t)p[idx+2]<<8)|p[idx+3];
                int16_t mm_ss = ((int16_t)p[idx+4]<<8)|p[idx+5];
                idx += 6;
                Serial.print(F("\n    ch")); Serial.print(ch+1);
                Serial.print(F(": mm=")); Serial.print(mm);
                Serial.print(F(" mm/s=")); Serial.print(mm_s);
                Serial.print(F(" mm/s²=")); Serial.print(mm_ss);
            }
        }
        break;

    case CMD_FILA_CTRL_SINGLE:
        if (pLen == 1) {
            Serial.print(F("  respond=")); Serial.print(respondName(p[0]));
        } else if (pLen >= 7) {
            int16_t mm    = ((int16_t)p[1]<<8)|p[2];
            int16_t mm_s  = ((int16_t)p[3]<<8)|p[4];
            int16_t mm_ss = ((int16_t)p[5]<<8)|p[6];
            Serial.print(F("  ch=")); Serial.print(p[0]);
            Serial.print(F("  mm=")); Serial.print(mm);
            Serial.print(F("  mm/s=")); Serial.print(mm_s);
            Serial.print(F("  mm/s²=")); Serial.print(mm_ss);
        }
        break;

    case CMD_FILA_CTRL_REACH_ACK:
    case CMD_FILA_CTRL_BOTH_ACK:
        if (pLen >= 1) { Serial.print(F("  respond=")); Serial.print(respondName(p[0])); }
        break;

    default:
        Serial.print(F("  [unknown cmd] payload: "));
        printHex(p, pLen);
        break;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Frame decoder
// ═══════════════════════════════════════════════════════════════════════════
static void decodeFrame(const uint8_t* frame, uint16_t len)
{
    if (len < 6 || frame[0] != FRAME_HEAD) return;  // silently drop junk

    uint8_t flag = frame[1];

    if (flag == FRAME_SHORT) {
        uint8_t flen = frame[2];
        if (len < flen)                              return;
        if (crc8(frame, 3) != frame[3])              return;  // CRC8 fail — drop silently
        uint16_t c16r = ((uint16_t)frame[flen-2]<<8)|frame[flen-1];
        if (crc16(frame, flen-2) != c16r)            return;  // CRC16 fail — drop silently

        uint8_t devId  = frame[4];
        uint8_t cmd    = frame[5];
        const uint8_t* payload = frame + 6;
        uint8_t payLen = (flen >= 8) ? flen - 8 : 0;
        bool isQuery   = (payLen == 0);

        // Print header line then decode payload.
        // decodePayload() returns false if nothing changed — skip header too.
        // We buffer the header and only commit it if payload says to print.

        // To avoid printing the header for suppressed frames we decode first
        // into a temporary check, then print if needed.
        // For non-ALL_STATUS / non-plug-in frames always print.
        // For ALL_STATUS and PLUG_IN decodePayload handles suppression internally.

        // Stage: build header string in Serial only after we know we'll print.
        // Simplest approach: call decodePayload, which returns false to suppress.
        // We need to print the separator and header first though — so we check
        // change BEFORE printing for the two commands that suppress:

        bool shouldPrint = true;

        // Pre-check for ALL_STATUS suppression without side effects
        if (cmd == CMD_ALL_STATUS && !isQuery) {
            uint8_t overall = (payLen >= 2) ? payload[1] : 0;
            shouldPrint = allStatusChanged(overall, payload, payLen);
            if (!shouldPrint) return;
        }
        if (cmd == CMD_FILAMENT_PLUG_IN && !isQuery && payLen >= 2) {
            if (payload[1] == lastPlugStatus) return;
        }

        Serial.println(F("────────────────────────────────────────"));
        Serial.print(F("RAW (")); Serial.print(flen); Serial.print(F("): "));
        printHex(frame, flen);
        Serial.println();

        Serial.print(F("TYPE: SHORT  devId=0x"));
        if (devId < 0x10) Serial.print('0'); Serial.print(devId, HEX);
        if (devId == CANVAS_LITE_DEVICE_ID) Serial.print(F(" (Canvas Lite)"));
        Serial.print(F("  cmd=0x"));
        if (cmd < 0x10) Serial.print('0'); Serial.print(cmd, HEX);
        Serial.print(F(" (")); Serial.print(cmdName(cmd));
        Serial.print(F(")  ")); Serial.println(isQuery ? F("QUERY") : F("RESPONSE"));

        decodePayload(cmd, payload, payLen, isQuery);
        Serial.println();
    }
    else if (flag == FRAME_LONG) {
        if (len < 9) return;
        uint16_t seq  = ((uint16_t)frame[2]<<8)|frame[3];
        uint16_t flen = ((uint16_t)frame[4]<<8)|frame[5];
        if (len < flen)                  return;
        if (crc8(frame, 6) != frame[6])  return;
        uint16_t c16r = ((uint16_t)frame[flen-2]<<8)|frame[flen-1];
        if (crc16(frame, flen-2) != c16r) return;

        const uint8_t* payload = frame + 7;
        uint8_t payLen = (flen >= 9) ? flen - 9 : 0;

        Serial.println(F("────────────────────────────────────────"));
        Serial.print(F("TYPE: LONG  seq=")); Serial.print(seq);
        Serial.print(F("  payLen=")); Serial.println(payLen);
        if (payLen > 0) { Serial.print(F("  payload: ")); printHex(payload, payLen); Serial.println(); }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RX state
// ═══════════════════════════════════════════════════════════════════════════
static uint8_t  rxBuf[256];
static uint16_t rxLen  = 0;
static bool     active = false;

// ═══════════════════════════════════════════════════════════════════════════
// Limit switch state
// ═══════════════════════════════════════════════════════════════════════════
static bool     swLastState  = HIGH;
static uint32_t swLastTimeMs = 0;

static void handleLimitSwitch()
{
    bool swNow = digitalRead(SW_PIN);
    uint32_t now = millis();

    // Trigger when switch is LOW (pressed), with debounce
    if (swNow == HIGH && (now - swLastTimeMs) > DEBOUNCE_MS) {
        swLastTimeMs = now;
        Serial.println(F("[SW] Limit switch pressed — sending 1mm feed on ch4"));
        sendFeedSingle(FEED_CHANNEL, FEED_MM, FEED_MM_S, FEED_MM_SS);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════════════════════════════════════
void setup()
{
    // MAX485: separate DE and /RE pins
    pinMode(DE_PIN, OUTPUT);
    pinMode(RE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW);   // driver disabled
    digitalWrite(RE_PIN, LOW);   // receiver enabled

    // Limit switch with internal pull-up (reads LOW when pressed)
    pinMode(SW_PIN, INPUT_PULLUP);

    Serial.begin(DEBUG_BAUD);
    Serial1.begin(RS485_BAUD);

    Serial.println(F("Elegoo Canvas RS485 Decoder + Limit Switch"));
    Serial.print  (F("Baud: ")); Serial.print(RS485_BAUD);
    Serial.print  (F("  IFG: ")); Serial.print(IFG_US); Serial.println(F("us"));
    Serial.println(F("Only printing frames when values change."));
    Serial.print  (F("Limit switch on pin ")); Serial.print(SW_PIN);
    Serial.print  (F(" -> feed ch")); Serial.print(FEED_CHANNEL);
    Serial.print  (F(" ")); Serial.print(FEED_MM);
    Serial.print  (F("mm @ ")); Serial.print(FEED_MM_S);
    Serial.print  (F("mm/s, ")); Serial.print(FEED_MM_SS);
    Serial.println(F("mm/s²"));
}

void loop()
{
    // ── RX: drain UART ───────────────────────────────────────────────────
    while (Serial1.available()) {
        uint8_t b  = (uint8_t)Serial1.read();
        lastRxUs   = micros();
        active     = true;

        if (rxLen < sizeof(rxBuf)) {
            rxBuf[rxLen++] = b;
        } else {
            decodeFrame(rxBuf, rxLen);
            rxLen = 0;
            rxBuf[rxLen++] = b;
        }
    }

    // ── RX: end-of-frame on IFG ──────────────────────────────────────────
    if (active && (micros() - lastRxUs) >= IFG_US) {
        decodeFrame(rxBuf, rxLen);
        rxLen  = 0;
        active = false;
    }

    // ── Limit switch ─────────────────────────────────────────────────────
    handleLimitSwitch();
}