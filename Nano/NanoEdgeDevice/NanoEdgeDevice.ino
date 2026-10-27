// ─────────────────────────────────────────────────────────────────────────────
// NanoEdgeDevice.ino
// Arduino Nano edge I/O device — communicates with RobotController via serial.
//
// Serial protocol (115200 baud, newline-terminated ASCII lines):
//
//  HOST → NANO
//    ID?                         Request device identifier, firmware version and capabilities
//    GET                         Get full state of all configured pins
//    CFG:<pin>,<type>[,<count>]  Configure pin  (I=Input, O=Output, N=Neopixel)
//    CFG:<dtPin>,L,<sckPin>      Configure HX711 load cell (DT=dtPin, SCK=sckPin)
//    SET:<pin>,<val>             Set digital output (val: 0 or 1)
//    NEO:<pin>,<r0>,<g0>,<b0>,<r1>,<g1>,<b1>,...  Set neopixel pixel colours
//
//  NANO → HOST
//    ID:<identifier>;fw=<major.minor.patch>;caps=<cap1,cap2,...>
//                                Response to ID? — one line. The `ID:<identifier>` prefix is
//                                byte-for-byte what pre-1.1.0 firmware sent, so old controllers
//                                still match; a controller that sees no `;fw=` treats the board
//                                as legacy (unknown capabilities). Capability names (lowercase,
//                                comma-separated, no spaces):
//                                  io        digital input/output pins (CFG/GET/SET/CHG/STATE)
//                                  neopixel  CFG type N + NEO
//                                  loadcell  CFG type L + LC streaming
//                                  stepper   M/C/S/X/E/D axis commands (not on this board)
//    STATE:<pin>,<t>,<v>;...     Full pin state (t: I/O/N, v: 0 or 1; L: v is raw count)
//    CHG:<pin>,<val>             Input pin changed value
//    LC:<pin>,<raw>              HX711 load-cell raw 24-bit reading (signed), streamed each sample
//
// Load cell: the Nano streams RAW counts only; the controller applies the tare offset + scale to
// get grams (calibration lives controller-side). Reads are non-blocking (only when the HX711 has a
// sample ready, i.e. DT is LOW), so they never stall the loop. Sample rate is set by the HX711 RATE
// pin wiring (10 SPS default, 80 SPS if RATE is tied high) — the firmware reads whatever is produced.
//
// Requires: Adafruit NeoPixel library
// ─────────────────────────────────────────────────────────────────────────────

#include <Adafruit_NeoPixel.h>

// ── Configuration ─────────────────────────────────────────────────────────────

// Change this identifier to match the nano_config.json entry for this device.
#define DEVICE_ID "ROBOT_NANO_001"

// Firmware version (semantic): bump patch for fixes, minor when a capability/command is
// added, major when the protocol changes incompatibly. 1.0.0 is the un-versioned firmware.
#define FW_VERSION "1.1.0"
// Capabilities advertised in the ID? reply (see protocol block above).
#define FW_CAPS    "io,neopixel,loadcell"

#define SERIAL_BAUD     115200
#define MAX_PINS        20      // Arduino Nano has D0-D13, A0-A7 (mapped 14-21)
#define MAX_PIXELS      150     // Max pixels any single neopixel pin can drive
#define INPUT_POLL_MS   5       // How often to check inputs (ms)

// ── Pin mode constants ─────────────────────────────────────────────────────────

#define MODE_UNCONFIGURED  0
#define MODE_INPUT         1
#define MODE_OUTPUT        2
#define MODE_NEOPIXEL      3
#define MODE_LOADCELL      4

// ── Per-pin state ──────────────────────────────────────────────────────────────

struct PinState {
    uint8_t           mode;
    uint8_t           value;        // current digital value (inputs & outputs)
    uint8_t           pixelCount;   // neopixel only
    Adafruit_NeoPixel* strip;       // neopixel only, heap-allocated
    uint8_t           sckPin;       // loadcell only: HX711 clock pin
    long              raw;          // loadcell only: latest raw 24-bit reading (signed)
};

static PinState   pins[MAX_PINS];
static uint8_t    lastInput[MAX_PINS];  // previous input values for change detection

// ── Serial input buffer ────────────────────────────────────────────────────────

static String inputBuf = "";

// ── Timing ────────────────────────────────────────────────────────────────────

static unsigned long lastPollMs = 0;

// ── Forward declarations ──────────────────────────────────────────────────────

void handleCommand(const String& line);
void sendFullState();
void checkInputChanges();
void configurePin(int pin, char typeChar, int pixelCount);
void pollLoadCells();
long readHx711(uint8_t dtPin, uint8_t sckPin);

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(SERIAL_BAUD);
    memset(pins, 0, sizeof(pins));
    memset(lastInput, 0, sizeof(lastInput));
}

void loop()
{
    // ── Read serial ──────────────────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            inputBuf.trim();
            if (inputBuf.length() > 0) {
                handleCommand(inputBuf);
            }
            inputBuf = "";
        } else if (c != '\r') {
            inputBuf += c;
        }
    }

    // ── Poll inputs ──────────────────────────────────────────────────────────
    unsigned long nowMs = millis();
    if (nowMs - lastPollMs >= INPUT_POLL_MS) {
        lastPollMs = nowMs;
        checkInputChanges();
    }

    // ── Poll load cells (non-blocking: reads only when a sample is ready) ──────
    pollLoadCells();
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handler
// ─────────────────────────────────────────────────────────────────────────────

void handleCommand(const String& line)
{
    // ── ID? ───────────────────────────────────────────────────────────────────
    if (line == "ID?") {
        // One line; literals concatenated at compile time, no String allocation.
        Serial.println("ID:" DEVICE_ID ";fw=" FW_VERSION ";caps=" FW_CAPS);
        return;
    }

    // ── GET ───────────────────────────────────────────────────────────────────
    if (line == "GET") {
        sendFullState();
        return;
    }

    // ── CFG:<pin>,<type>[,<pixelCount>] ───────────────────────────────────────
    if (line.startsWith("CFG:")) {
        int c1 = line.indexOf(',', 4);
        if (c1 < 0) return;

        int pin = line.substring(4, c1).toInt();
        if (pin < 0 || pin >= MAX_PINS) return;

        String rest = line.substring(c1 + 1);
        int c2 = rest.indexOf(',');

        char typeChar;
        int  pixelCount = 8;

        if (c2 >= 0) {
            typeChar   = rest.charAt(0);
            pixelCount = rest.substring(c2 + 1).toInt();
        } else {
            typeChar = rest.charAt(0);
        }

        configurePin(pin, typeChar, pixelCount);
        return;
    }

    // ── SET:<pin>,<value> ─────────────────────────────────────────────────────
    if (line.startsWith("SET:")) {
        int c1 = line.indexOf(',', 4);
        if (c1 < 0) return;

        int pin = line.substring(4, c1).toInt();
        int val = line.substring(c1 + 1).toInt();

        if (pin < 0 || pin >= MAX_PINS) return;
        if (pins[pin].mode != MODE_OUTPUT) return;

        pins[pin].value = val ? 1 : 0;
        digitalWrite(pin, pins[pin].value ? HIGH : LOW);
        return;
    }

    // ── NEO:<pin>,<r0>,<g0>,<b0>,<r1>,<g1>,<b1>,... ──────────────────────────
    if (line.startsWith("NEO:")) {
        int idx = 4;
        int c1  = line.indexOf(',', idx);
        if (c1 < 0) return;

        int pin = line.substring(idx, c1).toInt();
        if (pin < 0 || pin >= MAX_PINS) return;
        if (pins[pin].mode != MODE_NEOPIXEL || pins[pin].strip == nullptr) return;

        idx = c1 + 1;
        int pixelIdx = 0;
        int total    = pins[pin].pixelCount;

        while (idx < (int)line.length() && pixelIdx < total) {
            // Parse R
            int c2 = line.indexOf(',', idx);
            if (c2 < 0) break;
            int r = line.substring(idx, c2).toInt();
            idx = c2 + 1;

            // Parse G
            int c3 = line.indexOf(',', idx);
            if (c3 < 0) break;
            int g = line.substring(idx, c3).toInt();
            idx = c3 + 1;

            // Parse B  (may be last value or followed by another pixel)
            int c4 = line.indexOf(',', idx);
            int b;
            if (c4 < 0) {
                b = line.substring(idx).toInt();
                idx = line.length();
            } else {
                b = line.substring(idx, c4).toInt();
                idx = c4 + 1;
            }

            pins[pin].strip->setPixelColor(pixelIdx,
                pins[pin].strip->Color(
                    (uint8_t)constrain(r, 0, 255),
                    (uint8_t)constrain(g, 0, 255),
                    (uint8_t)constrain(b, 0, 255)
                )
            );
            pixelIdx++;
        }

        pins[pin].strip->show();
        return;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pin configuration
// ─────────────────────────────────────────────────────────────────────────────

void configurePin(int pin, char typeChar, int pixelCount)
{
    // Free any existing neopixel strip on this pin
    if (pins[pin].mode == MODE_NEOPIXEL && pins[pin].strip != nullptr) {
        pins[pin].strip->clear();
        pins[pin].strip->show();
        delete pins[pin].strip;
        pins[pin].strip = nullptr;
    }

    memset(&pins[pin], 0, sizeof(PinState));

    if (typeChar == 'I') {
        pins[pin].mode  = MODE_INPUT;
        pinMode(pin, INPUT_PULLUP);
        uint8_t v        = digitalRead(pin);
        pins[pin].value  = v;
        lastInput[pin]   = v;

    } else if (typeChar == 'O') {
        pins[pin].mode  = MODE_OUTPUT;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        pins[pin].value  = 0;

    } else if (typeChar == 'N') {
        int count = constrain(pixelCount, 1, MAX_PIXELS);
        pins[pin].mode       = MODE_NEOPIXEL;
        pins[pin].pixelCount = (uint8_t)count;
        pins[pin].strip      = new Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);
        pins[pin].strip->begin();
        pins[pin].strip->clear();
        pins[pin].strip->show();

    } else if (typeChar == 'L') {
        // HX711 load cell: `pin` is the DT (data) line, `pixelCount` carries the SCK (clock) line.
        int sck = pixelCount;
        if (sck < 0 || sck >= MAX_PINS) return;
        pins[pin].mode   = MODE_LOADCELL;
        pins[pin].sckPin = (uint8_t)sck;
        pins[pin].raw    = 0;
        pinMode(pin, INPUT);        // DT: driven by the HX711
        pinMode(sck, OUTPUT);
        digitalWrite(sck, LOW);     // SCK low keeps the HX711 awake (a long HIGH powers it down)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State reporting
// ─────────────────────────────────────────────────────────────────────────────

void sendFullState()
{
    Serial.print("STATE:");
    bool first = true;

    for (int i = 0; i < MAX_PINS; i++) {
        if (pins[i].mode == MODE_UNCONFIGURED) continue;

        if (!first) Serial.print(';');
        first = false;

        char typeChar = '?';
        if      (pins[i].mode == MODE_INPUT)    typeChar = 'I';
        else if (pins[i].mode == MODE_OUTPUT)   typeChar = 'O';
        else if (pins[i].mode == MODE_NEOPIXEL) typeChar = 'N';
        else if (pins[i].mode == MODE_LOADCELL) typeChar = 'L';

        Serial.print(i);
        Serial.print(',');
        Serial.print(typeChar);
        Serial.print(',');
        // Load cells report their latest raw 24-bit count; everything else its 0/1 value.
        Serial.print(pins[i].mode == MODE_LOADCELL ? pins[i].raw : (long)pins[i].value);
    }

    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// HX711 load cell
// ─────────────────────────────────────────────────────────────────────────────

// Shift one 24-bit sample out of the HX711 (MSB first), then a 25th pulse to select
// gain 128 / channel A for the next conversion. Sign-extends to a 32-bit long. The caller
// must only invoke this when DT is LOW (a sample is ready), so it never blocks waiting.
long readHx711(uint8_t dtPin, uint8_t sckPin)
{
    long value = 0;
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(sckPin, HIGH);
        value = (value << 1) | (digitalRead(dtPin) ? 1 : 0);
        digitalWrite(sckPin, LOW);
    }
    // 25th pulse: set channel A, gain 128 for the next reading.
    digitalWrite(sckPin, HIGH);
    digitalWrite(sckPin, LOW);
    // Sign-extend the 24-bit two's-complement value.
    if (value & 0x800000L) value |= 0xFF000000L;
    return value;
}

// Read each configured load cell whenever it has a fresh sample ready and stream the raw count.
void pollLoadCells()
{
    for (int i = 0; i < MAX_PINS; i++) {
        if (pins[i].mode != MODE_LOADCELL) continue;
        if (digitalRead(i) != LOW) continue;   // HX711 pulls DT low when a sample is ready
        pins[i].raw = readHx711((uint8_t)i, pins[i].sckPin);
        Serial.print("LC:");
        Serial.print(i);
        Serial.print(',');
        Serial.println(pins[i].raw);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Input change detection
// ─────────────────────────────────────────────────────────────────────────────

void checkInputChanges()
{
    for (int i = 0; i < MAX_PINS; i++) {
        if (pins[i].mode != MODE_INPUT) continue;

        uint8_t now = digitalRead(i);
        if (now != lastInput[i]) {
            lastInput[i]  = now;
            pins[i].value = now;

            Serial.print("CHG:");
            Serial.print(i);
            Serial.print(',');
            Serial.println(now);
        }
    }
}
