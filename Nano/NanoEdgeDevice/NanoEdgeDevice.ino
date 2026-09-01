// ─────────────────────────────────────────────────────────────────────────────
// NanoEdgeDevice.ino
// Arduino Nano edge I/O device — communicates with RobotController via serial.
//
// Serial protocol (115200 baud, newline-terminated ASCII lines):
//
//  HOST → NANO
//    ID?                         Request device identifier
//    GET                         Get full state of all configured pins
//    CFG:<pin>,<type>[,<count>]  Configure pin  (I=Input, O=Output, N=Neopixel)
//    SET:<pin>,<val>             Set digital output (val: 0 or 1)
//    NEO:<pin>,<r0>,<g0>,<b0>,<r1>,<g1>,<b1>,...  Set neopixel pixel colours
//
//  NANO → HOST
//    ID:<identifier>             Response to ID?
//    STATE:<pin>,<t>,<v>;...     Full pin state (t: I/O/N, v: 0 or 1)
//    CHG:<pin>,<val>             Input pin changed value
//
// Requires: Adafruit NeoPixel library
// ─────────────────────────────────────────────────────────────────────────────

#include <Adafruit_NeoPixel.h>

// ── Configuration ─────────────────────────────────────────────────────────────

// Change this identifier to match the nano_config.json entry for this device.
#define DEVICE_ID "ROBOT_NANO_001"

#define SERIAL_BAUD     115200
#define MAX_PINS        20      // Arduino Nano has D0-D13, A0-A7 (mapped 14-21)
#define MAX_PIXELS      150     // Max pixels any single neopixel pin can drive
#define INPUT_POLL_MS   5       // How often to check inputs (ms)

// ── Pin mode constants ─────────────────────────────────────────────────────────

#define MODE_UNCONFIGURED  0
#define MODE_INPUT         1
#define MODE_OUTPUT        2
#define MODE_NEOPIXEL      3

// ── Per-pin state ──────────────────────────────────────────────────────────────

struct PinState {
    uint8_t           mode;
    uint8_t           value;        // current digital value (inputs & outputs)
    uint8_t           pixelCount;   // neopixel only
    Adafruit_NeoPixel* strip;       // neopixel only, heap-allocated
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
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handler
// ─────────────────────────────────────────────────────────────────────────────

void handleCommand(const String& line)
{
    // ── ID? ───────────────────────────────────────────────────────────────────
    if (line == "ID?") {
        Serial.println("ID:" DEVICE_ID);
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

        Serial.print(i);
        Serial.print(',');
        Serial.print(typeChar);
        Serial.print(',');
        Serial.print(pins[i].value);
    }

    Serial.println();
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
