// ─────────────────────────────────────────────────────────────────────────────
// NanoEdgeDevice.ino
// Arduino Nano edge I/O device — communicates with RobotController via serial.
//
// Serial protocol (115200 baud, newline-terminated ASCII lines):
//
//  HOST → NANO
//    ID?                         Request device identifier
//    GET                         Get full state of all configured pins
//    CFG:<pin>,<type>[,<opt>...] Configure pin  (I=Input, O=Output, N=Neopixel)
//    SET:<pin>,<val>             Set digital output (val: 0 or 1)
//    NEO:<pin>,<r0>,<g0>,<b0>,<r1>,<g1>,<b1>,...  Set neopixel pixel colours
//
//  CFG options are <key>=<value>, in any order, all optional:
//    INIT=<0|1>   Level an output is driven to when configured, and the level it
//                 falls back to if the failsafe releases it. Default 0.
//                 An active LOW load wants INIT=1 so configuring does not
//                 energise it.
//    HOLD=<0|1>   Whether the pin keeps its state when the host goes quiet.
//                 Default 1, sticky: a pin holds what it was told regardless.
//                 HOLD=0 opts the pin into the failsafe below.
//    DEB=<ms>     Input debounce in milliseconds. Default 0, report a change as
//                 soon as it is seen.
//    PIX=<n>      Neopixel pixel count. Default 8.
//
//    A bare third field with no "=" is still read as a neopixel pixel count, so
//    "CFG:6,N,16" from an older host keeps working.
//
//  NANO → HOST
//    READY:<identifier>          Sent once from setup(), so a reset announces
//                                itself instead of being discovered by polling.
//                                Every pin is unconfigured when this appears.
//    ID:<identifier>             Response to ID?
//    STATE:<pin>,<t>,<v>;...     Full pin state (t: I/O/N, v: 0 or 1)
//    CHG:<pin>,<val>             Input pin changed value
//    !ERR:<cmd>,<pin>,<reason>   A command was refused. Nothing was silently
//                                dropped, so the host does not have to poll to
//                                find out a write went nowhere.
//    !FAILSAFE                   The host went quiet and every HOLD=0 pin was
//                                released to its INIT level.
//
// Requires: Adafruit NeoPixel library
// ─────────────────────────────────────────────────────────────────────────────

#include <Adafruit_NeoPixel.h>

// ── Configuration ─────────────────────────────────────────────────────────────

// Change this identifier to match the nano_config.json entry for this device.
#define DEVICE_ID "ROBOT_NANO_001"

#define SERIAL_BAUD     115200

// D0-D13 and A0-A5 are usable as digital pins (A0-A5 map to 14-19). A6 and A7 on
// a Nano are analog inputs only and cannot be digital, which this bound excludes.
#define MAX_PINS        20

#define MAX_PIXELS      150     // Max pixels any single neopixel pin can drive
#define INPUT_POLL_MS   5       // How often to check inputs (ms)

// How long the host may be silent before every HOLD=0 pin is released to its INIT
// level. A pin left latched by a host that crashed, was unplugged or hung is the
// thing this exists for: on a relay driving a pump that is a pump that never
// stops. Pins are sticky by default, so this only reaches the ones that asked.
// Set to 0 to disable the failsafe entirely.
#define FAILSAFE_TIMEOUT_MS  5000UL

// Longest command line accepted. A line longer than this is dropped and reported
// rather than being allowed to grow until the Nano runs out of memory.
#define INPUT_BUF_SIZE  96

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
    uint8_t           initLevel;    // configure level, and the failsafe level
    uint8_t           hold;         // 1 = keep state when the host goes quiet
    uint16_t          debounceMs;   // input only, 0 = report immediately
    Adafruit_NeoPixel* strip;       // neopixel only, heap-allocated
};

static PinState      pins[MAX_PINS];
static uint8_t       lastInput[MAX_PINS];    // last reported input values
static uint8_t       rawInput[MAX_PINS];     // last raw read, for debounce
static unsigned long rawChangedMs[MAX_PINS]; // when the raw read last moved

// ── Serial input buffer ────────────────────────────────────────────────────────

// A fixed buffer rather than a String. This runs for weeks at a time on a part
// with 2KB of SRAM, and growing a String a character at a time fragments the heap.
static char    inputBuf[INPUT_BUF_SIZE];
static uint8_t inputLen      = 0;
static bool    inputOverflow = false;

// ── Timing ────────────────────────────────────────────────────────────────────

static unsigned long lastPollMs    = 0;
static unsigned long lastCommandMs = 0;
static bool          failsafeTripped = false;

// ── Forward declarations ──────────────────────────────────────────────────────

void handleCommand(char* line);
void handleConfigure(char* body);
void handleSet(char* body);
void handleNeo(char* body);
void sendFullState();
void sendError(const char* command, int pin, const char* reason);
void checkInputChanges();
void checkFailsafe();
void configurePin(int pin, char typeChar, int pixelCount,
                  uint8_t initLevel, uint8_t hold, uint16_t debounceMs);

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(SERIAL_BAUD);
    memset(pins, 0, sizeof(pins));
    memset(lastInput, 0, sizeof(lastInput));
    memset(rawInput, 0, sizeof(rawInput));
    memset(rawChangedMs, 0, sizeof(rawChangedMs));

    lastCommandMs = millis();

    // A Nano resets whenever its port is opened, and everything above just wiped
    // every pin. Saying so turns that into an event the host can act on, instead
    // of something it discovers later by noticing its pins are gone.
    Serial.println("READY:" DEVICE_ID);
}

void loop()
{
    // ── Read serial ──────────────────────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n') {
            if (inputOverflow) {
                sendError("", -1, "OVERFLOW");
            } else if (inputLen > 0) {
                inputBuf[inputLen] = '\0';
                handleCommand(inputBuf);
            }

            inputLen      = 0;
            inputOverflow = false;

        } else if (c != '\r') {
            // Kept bounded. Line noise without a newline would otherwise grow
            // until there was no memory left to grow into.
            if (inputLen < (INPUT_BUF_SIZE - 1)) {
                inputBuf[inputLen++] = c;
            } else {
                inputOverflow = true;
            }
        }
    }

    // ── Poll inputs ──────────────────────────────────────────────────────────
    unsigned long nowMs = millis();
    if (nowMs - lastPollMs >= INPUT_POLL_MS) {
        lastPollMs = nowMs;
        checkInputChanges();
    }

    // ── Release anything that asked not to be left latched ───────────────────
    checkFailsafe();
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handler
// ─────────────────────────────────────────────────────────────────────────────

void handleCommand(char* line)
{
    // Any command at all counts as the host being alive, including a GET, which
    // is what a polling host sends anyway
    lastCommandMs = millis();

    if (strcmp(line, "ID?") == 0) {
        Serial.println("ID:" DEVICE_ID);
        return;
    }

    if (strcmp(line, "GET") == 0) {
        sendFullState();
        return;
    }

    if (strncmp(line, "CFG:", 4) == 0) { handleConfigure(line + 4); return; }
    if (strncmp(line, "SET:", 4) == 0) { handleSet(line + 4);       return; }
    if (strncmp(line, "NEO:", 4) == 0) { handleNeo(line + 4);       return; }

    sendError("", -1, "BADCMD");
}

// ── CFG:<pin>,<type>[,<key>=<value>...] ──────────────────────────────────────

void handleConfigure(char* body)
{
    char* save   = NULL;
    char* pinTok = strtok_r(body, ",", &save);
    if (pinTok == NULL) { sendError("CFG", -1, "NOPIN"); return; }

    int pin = atoi(pinTok);
    if (pin < 0 || pin >= MAX_PINS) { sendError("CFG", pin, "RANGE"); return; }

    char* typeTok = strtok_r(NULL, ",", &save);
    if (typeTok == NULL) { sendError("CFG", pin, "NOTYPE"); return; }

    char typeChar = typeTok[0];
    if (typeChar != 'I' && typeChar != 'O' && typeChar != 'N') {
        sendError("CFG", pin, "TYPE");
        return;
    }

    // Defaults: driven low, sticky, and as sensitive as the poll allows
    uint8_t  initLevel  = 0;
    uint8_t  hold       = 1;
    uint16_t debounceMs = 0;
    int      pixelCount = 8;

    for (char* opt = strtok_r(NULL, ",", &save); opt != NULL;
         opt = strtok_r(NULL, ",", &save)) {

        char* equals = strchr(opt, '=');

        // No "=", so this is an older host's positional pixel count
        if (equals == NULL) { pixelCount = atoi(opt); continue; }

        *equals = '\0';
        const char* key   = opt;
        const char* value = equals + 1;

        if      (strcmp(key, "INIT") == 0) initLevel  = atoi(value) ? 1 : 0;
        else if (strcmp(key, "HOLD") == 0) hold       = atoi(value) ? 1 : 0;
        else if (strcmp(key, "DEB")  == 0) debounceMs = (uint16_t)atoi(value);
        else if (strcmp(key, "PIX")  == 0) pixelCount = atoi(value);
        else                               sendError("CFG", pin, "BADOPT");
    }

    configurePin(pin, typeChar, pixelCount, initLevel, hold, debounceMs);
}

// ── SET:<pin>,<value> ────────────────────────────────────────────────────────

void handleSet(char* body)
{
    char* comma = strchr(body, ',');
    if (comma == NULL) { sendError("SET", -1, "NOVALUE"); return; }

    *comma = '\0';
    int pin = atoi(body);
    int val = atoi(comma + 1);

    if (pin < 0 || pin >= MAX_PINS) { sendError("SET", pin, "RANGE"); return; }

    // Refused out loud. Silently dropping this is indistinguishable from the write
    // working, and the host has no way to tell the two apart without polling.
    if (pins[pin].mode != MODE_OUTPUT) { sendError("SET", pin, "NOTOUT"); return; }

    pins[pin].value = val ? 1 : 0;
    digitalWrite(pin, pins[pin].value ? HIGH : LOW);
}

// ── NEO:<pin>,<r0>,<g0>,<b0>,... ─────────────────────────────────────────────

void handleNeo(char* body)
{
    char* comma = strchr(body, ',');
    if (comma == NULL) { sendError("NEO", -1, "NOCOLOUR"); return; }

    *comma = '\0';
    int pin = atoi(body);

    if (pin < 0 || pin >= MAX_PINS)  { sendError("NEO", pin, "RANGE");  return; }
    if (pins[pin].mode != MODE_NEOPIXEL || pins[pin].strip == NULL) {
        sendError("NEO", pin, "NOTNEO");
        return;
    }

    char* save     = NULL;
    char* token    = strtok_r(comma + 1, ",", &save);
    int   pixelIdx = 0;
    int   total    = pins[pin].pixelCount;

    while (token != NULL && pixelIdx < total) {
        int r = atoi(token);

        token = strtok_r(NULL, ",", &save);
        if (token == NULL) break;
        int g = atoi(token);

        token = strtok_r(NULL, ",", &save);
        if (token == NULL) break;
        int b = atoi(token);

        pins[pin].strip->setPixelColor(pixelIdx,
            pins[pin].strip->Color(
                (uint8_t)constrain(r, 0, 255),
                (uint8_t)constrain(g, 0, 255),
                (uint8_t)constrain(b, 0, 255)
            )
        );
        pixelIdx++;

        token = strtok_r(NULL, ",", &save);
    }

    pins[pin].strip->show();
}

// ─────────────────────────────────────────────────────────────────────────────
// Pin configuration
// ─────────────────────────────────────────────────────────────────────────────

void configurePin(int pin, char typeChar, int pixelCount,
                  uint8_t initLevel, uint8_t hold, uint16_t debounceMs)
{
    // Free any existing neopixel strip on this pin
    if (pins[pin].mode == MODE_NEOPIXEL && pins[pin].strip != NULL) {
        pins[pin].strip->clear();
        pins[pin].strip->show();
        delete pins[pin].strip;
        pins[pin].strip = NULL;
    }

    memset(&pins[pin], 0, sizeof(PinState));

    pins[pin].initLevel  = initLevel;
    pins[pin].hold       = hold;
    pins[pin].debounceMs = debounceMs;

    if (typeChar == 'I') {
        pins[pin].mode  = MODE_INPUT;
        pinMode(pin, INPUT_PULLUP);

        uint8_t v         = digitalRead(pin);
        pins[pin].value   = v;
        lastInput[pin]    = v;
        rawInput[pin]     = v;
        rawChangedMs[pin] = millis();

    } else if (typeChar == 'O') {
        pins[pin].mode  = MODE_OUTPUT;
        pinMode(pin, OUTPUT);

        // Driven to the level the caller asked for rather than always low. On an
        // active LOW load, low is energised, so configuring a bank of them used to
        // switch the lot on until each SET arrived to turn it back off.
        digitalWrite(pin, initLevel ? HIGH : LOW);
        pins[pin].value = initLevel;

    } else if (typeChar == 'N') {
        int count = constrain(pixelCount, 1, MAX_PIXELS);
        pins[pin].mode       = MODE_NEOPIXEL;
        pins[pin].pixelCount = (uint8_t)count;
        pins[pin].strip      = new Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);

        // new can fail on a part this small, and calling begin() on nothing hangs
        if (pins[pin].strip == NULL) {
            pins[pin].mode = MODE_UNCONFIGURED;
            sendError("CFG", pin, "NOMEM");
            return;
        }

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

void sendError(const char* command, int pin, const char* reason)
{
    Serial.print("!ERR:");
    Serial.print(command);
    Serial.print(',');
    if (pin >= 0) Serial.print(pin);
    Serial.print(',');
    Serial.println(reason);
}

// ─────────────────────────────────────────────────────────────────────────────
// Input change detection
// ─────────────────────────────────────────────────────────────────────────────

void checkInputChanges()
{
    unsigned long nowMs = millis();

    for (int i = 0; i < MAX_PINS; i++) {
        if (pins[i].mode != MODE_INPUT) continue;

        uint8_t now = digitalRead(i);

        // Track when the raw read last moved, which is what the debounce measures
        if (now != rawInput[i]) {
            rawInput[i]     = now;
            rawChangedMs[i] = nowMs;
        }

        // Already reported, nothing to say
        if (now == lastInput[i]) continue;

        // Held its new value long enough to be believed. With the default of 0 this
        // is true the moment it changes, which is how this has always behaved.
        if ((nowMs - rawChangedMs[i]) < pins[i].debounceMs) continue;

        lastInput[i]   = now;
        pins[i].value  = now;

        Serial.print("CHG:");
        Serial.print(i);
        Serial.print(',');
        Serial.println(now);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Failsafe
// ─────────────────────────────────────────────────────────────────────────────

void checkFailsafe()
{
    if (FAILSAFE_TIMEOUT_MS == 0) return;

    bool silent = (millis() - lastCommandMs) >= FAILSAFE_TIMEOUT_MS;

    // Latched, so a host that stays away is not told about it over and over
    if (!silent) { failsafeTripped = false; return; }
    if (failsafeTripped) return;

    failsafeTripped = true;

    for (int i = 0; i < MAX_PINS; i++) {
        // Sticky by default. Only a pin that asked to be released is released, so
        // this can never take down something that was deliberately left latched.
        if (pins[i].hold) continue;

        if (pins[i].mode == MODE_OUTPUT) {
            digitalWrite(i, pins[i].initLevel ? HIGH : LOW);
            pins[i].value = pins[i].initLevel;

        } else if (pins[i].mode == MODE_NEOPIXEL && pins[i].strip != NULL) {
            pins[i].strip->clear();
            pins[i].strip->show();
        }
    }

    Serial.println("!FAILSAFE");
}
