// AuxStepperDriver.ino
// Arduino Uno + CNC Shield V3/V4 — auxiliary stepper axis driver.
//
// The host sends one command with motion parameters; this firmware runs the
// full trapezoidal velocity profile internally and sends DONE when complete.
//
// Serial protocol (115200 baud, newline-terminated ASCII):
//
//  HOST -> ARDUINO
//    ID?                                  Request device identifier
//    E:<0|1>                              Enable (1) or disable (0) all drivers
//    D:<axis>,<dir>                       Set direction — axis 0-3, dir 0=CW 1=CCW
//    M:<axis>,<steps>,<vel>,<acc>,<dec>   Indexed move: trapezoidal profile
//                                         vel in steps/sec, acc/dec in steps/sec²
//                                         Responds OK immediately; DONE:<axis> on completion
//    C:<axis>,<vel>,<acc>                 Continuous stepping — ramps up to vel, holds until stopped
//    S:<axis>,<dec>                       Soft-stop: decelerate axis to rest
//    X                                    Immediate stop all axes
//
//  Edge-device digital IO on the spare pins (same line format the controller uses
//  for a Nano edge device, so they share one parser). Only the pins the stepper
//  driver does not use are allowed: D9, D10, D11, A0-A5. No HX711/NeoPixel here —
//  a load cell's blocking read would stall the software step loop.
//    CFG:<pin>,I            Configure pin as a digital input (INPUT_PULLUP)
//    CFG:<pin>,O            Configure pin as a digital output
//    GET                    Request full IO state -> STATE:...
//    SET:<pin>,<0|1>        Drive an output pin
//
//  ARDUINO -> HOST
//    RDY                  On boot
//    ID:AUX_STEPPER_001   Response to ID?
//    OK                   Command accepted
//    DONE:<axis>          Indexed M move complete
//    STATE:<pin>,<I|O>,<val>;...   Full IO state (response to GET)
//    CHG:<pin>,<val>      An input pin changed
//    ERR                  Bad command or invalid parameter
//
// Axis map (CNC Shield V3/V4):
//    0 = X  (Step D2, Dir D5)
//    1 = Y  (Step D3, Dir D6)
//    2 = Z  (Step D4, Dir D7)
//    3 = A  (Step D12, Dir D13)
//    Enable = D8 (active LOW, shared)

#define DEVICE_ID   "AUX_STEPPER_001"
#define SERIAL_BAUD 115200
#define MAX_AXES    4
#define RX_BUF_SIZE 64

static const uint8_t STEP_D[MAX_AXES] = { _BV(2), _BV(3), _BV(4), 0      };
static const uint8_t STEP_B[MAX_AXES] = { 0,      0,      0,      _BV(4) };
static const uint8_t DIR_D[MAX_AXES]  = { _BV(5), _BV(6), _BV(7), 0      };
static const uint8_t DIR_B[MAX_AXES]  = { 0,      0,      0,      _BV(5) };

typedef enum : uint8_t { IDLE, ACCEL, CRUISE, DECEL, CONTINUOUS } Phase;

static Phase    ax_phase[MAX_AXES];
static bool     ax_indexed[MAX_AXES];   // true = indexed M move (sends DONE on finish)
static float    ax_vel[MAX_AXES];       // current velocity, steps/sec
static float    ax_target[MAX_AXES];    // cruise velocity
static float    ax_accel[MAX_AXES];     // steps/sec²
static float    ax_decel[MAX_AXES];     // steps/sec²
static int32_t  ax_total[MAX_AXES];     // total steps for indexed move
static int32_t  ax_done[MAX_AXES];      // steps completed
static uint32_t ax_next_us[MAX_AXES];   // micros() timestamp for next step

#define MIN_VEL 50.0f  // minimum velocity (steps/sec) — prevents division by zero

// ── Edge-device digital IO on the spare pins ───────────────────────────────────
// The pins the CNC-shield stepper driver leaves free. INPUT_PULLUP inputs and plain
// OUTPUTs, polled cheaply so the software step loop keeps its timing (no HX711).
#define IO_PIN_COUNT 9
static const uint8_t IO_PINS[IO_PIN_COUNT] = { 9, 10, 11, A0, A1, A2, A3, A4, A5 };
static char     io_type[IO_PIN_COUNT];    // 0 = unconfigured, 'I' = input, 'O' = output
static uint8_t  io_val [IO_PIN_COUNT];    // last known level (0/1)
static uint32_t io_next_scan_ms = 0;

static int ioIndex(uint8_t pin)
{
    for (uint8_t i = 0; i < IO_PIN_COUNT; i++)
        if (IO_PINS[i] == pin) return i;
    return -1;   // not a configurable IO pin (stepper/serial pin or out of range)
}

static void stepAxis(uint8_t i)
{
    if (ax_phase[i] == IDLE) return;

    uint32_t now = micros();
    if ((int32_t)(now - ax_next_us[i]) < 0) return;

    // Fire step pulse
    PORTD |= STEP_D[i];
    PORTB |= STEP_B[i];
    delayMicroseconds(2);
    PORTD &= ~STEP_D[i];
    PORTB &= ~STEP_B[i];

    float v  = ax_vel[i];
    float dt = 1.0f / v;

    if (ax_phase[i] == CONTINUOUS) {
        if (v < ax_target[i]) {
            float nv = v + ax_accel[i] * dt;
            if (nv > ax_target[i]) nv = ax_target[i];
            ax_vel[i] = nv;
            v = nv;
        }
    } else {
        // Indexed or soft-stop decel
        ax_done[i]++;
        int32_t rem = ax_total[i] - ax_done[i];

        if (rem <= 0) {
            ax_phase[i] = IDLE;
            if (ax_indexed[i]) {
                Serial.print("DONE:");
                Serial.println(i);
            }
            return;
        }

        // Velocity update for current phase
        if (ax_phase[i] == ACCEL) {
            float nv = v + ax_accel[i] * dt;
            if (nv >= ax_target[i]) { nv = ax_target[i]; ax_phase[i] = CRUISE; }
            ax_vel[i] = nv;
            v = nv;
        } else if (ax_phase[i] == DECEL) {
            float nv = v - ax_decel[i] * dt;
            if (nv < MIN_VEL) nv = MIN_VEL;
            ax_vel[i] = nv;
            v = nv;
        }

        // Trigger decel when remaining steps equal braking distance
        if (ax_phase[i] != DECEL) {
            float brake = (v * v) / (2.0f * ax_decel[i]);
            if ((float)rem <= brake) ax_phase[i] = DECEL;
        }
    }

    uint32_t ival = (uint32_t)(1000000.0f / v);
    if (ival < 20) ival = 20;
    ax_next_us[i] = now + ival;
}

// ── Serial ────────────────────────────────────────────────────────────

static char    rx_buf[RX_BUF_SIZE];
static uint8_t rx_len = 0;

static uint32_t parseU32(const char** p)
{
    while (**p == ' ' || **p == ',') (*p)++;
    uint32_t v = 0;
    while (**p >= '0' && **p <= '9') v = v * 10 + (uint8_t)(*(*p)++ - '0');
    return v;
}

// ── Edge-device IO handlers ─────────────────────────────────────────────────────

// CFG:<pin>,<I|O>
static void handleIoCfg(const char* a)
{
    const char* p = a;
    uint8_t pin = (uint8_t)parseU32(&p);
    while (*p == ' ' || *p == ',') p++;
    char t = *p;
    int idx = ioIndex(pin);
    if (idx < 0 || (t != 'I' && t != 'O')) { Serial.println("ERR"); return; }
    io_type[idx] = t;
    if (t == 'O') { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); io_val[idx] = 0; }
    else          { pinMode(pin, INPUT_PULLUP); io_val[idx] = digitalRead(pin) ? 1 : 0; }
    Serial.println("OK");
}

// SET:<pin>,<0|1> — drive an output
static void handleIoSet(const char* a)
{
    const char* p = a;
    uint8_t pin = (uint8_t)parseU32(&p);
    uint8_t v   = (uint8_t)parseU32(&p);
    int idx = ioIndex(pin);
    if (idx < 0 || io_type[idx] != 'O') { Serial.println("ERR"); return; }
    digitalWrite(pin, v ? HIGH : LOW);
    io_val[idx] = v ? 1 : 0;
    Serial.println("OK");
}

// GET -> STATE:<pin>,<I|O>,<val>;...  (prints "STATE:" with no body when nothing configured)
static void emitIoState()
{
    Serial.print("STATE:");
    bool first = true;
    for (uint8_t i = 0; i < IO_PIN_COUNT; i++) {
        if (!io_type[i]) continue;
        uint8_t v = (io_type[i] == 'I') ? (digitalRead(IO_PINS[i]) ? 1 : 0) : io_val[i];
        io_val[i] = v;
        if (!first) Serial.print(';');
        first = false;
        Serial.print(IO_PINS[i]); Serial.print(','); Serial.print(io_type[i]); Serial.print(','); Serial.print(v);
    }
    Serial.println();
}

static void handleCommand(const char* line)
{
    if (strcmp(line, "ID?") == 0) { Serial.println("ID:" DEVICE_ID); return; }

    // Edge-device IO commands. Checked before the single-letter guard below, which
    // would otherwise ERR these multi-letter commands (SET: vs the stepper S: differ
    // at line[1], so there is no collision).
    if (strncmp(line, "CFG:", 4) == 0) { handleIoCfg(line + 4); return; }
    if (strcmp(line, "GET")   == 0)    { emitIoState();         return; }
    if (strncmp(line, "SET:", 4) == 0) { handleIoSet(line + 4); return; }

    if (line[0] == 'X' && line[1] == '\0') {
        for (uint8_t i = 0; i < MAX_AXES; i++) ax_phase[i] = IDLE;
        Serial.println("OK");
        return;
    }

    if (line[1] != ':') { Serial.println("ERR"); return; }
    const char* p = line + 2;

    switch (line[0]) {

        case 'E': {
            uint8_t en = (uint8_t)parseU32(&p);
            if (en) PORTB &= ~_BV(0);
            else    PORTB |=  _BV(0);
            Serial.println("OK");
            break;
        }

        case 'D': {
            uint8_t ax  = (uint8_t)parseU32(&p);
            uint8_t dir = (uint8_t)parseU32(&p);
            if (ax >= MAX_AXES) { Serial.println("ERR"); return; }
            if (dir) { PORTD |=  DIR_D[ax]; PORTB |=  DIR_B[ax]; }
            else      { PORTD &= ~DIR_D[ax]; PORTB &= ~DIR_B[ax]; }
            Serial.println("OK");
            break;
        }

        // M:<axis>,<steps>,<vel>,<accel>,<decel>
        case 'M': {
            uint8_t  ax    = (uint8_t)parseU32(&p);
            uint32_t steps = parseU32(&p);
            uint32_t vel   = parseU32(&p);
            uint32_t acc   = parseU32(&p);
            uint32_t dec   = parseU32(&p);
            if (ax >= MAX_AXES || steps == 0 || vel == 0) { Serial.println("ERR"); return; }
            ax_total[ax]   = (int32_t)steps;
            ax_done[ax]    = 0;
            ax_target[ax]  = (float)vel;
            ax_accel[ax]   = (float)(acc > 0 ? acc : vel);
            ax_decel[ax]   = (float)(dec > 0 ? dec : vel);
            ax_vel[ax]     = MIN_VEL;
            ax_indexed[ax] = true;
            ax_next_us[ax] = micros();
            ax_phase[ax]   = ACCEL;
            Serial.println("OK");
            break;
        }

        // C:<axis>,<vel>,<accel>
        case 'C': {
            uint8_t  ax  = (uint8_t)parseU32(&p);
            uint32_t vel = parseU32(&p);
            uint32_t acc = parseU32(&p);
            if (ax >= MAX_AXES || vel == 0) { Serial.println("ERR"); return; }
            ax_target[ax]  = (float)vel;
            ax_accel[ax]   = (float)(acc > 0 ? acc : vel);
            ax_vel[ax]     = MIN_VEL;
            ax_indexed[ax] = false;
            ax_next_us[ax] = micros();
            ax_phase[ax]   = CONTINUOUS;
            Serial.println("OK");
            break;
        }

        // S:<axis>,<decel> — soft stop
        case 'S': {
            uint8_t  ax  = (uint8_t)parseU32(&p);
            uint32_t dec = parseU32(&p);
            if (ax >= MAX_AXES) { Serial.println("ERR"); return; }
            if (ax_phase[ax] != IDLE) {
                float d  = (float)(dec > 0 ? dec : 1);
                float v  = ax_vel[ax];
                ax_decel[ax]  = d;
                ax_indexed[ax]= false;
                ax_total[ax]  = ax_done[ax] + (int32_t)(v * v / (2.0f * d)) + 2;
                ax_phase[ax]  = DECEL;
            }
            Serial.println("OK");
            break;
        }

        default:
            Serial.println("ERR");
    }
}

void setup()
{
    DDRD |= _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7);
    DDRB |= _BV(0) | _BV(4) | _BV(5);
    PORTB |= _BV(0);  // EN HIGH = disabled at startup

    for (uint8_t i = 0; i < MAX_AXES; i++) {
        ax_phase[i]   = IDLE;
        ax_vel[i]     = MIN_VEL;
        ax_next_us[i] = 0;
    }

    Serial.begin(SERIAL_BAUD);
    Serial.println("RDY");
}

void loop()
{
    for (uint8_t i = 0; i < MAX_AXES; i++)
        stepAxis(i);

    // Poll configured inputs and report changes. Throttled to ~10 ms and only a few
    // digitalReads, so it never meaningfully disturbs the step timing above.
    uint32_t nowMs = millis();
    if ((int32_t)(nowMs - io_next_scan_ms) >= 0) {
        io_next_scan_ms = nowMs + 10;
        for (uint8_t i = 0; i < IO_PIN_COUNT; i++) {
            if (io_type[i] != 'I') continue;
            uint8_t v = digitalRead(IO_PINS[i]) ? 1 : 0;
            if (v != io_val[i]) {
                io_val[i] = v;
                Serial.print("CHG:"); Serial.print(IO_PINS[i]); Serial.print(','); Serial.println(v);
            }
        }
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            while (rx_len && rx_buf[rx_len - 1] == '\r') rx_len--;
            rx_buf[rx_len] = '\0';
            if (rx_len > 0) handleCommand(rx_buf);
            rx_len = 0;
        } else if (rx_len < RX_BUF_SIZE - 1) {
            rx_buf[rx_len++] = c;
        }
    }
}
