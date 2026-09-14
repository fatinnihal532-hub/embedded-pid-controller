/*
 * Embedded PID temperature controller  -  ATmega328P / Arduino Uno
 * ---------------------------------------------------------------
 * A discrete PID controller running at a fixed 10 Hz, driving a PWM output,
 * with the heater and its thermal behaviour simulated inside the firmware so
 * the whole thing runs in a browser simulator with no hardware.
 *
 * The controller code is the real thing. Only the plant is simulated, and the
 * plant model is a first-order lag, which is what a heater, a motor winding
 * and a tank of water all look like from the controller's point of view:
 *
 *     tau * dT/dt = -(T - T_ambient) + K * u
 *
 * Serial shell at 115200 baud, one command per line:
 *
 *     ?            list the commands
 *     kp <value>   set the proportional gain
 *     ki <value>   set the integral gain      (per second)
 *     kd <value>   set the derivative gain    (seconds)
 *     sp <value>   set the target temperature in degrees C
 *     step <value> jump the target and restart CSV logging, for a step test
 *     dist <value> inject a disturbance in degrees C, e.g. an opened window
 *     noise <0|1>  add sensor noise
 *     csv <0|1>    stop or start the CSV stream
 *     reset        clear the controller state and return to ambient
 *
 * Output columns: seconds,setpoint,temperature,output_pct,p_term,i_term,d_term
 */

/* ---------------- pins ---------------- */
static const int PIN_HEATER = 9;   /* PWM output; an LED stands in for the heater */

/* ---------------- loop timing ---------------- */
static const uint32_t PERIOD_MS = 100;             /* 10 Hz control loop */
static const float    DT        = PERIOD_MS / 1000.0f;

/* ---------------- plant model ---------------- */
static const float T_AMBIENT = 25.0f;   /* degrees C with the heater off      */
static const float PLANT_K   = 60.0f;   /* degrees C rise at full power       */
static const float PLANT_TAU = 25.0f;   /* seconds to reach 63 % of the rise  */
static const float DEAD_TIME = 2.0f;    /* seconds of transport delay         */

static const int DEAD_SLOTS = (int)(DEAD_TIME / DT) + 1;
static float delayLine[24];             /* must be at least DEAD_SLOTS */
static int   delayIndex = 0;

/* ---------------- controller state ---------------- */
static float kp = 8.0f;
static float ki = 0.45f;
static float kd = 12.0f;

static float setpoint   = 45.0f;
static float integral   = 0.0f;
static float lastMeas   = T_AMBIENT;
static bool  firstPass  = true;

static const float OUT_MIN = 0.0f;
static const float OUT_MAX = 100.0f;    /* percent of full heater power */

/* ---------------- plant and I/O state ---------------- */
static float temperature = T_AMBIENT;
static float disturbance = 0.0f;
static bool  addNoise    = false;
static bool  csvOn       = true;
static uint32_t nextRun  = 0;
static uint32_t tickCount = 0;

static char   cmdBuf[32];
static uint8_t cmdLen = 0;

/* A tiny deterministic noise source. rand() on AVR is fine, but this keeps the
 * amplitude predictable and costs almost nothing. */
static float pseudoNoise()
{
    static uint16_t state = 0xACE1u;
    state ^= (uint16_t)(state << 7);
    state ^= (uint16_t)(state >> 9);
    state ^= (uint16_t)(state << 8);
    return ((float)(state & 0x3FF) / 1023.0f - 0.5f) * 0.6f;   /* +/- 0.3 C */
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ------------------------------------------------------------------ */
/* The controller                                                      */
/*                                                                     */
/* Two details separate this from a textbook transcription:            */
/*                                                                     */
/* 1. The derivative acts on the measurement, not on the error. If it  */
/*    acted on the error, every change of set point would produce an   */
/*    infinite derivative for one sample and slam the output to its    */
/*    limit. That artefact is called derivative kick.                  */
/*                                                                     */
/* 2. The integral only accumulates when the output is not saturated,  */
/*    and it is clamped as well. Without this, a long approach to the  */
/*    set point winds the integral up to a huge value and the output   */
/*    stays at full power well past the target. That is integral       */
/*    windup, and it is the most common reason a PID loop overshoots.  */
/* ------------------------------------------------------------------ */
static float pidUpdate(float measurement, float *pOut, float *iOut, float *dOut)
{
    float error = setpoint - measurement;

    float p = kp * error;

    if (firstPass) {
        lastMeas = measurement;
        firstPass = false;
    }
    float d = -kd * (measurement - lastMeas) / DT;
    lastMeas = measurement;

    float unsaturated = p + integral + d;

    /* conditional integration: only integrate if it will not push further
     * into a limit that is already reached */
    bool windingUp = (unsaturated >= OUT_MAX && error > 0.0f) ||
                     (unsaturated <= OUT_MIN && error < 0.0f);
    if (!windingUp) {
        integral += ki * error * DT;
        integral = clampf(integral, OUT_MIN - 20.0f, OUT_MAX);
    }

    float output = clampf(p + integral + d, OUT_MIN, OUT_MAX);

    *pOut = p;
    *iOut = integral;
    *dOut = d;
    return output;
}

/* ------------------------------------------------------------------ */
/* The simulated plant, discretised with the forward Euler rule        */
/* ------------------------------------------------------------------ */
static void plantUpdate(float outputPercent)
{
    /* transport delay: power applied now is felt DEAD_TIME later */
    delayLine[delayIndex] = outputPercent;
    delayIndex = (delayIndex + 1) % DEAD_SLOTS;
    float delayed = delayLine[delayIndex];

    float drive = PLANT_K * (delayed / 100.0f);
    float dTdt  = (-(temperature - T_AMBIENT - disturbance) + drive) / PLANT_TAU;
    temperature += dTdt * DT;
}

/* ------------------------------------------------------------------ */
/* Serial shell                                                        */
/* ------------------------------------------------------------------ */
static void printHelp()
{
    Serial.println(F("# commands: ? | kp v | ki v | kd v | sp v | step v |"));
    Serial.println(F("#           dist v | noise 0|1 | csv 0|1 | reset"));
    Serial.print(F("# now kp=")); Serial.print(kp);
    Serial.print(F(" ki="));      Serial.print(ki);
    Serial.print(F(" kd="));      Serial.print(kd);
    Serial.print(F(" sp="));      Serial.println(setpoint);
}

static void printCsvHeader()
{
    Serial.println(F("seconds,setpoint,temperature,output_pct,p_term,i_term,d_term"));
}

static void resetController()
{
    integral = 0.0f;
    firstPass = true;
    temperature = T_AMBIENT;
    for (int i = 0; i < DEAD_SLOTS; i++) delayLine[i] = 0.0f;
    tickCount = 0;
}

static bool matches(const char *cmd, const char *name, float *value)
{
    size_t n = strlen(name);
    if (strncmp(cmd, name, n) != 0) return false;
    if (cmd[n] != ' ' && cmd[n] != '\0') return false;
    *value = atof(cmd + n);
    return true;
}

static void handleCommand(char *cmd)
{
    float v = 0.0f;

    if (cmd[0] == '?' || cmd[0] == 'h') { printHelp(); return; }
    if (strcmp(cmd, "reset") == 0)      { resetController(); printCsvHeader(); return; }

    if (matches(cmd, "kp", &v))    { kp = v; }
    else if (matches(cmd, "ki", &v)) { ki = v; integral = 0.0f; }
    else if (matches(cmd, "kd", &v)) { kd = v; }
    else if (matches(cmd, "sp", &v)) { setpoint = v; }
    else if (matches(cmd, "step", &v)) { setpoint = v; tickCount = 0; printCsvHeader(); return; }
    else if (matches(cmd, "dist", &v)) { disturbance = v; }
    else if (matches(cmd, "noise", &v)) { addNoise = (v != 0.0f); }
    else if (matches(cmd, "csv", &v)) { csvOn = (v != 0.0f); if (csvOn) printCsvHeader(); return; }
    else { Serial.println(F("# unknown command, type ? for help")); return; }

    Serial.print(F("# ok "));
    Serial.println(cmd);
}

static void pollSerial()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmdBuf[cmdLen] = '\0';
            if (cmdLen > 0) handleCommand(cmdBuf);
            cmdLen = 0;
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);
    pinMode(PIN_HEATER, OUTPUT);
    resetController();
    delay(200);
    Serial.println(F("# Embedded PID temperature controller"));
    printHelp();
    printCsvHeader();
    nextRun = millis();
}

void loop()
{
    pollSerial();

    uint32_t now = millis();
    if ((int32_t)(now - nextRun) < 0)
        return;
    nextRun += PERIOD_MS;          /* fixed interval, no drift from jitter */

    float measured = temperature + (addNoise ? pseudoNoise() : 0.0f);

    float p, i, d;
    float output = pidUpdate(measured, &p, &i, &d);

    analogWrite(PIN_HEATER, (int)(output * 255.0f / 100.0f));
    plantUpdate(output);

    if (csvOn) {
        float seconds = tickCount * DT;
        Serial.print(seconds, 1);   Serial.print(',');
        Serial.print(setpoint, 2);  Serial.print(',');
        Serial.print(measured, 3);  Serial.print(',');
        Serial.print(output, 2);    Serial.print(',');
        Serial.print(p, 2);         Serial.print(',');
        Serial.print(i, 2);         Serial.print(',');
        Serial.println(d, 2);
    }
    tickCount++;
}
