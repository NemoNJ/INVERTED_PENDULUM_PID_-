#define ENCODER_A       16
#define ENCODER_B       4
#define PULSES_PER_REV  90.0f
#define GEAR_RATIO      12.0f
#define DEADZONE_RPM    2.0f
#define SAMPLE_MS       20
#define MAX_RPM         360.0f
#define EMA_ALPHA       0.3f

volatile long g_ticks    = 0;
static long   last_ticks = 0;
static long   last_diff  = 0;

static float raw_rpm      = 0.0f;
static float filtered_rpm = 0.0f;
static unsigned long last_sample_ms = 0;

void IRAM_ATTR isr_enc_a(void)
{
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) g_ticks++;
    else                                                   g_ticks--;
}
void IRAM_ATTR isr_enc_b(void)
{
    if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B)) g_ticks++;
    else                                                   g_ticks--;
}

static void calculate_rpm(void)
{
    unsigned long now = millis();
    if ((now - last_sample_ms) < SAMPLE_MS) return;

    double dt = (now - last_sample_ms) / 1000.0;
    if (dt < 0.001) dt = 0.001;
    if (dt > 0.500) dt = 0.500;

    long tick_diff = g_ticks - last_ticks;

    long max_exp = (long)((MAX_RPM * PULSES_PER_REV * GEAR_RATIO * dt / 60.0) * 1.5);
    if (max_exp > 0 && labs(tick_diff) > max_exp)
        tick_diff = (last_diff + tick_diff / 2) / 2;
    last_diff = tick_diff;

    double rpm = (tick_diff * 60.0) / (dt * PULSES_PER_REV * GEAR_RATIO);
    if (rpm > -DEADZONE_RPM && rpm < DEADZONE_RPM) rpm = 0.0;
    raw_rpm = (float)rpm;

    /* EMA */
    filtered_rpm = EMA_ALPHA * raw_rpm + (1.0f - EMA_ALPHA) * filtered_rpm;

    last_ticks     = g_ticks;
    last_sample_ms = now;
}

void setup(void)
{
    Serial.begin(115200);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isr_enc_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isr_enc_b, CHANGE);
    last_sample_ms = millis();
    Serial.println("Encoder Ready.");
}

void loop(void)
{
    calculate_rpm();

    static unsigned long last_print = 0;
    if ((millis() - last_print) >= 500UL) {
        last_print = millis();
        Serial.printf("Ticks: %ld | Raw RPM: %.2f | Filtered RPM: %.2f\n",
                      g_ticks, raw_rpm, filtered_rpm);
    }
}
