#include <Arduino.h>

#define MOTOR_IN1       19
#define MOTOR_IN2       18
#define ENCODER_A       16
#define ENCODER_B       4
#define PULSES_PER_REV  90.0f
#define GEAR_RATIO      12.0f
#define MAX_RPM         360.0f
#define MAX_PWM         255.0f
#define MIN_PWM         30.0f
#define DEADZONE_RPM    2.0f
#define SAMPLE_MS       20
#define CONTROL_MS      10
#define EMA_ALPHA       0.3f

/* ======== Encoder ======== */
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

/* ======== PID ======== */
static double PID_KP = 5.5;
static double PID_KI = 0.01;
static double PID_KD = 0.10;

static double pid_err   = 0.0;
static double pid_last  = 0.0;
static double pid_integ = 0.0;
static unsigned long pid_last_ms = 0;

static float setpoint = 0.0f;

static void pid_reset(void)
{
    pid_err = pid_last = pid_integ = 0.0;
    pid_last_ms = 0;
}

static double pid_update(double sp_mag, double fb_mag)
{
    unsigned long now = millis();
    double dt = (pid_last_ms == 0) ? 0.01 : (now - pid_last_ms) / 1000.0;
    if (dt <= 0.0) dt = 0.01;

    pid_err = sp_mag - fb_mag;

    if (pid_err > -50.0 && pid_err < 50.0) {
        pid_integ += pid_err * dt;
        if (pid_integ >  1000.0) pid_integ =  1000.0;
        if (pid_integ < -1000.0) pid_integ = -1000.0;
    }

    double d_err = (pid_err - pid_last) / dt;
    double u = PID_KP * pid_err + PID_KI * pid_integ + PID_KD * d_err;

    if (u < 0.0)     u = 0.0;
    if (u > MAX_PWM) u = MAX_PWM;

    pid_last    = pid_err;
    pid_last_ms = now;
    return u;
}

/* ======== Motor ======== */
static void set_motor_pwm(int pwm_signed)
{
    int pwm = (pwm_signed < 0) ? -pwm_signed : pwm_signed;
    if (pwm > 255) pwm = 255;

    if      (pwm_signed > 0) { analogWrite(MOTOR_IN1, pwm); analogWrite(MOTOR_IN2, 0);   }
    else if (pwm_signed < 0) { analogWrite(MOTOR_IN1, 0);   analogWrite(MOTOR_IN2, pwm); }
    else                     { analogWrite(MOTOR_IN1, 0);   analogWrite(MOTOR_IN2, 0);   }
}

/* ======== Control Loop ======== */
static unsigned long last_ctrl_ms = 0;

static void control_loop(void)
{
    unsigned long now = millis();
    if ((now - last_ctrl_ms) < CONTROL_MS) return;
    last_ctrl_ms = now;

    float sp_mag = (setpoint < 0.0f) ? -setpoint : setpoint;

    if (sp_mag < 1.0f) { pid_reset(); set_motor_pwm(0); return; }

    float abs_fb = (filtered_rpm < 0.0f) ? -filtered_rpm : filtered_rpm;
    float fb_pwm = (abs_fb / MAX_RPM) * MAX_PWM;
    if (fb_pwm < 5.0f) fb_pwm = 0.0f;

    int out_pwm = (int)pid_update((double)sp_mag, (double)fb_pwm);

    if (setpoint < 0.0f) out_pwm = -out_pwm;

    if (out_pwm != 0) {
        int a = (out_pwm < 0) ? -out_pwm : out_pwm;
        if (a < (int)MIN_PWM)
            out_pwm = (out_pwm > 0) ? (int)MIN_PWM : -(int)MIN_PWM;
    }

    if (out_pwm >  255) out_pwm =  255;
    if (out_pwm < -255) out_pwm = -255;

    set_motor_pwm(out_pwm);
}

/* ======== Serial Parser ======== */
static void parse_serial(void)
{
    if (!Serial.available()) return;
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;

    if (s.equalsIgnoreCase("stop")) {
        setpoint = 0.0f; pid_reset(); set_motor_pwm(0);
        Serial.println(">> Stopped.");
        return;
    }

    float val = s.toFloat();
    if (val >  255.0f) val =  255.0f;
    if (val < -255.0f) val = -255.0f;
    setpoint = val;
    Serial.printf(">> Setpoint: %.0f\n", setpoint);
}

/* ======== Debug ======== */
static void print_debug(void)
{
    static unsigned long last_p = 0;
    if ((millis() - last_p) < 500UL) return;
    last_p = millis();

    float abs_fb = (filtered_rpm < 0.0f) ? -filtered_rpm : filtered_rpm;
    float fb_pwm = (abs_fb / MAX_RPM) * MAX_PWM;
    float sp_mag = (setpoint < 0.0f) ? -setpoint : setpoint;

    Serial.printf("[PID] SP=%.0f  FB=%.1f  RPM=%.1f  err=%.2f\n",
                  sp_mag, fb_pwm, filtered_rpm, pid_err);
}

/* ======== Setup / Loop ======== */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("=== Motor PID Controller ===");
    Serial.println("Send: <-255..255>  or  'stop'");

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    analogWriteResolution(8);
    analogWriteFrequency(5000);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isr_enc_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isr_enc_b, CHANGE);

    last_sample_ms = millis();
    last_ctrl_ms   = millis();
    Serial.println("Ready.");
}

void loop(void)
{
    parse_serial();
    calculate_rpm();
    control_loop();
    print_debug();
}
