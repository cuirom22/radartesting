// --- Motor 1 Pins ---
#define M1_PWM   4
#define M1_INA   16
#define M1_INB   17
#define M1_EN    36   // optional: tie to 5V if not connected to ESP32

// --- Motor 2 Pins ---
#define M2_PWM   5
#define M2_INA   18
#define M2_INB   7    // replaced GPIO19 (USB D-) with GPIO7
#define M2_EN    37   // optional: tie to 5V if not connected to ESP32

// --- PWM Settings ---
#define PWM_FREQ     20000   // 20 kHz
#define PWM_RES      10      // 10-bit resolution (0–1023)

// --- LEDC Channels ---
#define CH_M1  0
#define CH_M2  1