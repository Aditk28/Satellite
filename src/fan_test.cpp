/*
  fan_test.cpp — STANDALONE translation-fan ESC test, DSHOT300.   env: fantest

  Built up from a clean 3-channel reference that was CONFIRMED spinning the motor.
  Channel 4 was then added as the smallest possible delta: it is on PA0, the SAME
  PORT as the others, so dshotSend() is byte-identical to the confirmed-good
  version and is simply called a fourth time. No second function, no port
  parameter, no lookup table of GPIO pointers -- every one of those variants broke
  the previously-working channels, repeatedly.

  WHY DSHOT AND NOT SERVO PWM
  ---------------------------
  Servo PWM was tried first and measured perfect (TIM1 at 50.000 Hz, 999.8 us
  pulses, MOE set, verified with a meter at the ESC's own pad) and the ESC never
  responded. Cause: this ESC ships with Bluejay, a BLHeli_S fork that REMOVED
  analog input -- it is DSHOT-only. That also explains why throttle-range
  calibration produced no beeps: analog range calibration does not exist in it.

  THE PROTOCOL
  ------------
  16-bit frame, MSB first:   [11-bit value][1-bit telemetry request][4-bit CRC]
    value 0        = disarmed / stop
    value 1..47    = reserved commands (beep, direction, save, ...)
    value 48..2047 = throttle, 48 = minimum
  CRC = XOR of the three nibbles of (value<<1 | telemetry), low 4 bits.

  DSHOT300 bit period = 3.333 us    '0' = high 1.25 us    '1' = high 2.50 us

  Bit-banged from GPIO using the DWT cycle counter, interrupts masked for the
  ~53 us of a frame so nothing stretches a bit.

  ------------------------------------------------------------------ SAFETY ---
  PROPS OFF. Throttle capped at THROTTLE_CAP_PCT. Unrecognised input stops all.

  ----------------------------------------------------------------- WIRING ----
      ESC pad 1 -> D7 -> PA8       ESC pad 3 -> D2 -> PA10
      ESC pad 2 -> D8 -> PA9       ESC pad 4 -> A0 -> PA0  (NOT D9)
      ESC '-'   -> GND
      ESC '+' and 'N' -> NOT CONNECTED
*/

#include <Arduino.h>

#define HC05_EN_PIN    PC12
#define HC05_BAUD      115200
HardwareSerial hc05Serial(PC11, PC10);

/* DSHOT300 timing in CPU cycles @180 MHz. 1 bit = 3.333 us = 600 cycles. */
#define CYC_BIT        600
#define CYC_T0H        225     /* 1.25 us -> logical 0 */
#define CYC_T1H        450     /* 2.50 us -> logical 1 */

#define FRAME_PERIOD_MS  2     /* resend every 2 ms; ESCs disarm if frames stop */
#define THROTTLE_CAP_PCT 75.0f /* bring-up cap. Raise deliberately.             */

/* Channels live on GPIOA bits 8/9/10 (PA8/PA9/PA10) -- one port, so the whole
   frame is written with direct BSRR stores and no port lookup per bit. */
/* CHANNEL 4 IS ON A0 (PA0), NOT D9/PC7 -- deliberately.
   PC7 is on GPIOC, a different port, which forced either a port parameter or a
   duplicate of dshotSend(). Every attempt at that broke the working channels.
   PA0 is free, on the Arduino (female) header, and on GPIOA -- so dshotSend()
   stays byte-identical and is simply called a fourth time. Move the channel-4
   signal wire from D9 to A0. */
static const uint8_t PIN_BIT[5] = { 0, 8, 9, 10, 0 };   /* idx 1..4 -> PA8/9/10/PA0 */
static uint16_t dshotValue[5]   = { 0, 0, 0, 0, 0 };    /* 0 = disarmed */
static int      selected        = 2;

static void say(const String& s) { Serial.println(s); hc05Serial.println(s); }

/* ---- cycle-accurate delay -------------------------------------------------- */
static inline void waitCycles(uint32_t from, uint32_t n) {
  while ((DWT->CYCCNT - from) < n) { __NOP(); }
}

/* ---- build a DSHOT frame (value + telemetry bit + CRC) --------------------- */
static uint16_t dshotFrame(uint16_t value, bool telem) {
  uint16_t packet = (uint16_t)((value << 1) | (telem ? 1 : 0));
  uint16_t csum = 0, tmp = packet;
  for (int i = 0; i < 3; i++) { csum ^= tmp; tmp >>= 4; }
  return (uint16_t)((packet << 4) | (csum & 0x0F));
}

/* ---- bit-bang one frame on one channel ------------------------------------- */
static void dshotSend(uint8_t bit, uint16_t value) {
  uint16_t frame = dshotFrame(value, false);
  const uint32_t setMask = (1UL << bit);
  const uint32_t clrMask = (1UL << (bit + 16));

  noInterrupts();                       /* ~53 us; a stretched bit corrupts the frame */
  for (int i = 15; i >= 0; i--) {
    uint32_t t0 = DWT->CYCCNT;
    GPIOA->BSRR = setMask;
    waitCycles(t0, (frame & (1U << i)) ? CYC_T1H : CYC_T0H);
    GPIOA->BSRR = clrMask;
    waitCycles(t0, CYC_BIT);
  }
  interrupts();
}

static void sendAll(void) {
  for (int ch = 1; ch <= 4; ch++) dshotSend(PIN_BIT[ch], dshotValue[ch]);
}

/* ---- commands -------------------------------------------------------------- */
static void stopAll(const char* why) {
  for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = 0;
  say("STOP (" + String(why) + ") -- all channels to DSHOT 0 (disarmed)");
}

static void setThrottlePct(float pct) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > THROTTLE_CAP_PCT) {
    pct = THROTTLE_CAP_PCT;
    say("(capped at " + String(THROTTLE_CAP_PCT, 0) + "%)");
  }
  /* 0% -> 48 (minimum throttle), 100% -> 2047 */
  uint16_t v = (pct <= 0.0f) ? 48 : (uint16_t)(48.0f + pct * (2047.0f - 48.0f) / 100.0f);
  dshotValue[selected] = v;
  say("ch" + String(selected) + "  throttle " + String(pct, 1)
      + "%  ->  DSHOT " + String(v));
}

static void handleLine(String s) {
  s.trim();
  if (!s.length()) return;
  char c = (char)toupper(s.charAt(0));

  if (c == 'C') {
    int n = s.substring(1).toInt();
    if (n >= 1 && n <= 4) { stopAll("channel change"); selected = n;
                            say("selected channel " + String(selected)); }
    else say("channel must be 1-4");
    return;
  }
  /* B : DSHOT command 1 = beep. Makes the ESC chirp WITHOUT spinning the motor --
     the cleanest possible proof that it is decoding our frames. */
  if (c == 'B') {
    dshotValue[selected] = 1;
    say("ch" + String(selected) + "  DSHOT cmd 1 (BEEP) -- listen for a chirp");
    for (int i = 0; i < 20; i++) { sendAll(); delay(2); }
    dshotValue[selected] = 0;
    say("beep command sent, back to 0");
    return;
  }
  if (c >= '0' && c <= '9') { setThrottlePct(s.toFloat()); return; }
  stopAll("operator");
}

static void pump(Stream& s, String& buf) {
  while (s.available()) {
    char ch = (char)s.read();
    if (ch == '\n' || ch == '\r') { if (buf.length()) { handleLine(buf); buf = ""; } }
    else if (buf.length() < 32) buf += ch;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  /* DWT cycle counter — our sub-microsecond time base for bit timing. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  pinMode(PA8,  OUTPUT); digitalWrite(PA8,  LOW);   /* ch1 = D7 */
  pinMode(PA9,  OUTPUT); digitalWrite(PA9,  LOW);   /* ch2 = D8 */
  pinMode(PA10, OUTPUT); digitalWrite(PA10, LOW);   /* ch3 = D2 */
  pinMode(PA0,  OUTPUT); digitalWrite(PA0,  LOW);   /* ch4 = A0 (NOT D9) */

  say("");
  say("=== fan ESC test — DSHOT300, 4 channels ===");
  say("PROPS OFF.");
  say("sending DSHOT 0 (disarmed) to arm the ESC...");
  for (int i = 0; i < 500; i++) { sendAll(); delay(2); }   /* ~1 s of zeros */
  say("armed (expect a tone from the ESC)");
  say("");
  say("commands (USB or HC-05, newline-terminated):");
  say("  <number>  throttle percent 0-" + String(THROTTLE_CAP_PCT, 0));
  say("  B         BEEP via DSHOT cmd 1 — proves decoding WITHOUT spinning");
  say("  C<n>      select channel 1-4   (1=D7 2=D8 3=D2 4=A0)");
  say("  X / any   STOP");
  say("try 10.");
}

void loop() {
  static String bufU, bufB;
  pump(Serial,     bufU);
  pump(hc05Serial, bufB);

  static uint32_t lastFrame = 0;
  if (millis() - lastFrame >= FRAME_PERIOD_MS) { lastFrame = millis(); sendAll(); }

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 2000) {
    lastBeat = millis();
    say("[alive " + String(millis() / 1000) + "s]  sel=ch" + String(selected)
        + "  dshot: " + String(dshotValue[1]) + " / " + String(dshotValue[2])
        + " / " + String(dshotValue[3]));
  }
}
