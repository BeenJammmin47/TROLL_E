/*
 * troll_e_uwb.ino -- TROLL-E UWB boards (Makerfabs MaUWB DW3000 with STM32 AT command)
 *
 * One sketch for both boards. Change ON_CART below before uploading:
 *   ON_CART 0 : the user's board = UWB ANCHOR 0 (the cart's tags range to it)
 *   ON_CART 1 : the cart's board = UWB TAG. Tags get the distance reports, so this board
 *               prints every UWB line to USB and sends just "D,<cm>" lines out
 *               CART_TX_PIN to the TROLL-E PCB (J10 pin 1).
 *               Second cart board later (J11): CART_TAG_ID 1.
 *
 * Arduino IDE: ESP32 board package, board "ESP32S3 Dev Module", Serial Monitor 115200.
 * You can type AT commands in the Serial Monitor; they are passed to the UWB module.
 */
#include <Arduino.h>

#define ON_CART      1      // 1 = cart board (UWB tag), 0 = user's board (UWB anchor)

// Pins between the ESP32 and the UWB AT module (Makerfabs' ESP32-S3 examples).
// If Makerfabs' example for your board version (esp32_at_a0.ino) uses other numbers, copy them here.
#define UWB_RST      16
#define UWB_RXD      18     // ESP32 RX <- UWB module TX
#define UWB_TXD      17     // ESP32 TX -> UWB module RX

// Cart board only: the ESP32 pin wired to the TROLL-E PCB, J10 pin 1.
// Use a GPIO on the board's 12-pin header (not 16/17/18/38/39). -1 = USB output only.
#define CART_TX_PIN  43     // <-- set this

#define CART_TAG_ID  0      // cart boards: tag 0 (J10), tag 1 (J11). The user's anchor is always ID 0.
#define UWB_RATE     1      // 1 = 6.8M (module's own default), 0 = 850K (longer range). Same on both boards.

// 1 = send the settings to the UWB module and save them (do this once per board),
// 0 = normal run: don't touch the settings, just listen and forward distances.
#define CONFIGURE    0

HardwareSerial uwb(2);
HardwareSerial cart(1);
String line;

// Send one AT command, print the reply, return it
String sendAT(const char *cmd, uint32_t waitMs)
{
  uwb.print(cmd);
  uwb.print("\r\n");
  String reply;
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    while (uwb.available()) reply += (char)uwb.read();
    if (reply.indexOf("OK") >= 0 || reply.indexOf("ERR") >= 0) { delay(20); break; }
  }
  while (uwb.available()) reply += (char)uwb.read();
  reply.trim();
  Serial.printf(">> %s   %s\n", cmd, reply.length() ? reply.c_str() : "(no reply)");
  return reply;
}

void setup()
{
  Serial.begin(115200);
  uwb.setRxBufferSize(1024);          // module start-up text is longer than the 256-byte default
  uwb.begin(115200, SERIAL_8N1, UWB_RXD, UWB_TXD);
#if ON_CART
  if (CART_TX_PIN >= 0) cart.begin(115200, SERIAL_8N1, -1, CART_TX_PIN);
#endif
  pinMode(UWB_RST, OUTPUT);           // held high, as in Makerfabs' examples
  digitalWrite(UWB_RST, HIGH);

  const int uwbId = ON_CART ? CART_TAG_ID : 0;

#if CONFIGURE
  delay(1500);
  Serial.printf("\n=== TROLL-E UWB SETUP: %s, %s %d ===\n", ON_CART ? "CART" : "USER",
                ON_CART ? "tag" : "anchor", uwbId);
  char cmd[40];
  sendAT("AT?", 1000);
  sendAT("AT+RESTORE", 5000);
  delay(1500);
  snprintf(cmd, sizeof(cmd), "AT+SETCFG=%d,%d,%d,1", uwbId, ON_CART ? 0 : 1, UWB_RATE);   // role: 0 tag, 1 anchor
  sendAT(cmd, 2000);
  // capacity 10 tags, slot 15 ms (850K) / 10 ms (6.8M). Newer firmware wants a 3rd parameter.
  snprintf(cmd, sizeof(cmd), "AT+SETCAP=10,%d,0", UWB_RATE ? 10 : 15);
  if (sendAT(cmd, 2000).indexOf("OK") < 0) {
    snprintf(cmd, sizeof(cmd), "AT+SETCAP=10,%d", UWB_RATE ? 10 : 15);
    sendAT(cmd, 2000);
  }
  sendAT("AT+SETRPT=1", 2000);        // report ranges automatically
  sendAT("AT+SAVE", 2000);
  // No AT+RESTART: it leaves this module silent. The saved settings start at the next power-up.
  Serial.println("Settings saved. Now set CONFIGURE 0, upload, then unplug the board for 5 s.");
#else
  // Normal run. The ESP32 and the UWB module power up together; capture the module's start-up
  // text (it lists the saved Role / Id / Rate) and print it once the Serial Monitor has reconnected.
  String boot;
  uint32_t t0 = millis();
  while (millis() - t0 < 6000) { while (uwb.available()) boot += (char)uwb.read(); }
  Serial.printf("\n=== TROLL-E UWB: %s, %s %d ===\n", ON_CART ? "CART" : "USER",
                ON_CART ? "tag" : "anchor", uwbId);
  Serial.println("--- UWB module start-up text ---");
  Serial.println(boot.length() ? boot.c_str() : "(none captured: it only prints when the board is first plugged in, not after RST)");
#if ON_CART
  if (CART_TX_PIN < 0) Serial.println("CART_TX_PIN not set: D lines go to USB only");
#endif
  Serial.println("Listening. Range lines start once the other board is on.");
#endif
}

#if ON_CART
// "AT+RANGE=tid:0,...,range:(105,0,0,0,0,0,0,0),rssi:(...)" -> first non-zero range, cm
// (a tag's report lists its distance to anchors 0..7; the user's anchor is 0)
static long firstRange(const String &s)
{
  int i = s.indexOf("range:(");
  if (i < 0) return 0;
  i += 7;
  for (int k = 0; k < 8; k++) {
    long v = atol(s.c_str() + i);
    if (v > 0) return v;
    int comma = s.indexOf(',', i);
    int close = s.indexOf(')', i);
    if (comma < 0 || (close >= 0 && close < comma)) break;
    i = comma + 1;
  }
  return 0;
}
#endif

static void handleLine(const String &s)
{
  if (s.length() == 0) return;
  Serial.println(s);                   // everything from the module, for debugging
#if ON_CART
  long cm = firstRange(s);
  if (cm > 0) {
    Serial.printf("D,%ld\n", cm);
    if (CART_TX_PIN >= 0) cart.printf("D,%ld\n", cm);
  }
#endif
}

void loop()
{
  static uint32_t lastByteMs = 0, lastNoteMs = 0;
  while (Serial.available()) uwb.write(Serial.read());     // Serial Monitor -> module
  while (uwb.available()) {
    char c = (char)uwb.read();
    lastByteMs = millis();
    if (c == '\n' || c == '\r') { handleLine(line); line = ""; }   // either ends a line
    else if (line.length() < 400) line += c;
  }
  uint32_t now = millis();
  if (line.length() && now - lastByteMs > 100) { handleLine(line); line = ""; }   // line with no ending
  if (now - lastByteMs > 5000 && now - lastNoteMs > 5000) {
    Serial.println("(nothing from the UWB module for 5 s)");
    lastNoteMs = now;
  }
}
