#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_system.h> 

const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD";

#define PIN_RELAY      27
#define PIN_BUZZER     14
#define PIN_LED_RED    13
#define PIN_LED_GREEN  33
#define PIN_LED_HEARTBEAT 25
#define PIN_DHT        26
#define PIN_SOIL       36   // VP
#define PIN_BUTTON     12
#define PIN_LDR        34

#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

#define RELAY_ACTIVE_HIGH true
#define BUZZER_ACTIVE_HIGH false

const unsigned long RELAY_SETTLE_MS         = 4000;

const unsigned long SENSOR_READ_INTERVAL_MS = 2000;
const unsigned long WATERING_DURATION_MS    = 4000;
const unsigned long SOAK_WAIT_MS            = 6000;
const unsigned long WATERING_COOLDOWN_MS    = 30000;
const unsigned long DEBOUNCE_MS             = 50;
const unsigned long RED_BLINK_INTERVAL_MS   = 400;
const unsigned long HEARTBEAT_PERIOD_MS     = 3000;
const unsigned long HEARTBEAT_PULSE_MS      = 150;
const unsigned long ALARM_CHIRP_INTERVAL_MS = 2500; 
const unsigned long ALARM_CHIRP_DURATION_MS = 80;   
unsigned long lastChirpTime = 0;

const bool  BUZZER_TYPE_PASSIVE  = true;
const int   BUZZER_ALARM_FREQ_HZ = 2700; 
const int   BUZZER_RING_FREQ_HZ  = 2900; 

const unsigned long RING_DURATION_MS       = 6000;
const unsigned long RING_BLINK_INTERVAL_MS = 150; 
bool ringActive = false;
unsigned long ringStartTime = 0;
unsigned long lastRingBlinkTime = 0;
bool ringBlinkState = false;

const int SOIL_ANALOG_SAMPLES        = 8; 
const int CONFIRMED_DRY_READS_NEEDED = 2; 
int dryReadStreak = 0;
int soilBeforeWatering = 0; 

const bool PUMP_ENABLED = true;

const int SOIL_DRY_THRESHOLD_PERCENT = 40;
const int SOIL_IMPROVEMENT_MARGIN_PERCENT = 8; 

int SOIL_DRY_RAW     = 3000;
int SOIL_WET_RAW     = 1200;
int LIGHT_DARK_RAW   = 100;
int LIGHT_BRIGHT_RAW = 4000;

enum WateringState { STATE_IDLE, STATE_WATERING, STATE_SOAK_CHECK };
WateringState wateringState = STATE_IDLE;

RTC_DATA_ATTR bool waterEmptyAlarm = false;
bool buzzerMuted        = false;
bool pumpIsOn           = false;
bool sensorsInitialized = false;
bool hasWateredOnce     = false;
bool overrideActiveGlobal = false; 
bool currentBuzzerState = false;

const unsigned long PUMP_HARD_SAFETY_CAP_MS = WATERING_DURATION_MS + 3000UL;
RTC_DATA_ATTR bool pumpFaultLatched = false; 
unsigned long pumpOnSince = 0;

RTC_DATA_ATTR int bootLoopCount = 0;
const int BOOT_LOOP_ALARM_THRESHOLD = 3;

unsigned long wateringStartTime   = 0;
unsigned long soakCheckStartTime  = 0;
unsigned long lastWateringEndTime = 0;
unsigned long lastSensorRead      = 0;
unsigned long lastRedBlinkTime    = 0;
bool redBlinkState = false;

int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

float lastTempC        = 0;
float lastHumidity     = 0;
int   lastSoilPercent  = 0;
int   lastLightPercent = 0;
int   lastSoilRaw      = 0;
int   lastLightRaw     = 0;

bool webWaterPulseActive = false;
unsigned long webWaterPulseStart = 0;


String lastPestLabel   = "";
int    lastPestConf    = 0;    
unsigned long lastPestTime = 0; 
bool   havePestReport   = false;

WebServer server(80);

void setPump(bool on) {
  if (on && pumpFaultLatched) on = false; 

  if (on && millis() < RELAY_SETTLE_MS) {
    on = false; 
  }

  bool wasOff = !pumpIsOn;
  pumpIsOn = on; 
  setLed(PIN_LED_GREEN, on);
  if (PUMP_ENABLED) {
    digitalWrite(PIN_RELAY, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
  }

  if (on && wasOff) pumpOnSince = millis(); 
}

void enforcePumpSafetyCap(unsigned long now) {
  if (pumpIsOn && !pumpFaultLatched && (now - pumpOnSince >= PUMP_HARD_SAFETY_CAP_MS)) {
    Serial.println(F(">>> SAFETY CUTOFF: pump ran past the hard time cap -- forcing OFF and locking out until reset. <<<"));
    pumpFaultLatched = true;
    wateringState = STATE_IDLE;
    setPump(false);
    waterEmptyAlarm = true; 
  }
}

void driveBuzzer(bool on, int freq) {
  if (BUZZER_TYPE_PASSIVE) {
    if (on) tone(PIN_BUZZER, freq);
    else noTone(PIN_BUZZER);
  } else {
    digitalWrite(PIN_BUZZER, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
  }
}

void setBuzzer(bool on) {
  if (on != currentBuzzerState) {
    currentBuzzerState = on;
    driveBuzzer(on, BUZZER_ALARM_FREQ_HZ);
  }
}

void setLed(int pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
}

int readSoilPercent() {
  long sum = 0;
  for (int i = 0; i < SOIL_ANALOG_SAMPLES; i++) { sum += analogRead(PIN_SOIL); delayMicroseconds(200); }
  lastSoilRaw = sum / SOIL_ANALOG_SAMPLES;
  int percent = map(lastSoilRaw, SOIL_DRY_RAW, SOIL_WET_RAW, 0, 100);
  return constrain(percent, 0, 100);
}

int readSoilPercentStable() {
  long sum = 0;
  const int STABLE_READS = 3;
  for (int i = 0; i < STABLE_READS; i++) {
    sum += readSoilPercent();
    delay(150);
  }
  return sum / STABLE_READS;
}

int readLightPercent() {
  long sum = 0;
  for (int i = 0; i < SOIL_ANALOG_SAMPLES; i++) { sum += analogRead(PIN_LDR); delayMicroseconds(200); }
  lastLightRaw = sum / SOIL_ANALOG_SAMPLES;
  int percent = map(lastLightRaw, LIGHT_DARK_RAW, LIGHT_BRIGHT_RAW, 0, 100);
  return constrain(percent, 0, 100);
}

void readAllSensors() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) lastHumidity = h;
  if (!isnan(t)) lastTempC = t;
  lastSoilPercent  = readSoilPercent();
  lastLightPercent = readLightPercent();

  if (lastSoilPercent < SOIL_DRY_THRESHOLD_PERCENT) {
    if (dryReadStreak < 255) dryReadStreak++;
  } else {
    dryReadStreak = 0;
  }
}

void runStartupSelfTest() {
  Serial.println(F("Starting Systen..."));
  Serial.println(F("  White LED x2"));
  for (int i = 0; i < 2; i++) { setLed(PIN_LED_HEARTBEAT, true); delay(200); setLed(PIN_LED_HEARTBEAT, false); delay(200); }
}

void onResetButtonPressed() {
  Serial.println(F(">>> Reset button pressed: clearing alarm, pump lockout, and unmuting buzzer <<<"));
  waterEmptyAlarm = false;
  buzzerMuted = false;
  pumpFaultLatched = false;
  setBuzzer(false);
}

void handleResetButton(unsigned long now) {
  int reading = digitalRead(PIN_BUTTON);
  if (reading != lastButtonReading) {
    lastDebounceTime = now;
  }
  if ((now - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        onResetButtonPressed();
      }
    }
  }
  lastButtonReading = reading;
}

void startWatering(unsigned long now) {
  wateringState = STATE_WATERING;
  wateringStartTime = now;
  soilBeforeWatering = lastSoilPercent; 
  dryReadStreak = 0;
  Serial.print(F(">>> Soil is dry ("));
  Serial.print(lastSoilPercent);
  Serial.println(F("%): starting watering cycle <<<"));
}

void runWateringStateMachine(unsigned long now) {
  switch (wateringState) {
    case STATE_IDLE:
      setPump(false);
      if (sensorsInitialized &&
          !waterEmptyAlarm &&
          (!hasWateredOnce || (now - lastWateringEndTime) >= WATERING_COOLDOWN_MS) &&
          dryReadStreak >= CONFIRMED_DRY_READS_NEEDED) {
        startWatering(now);
      }
      break;

    case STATE_WATERING:
      setPump(true); 
      setLed(PIN_LED_RED, false);
      if (now - wateringStartTime >= WATERING_DURATION_MS) {
        setPump(false);
        wateringState = STATE_SOAK_CHECK;
        soakCheckStartTime = now;
      }
      break;

    case STATE_SOAK_CHECK:
      setPump(false);
      if (now - soakCheckStartTime >= SOAK_WAIT_MS) {
        int freshSoil = readSoilPercentStable();
        lastSoilPercent = freshSoil;
        bool roseEnough = freshSoil >= (soilBeforeWatering + SOIL_IMPROVEMENT_MARGIN_PERCENT);
        if (freshSoil < SOIL_DRY_THRESHOLD_PERCENT || !roseEnough) {
          waterEmptyAlarm = true;
          Serial.print(F(">>> Soil didn't genuinely improve after watering (before: "));
          Serial.print(soilBeforeWatering);
          Serial.print(F("%, after: "));
          Serial.print(freshSoil);
          Serial.println(F("%): assuming reservoir is EMPTY. Alarm ON. <<<"));
        } else {
          waterEmptyAlarm = false;
          Serial.println(F(">>> Soil moisture genuinely rose after watering: reservoir OK. <<<"));
        }
        lastWateringEndTime = now;
        hasWateredOnce = true;
        wateringState = STATE_IDLE;
      }
      break;
  }
}

void handleHeartbeat(unsigned long now) {
  unsigned long phase = now % HEARTBEAT_PERIOD_MS;
  setLed(PIN_LED_HEARTBEAT, phase < HEARTBEAT_PULSE_MS);
}

void handleRingPresentation(unsigned long now) {
  if (!currentBuzzerState) {
    driveBuzzer(true, BUZZER_RING_FREQ_HZ);
    currentBuzzerState = true;
  }
  if (now - lastRingBlinkTime >= RING_BLINK_INTERVAL_MS) {
    lastRingBlinkTime = now;
    ringBlinkState = !ringBlinkState;
    setLed(PIN_LED_HEARTBEAT, ringBlinkState);
  }
}

void handleAlarmPresentation(unsigned long now, bool overrideActive) {
  if (waterEmptyAlarm) {
    if (!overrideActive) {
      if (now - lastRedBlinkTime >= RED_BLINK_INTERVAL_MS) {
        lastRedBlinkTime = now;
        redBlinkState = !redBlinkState;
        setLed(PIN_LED_RED, redBlinkState);
      }

      if (!buzzerMuted) {
        unsigned long elapsed = now - lastChirpTime;
        if (elapsed >= ALARM_CHIRP_INTERVAL_MS) {
          lastChirpTime = now; 
        }
        setBuzzer(elapsed < ALARM_CHIRP_DURATION_MS);
      } else {
        setBuzzer(false);
      }
    } else {
      setBuzzer(false);
    }
  } else {
    setBuzzer(false);
    if (!overrideActive && wateringState != STATE_WATERING) {
      setLed(PIN_LED_RED, false);
    }
  }
}

void printStatus(bool overrideActive) {
  Serial.print(F("Temp: "));    Serial.print(lastTempC, 1);     Serial.print(F("C | "));
  Serial.print(F("Hum: "));     Serial.print(lastHumidity, 1);  Serial.print(F("% | "));
  Serial.print(F("Soil: "));    Serial.print(lastSoilPercent);  Serial.print(F("% (raw "));  Serial.print(lastSoilRaw);  Serial.print(F(") | "));
  Serial.print(F("Light: "));   Serial.print(lastLightPercent); Serial.print(F("% (raw "));  Serial.print(lastLightRaw); Serial.print(F(") | "));
  Serial.print(F("Override: ")); Serial.print(overrideActive ? F("YES (web Water Now)") : F("no")); Serial.print(F(" | "));
  Serial.print(F("Pump: "));    Serial.print(pumpIsOn ? F("ON") : F("off")); Serial.print(F(" | "));
  Serial.print(F("State: "));
  switch (wateringState) {
    case STATE_IDLE:       Serial.print(F("IDLE"));       break;
    case STATE_WATERING:   Serial.print(F("WATERING"));   break;
    case STATE_SOAK_CHECK: Serial.print(F("SOAK_CHECK")); break;
  }
  Serial.print(F(" | Alarm: ")); Serial.print(waterEmptyAlarm ? F("YES") : F("no"));
  Serial.print(F(" | Muted: ")); Serial.println(buzzerMuted ? F("yes") : F("no"));
}

const char INDEX_HTML[] = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Terrarium Journal</title>
<style>
:root{
  --moss:#2F4A3C;
  --moss-deep:#1D2E25;
  --fern:#6FA37E;
  --fern-bright:#8FC49E;
  --brass:#C9A15A;
  --brass-bright:#E0BD7C;
  --glass:#EAF2ED;
  --paper:#F6F3EA;
  --clay:#8B5E3C;
  --clay-dark:#5E3E27;
  --ink:#1C2620;
  --alert:#D0604F;
  --alert-deep:#A6473A;
}
*{box-sizing:border-box;}
html,body{margin:0;padding:0;}
body{
  min-height:100vh;
  display:flex;
  align-items:flex-start;
  justify-content:center;
  background:radial-gradient(circle at 50% -10%, #3C5A48, #16211B 70%);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  color:var(--ink);
  padding:28px 14px;
}
.jar{
  width:100%;
  max-width:400px;
  border-radius:26px 26px 20px 20px;
  overflow:hidden;
  background:var(--paper);
  box-shadow:0 18px 40px rgba(20,30,24,0.28);
  position:relative;
}
.topbar{
  display:flex;
  align-items:baseline;
  justify-content:space-between;
  padding:12px 18px;
  background:var(--moss-deep);
  color:var(--glass);
}
.topbar .title{
  font-family:Georgia,"Iowan Old Style",Palatino,serif;
  font-style:italic;
  font-size:16px;
}
.topbar .byline{
  font-size:11px;
  color:var(--fern-bright);
  opacity:0.9;
}
.sky{
  position:relative;
  padding:20px 16px 30px;
  transition:background 1.4s ease;
}
.sky-bright{ background:linear-gradient(180deg,#BFE0D6,#EFE3B8); color:var(--ink); }
.sky-soft{ background:linear-gradient(180deg,#7E9FAE,#D9B98A); color:var(--paper); }
.sky-dark{ background:linear-gradient(180deg,#16233A,#3A3050); color:var(--paper); }
.skyRow{ display:flex; align-items:center; justify-content:space-between; }
.skyLeft{ text-align:left; }
.skyIcon{ font-size:30px; display:block; }
.skyCaption{ font-size:12px; opacity:0.85; margin-top:2px; }
.tempBadge{ text-align:right; font-size:15px; font-weight:600; }
.tempBar{ margin-top:5px; width:64px; height:5px; border-radius:3px; background:rgba(255,255,255,0.35); overflow:hidden; margin-left:auto; }
.tempBarFill{ height:100%; width:0%; background:var(--brass-bright); transition:width 1s ease; }

.stage{
  position:relative;
  text-align:center;
  padding:8px 16px 20px;
  background:var(--paper);
  overflow:hidden;
}
.confettiLayer{ position:absolute; inset:0; pointer-events:none; overflow:visible; }
.confetti{ position:absolute; top:45%; font-size:15px; opacity:0; animation:confettiPop 1.1s ease-out forwards; }
@keyframes confettiPop{
  0%{ transform:translateY(0) scale(0.6); opacity:0; }
  20%{ opacity:1; }
  100%{ transform:translateY(-70px) scale(1); opacity:0; }
}

.potWrap{ position:relative; width:140px; height:140px; margin:0 auto; }
.moistureRing{ position:absolute; inset:0; transform:rotate(-90deg); }
.ringTrack{ fill:none; stroke:#E3DCC9; stroke-width:6; }
.ringFill{ fill:none; stroke:var(--alert); stroke-width:6; stroke-linecap:round; stroke-dasharray:326.7; stroke-dashoffset:326.7; transition:stroke-dashoffset 1s ease, stroke 1s ease; }

.raindrops{ position:absolute; inset:0; pointer-events:none; opacity:0; transition:opacity .3s ease; }
.stage.watering .raindrops{ opacity:1; }
.drop{ position:absolute; top:6px; font-size:14px; animation:fall 0.85s linear infinite; }
@keyframes fall{
  0%{ transform:translateY(0); opacity:0; }
  15%{ opacity:1; }
  100%{ transform:translateY(120px); opacity:0; }
}

.critter{ position:absolute; inset:0; display:flex; flex-direction:column; align-items:center; justify-content:center; }
.plant{ font-size:52px; transform-origin:bottom center; }
.plant.healthy{ animation:sway 3.5s ease-in-out infinite; }
.plant.ok{ animation:sway 5s ease-in-out infinite; }
.plant.wilted{ animation:none; filter:grayscale(30%); transform:rotate(-10deg); }
@keyframes sway{
  0%,100%{ transform:rotate(-4deg); }
  50%{ transform:rotate(4deg); }
}

.pot{ width:58px; height:36px; margin-top:-4px; background:linear-gradient(180deg,#B98552,var(--clay)); border-radius:6px 6px 18px 18px; position:relative; box-shadow:inset 0 -7px 10px rgba(0,0,0,0.18); }
.face{ position:absolute; top:8px; left:0; right:0; display:flex; justify-content:center; gap:14px; }
.eye{ width:6px; height:6px; background:var(--ink); border-radius:50%; display:inline-block; animation:blink 5s infinite; }
@keyframes blink{
  0%,94%,100%{ transform:scaleY(1); }
  97%{ transform:scaleY(0.15); }
}
.mouth{ margin:5px auto 0; width:14px; height:6px; }
.mouth-happy{ border-bottom:2.5px solid var(--ink); border-radius:0 0 14px 14px; }
.mouth-neutral{ height:2.5px; background:var(--ink); border-radius:2px; }
.mouth-sad{ border-top:2.5px solid var(--ink); border-radius:14px 14px 0 0; margin-top:9px; }
.blush{ position:absolute; top:15px; width:7px; height:4px; background:#E8927C; border-radius:50%; opacity:0; transition:opacity .4s ease; }
.blushL{ left:5px; }
.blushR{ right:5px; }
.critter.healthy .blush{ opacity:0.8; }

.ringRipple{ position:absolute; inset:0; display:none; pointer-events:none; }
.ringRipple.active{ display:block; }
.ringRipple span{ position:absolute; left:50%; top:78%; width:20px; height:20px; margin:-10px; border:2px solid var(--brass); border-radius:50%; opacity:0; animation:ripple 1.4s ease-out infinite; }
.ringRipple span:nth-child(2){ animation-delay:.4s; }
.ringRipple span:nth-child(3){ animation-delay:.8s; }
@keyframes ripple{
  0%{ transform:scale(0.4); opacity:.9; }
  100%{ transform:scale(3.4); opacity:0; }
}

.mood{ font-family:Georgia,"Iowan Old Style",Palatino,serif; font-style:italic; font-size:18px; margin-top:6px; min-height:24px; }
.readouts{ margin-top:10px; display:flex; justify-content:center; gap:18px; font-size:14px; color:#5B6B60; }

.statusRow{ margin-top:12px; display:flex; justify-content:center; gap:8px; flex-wrap:wrap; }
.statusPill{ display:flex; align-items:center; gap:6px; padding:5px 11px; border-radius:20px; background:#E9E3D2; color:#7C8A7F; font-size:12px; font-weight:600; transition:background .3s ease, color .3s ease; }
.statusDot{ width:8px; height:8px; border-radius:50%; background:#C7BFA9; transition:background .3s ease, box-shadow .3s ease; }
.statusPill.pumpOn{ background:var(--fern); color:var(--paper); }
.statusPill.pumpOn .statusDot{ background:#DFF3E6; box-shadow:0 0 7px 2px rgba(223,243,230,0.9); }
.statusPill.alarmOn{ background:var(--alert); color:var(--paper); animation:pulse 1s ease-in-out infinite; }
.statusPill.alarmOn .statusDot{ background:#FFDAD3; box-shadow:0 0 7px 2px rgba(255,218,211,0.9); }
.statusPill.alarmMuted{ background:var(--brass); color:var(--ink); animation:none; }
.statusPill.alarmMuted .statusDot{ background:#FFF2D9; }
.statusPill.pestOn{ background:var(--alert); color:var(--paper); animation:pulse 1s ease-in-out infinite; }
.statusPill.pestOn .statusDot{ background:#FFDAD3; box-shadow:0 0 7px 2px rgba(255,218,211,0.9); }
.statusPill.aliveFlash .statusDot{ background:#8FC49E; box-shadow:0 0 7px 3px rgba(143,196,158,0.9); }
@keyframes pulse{
  0%,100%{ opacity:1; }
  50%{ opacity:0.75; }
}

.bed{ position:relative; height:80px; background:repeating-linear-gradient(180deg, #B08554, #B08554 6px, #A47A4C 6px, #A47A4C 12px); overflow:hidden; }
.bedFill{ position:absolute; bottom:0; left:0; right:0; height:0%; background:linear-gradient(180deg, rgba(111,163,126,0.55), var(--clay-dark)); transition:height 1.2s ease; }
.bedLabel{ position:relative; z-index:2; display:flex; align-items:baseline; justify-content:center; gap:8px; padding-top:10px; text-shadow:0 1px 3px rgba(0,0,0,0.4); }
.bedIcon{ font-size:16px; }
.bedPct{ font-size:23px; font-weight:800; color:var(--paper); letter-spacing:0.01em; transition:color .4s ease; }
.bedStatusWrap{ display:flex; flex-direction:column; align-items:flex-start; line-height:1.15; }
.bedCaption{ font-size:9.5px; text-transform:uppercase; letter-spacing:0.09em; color:rgba(246,243,234,0.65); }
.bedStatus{ font-size:12.5px; font-weight:700; transition:color .4s ease; }
.bedStatus.dry{ color:#FFB4A6; }
.bedStatus.ok{ color:#F4D89C; }
.bedStatus.healthy{ color:#B9E6C6; }

.banner{ display:none; align-items:center; justify-content:space-between; gap:10px; flex-wrap:wrap; padding:12px 16px; background:var(--alert); color:var(--paper); font-size:13.5px; }
.banner .icon{ display:inline-block; animation:shake 0.6s ease-in-out infinite; }
@keyframes shake{
  0%,100%{ transform:rotate(0); }
  25%{ transform:rotate(-8deg); }
  75%{ transform:rotate(8deg); }
}
.banner button{ border:none; background:var(--paper); color:var(--alert-deep); padding:7px 11px; border-radius:8px; font-weight:600; font-size:12.5px; }
.banner button:disabled{ opacity:0.65; }
.bannerNote{ width:100%; font-size:11px; opacity:0.85; font-style:italic; }

.overrideBanner{ display:none; background:linear-gradient(120deg, var(--moss), var(--fern), var(--moss)); background-size:220% 220%; animation:shimmer 4s ease infinite; text-align:center; padding:11px 16px; font-size:13.5px; color:var(--paper); }
.overrideBanner .icon{ display:inline-block; animation:wiggle 1.6s ease-in-out infinite; }
@keyframes wiggle{
  0%,100%{ transform:rotate(-6deg); }
  50%{ transform:rotate(6deg); }
}
@keyframes shimmer{
  0%{ background-position:0% 50%; }
  50%{ background-position:100% 50%; }
  100%{ background-position:0% 50%; }
}

.ringBanner{ display:none; background:var(--brass); text-align:center; padding:10px 16px; font-size:13.5px; color:var(--ink); }

.controls{ padding:16px 16px 6px; text-align:center; position:relative; }
#waterBtn{ width:100%; padding:14px; border:none; border-radius:12px; background:linear-gradient(180deg, var(--fern-bright), var(--fern)); color:var(--moss-deep); font-size:16px; font-weight:700; cursor:pointer; position:relative; overflow:hidden; }
#waterBtn:active{ transform:scale(0.98); }
#waterBtn:disabled{ opacity:0.75; cursor:default; }
#waterBtn:disabled::after{ content:""; position:absolute; right:14px; top:50%; width:16px; height:16px; margin-top:-8px; border-radius:50%; border:2.5px solid rgba(28,38,32,0.25); border-top-color:var(--moss-deep); animation:spin 0.8s linear infinite; }
@keyframes spin{ to{ transform:rotate(360deg); } }

#ringBtn{ width:100%; margin-top:10px; padding:12px; border:1.5px solid var(--brass); border-radius:12px; background:transparent; color:var(--clay-dark); font-size:14.5px; font-weight:600; cursor:pointer; }
#ringBtn:active{ transform:scale(0.98); }
#ringBtn:disabled{ opacity:0.7; cursor:default; border-color:var(--brass-bright); color:var(--brass-bright); background:rgba(201,161,90,0.08); }

:focus-visible{ outline:2.5px solid var(--brass); outline-offset:2px; }

.toastHost{ position:relative; padding:0 16px; min-height:4px; }
.toast{ background:var(--moss-deep); color:var(--glass); font-size:12.5px; padding:9px 12px; border-radius:10px; margin-top:8px; opacity:0; transform:translateY(6px); transition:opacity .3s ease, transform .3s ease; }
.toast.show{ opacity:1; transform:translateY(0); }

.footer{ padding:6px 16px 14px; font-size:11.5px; color:#8a9089; text-align:center; }
.credit{ padding-top:12px; margin-top:6px; border-top:1px solid #E6E0CF; padding:12px 16px 16px; text-align:center; font-size:11px; color:#93998E; }
.credit b{ color:var(--clay-dark); font-weight:700; }

@media (prefers-reduced-motion: reduce){
  *{ animation-duration:0.001ms !important; animation-iteration-count:1 !important; transition-duration:0.001ms !important; }
}
</style>
</head>
<body>
  <div class="jar">
    <div class="topbar">
      <span class="title">Terrarium journal</span>
      <span class="byline">by Gerbang Logika</span>
    </div>
    <div style="text-align:right;padding:6px 18px 0;background:var(--moss-deep);">
      <a href="/vision" style="color:#8FC49E;font-size:11px;font-weight:700;text-decoration:none;">&#128248; Open AI Leaf Scanner &rarr;</a>
    </div>

    <div class="sky sky-soft" id="sky">
      <div class="skyRow">
        <div class="skyLeft">
          <span class="skyIcon" id="skyIcon">&#9925;</span>
          <div class="skyCaption" id="skyCaption">soft afternoon light</div>
        </div>
        <div class="tempBadge">
          <span id="tempValue">--&deg;C</span>
          <div class="tempBar"><div class="tempBarFill" id="tempBarFill"></div></div>
        </div>
      </div>
    </div>

    <div class="stage" id="stage">
      <div class="confettiLayer" id="confettiLayer"></div>
      <div class="potWrap">
        <svg class="moistureRing" viewBox="0 0 120 120">
          <circle class="ringTrack" cx="60" cy="60" r="52"></circle>
          <circle class="ringFill" id="ringFill" cx="60" cy="60" r="52"></circle>
        </svg>
        <div class="raindrops" id="raindrops">
          <span class="drop" style="left:30%; animation-delay:0s">&#128167;</span>
          <span class="drop" style="left:50%; animation-delay:.2s">&#128167;</span>
          <span class="drop" style="left:70%; animation-delay:.4s">&#128167;</span>
        </div>
        <div class="critter ok" id="critter">
          <div class="plant ok" id="plant">&#127793;</div>
          <div class="pot">
            <div class="face">
              <span class="eye"></span>
              <span class="eye"></span>
            </div>
            <div class="mouth mouth-neutral" id="mouth"></div>
            <span class="blush blushL"></span>
            <span class="blush blushR"></span>
          </div>
        </div>
        <div class="ringRipple" id="ringRipple"><span></span><span></span><span></span></div>
      </div>

      <div class="mood" id="mood">waking up&hellip;</div>
      <div class="readouts">
        <span id="lightReadout">&#9728;&#65039; --%</span>
        <span id="humidityReadout">&#128167; --%</span>
        <span id="pumpReadout">&#128164; standby</span>
      </div>
      <div class="statusRow">
        <span class="statusPill" id="pumpPill"><span class="statusDot"></span>Pump</span>
        <span class="statusPill" id="alarmPill"><span class="statusDot"></span>Alarm</span>
        <span class="statusPill" id="alivePill"><span class="statusDot"></span>Online</span>
        <span class="statusPill" id="pestPill" style="display:none"><span class="statusDot"></span><span id="pestPillText">Pest watch</span></span>
      </div>
    </div>

    <div class="bed" id="bed">
      <div class="bedFill" id="bedFill"></div>
      <div class="bedLabel" id="bedLabel">
        <span class="bedIcon" id="bedIcon">&#128167;</span>
        <span class="bedPct" id="bedPct">--%</span>
        <span class="bedStatusWrap">
          <span class="bedCaption">soil moisture</span>
          <span class="bedStatus" id="bedStatus">reading&hellip;</span>
        </span>
      </div>
    </div>

    <div class="banner" id="alarmBanner">
      <span><span class="icon">&#128692;</span> Manual reset required. Refill water or change the battery, then press the reset button on the plant.</span>
      <button id="muteBtn" onclick="muteBuzzer()">Mute the alarm</button>
      <span class="bannerNote">Only the physical reset button clears this fully.</span>
    </div>
    <div class="overrideBanner" id="overrideBanner">
      <span class="icon">&#127918;</span> You've taken manual control
    </div>
    <div class="ringBanner" id="ringBanner">
      &#128276; Ringing &mdash; follow the sound to find your plant
    </div>

    <div class="controls">
      <button id="waterBtn" onclick="waterNow()">Water now</button>
      <button id="ringBtn" onclick="ringPlant()">&#128276; Find my plant</button>
    </div>
    <div class="toastHost" id="toastHost"></div>
    <div class="footer" id="footer">connecting&hellip;</div>
    <div class="credit">&#127807; Built by <b>Team Gerbang Logika</b> &mdash; AI &times; IoT Hackathon, FTI UK Petra</div>
  </div>

<script>
var failCount = 0;
var prevData = null;
var moodCategory = null;
var moodQuotes = {
  healthy: ["living my best life", "thriving out here", "soaking up the good vibes", "growing like nobody's business"],
  ok: ["doing alright, all things considered", "hanging in there", "steady as she grows", "no complaints today"],
  wilted: ["getting reeeally thirsty...", "could really use a drink", "wilting a little, send water", "this is urgent, friend"]
};
var ringTimer = null;
var RING_SECONDS = 6;
var skyCategory = null;
var skyQuotes = {
  bright: ['sunbathing, obviously', 'living for these rays', 'basking like a lizard on a rock', 'peak leaf-tanning hour', 'soaking up the spotlight', 'main character lighting'],
  soft: ['soft afternoon light', 'golden hour warm-up', 'lazy afternoon glow', 'gentle vibes, gentle light', 'basking in the mellow glow', 'coasting through a chill afternoon'],
  dark: ['quiet moonlit hours', 'the night shift', 'stargazing (allegedly)', 'moonlit and mysterious', 'counting fireflies in the dark', 'dreaming in chlorophyll']
};

function pick(arr){ return arr[Math.floor(Math.random()*arr.length)]; }

function setSky(lightPercent){
  var sky = document.getElementById('sky');
  var icon = document.getElementById('skyIcon');
  var caption = document.getElementById('skyCaption');
  sky.classList.remove('sky-bright','sky-soft','sky-dark');
  var category;
  if(lightPercent >= 60){ category = 'bright'; sky.classList.add('sky-bright'); icon.textContent = '\u2600\uFE0F'; }
  else if(lightPercent >= 25){ category = 'soft'; sky.classList.add('sky-soft'); icon.textContent = '\u26C5'; }
  else { category = 'dark'; sky.classList.add('sky-dark'); icon.textContent = '\uD83C\uDF19'; }

  if(category !== skyCategory){
    skyCategory = category;
    caption.textContent = pick(skyQuotes[category]);
  }
}

function setCritter(soilPercent){
  var critter = document.getElementById('critter');
  var plant = document.getElementById('plant');
  var mouth = document.getElementById('mouth');
  var mood = document.getElementById('mood');
  var category;
  if(soilPercent >= 60){ category = 'healthy'; plant.textContent = '\uD83C\uDF3F'; mouth.className = 'mouth mouth-happy'; }
  else if(soilPercent >= 40){ category = 'ok'; plant.textContent = '\uD83C\uDF31'; mouth.className = 'mouth mouth-neutral'; }
  else { category = 'wilted'; plant.textContent = '\uD83E\uDD40'; mouth.className = 'mouth mouth-sad'; }

  critter.className = 'critter ' + category;
  plant.className = 'plant ' + category;

  if(category !== moodCategory){
    moodCategory = category;
    mood.textContent = pick(moodQuotes[category]);
  }
}

function setRing(soilPercent){
  var ring = document.getElementById('ringFill');
  var circumference = 2 * Math.PI * 52;
  var pct = Math.max(0, Math.min(100, soilPercent));
  var offset = circumference * (1 - pct / 100);
  ring.style.strokeDashoffset = offset;
  ring.style.stroke = pct >= 60 ? '#6FA37E' : (pct >= 40 ? '#C9A15A' : '#D0604F');
}

function setBed(soilPercent){
  document.getElementById('bedFill').style.height = soilPercent + '%';
  document.getElementById('bedPct').textContent = soilPercent + '%';

  var icon = document.getElementById('bedIcon');
  var status = document.getElementById('bedStatus');
  status.classList.remove('dry','ok','healthy');

  if(soilPercent >= 60){
    icon.textContent = '\uD83D\uDCA7';
    status.textContent = 'moist & happy';
    status.classList.add('healthy');
  } else if(soilPercent >= 40){
    icon.textContent = '\uD83C\uDF31';
    status.textContent = 'doing okay';
    status.classList.add('ok');
  } else {
    icon.textContent = '\uD83D\uDD25';
    status.textContent = 'dry \u2014 needs water';
    status.classList.add('dry');
  }
}

function setStatusPills(data){
  var pumpPill = document.getElementById('pumpPill');
  pumpPill.classList.toggle('pumpOn', data.pumpActive);

  var alarmPill = document.getElementById('alarmPill');
  alarmPill.classList.remove('alarmOn','alarmMuted');
  if(data.alarmActive && data.buzzerMuted){ alarmPill.classList.add('alarmMuted'); }
  else if(data.alarmActive){ alarmPill.classList.add('alarmOn'); }

  var alivePill = document.getElementById('alivePill');
  alivePill.classList.add('aliveFlash');
  clearTimeout(alivePill._flashTimer);
  alivePill._flashTimer = setTimeout(function(){ alivePill.classList.remove('aliveFlash'); }, 700);

  var pestPill = document.getElementById('pestPill');
  if(data.havePestReport){
    pestPill.style.display = 'flex';
    var isPest = data.pestLabel && data.pestLabel.toLowerCase().indexOf('leaf') === -1;
    var fresh = data.pestAgeSec < 30; 
    pestPill.classList.toggle('pestOn', isPest && fresh);
    document.getElementById('pestPillText').textContent = fresh
      ? (data.pestLabel + ' ' + data.pestConf + '%')
      : 'Pest watch idle';
  }
}

function showToast(msg){
  var host = document.getElementById('toastHost');
  var t = document.createElement('div');
  t.className = 'toast';
  t.textContent = msg;
  host.appendChild(t);
  requestAnimationFrame(function(){ t.classList.add('show'); });
  setTimeout(function(){
    t.classList.remove('show');
    setTimeout(function(){ t.remove(); }, 350);
  }, 3200);
}

function burstConfetti(){
  var layer = document.getElementById('confettiLayer');
  var emojis = ['\u2728','\uD83C\uDF1F','\uD83D\uDC9A','\uD83C\uDF43'];
  for(var i = 0; i < 9; i++){
    (function(i){
      var s = document.createElement('span');
      s.className = 'confetti';
      s.textContent = emojis[i % emojis.length];
      s.style.left = (12 + Math.random()*76) + '%';
      s.style.animationDelay = (Math.random()*0.3) + 's';
      layer.appendChild(s);
      setTimeout(function(){ s.remove(); }, 1500);
    })(i);
  }
}

function announceChanges(data){
  if(prevData){
    if(!prevData.pumpActive && data.pumpActive){ showToast('\uD83D\uDCA7 Watering started'); }
    if(prevData.pumpActive && !data.pumpActive){ showToast("\u2728 All done \u2014 soil's a little happier now"); burstConfetti(); }
    if(!prevData.alarmActive && data.alarmActive){ showToast('\uD83D\uDEA8 Uh oh \u2014 the reservoir is empty'); }
    if(!prevData.buzzerMuted && data.buzzerMuted){ showToast('\uD83D\uDD07 Alarm shushed \u2014 the reset button is the only way to clear it fully'); }
    if(!prevData.overrideActive && data.overrideActive){ showToast('\uD83C\uDFAE Manual control engaged'); }
    if(prevData.overrideActive && !data.overrideActive){ showToast('\uD83C\uDFAE Back to auto-pilot'); }
    if(!prevData.ringActive && data.ringActive){ showToast('\uD83D\uDD14 Ringing \u2014 follow the sound!'); }
    var wasPest = prevData.havePestReport && prevData.pestLabel && prevData.pestLabel.toLowerCase().indexOf('leaf') === -1;
    var isPest = data.havePestReport && data.pestLabel && data.pestLabel.toLowerCase().indexOf('leaf') === -1 && data.pestAgeSec < 30;
    if(!wasPest && isPest){ showToast('\uD83D\uDC1B AI scanner spotted: ' + data.pestLabel); }
  }
  prevData = data;
}

function updateUI(data){
  announceChanges(data);

  setSky(data.lightPercent);
  setCritter(data.soilPercent);
  setRing(data.soilPercent);
  setBed(data.soilPercent);

  document.getElementById('tempValue').textContent = data.tempC.toFixed(1) + '\u00B0C';
  var tempPct = Math.max(0, Math.min(100, (data.tempC - 10) / (40 - 10) * 100));
  document.getElementById('tempBarFill').style.width = tempPct + '%';
  var lightIcon = data.lightPercent >= 60 ? '\u2600\uFE0F' : (data.lightPercent >= 25 ? '\u26C5' : '\uD83C\uDF19');
  document.getElementById('lightReadout').textContent = lightIcon + ' ' + data.lightPercent + '%';
  document.getElementById('humidityReadout').textContent = '\uD83D\uDCA7 ' + data.humidity.toFixed(0) + '%';
  document.getElementById('pumpReadout').textContent = data.pumpActive ? '\uD83D\uDCA6 watering now' : '\uD83D\uDCA4 standby';
  document.getElementById('stage').classList.toggle('watering', data.pumpActive);

  document.getElementById('alarmBanner').style.display = data.alarmActive ? 'flex' : 'none';
  document.getElementById('overrideBanner').style.display = data.overrideActive ? 'block' : 'none';
  document.getElementById('ringBanner').style.display = data.ringActive ? 'block' : 'none';
  document.getElementById('ringRipple').classList.toggle('active', data.ringActive);

  var muteBtn = document.getElementById('muteBtn');
  if(data.buzzerMuted){ muteBtn.textContent = 'Muted \u2014 reset button clears it'; muteBtn.disabled = true; }
  else { muteBtn.textContent = 'Mute the alarm'; muteBtn.disabled = false; }

  var waterBtn = document.getElementById('waterBtn');
  waterBtn.disabled = data.pumpActive;
  waterBtn.textContent = data.pumpActive ? 'Watering\u2026' : 'Water now';

  var ringBtn = document.getElementById('ringBtn');
  ringBtn.disabled = data.ringActive;
  if(!data.ringActive){ ringBtn.textContent = '\uD83D\uDD14 Find my plant'; }

  setStatusPills(data);
  document.getElementById('footer').textContent = 'updated just now';
}

function poll(){
  fetch('/data', {cache:'no-store'})
    .then(function(res){ return res.json(); })
    .then(function(data){ updateUI(data); failCount = 0; })
    .catch(function(){
      failCount++;
      if(failCount >= 3){
        document.getElementById('footer').textContent = 'connection lost, retrying...';
      }
    });
}

function waterNow(){
  document.getElementById('waterBtn').disabled = true;
  fetch('/water').catch(function(){});
}

function muteBuzzer(){
  fetch('/mute').catch(function(){});
}

function ringPlant(){
  var btn = document.getElementById('ringBtn');
  btn.disabled = true;
  var secondsLeft = RING_SECONDS;
  btn.textContent = '\uD83D\uDD0A Ringing\u2026 (' + secondsLeft + 's)';
  clearInterval(ringTimer);
  ringTimer = setInterval(function(){
    secondsLeft--;
    if(secondsLeft > 0 && btn.disabled){ btn.textContent = '\uD83D\uDD0A Ringing\u2026 (' + secondsLeft + 's)'; }
    else { clearInterval(ringTimer); }
  }, 1000);
  fetch('/ring').catch(function(){});
}

poll();
setInterval(poll, 2000);
</script>
</body>
</html>
)HTMLPAGE";

const char VISION_HTML[] = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AI Leaf Scanner</title>
<style>
:root{ --moss:#2F4A3C; --moss-deep:#1D2E25; --fern:#6FA37E; --fern-bright:#8FC49E; --brass:#C9A15A; --paper:#F6F3EA; --ink:#1C2620; --alert:#D0604F; }
*{box-sizing:border-box;}
html,body{margin:0;padding:0;}
body{ min-height:100vh; display:flex; align-items:flex-start; justify-content:center; background:radial-gradient(circle at 50% -10%, #3C5A48, #16211B 70%); font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; color:var(--ink); padding:28px 14px; }
.jar{ width:100%; max-width:400px; border-radius:26px; overflow:hidden; background:var(--paper); box-shadow:0 18px 40px rgba(20,30,24,0.28); }
.topbar{ display:flex; align-items:center; justify-content:space-between; padding:12px 18px; background:var(--moss-deep); color:var(--paper); }
.topbar a{ color:var(--fern-bright); text-decoration:none; font-size:13px; font-weight:600; }
.topbar .title{ font-family:Georgia,"Iowan Old Style",Palatino,serif; font-style:italic; font-size:15px; }
.status{ padding:10px 16px; font-size:12.5px; color:#5B6B60; text-align:center; background:#EFE9D8; }
.camWrap{ position:relative; background:#0F1712; display:flex; align-items:center; justify-content:center; min-height:260px; }
.camWrap canvas{ display:block; max-width:100%; }
.resultBadge{ position:absolute; bottom:8px; left:8px; right:8px; padding:8px 12px; border-radius:10px; background:rgba(29,46,37,0.82); color:var(--paper); font-weight:700; font-size:14px; text-align:center; }
.resultBadge.pestAlert{ background:var(--alert); animation:pulse 1s ease-in-out infinite; }
@keyframes pulse{ 0%,100%{opacity:1;} 50%{opacity:0.7;} }
.bars{ padding:12px 16px 4px; }
.barRow{ display:flex; align-items:center; gap:8px; font-size:12px; margin-bottom:6px; }
.barLabel{ width:80px; flex-shrink:0; color:#5B6B60; }
.barTrack{ flex:1; height:8px; border-radius:4px; background:#E6E0CF; overflow:hidden; }
.barFill{ height:100%; background:var(--fern); transition:width 0.25s ease; }
.barPct{ width:34px; text-align:right; color:#5B6B60; }
.controls{ padding:12px 16px 16px; text-align:center; }
button{ width:100%; padding:14px; border:none; border-radius:12px; background:linear-gradient(180deg, var(--fern-bright), var(--fern)); color:var(--moss-deep); font-size:15px; font-weight:700; cursor:pointer; margin-top:8px; }
button:disabled{ opacity:0.6; }
#switchBtn{ background:transparent; border:1.5px solid var(--brass); color:var(--moss-deep); }
.note{ padding:0 16px 16px; font-size:11px; color:#93998E; text-align:center; }
</style>
</head>
<body>
  <div class="jar">
    <div class="topbar">
      <a href="/">&larr; Dashboard</a>
      <span class="title">AI Leaf Scanner</span>
      <span></span>
    </div>
    <div class="status" id="status">Loading AI model&hellip;</div>
    <div class="status" id="diag" style="text-align:left;font-size:11px;line-height:1.45;background:#F7F4EA;color:#4E5B52;white-space:pre-wrap;">Checking browser camera support&hellip;</div>
    <div class="camWrap">
      <div id="webcam-container"></div>
      <div class="resultBadge" id="resultBadge" style="display:none"></div>
    </div>
    <div class="bars" id="bars"></div>
    <div class="controls">
      <button id="startBtn" onclick="startCamera()" disabled>Start camera</button>
      <button id="switchBtn" onclick="switchCamera()" style="display:none">Switch camera</button>
    </div>
    <div class="note">Model trained on Teachable Machine &middot; runs entirely on this phone, right in the browser &mdash; the ESP32 only receives the result.</div>
  </div>

<script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.11.0/dist/tf.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/@teachablemachine/image@0.8.5/dist/teachablemachine-image.min.js"></script>
<script>
var MODEL_URL = "https://storage.googleapis.com/tm-model/Lmrqw2ovw/model.json";
var METADATA_URL = "https://storage.googleapis.com/tm-model/Lmrqw2ovw/metadata.json";

var model, webcam, running = false, facing = "environment", lastReportTime = 0;

function setStatus(msg){ document.getElementById('status').textContent = msg; }

function setDiag(msg){ document.getElementById('diag').textContent = msg; }

function cameraDiagnostics(){
  var lines = [];
  lines.push('Page: ' + window.location.protocol + '//' + window.location.host);
  lines.push('Secure context: ' + (window.isSecureContext ? 'YES' : 'NO'));
  lines.push('mediaDevices API: ' + (navigator.mediaDevices ? 'YES' : 'NO'));
  lines.push('getUserMedia: ' + (navigator.mediaDevices && navigator.mediaDevices.getUserMedia ? 'YES' : 'NO'));
  lines.push('User agent: ' + navigator.userAgent);
  if (!window.isSecureContext) {
    lines.push('');
    lines.push('IMPORTANT: this page is not a secure context.');
    lines.push('On most mobile browsers, camera access needs HTTPS.');
    lines.push('Same WiFi does NOT make an http://192.168.x.x page HTTPS.');
  }
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    lines.push('');
    lines.push('Camera API is unavailable in this browser/context.');
  }
  setDiag(lines.join('\n'));
}

cameraDiagnostics();

function loadModel(){
  return tmImage.load(MODEL_URL, METADATA_URL).then(function(m){
    model = m;
    setStatus('Model ready \u2014 tap Start camera.');
    document.getElementById('startBtn').disabled = false;
  });
}

function startCamera(){
  document.getElementById('startBtn').disabled = true;
  setStatus('Requesting camera access\u2026');
  cameraDiagnostics();

  if (!window.isSecureContext) {
    setStatus('Camera blocked before permission prompt: page is not a secure context.');
    setDiag(document.getElementById('diag').textContent +
      '\n\nFix: open this scanner from HTTPS (or localhost on the same device). HTTP over local WiFi is usually not enough.');
    document.getElementById('startBtn').disabled = false;
    return;
  }

  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    setStatus('Camera API unavailable in this browser/context.');
    document.getElementById('startBtn').disabled = false;
    return;
  }

  var flip = false;
  webcam = new tmImage.Webcam(300, 300, flip);
  webcam.setup({ facingMode: facing }).then(function(){
    return webcam.play();
  }).then(function(){
    document.getElementById('webcam-container').innerHTML = '';
    document.getElementById('webcam-container').appendChild(webcam.canvas);
    document.getElementById('resultBadge').style.display = 'block';
    document.getElementById('switchBtn').style.display = 'inline-block';
    setStatus('Scanning\u2026');
    setDiag(document.getElementById('diag').textContent + '\n\nCamera stream started successfully.');
    running = true;
    window.requestAnimationFrame(loop);
  }).catch(function(e){
    var name = e && e.name ? e.name : 'UnknownError';
    var message = e && e.message ? e.message : String(e);
    var extra = '';
    if (name === 'NotAllowedError' || name === 'PermissionDeniedError') {
      extra = ' Browser denied camera access. Check site permissions/settings for this exact site.';
    } else if (name === 'NotFoundError' || name === 'DevicesNotFoundError') {
      extra = ' No usable camera was found.';
    } else if (name === 'NotReadableError' || name === 'TrackStartError') {
      extra = ' Camera may already be in use by another app/browser tab.';
    } else if (name === 'OverconstrainedError') {
      extra = ' Requested camera constraints were not satisfied; try Switch camera.';
    } else if (name === 'SecurityError') {
      extra = ' Browser security policy blocked camera access.';
    }
    setStatus('Camera error: ' + name + ' - ' + message);
    setDiag(document.getElementById('diag').textContent + '\n\nCamera error: ' + name + '\n' + message + extra);
    console.error('Camera error', e);
    document.getElementById('startBtn').disabled = false;
  });
}

function switchCamera(){
  facing = (facing === 'environment') ? 'user' : 'environment';
  running = false;
  if (webcam) webcam.stop();
  startCamera();
}

function loop(){
  if (!running) return;
  webcam.update();
  predict().then(function(){
    window.requestAnimationFrame(loop);
  });
}

function predict(){
  return model.predict(webcam.canvas).then(function(prediction){
    prediction.sort(function(a,b){ return b.probability - a.probability; });
    renderBars(prediction);
    var top = prediction[0];
    updateBadge(top);
    var now = Date.now();
    if (now - lastReportTime > 1500) {
      lastReportTime = now;
      reportToBoard(top.className, top.probability);
    }
  });
}

function renderBars(prediction){
  var host = document.getElementById('bars');
  host.innerHTML = '';
  prediction.forEach(function(p){
    var pct = Math.round(p.probability * 100);
    var row = document.createElement('div');
    row.className = 'barRow';
    row.innerHTML = '<span class="barLabel">' + p.className + '</span><div class="barTrack"><div class="barFill" style="width:' + pct + '%"></div></div><span class="barPct">' + pct + '%</span>';
    host.appendChild(row);
  });
}

function updateBadge(top){
  var badge = document.getElementById('resultBadge');
  var pct = Math.round(top.probability * 100);
  badge.textContent = top.className + ' \u2014 ' + pct + '%';
  var isPest = top.className.toLowerCase().indexOf('leaf') === -1;
  badge.classList.toggle('pestAlert', isPest && top.probability > 0.7);
}

function reportToBoard(label, prob){
  var url = '/pest?label=' + encodeURIComponent(label) + '&conf=' + Math.round(prob * 100);
  fetch(url).catch(function(){});
}

loadModel().catch(function(e){
  setStatus('Failed to load model: ' + e.message + ' (check phone has internet access on this WiFi)');
});
</script>
</body>
</html>
)HTMLPAGE";

void handleVision() {
  server.send(200, "text/html", VISION_HTML);
}

void handlePest() {
  if (server.hasArg("label")) {
    lastPestLabel = server.arg("label");
    lastPestConf  = server.hasArg("conf") ? server.arg("conf").toInt() : 0;
    lastPestTime  = millis();
    havePestReport = true;
    Serial.print(F(">>> AI vision report: "));
    Serial.print(lastPestLabel);
    Serial.print(F(" ("));
    Serial.print(lastPestConf);
    Serial.println(F("%) <<<"));
  }
  server.send(200, "text/plain", "ok");
}

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json = "{";
  json += "\"tempC\":" + String(lastTempC, 1) + ",";
  json += "\"humidity\":" + String(lastHumidity, 1) + ",";
  json += "\"soilPercent\":" + String(lastSoilPercent) + ",";
  json += "\"lightPercent\":" + String(lastLightPercent) + ",";
  json += "\"pumpActive\":" + String(pumpIsOn ? "true" : "false") + ",";
  json += "\"overrideActive\":" + String(overrideActiveGlobal ? "true" : "false") + ",";
  json += "\"alarmActive\":" + String(waterEmptyAlarm ? "true" : "false") + ",";
  json += "\"buzzerMuted\":" + String(buzzerMuted ? "true" : "false") + ",";
  json += "\"ringActive\":" + String(ringActive ? "true" : "false") + ",";
  json += "\"havePestReport\":" + String(havePestReport ? "true" : "false") + ",";
  json += "\"pestLabel\":\"" + lastPestLabel + "\",";
  json += "\"pestConf\":" + String(lastPestConf) + ",";
  json += "\"pestAgeSec\":" + String(havePestReport ? (millis() - lastPestTime) / 1000 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleWater() {
  webWaterPulseActive = true;
  webWaterPulseStart = millis();
  server.send(200, "text/plain", "Watering started");
}

void handleMute() {
  buzzerMuted = true;
  server.send(200, "text/plain", "Buzzer muted");
}

void handleRing() {
  ringActive = true;
  ringStartTime = millis();
  server.send(200, "text/plain", "Ringing");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, (RELAY_ACTIVE_HIGH) ? LOW : HIGH);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, (BUZZER_ACTIVE_HIGH ? LOW : HIGH));

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== Plant Watering System - WiFi dashboard sketch ==="));

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_POWERON) {
    bootLoopCount = 0;
  } else {
    bootLoopCount++;
    Serial.print(F("Non-power-on reset detected (reason code "));
    Serial.print((int)resetReason);
    if (resetReason == ESP_RST_BROWNOUT) Serial.print(F(", BROWNOUT"));
    Serial.print(F("), consecutive count: "));
    Serial.println(bootLoopCount);
    if (bootLoopCount >= BOOT_LOOP_ALARM_THRESHOLD) {
      Serial.println(F(">>> Board has reset repeatedly without a clean power-on -- likely the pump is browning out the supply. Setting the empty-reservoir alarm AND the pump lockout as a safety stop. <<<"));
      waterEmptyAlarm = true;
      pumpFaultLatched = true; 
    }
  }

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  setPump(false);
  setBuzzer(false);
  setLed(PIN_LED_RED, false);
  setLed(PIN_LED_GREEN, false);
  setLed(PIN_LED_HEARTBEAT, false);

  analogReadResolution(12);
  dht.begin();

  runStartupSelfTest();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("Connecting to WiFi"));
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(300);
    Serial.print(F("."));
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false); 
    Serial.print(F("Connected! Open this on your phone: http://"));
    Serial.println(WiFi.localIP());
    if (MDNS.begin("plant")) {
      Serial.println(F("You can also try: http://plant.local (works on most phones; if not, use the IP above)"));
    }
  } else {
    Serial.println(F("WiFi did NOT connect within 20 seconds."));
    Serial.println(F("The watering logic keeps running normally either way — only the phone dashboard needs WiFi."));
    Serial.println(F("Double-check WIFI_SSID / WIFI_PASSWORD, and that it's a 2.4GHz network (ESP32 can't join 5GHz-only WiFi)."));
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/water", handleWater);
  server.on("/mute", handleMute);
  server.on("/ring", handleRing);
  server.on("/vision", handleVision);
  server.on("/pest", handlePest);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println(F("Monitoring started."));
  Serial.println();
}

void loop() {
  server.handleClient();
  unsigned long now = millis();

  enforcePumpSafetyCap(now);

  handleResetButton(now);

  if (webWaterPulseActive && now - webWaterPulseStart >= WATERING_DURATION_MS) {
    webWaterPulseActive = false;
  }

  if (ringActive && now - ringStartTime >= RING_DURATION_MS) {
    ringActive = false;
    setLed(PIN_LED_HEARTBEAT, false);
    driveBuzzer(false, 0);
    currentBuzzerState = false;
  }

  bool overrideActive = webWaterPulseActive;
  overrideActiveGlobal = overrideActive;

  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    readAllSensors();
    sensorsInitialized = true;
    printStatus(overrideActive);
  }

  if (overrideActive) {
    setPump(true); 
    setLed(PIN_LED_RED, true);
    wateringState = STATE_IDLE;
  } else {
    runWateringStateMachine(now);
  }

  if (ringActive) {
    handleRingPresentation(now);
  } else if (!overrideActive && wateringState != STATE_WATERING) {
    handleHeartbeat(now);
  }

  handleAlarmPresentation(now, overrideActive);
}
