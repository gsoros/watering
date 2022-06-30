#include <Arduino.h>

#define SEC 0x3e8UL
#define MIN 0xea60UL
#define HOUR 0x36ee80UL
#define DAY 0x5265c00UL

const uint8_t pinTankVcc = 2;      // D2
const uint8_t pinTankSignal = 14;  // A0 47k pulldown
const uint8_t pinSoilVcc = 3;      // D3
const uint8_t pinSoilSignal = 15;  // A1 47k pulldown
const uint8_t pinRelay = 5;        // D5

const float tankThreshold = 1.0f;
const float soilThreshold = 2.0f;

const unsigned long checkPeriodTime = 3 * SEC;
const unsigned long wateringTime = 1 * SEC;
const unsigned long limitPeriodTime = 30 * SEC;
const unsigned int limitAmount = 3;

unsigned int waterings = 0;
unsigned long periodStartTime = 0;

void checkPeriod(unsigned long);
bool limitIsReached();
bool tankIsEmpty();
bool soilIsDry();
void water();
void log(const char *, ...);
float voltage(uint8_t, uint8_t, uint8_t numMeasurements = 100);

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));
    analogReference(DEFAULT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(pinTankVcc, OUTPUT);
    digitalWrite(pinTankVcc, LOW);
    pinMode(pinTankSignal, INPUT);
    pinMode(pinSoilVcc, OUTPUT);
    digitalWrite(pinSoilVcc, LOW);
    pinMode(pinSoilSignal, INPUT);
    pinMode(pinRelay, OUTPUT);
    digitalWrite(pinRelay, LOW);
    log("minute: %lu secs, hour: %lu minutes, day: %lu hours, rollover: %lu days",
        MIN / SEC,
        HOUR / MIN,
        DAY / HOUR,
        UINT32_MAX / DAY);
}

void loop() {
    // log("tank %s %.2fV, soil %s %.2fV",
    //     tankIsEmpty() ? "empty" : "ok",
    //     voltage(pinTankVcc, pinTankSignal),
    //     soilIsDry() ? "dry" : "wet",
    //     voltage(pinSoilVcc, pinSoilSignal));
    // delay(500);
    // return;

    unsigned long t = millis();
    log("check %lu:%02lu:%02lu:%02lu",
        t / DAY, t / HOUR, t / MIN % 60, t / SEC % 60);
    digitalWrite(LED_BUILTIN, HIGH);
    checkPeriod(t);
    if (!limitIsReached() && !tankIsEmpty() && soilIsDry()) {
        water();
    }
    digitalWrite(LED_BUILTIN, LOW);
    // unsigned long loopTime = millis() - t;
    //  if (loopTime < checkPeriodTime) {
    //   delay(checkPeriodTime - loopTime);
    // }
    delay(checkPeriodTime);
}

void checkPeriod(unsigned long t) {
    if (limitPeriodTime < t && periodStartTime < t - limitPeriodTime) {
        log("period reset");
        periodStartTime = t;
        waterings = 0;
    }
}

bool limitIsReached() {
    if (limitAmount <= waterings) {
        log("limit reached");
        return true;
    }
    return false;
}

bool tankIsEmpty() {
    float v = voltage(pinTankVcc, pinTankSignal);
    log("tank %.2fV", v);
    return v < tankThreshold;
    // if (1 < random(10)) {
    //     log("tank ok");
    //     return false;
    // }
    // log("tank empty");
    // return true;
}

bool soilIsDry() {
    float v = voltage(pinSoilVcc, pinSoilSignal);
    log("soil %.2fV", v);
    return v < soilThreshold;
    // if (7 < random(10)) {
    //     log("soil wet");
    //     return false;
    // }
    // log("soil dry");
    // return true;
}

void water() {
    log("watering %d", waterings);
    digitalWrite(pinRelay, HIGH);
    delay(wateringTime);
    digitalWrite(pinRelay, LOW);
    waterings++;
}

void log(const char *fmt, ...) {
    static char out[256];
    va_list argp;
    va_start(argp, fmt);
    vsnprintf(out, 256, fmt, argp);
    va_end(argp);
    Serial.println(out);
}

float voltage(uint8_t pinVcc, uint8_t pinSignal, uint8_t numMeasurements) {
    if (!numMeasurements) return 0.0f;
    static const double factor = 5.0 / 1023.0;
    double value = 0.0;
    digitalWrite(pinVcc, HIGH);
    for (uint8_t i = 0; i < numMeasurements; i++) {
        value += analogRead(pinSignal) * factor;
    }
    digitalWrite(pinVcc, LOW);
    return (float)(value / numMeasurements);
}