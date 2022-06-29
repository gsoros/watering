#include <Arduino.h>

#define SEC 0x3e8UL
#define MIN 0xea60UL
#define HOUR 0x36ee80UL
#define DAY 0x5265c00UL

const unsigned long checkPeriodTime = 5 * SEC;
const unsigned long wateringTime = 3 * SEC;
const unsigned long limitPeriodTime = 1 * MIN;
const unsigned int limitAmount = 3;

unsigned int waterings = 0;
unsigned long periodStartTime = 0;

void checkPeriod(unsigned long);
bool limitIsReached();
bool tankIsEmpty();
bool soilIsDry();
void water();

void log(const char *, ...);

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));
    pinMode(LED_BUILTIN, OUTPUT);
    log("minute: %lu secs, hour: %lu minutes, day: %lu hours, ulong max: %lu days",
        MIN / SEC,
        HOUR / MIN,
        DAY / HOUR,
        UINT32_MAX / DAY);
}

void loop() {
    unsigned long t = millis();
    log("check %lu:%02lu:%02lu:%02lu", t / DAY, t / HOUR, t / MIN % 60, t / SEC % 60);
    digitalWrite(LED_BUILTIN, HIGH);
    checkPeriod(t);
    if (!limitIsReached() && !tankIsEmpty() && soilIsDry()) {
        water();
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(checkPeriodTime - (millis() - t));
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
    if (1 < random(10)) {
        log("tank ok");
        return false;
    }
    log("tank empty");
    return true;
}

bool soilIsDry() {
    if (7 < random(10)) {
        log("soil wet");
        return false;
    }
    log("soil dry");
    return true;
}

void water() {
    log("watering %d", waterings);
    delay(wateringTime);
    waterings++;
}

void log(const char *fmt, ...) {
    char out[256];
    va_list argp;
    va_start(argp, fmt);
    vsnprintf(out, 256, fmt, argp);
    va_end(argp);
    Serial.println(out);
}
