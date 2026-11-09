#include <Arduino.h>

#define PIN_ENCODER     2   // Hardware interrupt
#define PIN_ON          4   // motor enable
#define PIN_START_STOP  5   // motor motion 
#define PIN_DIRECTION   6   // rotation direction

//default pin levels
//to check
#define DIR_RIGHT       HIGH
#define DIR_LEFT        LOW
#define MOTOR_START     HIGH
#define MOTOR_STOP      LOW
#define MOTOR_ON        HIGH
#define MOTOR_OFF       LOW

//motor parameters
//to check
#define IMPULSE_PER_ROTATE  350UL
#define WHEEL_R     204UL   // wheel circumference
#define ROTATION_DEG  140UL   // 90deg curve dist

#define IMPULSE_DIST(mm)  (((uint32_t)(mm) * IMPULSE_PER_ROTATE) / WHEEL_R)

volatile uint32_t IMPULSE_CURR = 0; // single maneuver distance counter
volatile uint32_t IMPULSE_TOTAL = 0; // total distance counter

void IMPULSE_COUNTER(void) {
    IMPULSE_CURR++;
    IMPULSE_TOTAL++;
}

void Movement(const char* maneuver_name, uint8_t direction, uint32_t impulse_count) {
    Serial.print("[START] Maneuver: ");
    Serial.println(maneuver_name);
    Serial.print("Impulse count: ");
    Serial.println(impulse_count);

    noInterrupts();
    IMPULSE_CURR = 0;
    interrupts();

    digitalWrite(PIN_DIRECTION, direction);
    delay(200);

    digitalWrite(PIN_START_STOP, MOTOR_START);

    uint32_t impulse_read = 0;
    uint32_t last_reported_ms = millis();

    while (1) {
        noInterrupts();
        impulse_read = IMPULSE_CURR;
        interrupts();

        if (impulse_read >= impulse_count) {
            break;
        }

        if (millis() - last_reported_ms >= 250) {
            last_reported_ms = millis();
            Serial.print("Impulses in maneuver: ");
            Serial.println(impulse_read);
        }
    }

    digitalWrite(PIN_START_STOP, MOTOR_STOP); //to check with analogWrite
    delay(500);

    uint32_t end_sum = 0;
    uint32_t total_sum = 0;
    noInterrupts();
    end_sum = IMPULSE_CURR;
    total_sum = IMPULSE_TOTAL;
    interrupts();

    Serial.print("[STOP]  Ended: ");
    Serial.println(maneuver_name);
    Serial.print(" -> impulse_read(encoder): ");
    Serial.print(end_sum);
    Serial.print("distance measurement error: ");
    Serial.print(end_sum - impulse_count);
    Serial.println(" imp)");
    Serial.print("IMPULSE_TOTAL: ");
    Serial.print(total_sum);
    Serial.println(" imp");

    delay(1000);
}
void setup(void) {
    Serial.begin(115200);

    pinMode(PIN_ON      , OUTPUT);
    pinMode(PIN_START_STOP, OUTPUT);
    pinMode(PIN_DIRECTION, OUTPUT);
    pinMode(PIN_ENCODER, INPUT_PULLUP);

    digitalWrite(PIN_START_STOP, MOTOR_STOP);
    digitalWrite(PIN_ON     , MOTOR_OFF);
    digitalWrite(PIN_DIRECTION, DIR_RIGHT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), IMPULSE_COUNTER, RISING);

    Serial.println("Start in 5 seconds");
    delay(1000);
    Serial.println("4");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);
    Serial.println("Test started");

    digitalWrite(PIN_ON     , MOTOR_ON);
    Serial.println("[SYSTEM] MOTOR WLACZONY (Enable = HIGH)");
    delay(100);

    uint32_t impulse_rotate_90  = IMPULSE_DIST(ROTATION_DEG);
    uint32_t impulse_rotate_dist    = IMPULSE_DIST(1000UL);             // 1000 mm = 1 m
    uint32_t impulse_rotate_180 = IMPULSE_DIST(ROTATION_DEG * 2UL);

    Movement("1. RIGHT 90 deg rotation", DIR_RIGHT, impulse_rotate_90);

    Movement("2. 1 meter drive", DIR_RIGHT, impulse_rotate_dist);

    Movement("3. LEFT 90 deg rotation", DIR_LEFT, impulse_rotate_90);

    Movement("4. 180 deg rotation", DIR_LEFT, impulse_rotate_180);

    digitalWrite(PIN_ON     , MOTOR_OFF);
    Serial.println("TEST ENDED");

}

void loop(void) {}