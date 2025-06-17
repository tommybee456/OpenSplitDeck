// ------------------------
// CONTROLLER CODE (Left or Right)
// ------------------------

// Uncomment this line for LEFT controller, comment it for RIGHT
#define LEFT_CONTROLLER

#include <Arduino.h>
#include "nrf_to_nrf.h"
#include "src\IQS7211E\IQS7211E.h"

#define DEMO_IQS7211E_ADDR 0x56
#define DEMO_IQS7211E_POWER_PIN 26
#define DEMO_IQS7211E_RDY_PIN 2

/*** Instances ***/
IQS7211E iqs7211e;
iqs7211e_gestures_e gesture;
iqs7211e_gestures_e last_gesture;

/*** Global Variables ***/
bool show_data = false;

uint8_t mode = 0;

uint16_t dataX[2] = { 0, 0 };
uint16_t dataY[2] = { 0, 0 };
int16_t deltaX = 0;
int16_t deltaY = 0;
int16_t deltaX_overflow = 0;
int16_t deltaY_overflow = 0;

int16_t stickX_min = 2350;
int16_t stickX_max = 12350;
int16_t stickX = 0;

int16_t stickY_min = 2355;
int16_t stickY_max = 12300;
int16_t stickY = 0;

int16_t trig_min = 6800;
int16_t trig_max = 7250;
int16_t trig = 0;

bool count = 0;
bool data_reset = 0;

// Filtered values
float smoothedX = 0;
float smoothedY = 0;
float smoothedTrig = 0;

// EMA smoothing factor (0.0 to 1.0)
// Lower = smoother, Higher = more responsive
const float alpha = 0.2;

nrf_to_nrf radio;
uint8_t address[][6] = { "1Node", "2Node" };

struct payload_t {
#ifdef LEFT_CONTROLLER
  uint8_t flags = 0b10000000;
#else
  uint8_t flags = 0b00000000;
#endif
  int8_t trigger;   //analog trigger
  int8_t stickX;    //analogstick X
  int8_t stickY;    //analogstick Y
  int8_t padX;      //trackpad X
  int8_t padY;      //trackpad Y
  uint8_t buttons;  //gamepad buttons
};

payload_t payload;
bool ping = false;

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}
  }

  radio.setPALevel(NRF_PA_MAX);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPayloadSize(sizeof(payload));
  radio.openReadingPipe(1, address[0]);
  radio.openReadingPipe(2, address[1]);
  radio.openWritingPipe(
#ifdef LEFT_CONTROLLER
    address[1]
#else
    address[0]
#endif
  );

  radio.startListening();

  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(2, INPUT);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  /* Power On IQS7211E */
  pinMode(DEMO_IQS7211E_POWER_PIN, OUTPUT);
  delay(200);
  digitalWrite(DEMO_IQS7211E_POWER_PIN, LOW);
  delay(200);
  digitalWrite(DEMO_IQS7211E_POWER_PIN, HIGH);
  // while (!Serial)
  //   ;
  delay(500);
  Serial.print("Start");
  /* Initialize the IQS7211E with input parameters device address and RDY pin */
  iqs7211e.begin(DEMO_IQS7211E_ADDR, DEMO_IQS7211E_RDY_PIN);
  Serial.println("IQS7211E Ready");

  analogReadResolution(14);

  delay(100);
}

void loop() {
  if (radio.available()) {
    radio.read(&ping, sizeof(ping));
    radio.stopListening();  //maybe move this to the if statement, needs testing

#ifdef LEFT_CONTROLLER
    if (ping) {
#else
    if (!ping) {
#endif
      // payload.mouse = !digitalRead(29);
      // payload.keyboard = !digitalRead(28);
      // payload.gamepad = !digitalRead(30);
      radio.write(&payload, sizeof(payload));
    }

    radio.startListening();
  }
  buttons();
  trackpad();
  analogreading();
}

void buttons() {

  if (!digitalRead(17)) {  // stick
    payload.buttons |= 0b00100000;
  } else {
    payload.buttons &= ~0b00100000;
  }

  if (!digitalRead(18)) {  // bumper
    payload.buttons |= 0b00010000;
  } else {
    payload.buttons &= ~0b00010000;
  }

#ifdef LEFT_CONTROLLER
  if (!digitalRead(19)) {
    payload.flags |= 0b00100000;
  } else {
    payload.flags &= ~0b00100000;
  }
#else
  if (!digitalRead(19)) {  // Home/Mode
    mode++;
    delay(200);
  }
  if (mode == 0) {
    payload.flags &= ~0b01100000;
  } else if (mode == 1) {
    payload.flags |= 0b00100000;
    payload.flags &= ~0b01000000;
  } else if (mode == 2) {
    payload.flags |= 0b01000000;
    payload.flags &= ~0b00100000;
  } else {
    mode = 0;
  }
#endif

  if (!digitalRead(13)) {  // A/Down
    payload.buttons |= 0b00001000;
  } else {
    payload.buttons &= ~0b00001000;
  }

  if (!digitalRead(12)) {  // B/Right
    payload.buttons |= 0b00000100;
  } else {
    payload.buttons &= ~0b00000100;
  }

  if (!digitalRead(11)) {  // X/left
    payload.buttons |= 0b00000010;
  } else {
    payload.buttons &= ~0b00000010;
  }

  if (!digitalRead(10)) {  // Y/Up
    payload.buttons |= 0b00000001;
  } else {
    payload.buttons &= ~0b00000001;
  }

  if (!digitalRead(9)) {  // Start/Select
    payload.buttons |= 0b10000000;
  } else {
    payload.buttons &= ~0b10000000;
  }

  if (!digitalRead(6)) {  // P4
    payload.flags |= 0b00000010;
  } else {
    payload.flags &= ~0b00000010;
  }

  if (!digitalRead(5)) {  // P5
    payload.flags |= 0b00000001;
  } else {
    payload.flags &= ~0b00000001;
  }

  // Serial.print(digitalRead(17));
  // Serial.print('\t');
  // Serial.print(digitalRead(18));
  // Serial.print('\t');
  // Serial.print(digitalRead(19));
  // Serial.print('\t');
  // Serial.print(digitalRead(13));
  // Serial.print('\t');
  // Serial.print(digitalRead(12));
  // Serial.print('\t');
  // Serial.print(digitalRead(11));
  // Serial.print('\t');
  // Serial.print(digitalRead(10));
  // Serial.print('\t');
  // Serial.print(digitalRead(9));
  // Serial.print('\t');
  // Serial.print(digitalRead(6));
  // Serial.print('\t');
  // Serial.print(digitalRead(5));
  // Serial.print('\t');
}

void analogreading() {
  stickX = analogRead(A0);
  stickY = analogRead(A1);
  trig = analogRead(A2);

  // Apply exponential moving average
  smoothedX = alpha * stickX + (1.0 - alpha) * smoothedX;
  smoothedY = alpha * stickY + (1.0 - alpha) * smoothedY;
  smoothedTrig = alpha * trig + (1.0 - alpha) * smoothedTrig;

  stickX = map(smoothedX, stickX_min, stickX_max, -128, 127);
  if (stickX > 127) {
    stickX = 127;
  }
  if (stickX < -128) {
    stickX = -128;
  }
  if (stickX < 5 && stickX > -5) {
    stickX = 0;
  }

  stickY = map(smoothedY, stickY_min, stickY_max, -128, 127);
  if (stickY > 127) {
    stickY = 127;
  }
  if (stickY < -128) {
    stickY = -128;
  }
  if (stickY < 5 && stickY > -5) {
    stickY = 0;
  }

  trig = map(smoothedTrig, trig_min, trig_max, -128, 127);
  if (trig > 127) {
    trig = 127;
  }
  if (trig < -128) {
    trig = -128;
  }
  // if (trig < -113) {
  //   trig = -128;
  // }

  // Serial.print((int)smoothedX);
  // Serial.print('\t');
  // Serial.print((int8_t)stickX);
  // Serial.print('\t');
  // Serial.print((int)smoothedY);
  // Serial.print('\t');
  // Serial.print((int8_t)stickY);
  // Serial.print('\t');
  // Serial.print((int)smoothedTrig);
  // Serial.print('\t');
  // Serial.println((int8_t)trig);

  payload.stickX = stickX;
  payload.stickY = stickY;
  payload.trigger = trig;
}

void trackpad() {
  /* Read new data from IQS7211E if available  (RDY Line Low) */
  iqs7211e.run();

  static uint32_t reset_val_timer = 0;
  static uint32_t ignore_press_timer = 0;
  static bool ignore_press = 1;
  if (iqs7211e.new_data_available) {

    uint16_t xbuff = iqs7211e.getAbsXCoordinate(FINGER_1);
    uint16_t ybuff = iqs7211e.getAbsYCoordinate(FINGER_1);

    if ((xbuff != 65535) && (ybuff != 65535)) {

      if ((millis() - ignore_press_timer > 150) || !ignore_press) {
        ignore_press = 0;
      }

      if (!ignore_press) {
        dataX[count] = xbuff;
        dataY[count] = ybuff;
        if (count) {
          deltaX = dataX[1] - dataX[0];
          deltaY = dataY[1] - dataY[0];
        } else {
          if (!data_reset) {
            deltaX = dataX[0] - dataX[1];
            deltaY = dataY[0] - dataY[1];
          }
        }
        data_reset = 0;

        deltaX = deltaX * 0.25;
        deltaY = deltaY * 0.25;

        count = !count;
      }

      // Serial.print(deltaX);
      // Serial.print('\t');
      // Serial.print(deltaX_overflow);
      // Serial.print('\t');
      // Serial.print(deltaY);
      // Serial.print('\t');
      // Serial.println(deltaY_overflow);

      iqs7211e.new_data_available = false;
      reset_val_timer = millis();
    } else {
      if (millis() - reset_val_timer > 150) {
        ignore_press = 1;
        reset_val_timer = millis();
        ignore_press_timer = millis();
        count = 0;
        data_reset = 1;
        dataX[0] = 0;
        dataX[1] = 0;
        dataY[0] = 0;
        dataY[1] = 0;
        deltaX = deltaX * 0.4;
        deltaY = deltaY * 0.4;
        if (deltaX < 7 && deltaX > -7) {
          deltaX = 0;
        }
        if (deltaY < 7 && deltaY > -7) {
          deltaY = 0;
        }
      }
    }
  }
  static uint32_t ms = 0;
  if (millis() - ms > 10) {
    ms = millis();
    trackpad_buttons();
  }
}

void trackpad_buttons() {
  // Combine new delta with stored overflow
  int16_t rawX = deltaX + deltaX_overflow;
  int16_t rawY = deltaY + deltaY_overflow;

  // Clip to int8_t range
  int8_t reportX = constrain(rawX, -127, 127);
  int8_t reportY = constrain(rawY, -127, 127);

  // Store remaining overflow
  deltaX_overflow = rawX - reportX;
  deltaY_overflow = rawY - reportY;

  payload.padX = reportX;
  payload.padY = reportY;


  static bool clicked = false;
  static uint32_t click_release_time = 0;

  static bool double_clicked = false;
  static uint32_t double_click_release_time = 0;

  // Check gesture
  gesture = iqs7211e.get_touchpad_event();
  uint32_t now = millis();

  // Handle single tap
  if (gesture != last_gesture && gesture == IQS7211E_GESTURE_SINGLE_TAP && !clicked) {
    payload.flags |= 0b00000100;  // Press left button
    clicked = true;
    click_release_time = now + 50;  // Hold for 50ms
  }

  // Handle double tap
  if (gesture != last_gesture && gesture == IQS7211E_GESTURE_DOUBLE_TAP && !double_clicked) {
    payload.flags |= 0b00001000;  // Press right button
    double_clicked = true;
    double_click_release_time = now + 50;  // Hold for 50ms
  }

  // Release single tap
  if (clicked && now >= click_release_time) {
    payload.flags &= ~0b00000100;
    clicked = false;
  }

  // Release double tap
  if (double_clicked && now >= double_click_release_time) {
    payload.flags &= ~0b00001000;
    double_clicked = false;
  }

  last_gesture = gesture;
}