// ------------------------
// DONGLE CODE
// ------------------------

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "nrf_to_nrf.h"

nrf_to_nrf radio;
uint8_t address[][6] = { "1Node", "2Node" };

struct payload_t {
  uint8_t flags;
  int8_t trigger;   //analog trigger
  int8_t stickX;    //analogstick X
  int8_t stickY;    //analogstick Y
  int8_t padX;      //trackpad X
  int8_t padY;      //trackpad Y
  uint8_t buttons;  //gamepad buttons
};

payload_t payload;

// HID Report IDs
enum {
  RID_KEYBOARD = 1,
  RID_MOUSE,
  RID_GAMEPAD,
};

uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
  TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE)),
  TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(RID_GAMEPAD))
};

Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;
hid_mouse_report_t m;

hid_gamepad_report_t gp_old;
hid_mouse_report_t m_old;

bool keyboard_button_state = 0, keyboard_button_state_old = 0;
bool gamepad_button_state = 0, gamepad_button_state_old = 0;
bool keyboard_changed = 0, gamepad_changed = 0;
bool mouse_changed;

bool poll_cmd = false;
unsigned long lastPoll = 0;
bool pollingLeft = true;

uint8_t mode = 0;

int l4 = 4;
int l5 = 5;
int r4 = 6;
int r5 = 7;

bool UP = 0;
bool DOWN = 0;
bool LEFT = 0;
bool RIGHT = 0;

bool A = 0;
bool B = 0;
bool X = 0;
bool Y = 0;

bool START = 0;
bool SELECT = 0;
bool GUIDE = 0;
bool L3 = 0;
bool R3 = 0;

bool RB = 0;
bool LB = 0;
bool L4 = 0;
bool L5 = 0;
bool R4 = 0;
bool R5 = 0;
bool STEAM = 0;

bool id;

void setup() {
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);

  Serial.begin(115200);

  usb_hid.setPollInterval(1);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("TommyB's Controller");
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

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
  radio.stopListening();

  Serial.println("Dongle polling controllers...");
}

void process_hid() {
  static uint8_t looper = 0;
  if (TinyUSBDevice.suspended()) {
    TinyUSBDevice.remoteWakeup();
  }

  // if (usb_hid.ready()) {
  //   static bool has_key = false;
  //   if (keyboard_changed) {
  //     if (keyboard_button_state) {
  //       uint8_t keycode[6] = { HID_KEY_A };
  //       usb_hid.keyboardReport(RID_KEYBOARD, 0, keycode);
  //       has_key = true;
  //     } else if (has_key) {
  //       usb_hid.keyboardRelease(RID_KEYBOARD);
  //       has_key = false;
  //     }
  //   }
  // }

  gamepad_changed = (gp.x != gp_old.x || gp.y != gp_old.y || gp.z != gp_old.z || gp.rz != gp_old.rz || gp.rx != gp_old.rx || gp.ry != gp_old.ry || gp.hat != gp_old.hat || gp.buttons != gp_old.buttons);
  mouse_changed = (m.buttons != m_old.buttons || m.x != m_old.x || m.y != m_old.y || m.wheel != m_old.wheel || m.pan != m_old.pan || m.x != 0 || m.y != 0 || m.wheel != 0 || m.pan != 0);

  if (usb_hid.ready()) {
    while (true) {
      if (looper == 0) {

        if (gamepad_changed) {
          usb_hid.sendReport(RID_GAMEPAD, &gp, sizeof(gp));
          gp_old = gp;
          looper++;
          break;
        }
        looper++;
      }

      if (looper == 1) {
        if (mouse_changed) {
          usb_hid.sendReport(RID_MOUSE, &m, sizeof(m));
          m_old = m;
          looper++;
          break;
        }
        looper++;
      }
      if (looper == 2) {
        looper = 0;
      }
      if (!mouse_changed && !gamepad_changed) {
        break;
      }
    }
  }
  if (looper == 2) {
    looper = 0;
  }
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (!TinyUSBDevice.mounted()) return;

  if (millis() - lastPoll >= 2) {
    lastPoll = millis();
    poll_cmd = pollingLeft;
    radio.stopListening();
    radio.openWritingPipe(address[pollingLeft ? 1 : 0]);
    radio.write(&poll_cmd, sizeof(poll_cmd));
    radio.startListening();
    pollingLeft = !pollingLeft;
  }

  if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    id = (payload.flags & 0b10000000) >> 7;
    if (!id) {  //right controller
      mode = (payload.flags & 0b01100000) >> 5;
      if (mode == 0) {  //mouse + gamepad mode

        //gamepad
        gp.z = payload.stickX;    //right stick X
        gp.rz = payload.stickY;   //right stick y
        gp.ry = payload.trigger;  //right trigger

        A = (payload.buttons & 0b00001000) >> 3;
        B = (payload.buttons & 0b00000100) >> 2;
        X = (payload.buttons & 0b00000010) >> 1;
        Y = (payload.buttons & 0b00000001) >> 0;

        START = (payload.buttons & 0b10000000) >> 7;

        R3 = (payload.buttons & 0b00100000) >> 5;
        RB = (payload.buttons & 0b00010000) >> 4;
        R4 = (payload.flags & 0b00000010) >> 1;
        R5 = (payload.flags & 0b00000001) >> 0;

        gp.buttons = A ? setBit(gp.buttons, 0) : clearBit(gp.buttons, 0);
        gp.buttons = B ? setBit(gp.buttons, 1) : clearBit(gp.buttons, 1);
        gp.buttons = X ? setBit(gp.buttons, 2) : clearBit(gp.buttons, 2);
        gp.buttons = Y ? setBit(gp.buttons, 3) : clearBit(gp.buttons, 3);

        gp.buttons = START ? setBit(gp.buttons, 7) : clearBit(gp.buttons, 7);

        gp.buttons = R3 ? setBit(gp.buttons, 9) : clearBit(gp.buttons, 9);
        gp.buttons = RB ? setBit(gp.buttons, 5) : clearBit(gp.buttons, 5);

        //mouse
        m.x = payload.padX;                             //pan on left trackpad
        m.y = payload.padY;                             //scroll on left trackpad
        m.buttons = (payload.flags & 0b00001100) >> 2;  //mouse button
      } else if (mode == 1) {

        //gamepad
        gp.z = payload.stickX;    //right stick X
        gp.rz = payload.stickY;   //right stick y
        gp.ry = payload.trigger;  //right trigger

        A = (payload.buttons & 0b00001000) >> 3;
        B = (payload.buttons & 0b00000100) >> 2;
        X = (payload.buttons & 0b00000010) >> 1;
        Y = (payload.buttons & 0b00000001) >> 0;

        START = (payload.buttons & 0b10000000) >> 7;

        R3 = (payload.buttons & 0b00100000) >> 5;
        RB = (payload.buttons & 0b00010000) >> 4;
        R4 = (payload.flags & 0b00000010) >> 1;
        R5 = (payload.flags & 0b00000001) >> 0;

        gp.buttons = A ? setBit(gp.buttons, 0) : clearBit(gp.buttons, 0);
        gp.buttons = B ? setBit(gp.buttons, 1) : clearBit(gp.buttons, 1);
        gp.buttons = X ? setBit(gp.buttons, 2) : clearBit(gp.buttons, 2);
        gp.buttons = Y ? setBit(gp.buttons, 3) : clearBit(gp.buttons, 3);

        gp.buttons = START ? setBit(gp.buttons, 7) : clearBit(gp.buttons, 7);

        gp.buttons = R3 ? setBit(gp.buttons, 9) : clearBit(gp.buttons, 9);
        gp.buttons = RB ? setBit(gp.buttons, 5) : clearBit(gp.buttons, 5);

        //trackpad
        if(payload.padX != 0 || payload.padY != 0)
        gp.z = payload.padX;                             //pan on left trackpad
        gp.rz = payload.padY;                             //scroll on left trackpad
        //m.buttons = (payload.flags & 0b00001100) >> 2;  //mouse button
      }
    } else if (id) {              //left controller
      if (mode == 0) {            //mouse + gamepad mode
        gp.x = payload.stickX;    //left stick X
        gp.y = payload.stickY;    //right stick y
        gp.rx = payload.trigger;  //right trigger

        UP = (payload.buttons & 0b00000001) >> 0;
        DOWN = (payload.buttons & 0b00001000) >> 3;
        LEFT = (payload.buttons & 0b00000010) >> 1;
        RIGHT = (payload.buttons & 0b00000100) >> 2;

        SELECT = (payload.buttons & 0b10000000) >> 7;

        L3 = (payload.buttons & 0b00100000) >> 5;
        LB = (payload.buttons & 0b00010000) >> 4;
        L4 = (payload.flags & 0b00000010) >> 1;
        L5 = (payload.flags & 0b00000001) >> 0;

        STEAM = (payload.flags & 0b00100000) >> 5;

        gp.buttons = SELECT ? setBit(gp.buttons, 6) : clearBit(gp.buttons, 6);

        gp.buttons = L3 ? setBit(gp.buttons, 8) : clearBit(gp.buttons, 8);
        gp.buttons = LB ? setBit(gp.buttons, 4) : clearBit(gp.buttons, 4);
        gp.buttons = STEAM ? setBit(gp.buttons, 14) : clearBit(gp.buttons, 14);

        if (UP && LEFT) {
          gp.hat = GAMEPAD_HAT_UP_LEFT;
        } else if (UP && RIGHT) {
          gp.hat = GAMEPAD_HAT_UP_RIGHT;
        } else if (DOWN && LEFT) {
          gp.hat = GAMEPAD_HAT_DOWN_LEFT;
        } else if (DOWN && RIGHT) {
          gp.hat = GAMEPAD_HAT_DOWN_RIGHT;
        } else if (UP) {
          gp.hat = GAMEPAD_HAT_UP;
        } else if (DOWN) {
          gp.hat = GAMEPAD_HAT_DOWN;
        } else if (LEFT) {
          gp.hat = GAMEPAD_HAT_LEFT;
        } else if (RIGHT) {
          gp.hat = GAMEPAD_HAT_RIGHT;
        } else {
          gp.hat = GAMEPAD_HAT_CENTERED;
        }

        static float scroll_accum_y = 0;
        static float scroll_accum_x = 0;

        scroll_accum_y += payload.padY / 64.0;  // tune divisor to taste
        scroll_accum_x += payload.padX / 64.0;

        int8_t scroll_y = (int8_t)scroll_accum_y;
        int8_t scroll_x = (int8_t)scroll_accum_x;

        scroll_accum_y -= scroll_y;
        scroll_accum_x -= scroll_x;

        m.wheel = scroll_y;
        m.pan = scroll_x;

      } else if (mode == 1) {
        gp.x = payload.stickX;    //left stick X
        gp.y = payload.stickY;    //right stick y
        gp.rx = payload.trigger;  //right trigger

        UP = (payload.buttons & 0b00000001) >> 0;
        DOWN = (payload.buttons & 0b00001000) >> 3;
        LEFT = (payload.buttons & 0b00000010) >> 1;
        RIGHT = (payload.buttons & 0b00000100) >> 2;

        SELECT = (payload.buttons & 0b10000000) >> 7;

        L3 = (payload.buttons & 0b00100000) >> 5;
        LB = (payload.buttons & 0b00010000) >> 4;
        L4 = (payload.flags & 0b00000010) >> 1;
        L5 = (payload.flags & 0b00000001) >> 0;

        STEAM = (payload.flags & 0b00100000) >> 5;

        gp.buttons = SELECT ? setBit(gp.buttons, 6) : clearBit(gp.buttons, 6);

        gp.buttons = L3 ? setBit(gp.buttons, 8) : clearBit(gp.buttons, 8);
        gp.buttons = LB ? setBit(gp.buttons, 4) : clearBit(gp.buttons, 4);
        gp.buttons = STEAM ? setBit(gp.buttons, 14) : clearBit(gp.buttons, 14);


        static float scroll_accum_y = 0;
        static float scroll_accum_x = 0;

        scroll_accum_y += payload.padY / 64.0;  // tune divisor to taste
        scroll_accum_x += payload.padX / 64.0;

        int8_t scroll_y = (int8_t)scroll_accum_y;
        int8_t scroll_x = (int8_t)scroll_accum_x;

        scroll_accum_y -= scroll_y;
        scroll_accum_x -= scroll_x;

        if (!UP) {
          if (scroll_y < 0) {
            UP = 1;
          }
        }
        if (!DOWN) {
          if (scroll_y > 0) {
            DOWN = 1;
          }
        }
        
        if (!RIGHT) {
          if (scroll_x > 0) {
            RIGHT = 1;
          }
        }
        if (!LEFT) {
          if (scroll_x < 0) {
            LEFT = 1;
          }
        }

        if (UP && LEFT) {
          gp.hat = GAMEPAD_HAT_UP_LEFT;
        } else if (UP && RIGHT) {
          gp.hat = GAMEPAD_HAT_UP_RIGHT;
        } else if (DOWN && LEFT) {
          gp.hat = GAMEPAD_HAT_DOWN_LEFT;
        } else if (DOWN && RIGHT) {
          gp.hat = GAMEPAD_HAT_DOWN_RIGHT;
        } else if (UP) {
          gp.hat = GAMEPAD_HAT_UP;
        } else if (DOWN) {
          gp.hat = GAMEPAD_HAT_DOWN;
        } else if (LEFT) {
          gp.hat = GAMEPAD_HAT_LEFT;
        } else if (RIGHT) {
          gp.hat = GAMEPAD_HAT_RIGHT;
        } else {
          gp.hat = GAMEPAD_HAT_CENTERED;
        }
      }
    }
    static uint32_t ms = 0;
    if (millis() - ms > 4) {
      ms = millis();
      process_hid();
    }
  }
}

inline uint32_t setBit(uint32_t value, uint8_t bit) {
  return value | (1UL << bit);
}

inline uint32_t clearBit(uint32_t value, uint8_t bit) {
  return value & ~(1UL << bit);
}