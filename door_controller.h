#include "pinout.h"
#include "utils.h"
#include <stdint.h>
#include <Arduino.h>

#define DOOR_CTL_MAX_RETRIES 3
#define DOOR_CTL_TOGGLE_PERIOD 500
#define DOOR_CTL_TIMEOUT 30000
#define DOOR_CTL_PUBLISH_DOOR_PERIOD 2500
#define DOOR_CTL_DELAY_TIMER 1000

#define IS_DOOR_CLOSED() (!digitalRead(DOOR_CLOSED_SWITCH)) 
#define IS_DOOR_OPENED() (!digitalRead(DOOR_OPEN_SWITCH)) 
#define TOGGLE_ON() digitalWrite(TOGGLE_GARAGE_DOOR, HIGH)
#define TOGGLE_OFF() digitalWrite(TOGGLE_GARAGE_DOOR, LOW);

typedef enum {
  DOOR_IDLE = 0,
  DOOR_STOP = 1,
  DOOR_CLOSE = 2,
  DOOR_OPEN = 3
} DoorCtl; 

typedef enum {
  DS_STOPPED = 0,
  DS_CLOSED = 1,
  DS_OPENED = 2,
  DS_CLOSING = 3,
  DS_OPENING = 4,
  DS_TO_OPENING = 5,
  DS_TO_CLOSING = 6,
  DS_STOPPED_CLOSING = 7,
  DS_STOPPED_OPENING = 8,
  DS_UNDEFINED = 9
} DoorState;

typedef enum {
  TOGGLE_CTL_IDLE = 0,
  TOGGLE_CTL_ON,
  TOGGLE_CTL_WAIT,
  TOGGLE_CTL_OFF
} ToggleCtlrState;

class ToggleController{
  public:
    ToggleController(void);
    void toggle(void);
    bool isMoving(void);
    bool isIdle(void);
    void loop(void);
    DoorState get_door_state(void);

   private:
    bool b_toggle_pending;
    ToggleCtlrState e_toggle_state; 
    DoorState e_door_state;
    unsigned long ul_toggle_delay_val;
    unsigned long ul_door_moving_timeout;
};

ToggleController::ToggleController(void) {
  this->b_toggle_pending = false;
  this->e_toggle_state = TOGGLE_CTL_IDLE;
  if (IS_DOOR_CLOSED()) 
    this->e_door_state = DS_CLOSED;
  else if (IS_DOOR_OPENED())
    this->e_door_state = DS_OPENED;
  else
    this->e_door_state = DS_STOPPED;
}

void ToggleController::toggle(void) {
  this->b_toggle_pending = true;
}

bool ToggleController::isIdle(void) {
  return (this->e_toggle_state == TOGGLE_CTL_IDLE);
}

bool ToggleController::isMoving(void) {
  return (this->e_door_state == DS_OPENING || this->e_door_state == DS_CLOSING);
}

void ToggleController::loop(void) {
  SERIAL_PRINT("Door State: ");
  SERIAL_PRINTLN(this->e_door_state);
  switch(this->e_toggle_state) {
    case TOGGLE_CTL_IDLE:
//      SERIAL_PRINTLN("TOGGLE_CTL_IDLE");
      if (this->b_toggle_pending) {
//        SERIAL_PRINTLN("Tgl Pend");
        this->b_toggle_pending = false;
         if (this->e_door_state == DS_CLOSED || this->e_door_state == DS_STOPPED_CLOSING || this->e_door_state == DS_TO_CLOSING)
            this->e_door_state = DS_OPENING;
         else if (this->e_door_state == DS_OPENED || this->e_door_state == DS_STOPPED_OPENING || this->e_door_state == DS_TO_OPENING)
            this->e_door_state = DS_CLOSING;
         else if (this->e_door_state == DS_OPENING)
           this->e_door_state = DS_STOPPED_OPENING;
         else if (this->e_door_state == DS_CLOSING)
           this->e_door_state = DS_STOPPED_CLOSING;
         else if (this->e_door_state == DS_STOPPED)
           this->e_door_state = DS_OPENING;
        this->e_toggle_state = TOGGLE_CTL_ON;
      } else if (this->e_door_state == DS_OPENING || this->e_door_state == DS_CLOSING) {
        if (IS_DOOR_CLOSED() && this->e_door_state == DS_CLOSING) {
          SERIAL_PRINTLN("Door Closed");
          this->e_door_state = DS_CLOSED;
        }
        else if (IS_DOOR_OPENED() && this->e_door_state == DS_OPENING) {
          SERIAL_PRINTLN("Door Opened");
          this->e_door_state = DS_OPENED;
        }
        else if (millis() >= this->ul_door_moving_timeout) {
          SERIAL_PRINTLN("TIMEOUT");
          if (this->e_door_state == DS_OPENING) {
//            SERIAL_PRINTLN("Door Opening Timeout!");
            this->e_door_state = DS_TO_OPENING;
          }
          else if (this->e_door_state == DS_CLOSING) {
//            SERIAL_PRINTLN("Door Closing Timeout!");
            this->e_door_state = DS_TO_CLOSING;
          }
        }
      } else if (IS_DOOR_CLOSED() && (this->e_door_state == DS_CLOSING || this->e_door_state == DS_TO_CLOSING || this->e_door_state == DS_UNDEFINED || this->e_door_state == DS_OPENED || this->e_door_state == DS_STOPPED_CLOSING || this->e_door_state == DS_STOPPED_OPENING)) {
//        SERIAL_PRINTLN("Door Closed");
        this->e_door_state = DS_CLOSED;
      } else if (IS_DOOR_OPENED() && (this->e_door_state == DS_OPENING || this->e_door_state == DS_TO_OPENING || this->e_door_state == DS_UNDEFINED || this->e_door_state == DS_CLOSED || this->e_door_state == DS_STOPPED_CLOSING || this->e_door_state == DS_STOPPED_OPENING)) {
//        SERIAL_PRINTLN("Door Opened");
        this->e_door_state = DS_OPENED;
      } else if (!(this->e_door_state == DS_OPENING || this->e_door_state == DS_CLOSING || IS_DOOR_CLOSED() || IS_DOOR_OPENED() || this->e_door_state == DS_STOPPED_CLOSING || this->e_door_state == DS_STOPPED_OPENING)) {
        this->e_door_state = DS_UNDEFINED;
      }
      
      break;
    case TOGGLE_CTL_ON:
//      SERIAL_PRINTLN("TOGGLE_CTL_ON");
      TOGGLE_ON();
      
     
      this->e_toggle_state = TOGGLE_CTL_WAIT;
      this->ul_toggle_delay_val = millis() + DOOR_CTL_TOGGLE_PERIOD;
      this->ul_door_moving_timeout = millis() + DOOR_CTL_TIMEOUT;
      break;
    case TOGGLE_CTL_WAIT:
      if (millis() >= this->ul_toggle_delay_val) {
        this->e_toggle_state = TOGGLE_CTL_OFF;
      }
      break;
    case TOGGLE_CTL_OFF:
//      SERIAL_PRINTLN("TOGGLE_CTL_OFF");
      TOGGLE_OFF();
      this->e_toggle_state = TOGGLE_CTL_IDLE;
      break;
     default:
      this->e_toggle_state = TOGGLE_CTL_IDLE;
  }
}

DoorState ToggleController::get_door_state(void) {
  return this->e_door_state;
}
class DoorController{
  public: 
    DoorController(void);
    void setDoor(DoorCtl e_set_state);
    void loop(void);
    bool needs_update(void);
    bool isIdle(void);
    void get_door_state_str(char* psz_door_state);
  private:
    DoorCtl e_set_state;
    ToggleController toggleController;
    unsigned long ul_repeat_delay_trigger;
    unsigned long ul_delay_timer;
    uint8_t u8_retry_count;
    DoorState e_last_door_state; 
};

bool DoorController::isIdle(void) {
  return this->e_set_state == DOOR_IDLE;
}

DoorController::DoorController(void) {
  this->e_set_state = DOOR_IDLE;
  this->u8_retry_count = 0;
  this->ul_delay_timer = 0;
  this->e_last_door_state = DS_UNDEFINED;
}

void DoorController::setDoor(DoorCtl e_set_state) {
  this->e_set_state = e_set_state;
}

bool DoorController::needs_update(void){
  static DoorState e_last_door_state;
  DoorState e_door_state = this->toggleController.get_door_state();
  if (e_last_door_state != e_door_state) {
    e_last_door_state = e_door_state;
    return true;
  } else
    return false;  
}

void DoorController::get_door_state_str(char* psz_door_state) {
  switch(this->toggleController.get_door_state()) {
    case DS_OPENED:
      strcpy(psz_door_state, "OPENED");
      break;
    case DS_CLOSED:
      strcpy(psz_door_state, "CLOSED");
      break;
    case DS_CLOSING:
      strcpy(psz_door_state, "CLOSING");
      break;
    case DS_OPENING:
      strcpy(psz_door_state, "OPENING");
      break;
    case DS_STOPPED:
    case DS_TO_OPENING:
    case DS_TO_CLOSING:
    case DS_STOPPED_CLOSING:
    case DS_STOPPED_OPENING:
      strcpy(psz_door_state, "STOPPED");
      break;
    default:
      strcpy(psz_door_state, "UNDEFINED");
      break;
  }
}

void DoorController::loop(void) {
  SERIAL_PRINT("Set State: ");
  SERIAL_PRINTLN(this->e_set_state);
  SERIAL_PRINT("Delay tmr: ");
  SERIAL_PRINTLN(ul_delay_timer);
  if (millis() >= this->ul_delay_timer && this->e_set_state == DOOR_STOP && (this->toggleController.get_door_state() == DS_OPENING or this->toggleController.get_door_state() == DS_CLOSING)) {
//    SERIAL_PRINTLN("TOGGLE 0");
    this->toggleController.toggle();
    this->ul_repeat_delay_trigger = 0;
    this->e_set_state = DOOR_IDLE;
    this->u8_retry_count = 0;
    this->ul_delay_timer = 0;
  }
  else if (millis() >= this->ul_delay_timer && millis() >= this->ul_repeat_delay_trigger && this->e_set_state == DOOR_OPEN && !(this->toggleController.get_door_state() == DS_OPENED || this->toggleController.get_door_state() == DS_OPENING) && this->u8_retry_count < DOOR_CTL_MAX_RETRIES) {
//    SERIAL_PRINTLN("TOGGLE 1");
    this->u8_retry_count++;
    this->ul_repeat_delay_trigger = millis() + DOOR_CTL_TIMEOUT;
    this->toggleController.toggle();
    this->ul_delay_timer = millis() + DOOR_CTL_DELAY_TIMER;
  }

  else if (millis() >= this->ul_delay_timer && millis() >= this->ul_repeat_delay_trigger && this->e_set_state == DOOR_CLOSE && !(this->toggleController.get_door_state() == DS_CLOSED || this->toggleController.get_door_state() == DS_CLOSING) && this->u8_retry_count < DOOR_CTL_MAX_RETRIES) {
//    SERIAL_PRINTLN("TOGGLE 2");
    this->u8_retry_count++;
    this->ul_repeat_delay_trigger = millis() + DOOR_CTL_TIMEOUT;
    this->toggleController.toggle();
    this->ul_delay_timer = millis() + DOOR_CTL_DELAY_TIMER;
  }
  // This Open state achieved, reset the repeat delay timer
  else if (this->e_set_state == DOOR_OPEN && this->toggleController.get_door_state() == DS_OPENED) {
    this->ul_repeat_delay_trigger = 0;
    this->e_set_state = DOOR_IDLE;
  }
  // This Close state achieved, reset the repeat delay timer
  else if (this->e_set_state == DOOR_CLOSE && this->toggleController.get_door_state() == DS_CLOSED) {
    this->ul_repeat_delay_trigger = 0;
    this->e_set_state = DOOR_IDLE;
  }

  // This handles cases where it should be in one direction, but the state gets set in the other direction
  if (this->e_last_door_state != this->toggleController.get_door_state() && (this->toggleController.get_door_state() == DS_CLOSED || this->toggleController.get_door_state() == DS_OPENED)) {
    this->ul_repeat_delay_trigger = 0;
    this->u8_retry_count = 0;
    this->ul_delay_timer = millis() + DOOR_CTL_DELAY_TIMER;
  }
  this->e_last_door_state = this->toggleController.get_door_state();

  
  this->toggleController.loop();
}

