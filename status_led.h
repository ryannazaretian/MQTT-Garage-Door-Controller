#include <stdint.h>
#include <Arduino.h>

#define STATUS_OFF 0x0
#define STATUS_GRN 0x1
#define STATUS_RED 0x2
#define STATUS_YEL (STATUS_GRN | STATUS_RED)
#define STATUS_BLINK 0x4
#define STATUS_BLINK_GRN (STATUS_GRN | STATUS_BLINK)
#define STATUS_BLINK_YEL (STATUS_YEL | STATUS_BLINK)
#define STATUS_BLINK_RED (STATUS_RED | STATUS_BLINK)

class StatusLed {
  public:
    StatusLed(uint8_t u8_red_pin, uint8_t u8_green_pin); 
    void setStatus(uint8_t u8_status);
    void update_leds(void);

  private:
    bool b_blink;
    uint8_t u8_red_pin;
    uint8_t u8_green_pin;
    uint8_t u8_status = 0;
   
};

StatusLed::StatusLed(uint8_t u8_red_pin, uint8_t u8_green_pin) {
  this->u8_red_pin = u8_red_pin;
  this->u8_green_pin = u8_green_pin; 
  pinMode(this->u8_red_pin, OUTPUT);
  pinMode(this->u8_green_pin, OUTPUT);  
}

void StatusLed::setStatus(uint8_t u8_status) {
 if (this->u8_status != u8_status) {
   this->u8_status = u8_status;
   this->b_blink = true; // Turn b_blink on to cause the LEDs to display some status (in the event that the CPU locks up)
   this->update_leds();// Update the LED
 }
}

void StatusLed::update_leds(void) {
//  Serial.println("update_leds called");
  if (!(this->u8_status & STATUS_BLINK) || this->b_blink) {
      digitalWrite(u8_red_pin,   STATUS_RED & this->u8_status);
      digitalWrite(u8_green_pin, STATUS_GRN & this->u8_status);
    } else {
      digitalWrite(u8_red_pin,   STATUS_OFF);
      digitalWrite(u8_green_pin, STATUS_OFF);
    }
  this->b_blink = !this->b_blink;
}
