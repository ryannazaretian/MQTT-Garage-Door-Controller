/*
 Basic MQTT example

 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
	- publishes "hello world" to the topic "outTopic"
	- subscribes to the topic "inTopic", printing out any messages
		it receives. NB - it assumes the received payloads are strings not binary

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'MQTT_STATE_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

*/

//#define DEBUG

#include <EthernetV2_0.h>
#include <NewPing.h>
#include <PubSubClient.h>

#include "pinout.h"
#include "SimpleTaskScheduler.h"
#include "status_led.h"
#include "door_controller.h"
#include "utils.h"
#include "secrets.h"


#define TEMP_SAMPLES 16

// Ethernet Seutp
#define ETH_RESET_ASSERT 500
#define ETH_RESET_WAIT 2500
#define ETH_CONNECT_WAIT 1000
EthernetClient ethClient;

// MQTT Setup
#define MQTT_STATE_RESET_WAIT 5000
PubSubClient client(ethClient);

// Distance Sensor Setup
#define DEFAULT_DISTANCE_TRIG 0
uint16_t gu16_car0_distance_trig = DEFAULT_DISTANCE_TRIG;
uint16_t gu16_car1_distance_trig = DEFAULT_DISTANCE_TRIG;
#define SAMPLES 32
NewPing PingCar0(CAR0_TRIG, CAR0_ECHO);
NewPing PingCar1(CAR1_TRIG, CAR1_ECHO);

// Status LED Setup
StatusLed status(STATUS_LED_RED, STATUS_LED_GRN);

// Door Controller
DoorController door_controller;

// Function Declarations


void setup_runner(void);
void disable_publishing(void);
void enable_publishing(void);

void mqtt_callback(char* psz_topic, char* ps_payload, unsigned int len);
void printIPAddress(void);
bool checkEthernet(void);
void loop_ethernet(void);
void loop_mqtt(void);
void loop_toggle_door(void);
void loop_publish_door_state(void);

void tcb_publish_temp(void);
void tcb_publish_car0(void);
void tcb_publish_car1(void);
void tcb_toggle_door(void);
void tcb_update_leds(void);



// Task Scheduler
#define TASK_RATE_TEMPERATURE 60000
#define TASK_RATE_CAR0_STATE 5000
#define TASK_RATE_CAR1_STATE 5000
#define TASK_RATE_MAINTAIN_ETH 60000
#define TASK_RATE_TOGGLE_DOOR 500
#define TASK_RATE_BLINK_PERIOD 500
#define TASK_RATE_RESET_ETH 5000
#define TASK_RATE_RECONNECT_MQTT 5000
#define TASK_RATE_PUBLISH_DOOR 10000

SimpleTaskScheduler taskScheduler(4, TS_MILLIS);
uint8_t t_status_led = taskScheduler.addTask(tcb_update_leds, TASK_RATE_BLINK_PERIOD, true);
uint8_t t_publish_temp = taskScheduler.addTask(tcb_publish_temp, tcb_publish_temp, true);
uint8_t t_publish_car0 = taskScheduler.addTask(tcb_publish_car0, TASK_RATE_CAR0_STATE, true);
uint8_t t_publish_car1 = taskScheduler.addTask(tcb_publish_car1, TASK_RATE_CAR1_STATE, true);

void tcb_update_leds(void) {
	status.update_leds();
}

void mqtt_callback(char* psz_topic, uint8_t* pu8_payload, unsigned int len) {
	char psz_payload[32];

	// Fill the payload with zeros. This will handle the zero-termination
	memset(psz_payload, 0, 32*sizeof(char));
	memcpy(psz_payload, pu8_payload, len); // Copy the payload to local memory

	// Print off some debug messages
	SERIAL_PRINT("MQTT PAYLD [");
	SERIAL_PRINT(psz_topic);
	SERIAL_PRINT("] ");
	SERIAL_PRINTLN(psz_payload);

	if (strcmp(psz_topic, "garage_ctl/toggle_door") == 0) {
	 if (strcmp(psz_payload, "OPEN") == 0) {
		door_controller.setDoor(DOOR_OPEN);
	 } else if (strcmp(psz_payload, "CLOSE") == 0) {
		door_controller.setDoor(DOOR_CLOSE);
	 } else if (strcmp(psz_payload, "STOP") == 0) {
    door_controller.setDoor(DOOR_STOP);
	 } else {
		SERIAL_PRINTLN("ERR: UKNWN PAYLD");
	 }
	} else if (strcmp(psz_topic, "garage_ctl/car0/dist_trig") == 0){
    gu16_car0_distance_trig = atoi(psz_payload);
    SERIAL_PRINT("Car 0 Distance Trig set to (in): ");
    SERIAL_PRINTLN(gu16_car0_distance_trig);
  } else if (strcmp(psz_topic, "garage_ctl/car1/dist_trig") == 0){
    gu16_car1_distance_trig = atoi(psz_payload);
    SERIAL_PRINT("Car 0 Distance Trig set to (in): ");
    SERIAL_PRINTLN(gu16_car1_distance_trig);
	} else {
    SERIAL_PRINTLN("ERR: UKNWN TOPIC");
	}
}

void loop_mqtt(MqttState *pe_MQTT_STATE_state) {
	static unsigned long ul_delay_trigger;
	switch(*pe_MQTT_STATE_state) {
		case MQTT_STATE_NOT_SETUP:
			*pe_MQTT_STATE_state = MQTT_STATE_CONNECTING;
			break;
		case MQTT_STATE_CONNECTING:
      SERIAL_PRINTLN("MQTT: Connecting...");
			if (client.connect("garageClient", USERNAME, PASSWORD)) {
				SERIAL_PRINTLN("MQTT:  Connected");
				client.subscribe("garage_ctl/toggle_door");
        client.subscribe("garage_ctl/car0/dist_trig");
        client.subscribe("garage_ctl/car1/dist_trig");
        // Once connected & subscribed, publish an announcement...
        client.publish("garage_state", "started");
        
				*pe_MQTT_STATE_state = MQTT_STATE_CONNECTED;
			} else {
				SERIAL_PRINT("MQTT: Error, RC=");
				SERIAL_PRINT(client.state());
				SERIAL_PRINTLN(", Retrying...");
				ul_delay_trigger = millis() + MQTT_STATE_RESET_WAIT;
				*pe_MQTT_STATE_state = MQTT_STATE_DELAY;
			}
			break;
		case MQTT_STATE_DELAY:
			if (millis() > ul_delay_trigger)
				*pe_MQTT_STATE_state = MQTT_STATE_CONNECTING;
      break;
		case MQTT_STATE_CONNECTED:
			if (!client.connected()) {
				SERIAL_PRINTLN("Lost MQTT Connection");
				*pe_MQTT_STATE_state = MQTT_STATE_CONNECTING;
			}
      break;
		default:
			*pe_MQTT_STATE_state = MQTT_STATE_NOT_SETUP;
	}
}



void loop_ethernet(EthernetState *pe_ethernet_state) {
	static unsigned long ul_delay_trigger;
	switch(*pe_ethernet_state) {
		case ETH_NOT_SETUP:
			*pe_ethernet_state = ETH_TOGGLE_RESET;
			break;
			
		case ETH_TOGGLE_RESET:
			SERIAL_PRINTLN("ETH: Toggling Ethernet Reset pin... ");
			digitalWrite(ETH_RST, HIGH);
			ul_delay_trigger = millis() + ETH_RESET_ASSERT;
			*pe_ethernet_state = ETH_DELAY_TOGGLE_RESET;
			break;
		case ETH_DELAY_TOGGLE_RESET:
			if (millis() > ul_delay_trigger) {
				SERIAL_PRINTLN("ETH: Waiting for Ethernet Control to come out of reset... ");
				digitalWrite(ETH_RST, LOW);
				ul_delay_trigger = millis() + ETH_RESET_WAIT;
				*pe_ethernet_state = ETH_DELAY_IN_RESET;
			}
			break;
		case ETH_DELAY_IN_RESET:
			if (millis() > ul_delay_trigger) {
				SERIAL_PRINTLN("ETH: Out of reset.");
        *pe_ethernet_state = ETH_CONNECTING;
			}
     break;
    case ETH_CONNECTING:
        SERIAL_PRINTLN("ETH: Connecting...");
				Ethernet.begin(MAC_ADDRESS, CLIENT_IP) ;
        ul_delay_trigger = millis() + ETH_CONNECT_WAIT;
        *pe_ethernet_state = ETH_CONNECTING_DELAY;
        break;
     case ETH_CONNECTING_DELAY:
        if (millis() > ul_delay_trigger) {
				SERIAL_PRINT("ETH: ");
				if (checkEthernet()) {
					SERIAL_PRINTLN("Connected.");
					*pe_ethernet_state = ETH_CONNECTED_MAINTAIN;
				} else {
					SERIAL_PRINTLN("Failed to connect. Resetting...");
					*pe_ethernet_state = ETH_TOGGLE_RESET;
				}
			}
			break;
		case ETH_CONNECTED_MAINTAIN:
			if (!checkEthernet()) {
				SERIAL_PRINTLN("ETH: Lost connection.");
				*pe_ethernet_state = ETH_TOGGLE_RESET;
			} else {
				SERIAL_PRINT("ETH: Performing maintenance... ");
				switch (Ethernet.maintain())
				{
					case 1:
						//renewed fail
						SERIAL_PRINTLN("Renew error");
						*pe_ethernet_state = ETH_TOGGLE_RESET;
						break;
					case 2:
						//renewed success
						SERIAL_PRINTLN("Renew ok");
						//print your local IP address:
						ul_delay_trigger = millis() + TASK_RATE_MAINTAIN_ETH;
						*pe_ethernet_state = ETH_CONNECTED_DELAY_MAINTAIN;
						break;
					case 3:
						//rebind fail
						SERIAL_PRINTLN("Rebind error");
						*pe_ethernet_state = ETH_TOGGLE_RESET;
						break;
					case 4:
						//rebind success
						SERIAL_PRINTLN("Rebind ok");
						ul_delay_trigger = millis() + TASK_RATE_MAINTAIN_ETH;
						*pe_ethernet_state = ETH_CONNECTED_DELAY_MAINTAIN;

						break;
					default:
            SERIAL_PRINTLN("Done");
						ul_delay_trigger = millis() + TASK_RATE_MAINTAIN_ETH;
						*pe_ethernet_state = ETH_CONNECTED_DELAY_MAINTAIN;
						break;
				}
			}
			break;
		case ETH_CONNECTED_DELAY_MAINTAIN:
			if (!checkEthernet()) {
					SERIAL_PRINTLN("ETH: Lost connection.");
					*pe_ethernet_state = ETH_TOGGLE_RESET;
				}
			if (millis() > ul_delay_trigger)
				*pe_ethernet_state = ETH_CONNECTED_MAINTAIN;
			break;
		default:
			*pe_ethernet_state = ETH_NOT_SETUP;
	}
}

bool checkEthernet(void) {
//  return true;
  bool is_valid = false;
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    if (Ethernet.localIP()[thisByte] != 0) {
      is_valid = true;
    }
  }
  return is_valid;
}

void loop_publish_door_state(void) {
  static unsigned long ul_next_update = 0;
  char psz_door_state[10];
  if (door_controller.needs_update() || millis() > ul_next_update) {
    SERIAL_PRINTLN("->DOOR STATE");
    ul_next_update = millis() + TASK_RATE_PUBLISH_DOOR;
    door_controller.get_door_state_str(psz_door_state);
    client.publish("garage_state/door_state", psz_door_state);
  }
}

void tcb_publish_temp(void) {
	uint16_t au16_lm60_adc_counts[TEMP_SAMPLES];
	uint16_t u16_lm60_adc_counts;
	uint16_t u16_lm60_adc_mv;
	uint16_t u16_last;
	float f_temp_c;
	float f_temp_f;
	uint8_t u8_j;
	char sz_temp_f[4];
	SERIAL_PRINTLN("->TEMP");
	for(uint8_t u8_i = 0; u8_i<TEMP_SAMPLES; u8_i++) {
		u16_last = analogRead(LM60_TEMP_SENSE);
		if (u8_i > 0) {
			for (u8_j = u8_i; u8_j > 0 && au16_lm60_adc_counts[u8_j - 1] < u16_last; u8_j--) // Insertion sort loop.
				au16_lm60_adc_counts[u8_j] = au16_lm60_adc_counts[u8_j - 1]; // Shift ping array to correct position for sort insertion.
		} else u8_j = 0;
		au16_lm60_adc_counts[u8_j] = u16_last;
		delay(10);
	}
	u16_lm60_adc_counts = (au16_lm60_adc_counts[TEMP_SAMPLES >> 1]); // Return the temperature median.
	u16_lm60_adc_mv = ((uint32_t) u16_lm60_adc_counts * 5000) / 1023; // 10-bit reading, range 0-1023
	if (u16_lm60_adc_mv >= 424) {
		f_temp_c = (u16_lm60_adc_mv - 424) / 6.25;
		f_temp_f = (f_temp_c * 9.0) / 5.0 + 32.0;
		itoa((int)f_temp_f, sz_temp_f, 10);
		client.publish("garage_state/temperature", sz_temp_f);
	}
	else {
		client.publish("garage_state/temperature", "-1");
	}
}

void tcb_publish_car0(void) {
	uint16_t u16_inches;
	char sz_dist[8];
	SERIAL_PRINTLN("->CAR0");
	u16_inches = PingCar0.convert_in(PingCar0.ping_median(SAMPLES));
	itoa(u16_inches, sz_dist, DECIMAL);
	client.publish("garage_state/car0/dist", sz_dist);
	client.publish("garage_state/car0/state", BINARY_SENSOR(u16_inches < gu16_car0_distance_trig));
  itoa(gu16_car0_distance_trig, sz_dist, DECIMAL);
	client.publish("garage_state/car0/trigger", sz_dist);
}

void tcb_publish_car1(void) {
	uint16_t u16_inches;
	char sz_dist[8];
	SERIAL_PRINTLN("->CAR1");
	u16_inches = PingCar1.convert_in(PingCar1.ping_median(SAMPLES));
	itoa(u16_inches, sz_dist, DECIMAL);
	client.publish("garage_state/car1/dist", sz_dist);
	client.publish("garage_state/car1/state", BINARY_SENSOR(u16_inches < gu16_car1_distance_trig));
  itoa(gu16_car1_distance_trig, sz_dist, DECIMAL);
  client.publish("garage_state/car1/trigger", sz_dist);
}



void setup() {
	// Setup input pins
	analogReference(DEFAULT);
	pinMode(DOOR_OPEN_SWITCH,		INPUT);
	pinMode(DOOR_CLOSED_SWITCH, INPUT);
	pinMode(LM60_TEMP_SENSE,		INPUT);

	// Setup output pins
	pinMode(ETH_RST,						OUTPUT);
	pinMode(TOGGLE_GARAGE_DOOR, OUTPUT);
  pinMode(SDCARD_CS,OUTPUT);
  digitalWrite(SDCARD_CS,HIGH);//Deselect the SD card
	digitalWrite(ETH_RST, LOW);
	digitalWrite(TOGGLE_GARAGE_DOOR, LOW);

	START_SERIAL(9600);
  SERIAL_PRINTLN();
	SERIAL_PRINTLN("MASTER RESET");
	
	taskScheduler.enableTask(t_status_led, true);

	// Setup MQTT
	status.setStatus(STATUS_BLINK_RED);
	client.setServer(SERVER_IP, SERVER_PORT);
	client.setCallback(mqtt_callback);

}

void enable_publishing(void) {
  taskScheduler.enableTask(t_publish_temp, true);
  taskScheduler.enableTask(t_publish_car0, true);
  taskScheduler.enableTask(t_publish_car1, true);
}

void disable_publishing(void) {
  taskScheduler.disableTask(t_publish_temp);
  taskScheduler.disableTask(t_publish_car0);
  taskScheduler.disableTask(t_publish_car1);
}

void loop() {
	static EthernetState ethernet_state;
	static MqttState mqtt_state;
	door_controller.loop();
	loop_mqtt(&mqtt_state);
	loop_ethernet(&ethernet_state);
  door_controller.loop();
  taskScheduler.loop();
	
	if (!(ethernet_state & ETH_CONNECTED_MASK)) {
		// No Ethernet
		status.setStatus(STATUS_BLINK_RED);
		disable_publishing();
		mqtt_state = MQTT_STATE_NOT_SETUP;
	} else {
		if (!(mqtt_state & MQTT_STATE_CONNECTED_MASK)) {
			// No MQTT Connection
			status.setStatus(STATUS_YEL);
			disable_publishing();
		} else {
			// Normal Mode
			enable_publishing();
      loop_publish_door_state();
      if (door_controller.isIdle())
        status.setStatus(STATUS_BLINK_GRN);
      else
        status.setStatus(STATUS_BLINK_YEL);
			client.loop();
      
		}
	}
}

