 /*
 * mqtt_ac_control.c
 * Date: 12-May-2017
 * Revision: 1.0
 *  _____               __  _
   |__  _|             / / | |
	 | | (_)__ __ ___ _| |_| |____ __   __
	 | | | |  '  | __(_   _) | ___)\ \ / /
	 | | | | | | | ___ | | | | ___  \   /
	 |_| |_|_|_|_|____)|_| |_|____) /_/\_\
 *  Copyright (C) 2016 - 2017 Timeflex Data pvt ltd
 *
 *
 *   Maintainer: Sijo k saju
 *	 Usage:
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp8266.h"
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>
#include "fcntl.h"
#include "unistd.h"
#include "spiffs.h"
#include "esp_spiffs.h"
#include "jsmn.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "lwip/api.h"
// DS18B20 driver
#include "ds18b20/ds18b20.h"
//DS2413 driver
#include "ds2413/ds2413.h"
/*
#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTF( ...) printf( "opt-contrl: "__VA_ARGS__)
#else
  #define DEBUG_PRINTF(...) (void)0
#endif
*/

#define DEBUG 0
#ifdef DEBUG
    #define DEBUG_PRINTF( ...) printf( "opt-contrl: "__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...) (void)0
#endif

#define TM_PRINTF( ...) printf( "opt-contrl: "__VA_ARGS__)

/*
#define DISPLAYLOG(args...)    printf(stderr,"opt-contrl: " args)

#define TM_PRINTF( ...) printf( "opt-contrl: "__VA_ARGS__)

#define DEBUG_PRINTF( ...) printf( "opt-contrl: "__VA_ARGS__)

*/

#define MQTT_HOST ("52.76.228.190")
//#define MQTT_HOST ("52.220.19.2")
#define MQTT_PORT 1883

#define MQTT_USER "600194803E7D"
#define MQTT_PASS NULL


#define HEARTBEATLED 16
#define LINKLED 10
#define RELAYPW1 13
#define RELAYPW2 12

#define RELAYLP1 15
#define RELAYLP2 14

#define INPUT1 2
#define INPUT2 0

#define SENSOR_GPIO 4
#define MAX_SENSORS 4
#define RESCAN_INTERVAL 8
#define LOOP_DELAY_MS 10000

QueueHandle_t publish_queue;

uint8_t statlink=0;
uint8_t wifilink=0;

SemaphoreHandle_t wifi_alive;
//link stat mutex
SemaphoreHandle_t linkstatus= NULL;
//temp up mutex
SemaphoreHandle_t tmp= NULL;

//motion count mutex
SemaphoreHandle_t motion= NULL;

#define PUB_MSG_LEN 87


const int heartbeatled          = HEARTBEATLED;
const int linkled 		= LINKLED;
const int relaylp1		= RELAYLP1;
const int relaylp2		= RELAYLP2;
const int relaypw1		= RELAYPW1;
const int relaypw2		= RELAYPW2;

const uint8_t input1        = INPUT1;
const uint8_t input2        = INPUT2;

TaskHandle_t xHandle_link_wifi = NULL;
TaskHandle_t xHandle_link_net = NULL;

/* pin config */
const uint8_t gpio_intrrupt_pir = 5;   /* PIR */
const uint8_t gpio_intrrupt_in1 = 2;   /* input1 */
const uint8_t gpio_intrrupt_in2 = 0;   /* input2 */

const uint8_t active = 0; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
const gpio_inttype_t int_typein1=GPIO_INTTYPE_EDGE_POS;//GPIO_INTTYPE_LEVEL_LOW;
const gpio_inttype_t int_typein2=GPIO_INTTYPE_EDGE_POS;//GPIO_INTTYPE_LEVEL_HIGH;


ds2413_addr_t ds2413addrs[MAX_SENSORS];


char *JSON_STRING;
char *USER_WIFI_SSID_;
char *USER_WIFI_PASS_;
char *KEYS[] = { "ssid", "pass" };

struct dev_stat {
	    bool rpw1;
		bool rpw2;
		bool rlp1;
		bool rlp2;
		bool in1;
		bool in2;
};

typedef struct dev_stat opt_stat;

struct optimiser_config {

	float temp_setpoint;
	char * devid;
	float hysteresis;
	opt_stat default_stat;
};
typedef struct optimiser_config opt_config;

opt_config dev_config;

opt_stat cur_stat={0};


/* store temp at 1 sec*/
float temps;

/* motion count*/
static int motion_count;

char * gpioval;
char * method;
char * pin;
char * value;
struct sdk_station_config default_config = {
    .ssid = WIFI_SSID,
    .password = WIFI_PASS,
};


struct sdk_station_config user_config;

void get_temperature(void *pvParameters);

static int jsoneq(const char *json, jsmntok_t *tok, const char *s);

static const char *  get_my_id(void);

static void  mqtt_task(void *pvParameters);

static void dataparse_json();

static void read_file_posix();
static char write_file_posix(char buffjson);

static void  wifi_task(void *pvParameters);

void heartbeat(void *pvParameters);

void link_wifi(void *pvParameters);

void  stats_send(void *pvParameters);

void intrruptTask(void *pvParameters);

void relay_ds2413_devices(void *pvParameters);

void gpio_intr_handler(uint8_t gpio_num);

void gpio_intr_handler2(uint8_t gpio_num);

void gpio_intr_handler3(uint8_t gpio_num);

static void  topic_received(mqtt_message_data_t *md);

void stats_send(void *pvParameters){

	static int m_count;
	bool  val;
	 const TickType_t xDelay = 1000*20 / portTICK_PERIOD_MS;
	//  TickType_t stat_send = xTaskGetTickCount();
	  char msg[87];
	//  char jsenddata[PUB_MSG_LEN];
	  for(;;){
		//  vTaskDelayUntil(&stat_send, 30000 / portTICK_PERIOD_MS);
		 vTaskDelay( xDelay );

		 if ( xSemaphoreTake( motion, ( TickType_t ) 15  ) == pdTRUE) {

					         m_count=motion_count;
					         motion_count=0;
				        	 xSemaphoreGive(motion);
			 }

		  if ( xSemaphoreTake( tmp, ( TickType_t ) 15  ) == pdTRUE) {
				if (temps<= ((dev_config.temp_setpoint)- (dev_config.hysteresis / 2))) {
					DEBUG_PRINTF("trigger off / cur_stat.rpw1 %d\r\n", cur_stat.rpw1);
					if (cur_stat.rpw1) {
						gpio_write(relaypw1, 0);
						cur_stat.rpw1 = false;
					}
				} else if (temps>= ((dev_config.temp_setpoint)	+ (dev_config.hysteresis / 2))) {
					DEBUG_PRINTF("trigger on / cur_stat.rpw1 %d\r\n", cur_stat.rpw1);
					if (!cur_stat.rpw1)
						gpio_write(relaypw1, 1);
					cur_stat.rpw1 = true;
				}
			//	val=cur_stat.rpw1;
		  }
		  DEBUG_PRINTF("m_count %d \r\n",m_count);
		  memset(msg, 0, sizeof(msg));
       //   snprintf(msg,PUB_MSG_LEN, "{\"id\":\"%s\",\"T\":%0.2f,\"P1\":%d,\"M\":%d}\r\n",dev_config.devid,temps,cur_stat.rpw1,m_count);
          snprintf(msg,PUB_MSG_LEN, "{\"Temp\":\"%0.2f\",\"P1\":\"%d\",\"M\":\"%d\"}",temps,cur_stat.rpw1,m_count);

          DEBUG_PRINTF("strlen msg buff %d",strlen(msg));
		    if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
		                          //   printf("Publish queue overflow.\r\n");
		                    }
		    xSemaphoreGive(tmp);
	//	  free(msg);
	//	  free(jsenddata);
		  taskYIELD();
	  }


}

void get_temperature(void *pvParameters) {

	ds18b20_addr_t addrs[MAX_SENSORS];
	float tempsr[MAX_SENSORS];
	uint8_t sensor_count;
	char msg[PUB_MSG_LEN];
	gpio_set_pullup(SENSOR_GPIO, true, true);

	for(;;) {
		// Every RESCAN_INTERVAL samples, check to see if the sensors connected
		// to our bus have changed.
		sensor_count = ds18b20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS);

		if (sensor_count < 1) {
			//  printf("\nNo sensors detected!\n");
			vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
		} else {
			//  printf("\n%d sensors detected:\n", sensor_count);
			// If there were more sensors found than we have space to handle,
			// just report the first MAX_SENSORS..
			if (sensor_count > MAX_SENSORS)
				sensor_count = MAX_SENSORS;

			// Do a number of temperature samples, and print the results.
			// for (int i = 0; i < RESCAN_INTERVAL; i++) {
			ds18b20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count,
					tempsr);
			for (uint8_t j = 0; j < sensor_count; j++) {

				  if ( xSemaphoreTake( tmp, ( TickType_t ) 15  ) == pdTRUE) {
					  temps=tempsr[j];
					  xSemaphoreGive(tmp);
				  }

				  DEBUG_PRINTF(" Current temp  %0.2f deg C \r\n", tempsr[j]);
			//	printf(" dev_config.temp_setpoint  %f deg C \r\n",
			//			dev_config.temp_setpoint);
			//	printf(" dev_config.hysteresis  %f deg C \r\n",
			//			dev_config.hysteresis);

			}


			//  printf("\n");

			// Wait for a little bit between each sample (note that the
			// ds18b20_measure_and_read_multi operation already takes at
			// least 750ms to run, so this is on top of that delay).
			vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
			//}

		}
		taskYIELD();
	}

}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

static void  topic_received(mqtt_message_data_t *md)
{
	uint8_t i;
    mqtt_message_t *message = md->message;
    const int buf_size = message->payloadlen ;//0xFF * 2;
    DEBUG_PRINTF("INFO:  Payload Length : %d \n",buf_size);
   // printf(" Payload Length : %d \n",buf_size);
    char buf[buf_size+1];
      memset(buf, 0, sizeof(buf));
      for( i = 0; i < (int)message->payloadlen; ++i){
        //printf("%c ", ((char *)(message->payload))[i]);
        buf[i]=((char *)(message->payload))[i];

    }
      DEBUG_PRINTF("INFO:  Buff Data %s \n",buf);
    uint8_t r;
  	jsmn_parser p;
  	jsmntok_t t[128];//  We expect no more than 128 tokens
  	//jsmntok_t *tokens = json_tokenise(JSON_STRING);
	jsmn_init(&p);
	r = jsmn_parse(&p,buf, strlen(buf), t, sizeof(t)/sizeof(t[0]));
		if (r < 0) {
			DEBUG_PRINTF("INFO: Failed to parse JSON: %d\n", r);
		}

		DEBUG_PRINTF("INFO: to parse JSON:\n");
  	//	 Assume the top-level element is an object
  		if (r < 1 || t[0].type != JSMN_OBJECT) {
  			DEBUG_PRINTF("INFO: Object expected\n");
  		}
  		for (i = 1; i < r; i++) {

  			DEBUG_PRINTF("INFO: to parse JSON1:\n");

  			if (jsoneq(buf, &t[i], "method") == 0) {
  						 //We may use strndup() to fetch string value
  						DEBUG_PRINTF("INFO:  method found ! \n");
  						method=strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);

  						i++;
  			 }else if (jsoneq(buf, &t[i], "params") == 0) {
  				int j,k;
  				DEBUG_PRINTF("INFO:  params found ! \n");
  				if (t[i+1].type != JSMN_OBJECT) {
  					continue; /* We expect groups to be an array of strings */
  				}
  				for (k = i+1; k < r; k++) {

  					if (jsoneq(buf, &t[k], "pin") == 0) {
  						//We may use strndup() to fetch string value
  						DEBUG_PRINTF("INFO:  pin found ! \n");
  						pin=strndup(buf + t[k+1].start,t[k+1].end-t[k+1].start);
  						k++;
  					} else if (jsoneq(buf, &t[k], "value") == 0) {
  						//We may use strndup() to fetch string value
  						DEBUG_PRINTF("INFO:  value found ! \n");
  						value=strndup(buf + t[k+1].start,t[k+1].end-t[k+1].start);
  						k++;
  					} else if (jsoneq(buf, &t[k], "ssid") == 0){
  						DEBUG_PRINTF("INFO:  ssid found ! \n");
  						USER_WIFI_SSID_=strndup(buf + t[k+1].start,t[k+1].end-t[k+1].start);

  					}else if (jsoneq(buf, &t[k], "pass") == 0){
  						DEBUG_PRINTF("INFO:  pass found ! \n");
  						USER_WIFI_PASS_=strndup(buf + t[k+1].start,t[k+1].end-t[k+1].start);

  					}

  				}


  				//i += t[i+1].size + 1;
  			}

/*  			if (jsoneq(buf, &t[i], "params") == 0) {
  				int j;
  						 //We may use strndup() to fetch string value
  						DEBUG_PRINTF("INFO:  params found ! \n");
  						if (t[i+1].type != JSMN_OBJECT) {
  										continue;  We expect groups to be an array of strings
  										DEBUG_PRINTF("INFO:  JSMN_OBJECT ! \n");
  									}
  						for (j = 0; j < t[i+1].size; j++) {
  							jsmntok_t *g = &t[i+j+2];
  							pin =strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);
  							DEBUG_PRINTF("  * %.*s\n", g->end - g->start, buf + g->start);

  							DEBUG_PRINTF(" PIN *s\n", strndup(buf + g[i+1].start,g[i+1].end-g[i+1].start));
  						}

  						i += t[i+1].size + 1;
  			 }*/


/* else if (jsoneq(buf, &t[i], "ssid") == 0) {
			// We may additionally check if the value is either "true" or "false"
			printf("- ssid: %.*s\n", t[i+1].end-t[i+1].start,buf+ t[i+1].start);
//			gpioval=strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);
			USER_WIFI_SSID_=strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);
			i++;
			 write_file_posix(buf);
	   }else if (jsoneq(buf, &t[i], "pass") == 0) {
				// We may additionally check if the value is either "true" or "false"
				printf("- ssid: %.*s\n", t[i+1].end-t[i+1].start,buf+ t[i+1].start);
				USER_WIFI_PASS_=strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);

	//			gpioval=strndup(buf + t[i+1].start,t[i+1].end-t[i+1].start);
				i++;
	  }*/

  			//


  		}
  		if(!strcmp(method,"setGpio")){
  			DEBUG_PRINTF("INFO: RPC Method %s\n",method);
  			/* method is set gpio */
  			if(!strcmp(pin,"rlp1")){
  				DEBUG_PRINTF("INFO: RPC Method %s\n",pin);
  				/* method is set gpio */
  				if(!strcmp(value,"ON")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set high gpio */
  				}else if(!strcmp(value,"OFF")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set toggle gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					gpio_write(relaylp1,1);
  					vTaskDelay(200 / portTICK_PERIOD_MS);
  					gpio_write(relaylp1,0);
  				}
  			}else if(!strcmp(pin,"rlp2")){
  				DEBUG_PRINTF("INFO: RPC Method %s\n",pin);
  				/* method is set gpio */
  				if(!strcmp(value,"ON")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set high gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					gpio_write(relaylp2,1);
  				}else if(!strcmp(value,"OFF")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set low gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					gpio_write(relaylp2,0);

  				}
  			}else if(!strcmp(pin,"rpw1")){
  				DEBUG_PRINTF("INFO: RPC Method %s\n",pin);
  				/* method is set gpio */
  				if(!strcmp(value,"ON")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set high gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					if ( xSemaphoreTake( tmp, ( TickType_t ) 20 ) == pdTRUE) {
  						gpio_write(relaypw1, 1);
  						//	printf("gpiosetval %d\n", cur_stat.rpw1);
  						cur_stat.rpw1 = true;
  						DEBUG_PRINTF("gpiosetval %d\n", cur_stat.rpw1);
  						xSemaphoreGive(tmp);
  					}
  				}else if(!strcmp(value,"OFF")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set low gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					if ( xSemaphoreTake( tmp, ( TickType_t ) 20  ) == pdTRUE) {
  						gpio_write(relaypw1,0);

  						cur_stat.rpw1 = false;
  						printf("gpiosetval %d\n",cur_stat.rpw1);
  						xSemaphoreGive(tmp);
  					}

  				}
  			}else if(!strcmp(pin,"rpw2")){
  				DEBUG_PRINTF("INFO: RPC Method %s\n",pin);
  				/* method is set gpio */
  				if(!strcmp(value,"ON")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set high gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					gpio_write(relaypw2,1);
  				}else if(!strcmp(value,"OFF")){
  					DEBUG_PRINTF("INFO: RPC Method %s\n",value);
  					/* set low gpio */
  					DEBUG_PRINTF("gpiosetval %s\n",value);
  					gpio_write(relaypw2,0);

  				}
  			}
  		}
  		DEBUG_PRINTF("INFO: to parse JSON2:\n");
    printf("\r\n");
}


static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void  mqtt_task(void *pvParameters)
{
    int ret   = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP8266");
    strcat(mqtt_client_id, get_my_id());
  //  strcat(mqtt_client_id, dev_config.devid);
    char subdata[39];
    char attr_request[]="v1/devices/me/attributes";
    strcpy(subdata, "v1/devices/me/rpc/request/+");
 //  strcat(subdata, get_my_id());
  //  sprintf(subdata,"sensor/%s",get_my_id());
  //  strcat(subdata,"/request/+/+");
   char pubdata[24]="v1/devices/me/telemetry";
  //  char pubdata[]="sensors";
   // sprintf(topicdata,"/%s",mqtt_client_id);

 //  sprintf(pubdata,"optimiser/temp/%s",dev_config.devid);
   TM_PRINTF("INFO:  Subcribe path : %s \n",subdata);
//	printf("sub line %s \n",subdata);
   TM_PRINTF("INFO:  Publishe path : %s \n",pubdata);
	//6001948034C4
    while(1) {

    	static int attr=0;
    	xSemaphoreTake(wifi_alive, portMAX_DELAY);
    	TM_PRINTF("%s: started\n\r", __func__);
    	TM_PRINTF("%s: (Re)connecting to MQTT server %s ... ",__func__,MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
        	TM_PRINTF("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        TM_PRINTF("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100,
                      mqtt_readbuf, 100);
        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = mqtt_client_id;
        data.username.cstring   = MQTT_USER;
        //data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
        TM_PRINTF("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
        	TM_PRINTF("error: %d\n\r", ret);
            // fail link
			if (linkstatus != NULL) {
				if ( xSemaphoreTake( linkstatus, ( TickType_t ) 5 ) == pdTRUE) {
					statlink = 0;
					xSemaphoreGive(linkstatus);
				} else {
					/* We could not obtain the semaphore and can therefore not access
					 the shared resource safely. */
				}
			}
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }

        TM_PRINTF("done\r\n");

       // mqtt_subscribe(&client,attr_request, MQTT_QOS1, topic_received);
        mqtt_subscribe(&client,subdata, MQTT_QOS1, topic_received);
        DEBUG_PRINTF("subdata %s\r\n",subdata);
        xQueueReset(publish_queue);

        //connect successs check
		if (linkstatus != NULL) {
			if ( xSemaphoreTake( linkstatus, ( TickType_t ) 5 ) == pdTRUE) {
				statlink = 1;
				xSemaphoreGive(linkstatus);
			} else {
				/* We could not obtain the semaphore and can therefore not access
				 the shared resource safely. */
			}
		}
///

        while(1){

        	if(!attr){
            	/* Attributes data publish*/
               mqtt_message_t message;
        	   DEBUG_PRINTF("Attributes data publish\n");
              // message.payload = ("{\"SN\":\"%s\",\"model\":\"AC-C\"}",dev_config.devid);
               message.payload = ("{\"SN\":\"600194803E7D\",\"model\":\"AC-C\"}");
               message.payloadlen =strlen("{\"SN\":\"600194803E7D\",\"model\":\"AC-C\"}");
               // message.payloadlen = 80;
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 1;
                ret = mqtt_publish(&client, "v1/devices/me/attributes", &message);
                DEBUG_PRINTF("Attributes data publish done\n");
              //  printf(" Json Data  :  %s\n\r",msg);
                if (ret != MQTT_SUCCESS ){
                	TM_PRINTF("error while publishing Attributes data message: %d\n", ret );
                    break;
                }

                attr=1;

        	}

        	char msg[PUB_MSG_LEN+1 ] = "\0";
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==pdTRUE){

               DEBUG_PRINTF("got telemetry message to publish\r\n");
        	   mqtt_message_t message;
        	   DEBUG_PRINTF("telemetry pub start\n");
               message.payload = msg;
               message.payloadlen =strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 1;
                ret = mqtt_publish(&client, "v1/devices/me/telemetry", &message);
                DEBUG_PRINTF("telemetry pub done\n");
              //  printf(" Json Data  :  %s\n\r",msg);
                if (ret != MQTT_SUCCESS ){
                	TM_PRINTF("error while publishing telemetry message: %d\n", ret );
                    break;
                }

                //

            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
		TM_PRINTF("Connection dropped, request restart\n\r");
       //link failed
		if (linkstatus != NULL) {
			if ( xSemaphoreTake( linkstatus, ( TickType_t ) 5 ) == pdTRUE) {
				statlink = 0;
				xSemaphoreGive(linkstatus);
			} else {
				/* We could not obtain the semaphore and can therefore not access
				 the shared resource safely. */
			}
		}
		//printf("error test");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void dataparse_json(){

//	printf("Inside json_read Task \n ");

	//vTaskDelay(5000 / portTICK_PERIOD_MS);
	//	 printf("Inside json_read Task %s \n ",JSON_STRING);
    int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[128]; /* We expect no more than 128 tokens */
	//jsmntok_t *tokens = json_tokenise(JSON_STRING);

	jsmn_init(&p);
	r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
		if (r < 0) {
			TM_PRINTF("Failed to parse JSON: %d\n", r);
		}

		/* Assume the top-level element is an object */
		if (r < 1 || t[0].type != JSMN_OBJECT) {
			TM_PRINTF("Object expected\n");
		}
		char *pEnd;
		for (i = 1; i < r; i++) {
			if (jsoneq(JSON_STRING, &t[i], "ssid") == 0) {
						/* We may use strndup() to fetch string value */
				DEBUG_PRINTF("- SSID: %.*s\n", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);
						USER_WIFI_SSID_=strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start);
						//USER_WIFI_SSID=strcpy(t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);
						//sprintf(USER_WIFI_SSID,"%s%s",t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);
						i++;
			 }else if (jsoneq(JSON_STRING, &t[i], "pass") == 0) {
						/* We may additionally check if the value is either "true" or "false" */
				 DEBUG_PRINTF("- Pass: %.*s\n", t[i+1].end-t[i+1].start,
								JSON_STRING + t[i+1].start);
						USER_WIFI_PASS_=strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start);
						//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

						i++;
			}else if (jsoneq(JSON_STRING, &t[i], "temp_set") == 0) {
				/* We may additionally check if the value is either "true" or "false" */
				DEBUG_PRINTF("- temp_set: %.*s\n", t[i+1].end-t[i+1].start,
						JSON_STRING + t[i+1].start);

				dev_config.temp_setpoint=strtof(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start),&pEnd);

				//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

				i++;
	       }else if (jsoneq(JSON_STRING, &t[i], "hyst") == 0) {
				/* We may additionally check if the value is either "true" or "false" */
	    	   DEBUG_PRINTF("- hyst_set: %.*s\n", t[i+1].end-t[i+1].start,
						JSON_STRING + t[i+1].start);

			 dev_config.hysteresis=strtof(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start),&pEnd);
				//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

				i++;
				}else if (jsoneq(JSON_STRING, &t[i], "rpw1") == 0) {
					/* We may additionally check if the value is either "true" or "false" */
					DEBUG_PRINTF("- rpw1_set: %.*s\n", t[i+1].end-t[i+1].start,
							JSON_STRING + t[i+1].start);


				 	dev_config.default_stat.rpw1=atoi(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start));
					//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);
				 	DEBUG_PRINTF("rpw1 %d \r\n",(int)dev_config.default_stat.rpw1);
				 	gpio_write(relaypw1,(int)dev_config.default_stat.rpw1);
				 	cur_stat.rpw1=1;


					i++;
			}else if (jsoneq(JSON_STRING, &t[i], "rpw2") == 0) {
				/* We may additionally check if the value is either "true" or "false" */
				DEBUG_PRINTF("- rpw2_set: %.*s\n", t[i+1].end-t[i+1].start,
						JSON_STRING + t[i+1].start);
				 dev_config.default_stat.rpw2=atoi(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start));
				//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

				i++;
			}else if (jsoneq(JSON_STRING, &t[i], "rlp1") == 0) {
				/* We may additionally check if the value is either "true" or "false" */
				DEBUG_PRINTF("- rlp1_set: %.*s\n", t[i+1].end-t[i+1].start,
						JSON_STRING + t[i+1].start);
				 dev_config.default_stat.rlp1=atoi(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start));
				//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

				i++;
			}else if (jsoneq(JSON_STRING, &t[i], "rlp2") == 0) {
				/* We may additionally check if the value is either "true" or "false" */
				DEBUG_PRINTF("- rlp2_set: %.*s\n", t[i+1].end-t[i+1].start,
						JSON_STRING + t[i+1].start);
				 dev_config.default_stat.rlp2=atoi(strndup(JSON_STRING + t[i+1].start,t[i+1].end-t[i+1].start));
				//sprintf(USER_WIFI_PASS," %.*s", t[i+1].end-t[i+1].start,JSON_STRING + t[i+1].start);

				i++;
			}
		}

		memcpy(user_config.ssid,USER_WIFI_SSID_,strlen(USER_WIFI_SSID_));
		memcpy(user_config.password,USER_WIFI_PASS_,strlen(USER_WIFI_PASS_));
}

static void read_file_posix()
{
	// vTaskDelay( 50000 / portTICK_PERIOD_MS );
    #if SPIFFS_SINGLETON == 1
		esp_spiffs_init();
	#else
		// for run-time configuration when SPIFFS_SINGLETON = 0
		esp_spiffs_init(0x050000, 0x10000);
	#endif

	if (esp_spiffs_mount() != SPIFFS_OK) {
		TM_PRINTF("Error mount SPIFFS\n");
	}
	const int buf_size = 0xFF * 2;
	uint8_t buf[buf_size];
	int fd = open("setup.txt", O_RDONLY);
	if (fd < 0) {
		TM_PRINTF("Error opening file\n");
		return;
	}
//	printf("Reading file  file\n");
	int read_bytes = read(fd, buf, buf_size);
//	printf("Read %d bytes\n", read_bytes);
	JSON_STRING = (char*) &buf;
	//strncpy(JSON_STRING,(char*) &buf);
	memcpy(JSON_STRING, (char*) &buf, strlen((char*) &buf));
	buf[read_bytes] = '\0';    // zero terminate string
	DEBUG_PRINTF("Data: %s\n", buf);

	close(fd);

    dataparse_json();

}

static char write_file_posix(char buffjson)
{
	// vTaskDelay( 50000 / portTICK_PERIOD_MS );
    #if SPIFFS_SINGLETON == 1
		esp_spiffs_init();
	#else
		// for run-time configuration when SPIFFS_SINGLETON = 0
		esp_spiffs_init(0x050000, 0x10000);
	#endif

	if (esp_spiffs_mount() != SPIFFS_OK) {
		TM_PRINTF("Error mount SPIFFS\n");
	}
	const int buf_size = 0xFF * 2;
	uint8_t buf[buf_size];
	int fd = open("setup.txt", O_WRONLY);
	if (fd < 0) {
		TM_PRINTF("Error opening file\n");
		return;
	}

	close(fd);

	 fd = open("setup.txt", O_WRONLY);
	if (fd < 0) {
		TM_PRINTF("Error opening file\n");
		return;
	}

	int written = write(fd, buffjson, sizeof(buffjson));
	TM_PRINTF("Written %d bytes\n", buffjson);


	close(fd);

}

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 10;
   sdk_wifi_station_disconnect();
  //  printf("WiFi: example_read_file_posix()\n\r");
    read_file_posix();
  //  vTaskDelay( 10000 / portTICK_PERIOD_MS );
  //  printf("WiFi: connecting to WiFi\n\r");
//    printf("SSID %s\n\r",user_config.ssid);
//    printf("PASS %s\n\r",user_config.password);
    sdk_wifi_station_set_config(&user_config);
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_connect();
  //  vTaskDelay(1000/ portTICK_PERIOD_MS );
    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
         //   printf("%s: status = %d\n\r", __func__, status );
            TM_PRINTF("Retries %d\n",retries);
            if( status == STATION_WRONG_PASSWORD ){
            	TM_PRINTF("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
            	TM_PRINTF("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
            	TM_PRINTF("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
        if (status == STATION_GOT_IP) {
        	TM_PRINTF("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
        	if (strcmp(user_config.ssid, WIFI_SSID) != 0)
        	//if(user_config.ssid==DEFAULT_WIFI_SSID)
        	{
        		//vTaskDelay( 2000 / portTICK_PERIOD_MS );
        	//	printf("USER_Config %s\n\r",user_config.ssid);
        	}else{
        		//vTaskDelay( 2000 / portTICK_PERIOD_MS );
        	//	printf("DEFAULT_config %s\n\r",user_config.ssid);
        		//update variable dont start the temp reading nd normal mqtt pub
        		//get config data
        	}
        	//printf("inside another while\n\r");
        	vTaskDelay( 1000 / portTICK_PERIOD_MS );
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        TM_PRINTF("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        user_config=default_config;
        //printf("SSID %s\n\r",user_config.ssid);
        //printf("PASS %s\n\r",user_config.password);
        sdk_wifi_station_set_config(&user_config);
        sdk_wifi_station_connect();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void heartbeat(void *pvParameters)
{
    gpio_enable(heartbeatled, GPIO_OUTPUT);
    TickType_t hb = xTaskGetTickCount();
    for(;;) {
    	 vTaskDelayUntil(&hb, 1000 / portTICK_PERIOD_MS);
        gpio_write(heartbeatled, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_write(heartbeatled, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    taskYIELD();
}

void link_wifi(void *pvParameters)
{
	//vTaskDelay(10000 / portTICK_PERIOD_MS);
	TickType_t link = xTaskGetTickCount();
    gpio_enable(linkled, GPIO_OUTPUT);
	for(;;) {

		  vTaskDelayUntil(&link, 1000 / portTICK_PERIOD_MS);
		//semp
		if (linkstatus != NULL) {

			if ( xSemaphoreTake( linkstatus, ( TickType_t ) 10  ) == pdTRUE) {
				if (statlink) {
					xSemaphoreGive(linkstatus);

					//printf(" Link Stat %d \n\r", statlink);

							gpio_write(linkled, 0);
					        vTaskDelay(100 / portTICK_PERIOD_MS);
					        gpio_write(linkled, 1);
					        vTaskDelay(100 / portTICK_PERIOD_MS);

				} else if (!statlink) {
					xSemaphoreGive(linkstatus);
				//	printf(" Link Stat ! %d \n\r", statlink);

					 uint8_t blink=10;
										while(blink){
											gpio_write(linkled, 0);
											vTaskDelay(100/ portTICK_PERIOD_MS);
											gpio_write(linkled, 1);
											vTaskDelay(100/ portTICK_PERIOD_MS);
											--blink;
										}
				}
			} else {
				/* We could not obtain the semaphore and can therefore not access
				 the shared resource safely. */
			}
		}
		taskYIELD();
	}
}

void intrruptTaskMotion(void *pvParameters)
{
    QueueHandle_t *tsqueue_motion = (QueueHandle_t *)pvParameters;
    gpio_set_interrupt(gpio_intrrupt_pir, int_type, gpio_intr_handler);

    char msg[PUB_MSG_LEN];
    uint32_t last = 0;
    for(;;) {

        uint32_t button_ts;
        xQueueReceive(*tsqueue_motion, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;

        if(last < button_ts-200) {
        	 if ( xSemaphoreTake( motion, ( TickType_t ) 5  ) == pdTRUE) {
        		 motion_count++;
        		 DEBUG_PRINTF("motion count %d \r\n",motion_count);
        		 xSemaphoreGive(motion);
        	 }
            last = button_ts;
        }
        taskYIELD();}
}

void intrruptTask(void *pvParameters)
{
   // printf("Waiting for  interrupt on gpio %d...\r\n", gpio_intrrupt_pir);

    QueueHandle_t *tsqueue = (QueueHandle_t *)pvParameters;

  //  gpio_set_interrupt(gpio_intrrupt_pir, int_type, gpio_intr_handler);

    gpio_set_interrupt(gpio_intrrupt_in2, int_typein2, gpio_intr_handler2);

    gpio_set_interrupt(gpio_intrrupt_in1, int_typein2, gpio_intr_handler3);

    char msg[PUB_MSG_LEN];
    uint32_t last = 0;
    for(;;) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;

        if(last < button_ts-200) {
           // printf("interrupt fired at %dms\r\n", button_ts);
        	DEBUG_PRINTF("Intrrupt @ pin %d \n",button_ts);

        	memset(msg, 0, sizeof(msg));
          //  snprintf(msg, PUB_MSG_LEN,"Intrrupt @%d\n", button_ts);
            snprintf(msg,PUB_MSG_LEN, " {\"d\":\"600194803E7D\",\"i\":%d }\r\n ",button_ts);

         //   xSemaphoreGive( updateTemp );
           // printf("Sensor %f deg C \n", temps[j]);
            if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            	DEBUG_PRINTF("Publish queue overflow.\r\n");
              }

            last = button_ts;
        }
        taskYIELD();}
}

//ds2413

void relay_ds2413_devices(void *pvParameters) {

    ds2413_addr_t ds2413_addrs[MAX_SENSORS];
    int sensor_count;
    gpio_set_pullup(SENSOR_GPIO, true, true);

     

	sensor_count = ds2413_scan_devices(SENSOR_GPIO, ds2413_addrs, MAX_SENSORS);
	        if (sensor_count < 1) {
            printf("\nNo sensors detected!\n");
        } else {
		
	    printf("\n%d sensors detected:\n", sensor_count);

            // If there were more sensors found than we have space to handle,
            // just report the first MAX_SENSORS..
            if (sensor_count > MAX_SENSORS) sensor_count = MAX_SENSORS;

           // Wait for a little bit between each sample (note that the
           // ds2413_measure_and_read_multi operation already takes at
           // least 750ms to run, so this is on top of that delay).
           vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);

	   }

	        while(1) {

	        	for (int j = 0; j < sensor_count; j++) {

	        		// The DS2413 address is a 64-bit integer, but newlib-nano
	        		// printf does not support printing 64-bit values, so we
	        		// split it up into two 32-bit integers and print them
	        		// back-to-back to make it look like one big hex number.
	        		uint64_t addrds2413=ds2413_addrs[j]>>8;
	        		addrds2413&=0xffffffffffff;

	        		uint32_t addr0 =addrds2413>>32;// ds2413_addrs[j] >> 32;
	        		//   uint32_t addr0 = addrs[j] >> 8;
	        		uint32_t addr1 = addrds2413; //ds2413_addrs[j];
	        		printf("\n");
	        		printf("Sensor %08x%08x  \n", addr0, addr1);
	        		printf("\n");

	        		//   (ds18b20_pio_access_write(SENSOR_GPIO,addrs[0],0x2) == true ? printf("Write succes \n") : printf("Write failed \n"));
	        		//  (ds18b20_pio_access_write(SENSOR_GPIO,addrs[0],0x0) == true ? printf("Write succes \n") : printf("Write failed \n"));
	        		printf("\n");
	        		uint8_t pin_vlaue =ds2413_read_pin(SENSOR_GPIO,ds2413_addrs[j]);
	        		printf("Pin Value %02x  \n", pin_vlaue);
	        		// (ds18b20_pio_access_write(SENSOR_GPIO,addrs[1],0x0) == true ? printf("Write succes \n") : printf("Write failed \n"));
	        		/*find the input pin with the ds2413 sensor then keep value to the global access*/
	        		vTaskDelay(2000 / portTICK_PERIOD_MS);

	        	}


	        	taskYIELD();
	        }

}




static QueueHandle_t tsqueue;
static QueueHandle_t tsqueue_motion;

void gpio_intr_handler(uint8_t gpio_num)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue_motion, &now, NULL);
}

void gpio_intr_handler2(uint8_t gpio_num)
{
    uint32_t now1 = 0 ;//xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now1, NULL);
}

void gpio_intr_handler3(uint8_t gpio_num)
{
    uint32_t now2 = 2;// xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now2, NULL);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    TM_PRINTF("SDK version:%s\n", sdk_system_get_sdk_version());

    dev_config.devid=get_my_id();

    TM_PRINTF("Dev ID:%s\n",dev_config.devid);
    gpio_enable(heartbeatled, GPIO_OUTPUT);
    gpio_enable(linkled, GPIO_OUTPUT);
  
    gpio_enable(relaylp1, GPIO_OUTPUT);
    gpio_enable(relaylp2, GPIO_OUTPUT);
    gpio_enable(relaypw1, GPIO_OUTPUT);
    gpio_enable(relaypw2, GPIO_OUTPUT);

   // gpio_enable(input1, GPIO_INPUT);
    //gpio_enable(input2, GPIO_INPUT);

    gpio_write(heartbeatled,1);
    gpio_write(linkled, 1);




    gpio_write(relaylp1,0);
    gpio_write(relaylp2,0);
    gpio_write(relaypw1,0);
    gpio_write(relaypw2,1);

    linkstatus = xSemaphoreCreateMutex();
    tmp = xSemaphoreCreateMutex();
    wifi_alive=xSemaphoreCreateBinary(); //wifi mutex
    motion=xSemaphoreCreateMutex();

    xTaskCreate(heartbeat, "heartbeat", 128, NULL, 1, NULL);

    xTaskCreate(&wifi_task, "wifi_task",  1024, NULL, 5, NULL);

    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);

    xTaskCreate(link_wifi, "linkled", 256, NULL, 2, &xHandle_link_wifi);

    xTaskCreate(stats_send, "sendstatus", 512, NULL, 2, NULL);

    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    tsqueue_motion = xQueueCreate(2, sizeof(uint32_t));

    xTaskCreate(&get_temperature, "get_temperature", 512, NULL, 3, NULL);

  // gpio_enable(gpio, GPIO_INPUT);

  //  xTaskCreate(intrruptTask, "IntrruptTask", 512, &tsqueue, 6, NULL);


    xTaskCreate(intrruptTaskMotion, "IntrruptTask", 512, &tsqueue_motion, 6, NULL);

    xTaskCreate(&relay_ds2413_devices, "relay_ds2413_devices", 256, NULL, 2, NULL);

}
