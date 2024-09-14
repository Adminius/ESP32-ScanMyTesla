/*
ESP32-ScanMyTesla 2.0.0 (2024)
Author: E.Burkowski
GENERAL PUBLIC LICENSE
*/

#include "driver/twai.h"
#include <BluetoothSerial.h>


#define BUFFER_LENGTH 16       //play with this size to get better packets/second

#define RX_PIN 16
#define TX_PIN 17

//#define DEBUG                  //serial debug output

uint8_t messageCounter = 0;    //should not be greater than BUFFER_LENGTH

bool ids[2048];                             //save here ID that we care about.
uint16_t canDataBufferId[BUFFER_LENGTH];    
uint8_t canDataBufferLength[BUFFER_LENGTH];
uint8_t canDataBufferData[BUFFER_LENGTH][8];

bool noFilter = true;
char btBufferCounter = 0;
char buffer[128];
String lineEnd = String("\n");

static bool driver_installed = false;

BluetoothSerial SerialBT;

#ifdef DEBUG
void printFrame(twai_message_t &canMessage)
{
    Serial.printf("%03x: ", canMessage.identifier);
    for (uint8_t i = 0; i <  canMessage.data_length_code; i++) {
        Serial.printf("%02x ", canMessage.data[i]);
    }
    Serial.println();
}

#define debug_println(msg) Serial.println(msg);
#define debug_print(msg) Serial.print(msg);

#else
#define printFrame(canMessage) do {} while(0)
#define debug_println(msg) do {} while(0)
#define debug_print(msg) do {} while(0)
#endif

void processCanMessage(twai_message_t &canMessage){
    uint8_t length = canMessage.data_length_code;
    uint8_t lineNumber = messageCounter;

    //print can message before filter
    //printFrame(canMessage);

    //if the can message id is in the list process it
    if(!ids[canMessage.identifier]){
        return;
    }

    //print can message after filter
    //printFrame(canMessage);
    
    //check if this ID is already in the buffer !!! 
    //doesn't work with mutiplexed messages, like cell voltages -> check first byte also (it's mostly multiplex index)
    for(uint8_t i = 0; i < BUFFER_LENGTH; i++){
        if(canDataBufferId[i] == canMessage.identifier && canDataBufferData[lineNumber][0] == canMessage.data[0]){
            debug_print("ID ");
            debug_print(canMessage.identifier);
            debug_println(" allready in buffer");
            lineNumber = i;
        }
    }
    
    //messages can be 1..8 bytes long only, if not drop this message (propably corrupt)
    if(0 < length && length < 9){ 
        canDataBufferId[lineNumber] = canMessage.identifier;
        canDataBufferLength[lineNumber] = length;
        for(uint8_t i = 0; i < length; i++){
            canDataBufferData[lineNumber][i] = canMessage.data[i];
        }
        messageCounter++;
        if(messageCounter >= BUFFER_LENGTH) messageCounter = 0;
    }else{
        debug_println("Message dropped, wrong length");
    }
}

void setup() {
#ifdef DEBUG
    delay(1000);
    Serial.begin(250000);
    debug_println("ESP_SMT init");
#endif
    SerialBT.begin("ESP-SMT");

    //Initialize configuration structures using macro initializers
    //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
                                    .tx_io = (gpio_num_t)TX_PIN,
                                    .rx_io = (gpio_num_t)RX_PIN,
                                    .clkout_io = TWAI_IO_UNUSED,
                                    .bus_off_io = TWAI_IO_UNUSED,
                                    .tx_queue_len = 1,
                                    .rx_queue_len = 127,
                                    .alerts_enabled = TWAI_ALERT_ALL,
                                    .clkout_divider = 0
                                        };
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
        debug_println("Driver installed");
    }else{
        debug_println("Failed to install driver");
        return;
    }
    
    //Start TWAI driver
    if(twai_start() == ESP_OK) {
        debug_println("Driver started");
    }else{
        debug_println("Failed to start driver");
        return;
    }

    //Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if(twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK){
        debug_println("CAN Alerts reconfigured");
    }else{
        debug_println("Failed to reconfigure alerts");
        return;
    }

    //TWAI driver is now successfully installed and started
    driver_installed = true; 

    noFilter = false; //there are some filters
    //set true for all IDs, because no filters applied yet
    memset(ids, true, sizeof(ids));
    memset(canDataBufferId, 0, sizeof(canDataBufferId) / sizeof(canDataBufferId[0]));
    memset(canDataBufferLength, 0, sizeof(canDataBufferLength) / sizeof(canDataBufferLength[0]));
    memset(canDataBufferData, 0, sizeof(canDataBufferData) / sizeof(canDataBufferData[0][0]));
}


void canLoop(){
    if (!driver_installed){
        // Driver not installed
        delay(1000);
        return;
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
        debug_println("Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        debug_println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        debug_print("Bus error count: ");
        debug_println(twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
        debug_println("Alert: The RX queue is full causing a received frame to be lost.");
        debug_print("RX buffered: ");
        debug_println(twaistatus.msgs_to_rx);
        debug_print("RX missed: ");
        debug_println(twaistatus.rx_missed_count);
        debug_print("RX overrun ");
        debug_println(twaistatus.rx_overrun_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        // One or more messages received. Handle all.
        twai_message_t message;
        while (twai_receive(&message, 0) == ESP_OK) {
            processCanMessage(message);
        }
    }
}

String processSmtCommands(char *smtCmd){
    //a lot of string manipulation to cut filter ID from SMT message. Any better ideas?
    String cmd = String(smtCmd);
    String returnToSmt = String();
    String sFilter = String();
    uint16_t filter = 0;

    debug_print("smtCmd: ");
    debug_println(smtCmd);

    //wait for at-commands (ELM327) or st-commands (ST1110)
    if (strncmp(smtCmd, "at", 2) && strncmp(smtCmd, "st", 2)){
        returnToSmt.concat(lineEnd); //we are not allowed to send "NULL" to BT, send at least "CR"
        debug_println("Unknown request");
        //returnToSmt.concat("OK"); //sent something, but not "null"
        return returnToSmt;  
    }

  	//data polling
  	if(!strncmp(smtCmd, "atma", 4) || !strncmp(smtCmd, "stm", 3)){
    		//create response string here
    		for(uint8_t i = 0; i < BUFFER_LENGTH; i++){
      			//only IDs >0 are allowed, important at initialisation
      			if(canDataBufferId[i] == 0) continue;
      			//all message IDs should be 3 digits long
      			if(canDataBufferId[i] < 256) returnToSmt.concat("0"); //make id 3 hex digits long
      			if(canDataBufferId[i] < 16) returnToSmt.concat("0"); //make id 2 hex digits long
            returnToSmt.concat(String(canDataBufferId[i], HEX));
      			
      			for(uint8_t l = 0; l < canDataBufferLength[i]; l++){
        				//all data bytes should be 2 digits long
                if(canDataBufferData[i][l] < 16) returnToSmt.concat("0"); //make data 2 digits long
                returnToSmt.concat(String(canDataBufferData[i][l], HEX));
      			}
      			canDataBufferId[i] = 0; //set ID to zero to ignore it on next run
      			returnToSmt.concat(lineEnd);
		    }
	  //set filters
	  }else if(!strncmp(smtCmd, "stfap ", 6)){//e.g. "stfap 3d2,7ff", we need 3d2, first character 6 and 3 length: 3d2
		
    		sFilter = cmd.substring(9,6); //why 9,6?!? it should be 6,3!! 
    		const char * chCmd = sFilter.c_str(); //HEX string (3d2)
    		filter = strtol(chCmd, 0, 16); //convert HEX string to integer (978)
    		if(noFilter){ 
    			  memset(ids, false, sizeof(ids)); //if there no filters, all IDs are allowed. But we need only one => disallow all and allow one
    			  noFilter = false; //no filtes at all
        }
    
    		debug_print("New filter from SMT: ");
    		debug_println(filter);  
    
    		ids[filter] = true;
    		returnToSmt.concat("OK");
    //clear filters
	  }else if(!strncmp(smtCmd, "stfcp", 5)){

    		debug_println("Clear all filters = allow all IDs!");
    
    		memset(ids, true, sizeof(ids)); //clear all filters = allow all IDs.
    		noFilter = true; //no filtes at all
        debug_println("No IDs are allowed now");
    		returnToSmt.concat("OK");
	  }else{
		//all other at*/st* commands, we don't care about, send "OK"...
		    returnToSmt.concat("OK");
	  }
	  returnToSmt.concat(">");
    
    return returnToSmt;  
}


void processBtMessage(){
    String responseToBt = processSmtCommands(buffer);

    //debug_println("BT out message: ");
    //debug_println(responseToBt);

    SerialBT.print(responseToBt); //send String to BT
}

void btLoop(){
    uint8_t tmp;
    while(SerialBT.available()){
        tmp = SerialBT.read();
        //check if current char is "Carriage Return"
        if (tmp == 13 || btBufferCounter > 126){
            buffer[btBufferCounter] = 0; //terminate the string by null
            btBufferCounter = 0; // reset buffer counter, buffer is empty
            processBtMessage(); 
        }else{
            //check if current char is "new line" or "space", if so, ignore it :)
            if(tmp != 10 || tmp != 32){
                buffer[btBufferCounter++] = (char)tolower(tmp); //use lowercase only
            }
        }
    }
}


void loop() {
    canLoop();
    btLoop();
}
