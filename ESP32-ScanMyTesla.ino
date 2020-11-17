/*
ESP32-ScanMyTesla 1.0.0 (2020)
Author: E.Burkowski
GENERAL PUBLIC LICENSE
*/

#include <esp32_can.h>  //https://github.com/collin80/esp32_can and https://github.com/collin80/can_common
#include <BluetoothSerial.h>

/* CAN-ID settings:
 *  7 6 5 4 3 2 1 0
 *  Bit 0: 0 - ignore, 1 - relevant
 *  Bit 1: tbd
 *  Bit 2: tbd
 *  Bit 3: tbd
 *  Bit 4: tbd
 *  Bit 5: tbd
 *  Bit 6: tbd
 *  Bit 7: tbd
 */

#define CAN_ID_BIT_RELEVANT 0  //mask for CAN-ID settings
#define BUFFER_LENGTH 16       //play with this size to get better packtes/second

//#define DEBUG                 //serial debug output
//#define DATA_SIMULATION       //use test data without can hardware

byte messageCounter = 0;
 
//it's not RAM optimized (we have enough), but it should be very fast. do it better later...
byte ids[2048];
byte canDataBuffer[BUFFER_LENGTH][8];
byte canDataBufferLength[BUFFER_LENGTH];
uint32_t canDataBufferId[BUFFER_LENGTH];

//some CAN dumps to simulate can bus without physical HW
#ifdef DATA_SIMULATION
byte canDataBufferTest[5][8] = {
                        {0x2e, 0x97, 0xF6, 0xFF, 0xFD, 0x26, 0xFF, 0x0F}, //1322e97f6fffd26ff0f
                        {0xE3, 0x24, 0x55, 0x20, 0xFF, 0x1F, 0xFF, 0x3F}, //129E3245520FF1FFF3F 
                        {0xE6, 0x2E, 0x6A, 0xA8, 0xA1, 0x15, 0x84, 0x00}, //352E62E6AA8A1158400
                        {0x73, 0xB7, 0x9D, 0x77, 0xDD, 0x0A, 0x03, 0x00}, //29273B79D77DD0A0300
                        {0x00, 0x00, 0x0F, 0x09, 0x00, 0x00, 0xA0, 0x0F}}; //25200000F090000A00F
byte canDataBufferLengthTest[5] = {8, 8, 8, 8, 8};
uint32_t canDataBufferIdTest[5] = {0x132, 0x129, 0x352, 0x292, 0x252};
byte lastTestMessage = 0;
#endif

//static, for ELM327
uint32_t relevantIds[] = {0x108, 0x118, 0x129, 0x132, 0x1D5, 0x1D8, 0x186,
                          0x20C, 0x212, 0x241, 0x244, 0x252, 0x257, 0x261, 0x264, 0x266, 
                          0x267, 0x268, 0x292, 0x29D, 0x2A8, 0x2B4, 0x2C1, 0x2D2, 0x2E5, 
                          0x312, 0x315, 0x321, 0x332, 0x334, 0x336, 0x33A, 0x352, 0x376, 
                          0x381, 0x382, 0x395, 0x396, 0x3A1, 0x3B6, 0x3D2, 0x3F2, 0x3FE, 
                          0x401, 0x541, 0x557, 0x5D7, 0x692};

char btBufferCounter = 0;
char buffer[128];
String lineEnd = String("\n");

bool getBit(byte b, byte bitNumber) {
   return (b & (1 << bitNumber)) != 0;
}

BluetoothSerial SerialBT;

#ifdef DEBUG
void printFrame(CAN_FRAME *message)
{
    Serial.print(message->id, HEX);
    Serial.print(" ");
    for (int i = 0; i < message->length; i++) {
        if(message->data.byte[i] < 16) Serial.print("0");
        Serial.print(message->data.byte[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
#endif


bool copyDataToBuffer(CAN_FRAME *canData, byte lineNumber){
    if(canData->length > 0 && canData->length < 9){ //messages can be 1..8 bytes long only, if not drop this message (propably corrupt)
        canDataBufferId[lineNumber] = canData->id;
        canDataBufferLength[lineNumber] = canData->length;
        for(byte i = 0; i < canData->length; i++){
            canDataBuffer[lineNumber][i] = canData->data.byte[i];
        }
        return true;
    }else{
#ifdef DEBUG
        Serial.println("Message dropped, wrong length");
#endif              
        return false;
    }
}

void setup() {
#ifdef DEBUG
    Serial.begin(250000);
    Serial.println("ESP_SMT init");
#endif
#ifdef DATA_SIMULATION
    SerialBT.begin("ESP-SMT-Simulation");
#else //normal mode
    SerialBT.begin("ESP-SMT");
#endif
    CAN0.begin(500000); //for Tesla
    CAN0.watchFor();
    memset(canDataBuffer, 0x00, sizeof(canDataBuffer) / sizeof(canDataBuffer[0])); //clear data buffer
    memset(ids, B00000000, sizeof(ids)); //set ID-settings to per default b00000000
    //set default relevant ids (for ELM327. ST1110 sets its own filters)
    for(byte i = 0; i < sizeof(relevantIds)/sizeof(relevantIds[0]); i++){
        ids[relevantIds[i]] = 0x01; //it will work until other bits in CAN-ID settings are not defined
    }
}


void canLoop(){
    CAN_FRAME canMessage;
#ifdef DATA_SIMULATION
//test can data
    canMessage.rtr = 0;
    canMessage.id = canDataBufferIdTest[lastTestMessage];
    canMessage.extended = false;
    canMessage.length = canDataBufferLengthTest[lastTestMessage];
    for(byte b = 0; b < canMessage.length; b++){
        canMessage.data.uint8[b] = canDataBufferTest[lastTestMessage][b];
    }
    lastTestMessage++;
    if(lastTestMessage >= sizeof(canDataBufferLengthTest)) lastTestMessage = 0;
#else
//real can data
    if (CAN0.read(canMessage)) {
#ifdef DEBUG
    printFrame(&canMessage); //print ony real messages, not a simulation
#endif //DEBUG
#endif //DATA_SIMULATION
        if(getBit(ids[canMessage.id], CAN_ID_BIT_RELEVANT)){ //check if this ID relevant for us
//            Serial.print("Relevant ID received: ");
//            Serial.println(canMessage.id, HEX);
//            printFrame(&canMessage);
            if(copyDataToBuffer(&canMessage, messageCounter)) messageCounter++;
            if(messageCounter >= BUFFER_LENGTH) {
                messageCounter = 0;
                return;
            }
        }
#ifndef DATA_SIMULATION //not DATA_SIMULATION     
    }
#endif
}

String processSmtCommands(char *smtCmd){
    //a lot of string manipulation to cut filter ID from SMT message. Any better ideas?
    String cmd = String(smtCmd);
    String returnToSmt = String();
    String sFilter = String();
    uint32_t filter = 0;
#ifdef DEBUG    
    Serial.print("smtCmd: ");
    Serial.println(smtCmd);
#endif
    //wait for at-commands (ELM327) or st-commands (ST1110)
    if (!strncmp(smtCmd, "at", 2) || !strncmp(smtCmd, "st", 2)){
        if(!strncmp(smtCmd, "atma", 4) || !strncmp(smtCmd, "stm", 3)){//data polling
            //send response here
            for(byte i = 0; i < BUFFER_LENGTH; i++){
                if(canDataBufferLength[i] > 0){
                    if(canDataBufferId[i] < 256) returnToSmt.concat("0"); //make id 3 hex digit long
                    if(canDataBufferId[i] < 16) returnToSmt.concat("0"); //make id 2 hex digit long
                    returnToSmt.concat(String(canDataBufferId[i], HEX));
                    for(byte l = 0; l < canDataBufferLength[i]; l++){
                        if(canDataBuffer[i][l] < 16) returnToSmt.concat("0"); //make data 2 digits long
                        returnToSmt.concat(String(canDataBuffer[i][l], HEX));
                    }
                returnToSmt.concat(lineEnd);
                }
            }
        }else if(!strncmp(smtCmd, "stfap ", 6)){//e.g. "stfap 3d2,7ff", we need 3d2, first charachters 6 and 3 length: 3d2
            
            sFilter = cmd.substring(9,6); //why 9,6?!? it should be 6,3!! 
            const char * chCmd = sFilter.c_str(); //HEX string (3d2)
            filter = strtol(chCmd, 0, 16); //convert HEX string to integer (978)
#ifdef DEBUG
            Serial.print("New filter from SMT: ");
            Serial.println(filter);  
#endif 
            ids[filter] = 0x01; //it will work until other bits in CAN-ID settings are not defined
            returnToSmt.concat("OK");
        }else if(!strncmp(smtCmd, "stfcp", 5)){
#ifdef DEBUG
            Serial.println("Clear all filters");
#endif
            memset(ids, B00000000, sizeof(ids));
            returnToSmt.concat("OK");
        }else{
            //all other at* commands, we don't care, send "OK"...
            returnToSmt.concat("OK");
        }
        returnToSmt.concat(">");
    }else{
        returnToSmt.concat(lineEnd); //we are not allowed to send "NULL" to BT, send atleast "CR" 
    }
    return returnToSmt;  
}


void processBtMessage(){
    String responseToBt = processSmtCommands(buffer);
#ifdef DEBUG
    Serial.println("BT out message: ");
    Serial.println(responseToBt);
#endif
    SerialBT.print(responseToBt); //send String to BT
}

void btLoop(){
    byte tmp;
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
