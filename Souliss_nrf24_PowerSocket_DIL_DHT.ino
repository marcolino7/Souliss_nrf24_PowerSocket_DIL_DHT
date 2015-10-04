/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */
/**********************************************************************************
 * This software provide logic and comunication for a simple power socket
 * equiped with nrf24 radio module, and running a full souliss node.
 *
 * 2014 Marcolino
 * 
 **********************************************************************************/
// Comment out the below lines to use the conf/QuickCfg.h method
//
//Altezza LED 2,45mm

#define	VNET_DEBUG_INSKETCH
#define VNET_DEBUG  		1

#define	MaCaco_DEBUG_INSKETCH
#define MaCaco_DEBUG  		1

//--\conf\frame\vNetCfg.h
#define	VNET_RESETTIME_INSKETCH
#define VNET_RESETTIME		   0x85ED

// Configure the framework
#include "bconf/StandardArduino.h"			// Use a standard Arduino
#include "conf/nRF24L01.h"

// Include framework code and libraries
#include <SPI.h>
#include "Souliss.h"
#include "DHTlib\dht.h"

// End of configuration block

//Souliss Slot
#define POWER_SOCKET	0
#define DHT_TEMP		1
#define DHT_HUMI		3

//Arduino PINs
#define PIN_RELE		2
#define PIN_LED			3
#define PIN_BUTTON		19 //PIN A5
#define PIN_DHT			17 //PIN A3

//Useful Variable
byte led_status = 0;
byte joined = 0;

//Souliss Addressing
#define network_address         0x6503        // Local address
#define network_my_subnet       0xFF00
#define network_my_supern       0x6501        // RF24 gateway address

//Funzione e variabile necessarie per il reset
void(* resetFunc) (void) = 0; //declare reset function @ address 0
U8 value_hold=0x068;


// Identify the sensor, in case of more than one used on the same board	
dht DHT;
#define DEADBANDLOW	  0.005

//*******************************************************
// setup, called once on startup
//*******************************************************
void setup() {
        Serial.begin(115200);
        Serial.println("Node Init");
        
		pinMode(PIN_BUTTON, INPUT);
		pinMode(PIN_RELE, OUTPUT);
		pinMode(PIN_LED, OUTPUT);
        
		// Setup the network configuration
        Souliss_SetAddress(network_address,network_my_subnet,network_my_supern);
		
		//Relè power Socket
		Souliss_SetT11(memory_map, POWER_SOCKET);
		//T52 Temperatur DHT
		Souliss_SetT52(memory_map, DHT_TEMP);
		//T53 Umidità
		Souliss_SetT53(memory_map, DHT_HUMI);


}
//*******************************************************
// main loop
//*******************************************************
void loop(){
	EXECUTEFAST() {						
		UPDATEFAST();
		FAST_50ms() {	// We process the logic and relevant input and output every 50 milliseconds

			//Gestisco la pressione Lunga o corta del bottone
			uint8_t invalue = Souliss_DigInHold(PIN_BUTTON, Souliss_T1n_ToggleCmd, value_hold, memory_map, POWER_SOCKET, 5000);
			if(invalue==Souliss_T1n_ToggleCmd){
				Serial.println("TOGGLE");
				mInput(POWER_SOCKET)=Souliss_T1n_ToggleCmd;
			} else if(invalue==value_hold) {
				// reset
				Serial.println("REBOOT");
				delay(1000);
				resetFunc();
			}

			//Checking Pushbutton
			Souliss_DigIn(PIN_BUTTON, Souliss_T1n_ToggleCmd, memory_map, POWER_SOCKET);
			//Writing relè status
			Souliss_DigOut(PIN_RELE, Souliss_T1n_Coil, memory_map, POWER_SOCKET);
			
			//Check if joined and take control of the led
			if (joined==1) {
				if (mOutput(POWER_SOCKET)==1) {
					digitalWrite(PIN_LED,HIGH);
				} else {
					digitalWrite(PIN_LED,LOW);
				}
			}
		}

		FAST_70ms() {
		}

		FAST_90ms() { 
			//Apply logic if statuses changed
			Souliss_Logic_T11(memory_map, POWER_SOCKET, &data_changed);
		}

		FAST_110ms() {
			//Update Gateway if statuses changed
			Souliss_CommunicationData(memory_map, &data_changed);
		}

		FAST_510ms() {
			//Check if joined to gateway
			check_if_joined();
		}

		FAST_1110ms() {
			Souliss_Logic_T52(memory_map, DHT_TEMP, DEADBANDLOW, &data_changed);
			Souliss_Logic_T53(memory_map, DHT_HUMI, DEADBANDLOW, &data_changed);
		}

        FAST_2110ms() {
        }

}
	
	EXECUTESLOW() {
		UPDATESLOW();

		SLOW_10s() {
			DHTRead();
		}
	}		

}

//This routine check for peer id joined to Souliss Network
//If not blink the led every 500ms, else led is a mirror of relè status
void check_if_joined() {
	if(JoinInProgress() && joined==0){
		joined=0;
		if (led_status==0){
			digitalWrite(PIN_LED,HIGH);
			led_status=1;
		}else{
			digitalWrite(PIN_LED,LOW);
			led_status=0;
		}
	}else{
		joined=1;
	}		
}

void DHTRead() {
	int chk = DHT.read22(PIN_DHT);
	if (chk == DHTLIB_OK) {
		float temperature = DHT.temperature;
		Souliss_ImportAnalog(memory_map, DHT_TEMP, &temperature);

		float humidity = DHT.humidity;
		Souliss_ImportAnalog(memory_map, DHT_HUMI, &humidity);
	}
}