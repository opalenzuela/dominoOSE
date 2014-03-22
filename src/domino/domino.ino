/**
 \file domino.ino
 \brief Domino (OpenDomo for Arduino)
 ****************************************************************************
 *  This file is part of the OpenDomo project.
 *  Copyright(C) 2013 OpenDomo Services S.L.
 *
 *  Oriol Palenzuela Roses <opalenzuela (at) opendomo (dot) com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ****************************************************************************
 \mainpage Domino
 Domino is a multi-purpose driver, designed for creating quick prototypes and
 automated products using a normalized configuration syntax.

 A detailed explanation of all the supported commands and architecture can be
 found in the following webpages:

 -# http://es.opendomo.org/domino (Main page)
 -# http://es.opendomo.org/domino_reference (Reference manual)

 This document is only intended to be useful for the firmware's programmers.
*/

/// Current library version
#define VERSION "1.1.4"

//define para habilitar la funcionalidad de red
#define ENABLE_NETWORKING

#define UID "12345678901A"

//! MAC address: Write here your own MAC
byte mac[] = { 0xBE, 0xAB, 0xEA, 0xEF, 0xFE, 0xED };

//! IP address: Write here your own IP
byte defip[] = {169, 254, 0, 10 };

//! IP address (global)
byte ip[4];

#ifdef ENABLE_NETWORKING
// Library for serial communication (required by Ethernet.h)
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#endif

#include <avr/eeprom.h>

//! Possible outputs for a \ref writef
enum outputChannel {
	NONE = 0,
	SERIALPORT = 1,
	TELNET = 2,
	HTTP = 3
} output;

#define IO_BUFFER_SIZE 50
#define UDP_PORT 1729


/// table for encoding characters in precompilation
#define BYTE1_A 0b000010000000000
#define BYTE1_B 0b000100000000000
#define BYTE1_C 0b000110000000000
#define BYTE1_D 0b001000000000000
#define BYTE1_E 0b001010000000000
#define BYTE1_F 0b001100000000000
#define BYTE1_G 0b001110000000000
#define BYTE1_H 0b010000000000000
#define BYTE1_I 0b010010000000000
#define BYTE1_J 0b010100000000000
#define BYTE1_K 0b010110000000000
#define BYTE1_L 0b011000000000000
#define BYTE1_M 0b011010000000000
#define BYTE1_N 0b011100000000000
#define BYTE1_O 0b011110000000000
#define BYTE1_P 0b100000000000000
#define BYTE1_Q 0b100010000000000
#define BYTE1_R 0b100100000000000
#define BYTE1_S 0b100110000000000
#define BYTE1_T 0b101000000000000
#define BYTE1_U 0b101010000000000
#define BYTE1_V 0b101100000000000
#define BYTE1_W 0b101110000000000
#define BYTE1_X 0b110000000000000
#define BYTE1_Y 0b110010000000000
#define BYTE1_Z 0b110100000000000

#define BYTE2_A 0b0000100000
#define BYTE2_B 0b0001000000
#define BYTE2_C 0b0001100000
#define BYTE2_D 0b0010000000
#define BYTE2_E 0b0010100000
#define BYTE2_F 0b0011000000
#define BYTE2_G 0b0011100000
#define BYTE2_H 0b0100000000
#define BYTE2_I 0b0100100000
#define BYTE2_J 0b0101000000
#define BYTE2_K 0b0101100000
#define BYTE2_L 0b0110000000
#define BYTE2_M 0b0110100000
#define BYTE2_N 0b0111000000
#define BYTE2_O 0b0111100000
#define BYTE2_P 0b1000000000
#define BYTE2_Q 0b1000100000
#define BYTE2_R 0b1001000000
#define BYTE2_S 0b1001100000
#define BYTE2_T 0b1010000000
#define BYTE2_U 0b1010100000
#define BYTE2_V 0b1011000000
#define BYTE2_W 0b1011100000
#define BYTE2_X 0b1100000000
#define BYTE2_Y 0b1100100000
#define BYTE2_Z 0b1101000000

#define BYTE3_A 0b00001
#define BYTE3_B 0b00010
#define BYTE3_C 0b00011
#define BYTE3_D 0b00100
#define BYTE3_E 0b00101
#define BYTE3_F 0b00110
#define BYTE3_G 0b00111
#define BYTE3_H 0b01000
#define BYTE3_I 0b01001
#define BYTE3_J 0b01010
#define BYTE3_K 0b01011
#define BYTE3_L 0b01100
#define BYTE3_M 0b01101
#define BYTE3_N 0b01110
#define BYTE3_O 0b01111
#define BYTE3_P 0b10000
#define BYTE3_Q 0b10001
#define BYTE3_R 0b10010
#define BYTE3_S 0b10011
#define BYTE3_T 0b10100
#define BYTE3_U 0b10101
#define BYTE3_V 0b10110
#define BYTE3_W 0b10111
#define BYTE3_X 0b11000
#define BYTE3_Y 0b11001
#define BYTE3_Z 0b11010


// {{{ instructions are encoded into integer a + b + c
#define CMD_AMN BYTE1_A + BYTE2_M + BYTE3_N
#define CMD_AMX BYTE1_A + BYTE2_M + BYTE3_X
#define CMD_CFG BYTE1_C + BYTE2_F + BYTE3_G
#define CMD_DEB BYTE1_D + BYTE2_E + BYTE3_B
#define CMD_DEF BYTE1_D + BYTE2_E + BYTE3_F
#define CMD_ECH BYTE1_E + BYTE2_C + BYTE3_H
#define CMD_ETH BYTE1_E + BYTE2_T + BYTE3_H
#define CMD_EXE BYTE1_E + BYTE2_X + BYTE3_E
#define CMD_FXA BYTE1_F + BYTE2_X + BYTE3_A
#define CMD_FXB BYTE1_F + BYTE2_X + BYTE3_B
#define CMD_HIB BYTE1_H + BYTE2_I + BYTE3_B
#define CMD_LBL BYTE1_L + BYTE2_B + BYTE3_L
#define CMD_LCF BYTE1_L + BYTE2_C + BYTE3_F
#define CMD_LLN BYTE1_L + BYTE2_L + BYTE3_N
#define CMD_LNK BYTE1_L + BYTE2_N + BYTE3_K
#define CMD_LOA BYTE1_L + BYTE2_O + BYTE3_A
#define CMD_LST BYTE1_L + BYTE2_S + BYTE3_T
#define CMD_LSC BYTE1_L + BYTE2_S + BYTE3_C
#define CMD_MAP BYTE1_M + BYTE2_A + BYTE3_P
#define CMD_MEM BYTE1_M + BYTE2_E + BYTE3_M
#define CMD_PUT BYTE1_P + BYTE2_U + BYTE3_T
#define CMD_RES BYTE1_R + BYTE2_E + BYTE3_S
#define CMD_RNG BYTE1_R + BYTE2_N + BYTE3_G
#define CMD_SAV BYTE1_S + BYTE2_A + BYTE3_V
#define CMD_SET BYTE1_S + BYTE2_E + BYTE3_T
#define CMD_SNM BYTE1_S + BYTE2_N + BYTE3_M
#define CMD_SUB BYTE1_S + BYTE2_U + BYTE3_B
#define CMD_TOL BYTE1_T + BYTE2_O + BYTE3_L
#define CMD_TST BYTE1_T + BYTE2_S + BYTE3_T
#define CMD_UNL BYTE1_U + BYTE2_N + BYTE3_L
#define CMD_UPT BYTE1_U + BYTE2_P + BYTE3_T
#define CMD_VER BYTE1_V + BYTE2_E + BYTE3_R
#define CMD_WMX BYTE1_W + BYTE2_M + BYTE3_X
#define CMD_WMN BYTE1_W + BYTE2_M + BYTE3_N
#define CMD_GRP BYTE1_G + BYTE2_R + BYTE3_P
#define CMD_SOP BYTE1_S + BYTE2_O + BYTE3_P
#define CMD_CSS BYTE1_C + BYTE2_S + BYTE3_S
// }}}

// {{{
#define INS_END 0           //!< Exit the program
#define INS_SET 83          //!< Set the value of the port
#define INS_AND 97          //!< Logic AND
#define INS_OR 111          //!< Logic OR
#define INS_DECREASE 68     //!< Decrease the value of the port
#define INS_INCREASE 73     //!< Increase the value of the port
#define INS_JMP 74          //!< Jump to another instruction
#define INS_JIF_ISZERO 90   //!< Jump if port value is zero
#define INS_JIF_NOTZERO 78  //!< Jump if port value is not zero
#define INS_STORE 115       //!< Save the value to main register
#define INS_JIF_GT 62       //!< Jump if value is greater than (main register)
#define INS_JIF_LT 60       //!< Jump if value is greater than (main register)
// }}}




/** \page EEPROM EEPROM Map
 The EEPROM in Arduino Duemilanove with ATMEGA328 has 1024Bytes availables.
 Domino uses this space to store all the configuration variables required,
 organized in the following segments:

 -# Ports configuration: starting from the first byte (\ref EMPORTSOFFSET), with
	13 Bytes (\ref EMPORTSLOT) for each port, it contains the alias and type of
	each port.
 -# Virtual port information: starting from \ref EMVPORTOFFSET, contains the
	data of extended ports (virtual and grouped ports)
 -# Links: starting after the ports segment (\ref EMLINKSOFFSET) contains the
	index of two ports and a third byte for the type. 
 -# Network information: starting from \ref EMNETCFOFFSET, contains 20 bytes
	with the network information (MAC(6) + IP(4) + GW(4) + MASK(4))
 -# Board information: starting in \ref EMBOARDOFFSET contains some information
        about the board, like the name
*/


#define DIGITALPORTS    14	//!< Number of digital ports
#define ANALOGPORTS     6	//!< Number of analog ports
#define VIRTUALPORTS    30	//!< Number of virtual ports
#define EMPORTSLOT 	13	//!< Reserved bytes for each port
#define MAXLINKS 	30	//!< Number of links.
#define EMLINKSLOT 	3	//!< Reserved bytes for each link

// EEPROM Memory distribution. Preprocessor calculation

#define EMPORTSOFFSET   0
#define EMAPORTSOFFSET 	(EMPORTSOFFSET +(DIGITALPORTS * EMPORTSLOT))   	//=> 14x13 =182
#define EMVPORTSOFFSET 	(EMAPORTSOFFSET + (ANALOGPORTS * EMPORTSLOT))  	//=> 182+(6X13)=260
#define EMLINKSOFFSET  	(EMVPORTSOFFSET + (VIRTUALPORTS * EMPORTSLOT)) 	//=> 260+(30X13)=650
#define EMNETCFOFFSET 	(EMLINKSOFFSET + (MAXLINKS * EMLINKSLOT)) 	//=> 650+(30X3)=740
#define	EMBOARDOFFSET 	(EMNETCFOFFSET+20)                              //=> 750+20=770

// EEPROM position data ports slot.
#define EMPOSTYPEPORT 	5
#define EMPOSVISIBILITY 6
#define EMPOSTYPEVIRTUAl 7
#define EMPOSPARAM1 	8
#define EMPOSPARAM2 	10
#define EMPOSPARAM3 	12	

#define EMSEGMENTS 50		//!< Number of segments in \ref EEPROM
#define DELAYCYCLE 100		//!< Delay in each cycle, in milliseconds
#define BUFFERSIZE 50		//!< Maximum lenght for the command
#define MAXCHANGES 6		//!< Maximum number of changes allowed per second



/// Encoded values
#define ANALOG  65		//!< Alias for ANALOG    ! ANALOG  DIGITAL
#define DIGITAL 68		//!< Alias for DIGITAL --+----------------
#define IN      73		//!< Alias for INPUT     !   a       d
#define OUT     79		//!< Alias for OUTPUT    !   A       D
//#define OFF      0		//!< Alias for OFF
//#define ON    HIGH		//!< Alias for ON

#define TIMEPULSE 5
///! Event types
enum eventType {
	DEBUG = 68,
	NOTICE = 78,
	WARNING = 87,
	ALARM = 65,
	ERROR = 69,
	INFO = 73
};

#define CONTENT_TYPE_TEXTHTML 1
#define CONTENT_TYPE_CSS 2


//! Delete the specified link
#define emptyLink(i) {links[i][0]=0; links[i][1]=0; links[i][2]=0;}
//! Total amount of ports
#define TOTALPORTS (DIGITALPORTS + ANALOGPORTS + VIRTUALPORTS)

// This macros will make a little easier to work with the memory structures
//! Return TRUE if the specified port is an INPUT port
#define ISINPUT(n)   ((ports[n].type=='a')||(ports[n].type=='d')||(ports[n].type=='p'))
//! Return TRUE if the specified port is an OUTPUT port
#define ISOUTPUT(n)  ((ports[n].type=='A')||(ports[n].type=='D')||(ports[n].type=='P'))
//! Return TRUE if the specified port is an ANALOG port
#define ISANALOG(n)  ((ports[n].type=='a')||(ports[n].type=='A'))
//! Return TRUE if the specified port is a DIGITAL port
#define ISDIGITAL(n) ((ports[n].type=='d')||(ports[n].type=='D'))
//! Return TRUE if the specified port is a PULSE port
#define ISPULSE(n)   ((ports[n].type=='p')||(ports[n].type=='P'))
//! Return TRUE if the specified port is virtual
#define ISVIRTUAL(n) (n>=ANALOGPORTS+DIGITALPORTS)
//! Return TRUE if the specified port is disabled
#define ISDISABLED(n) ((ports[n].type=='x')||(ports[n].type=='X'))

//! Copy the contents of the EEPROM specified region into the string
//#define EEPROM2string(offset,lenght,string) for(i=0;i<lenght;i++) \
//	{string[i]=EEPROM.read(offset+i);}

// lnk puls1 alarm
// 012345678901234
//! Copy the first argument into the string
#define GETARG1(str,arg1) arg1[0]=str[4];arg1[1]=str[5];arg1[2]=str[6];\
	arg1[3]=str[7];arg1[4]=str[8];arg1[5]=0;
//! Copy the second argument into the string
#define GETARG2(str,arg2) arg2[0]=str[10];arg2[1]=str[11];arg2[2]=str[12];\
	arg2[3]=str[13];arg2[4]=str[14];arg2[5]=0;
//! Return TRUE if the byte is a valid character (a-z, A-Z and 0-9)
#define ISVALIDCHAR(a) ((a>=48 && a<=57) || (a>=65 && a<=90) || (a>=97 && a<=122))


//! Data structure for ports
struct portStruct {
	byte type;			///< Type of the port (a, d, A, D, v, ...)
	byte value;			///< Last value of the port
	byte counter;
//	struct portRanges *range;	///< Ranges (only for analog ports)
//	struct extendedData *extra;	///< Additional data for virtual ports
} ports[TOTALPORTS];

//! Link table (3 bytes: port1, port2 and type)
byte links[MAXLINKS][3];

//! Board name
char bname[6];



// {{{ Flash Strings
const char strfmt_std_num[] PROGMEM = "%c:%s.%s %s %d\n"; ///< Standard string format for numbers
const char strfmt_std[] PROGMEM = "%c:%s.%s %s %s\n"; ///< Standard string format
const char strfmt_lnk[] PROGMEM = "%c:%s %s-%s %c\n"; ///< Standard string format for links
const char strfmt_cmdok[] PROGMEM = "N:board cmdok";
const char strfmt_error[] PROGMEM = "E:board error ";

#define ON "ON"
#define OFF "OFF"
const char type_link[] PROGMEM = { 'd', 'i','p', 'c', 'r', 't','s','n','N','o','f','l', 0x00};

// {{{
const char str_upsec[] PROGMEM = "upsec";
const char str_upday[] PROGMEM = "upday";

const char str_versn[] PROGMEM = "versn";
const char str_bdate[] PROGMEM = "bdate";
const char str_build[] PROGMEM = "build";

const char str_mfree[] PROGMEM = "mfree";
const char str_mallo[] PROGMEM = "mallo";
const char str_mused[] PROGMEM = "mused";

const char str_value[] PROGMEM = "value";
const char str_empty[] PROGMEM = "empty";
// }}}

#ifdef ENABLE_NETWORKING
const char str_http_header_1[] PROGMEM = "<html>\n<head><title>Domino</title><meta name=";
const char str_http_header_2[] PROGMEM = "viewport content='width=320'/><link rel=styles";
const char str_http_header_3[] PROGMEM = "heet type='text/css' href='http://cloud.opendo";
const char str_http_header_4[] PROGMEM = "mo.com/odctp/ar.css' /><script src='http://cl";
const char str_http_header_5[] PROGMEM = "oud.opendomo.com/odctp/ar.js'></script></head>";
const char str_http_header_6[] PROGMEM = "<body><div id=header></div><ul id=frm class=lst>";

const char str_http_footer_1[] PROGMEM = "</ul><a id=ftr href=http://opendomo.com>v";
const char str_http_footer_2[] PROGMEM = " </a></body></html>";
#endif

byte h2d(char a, char b) __attribute__((noinline));
void print_error(byte out, int code) __attribute__((noinline));
void print_cmdok(byte out) __attribute__((noinline));
boolean setPortValue(char id, int value) __attribute__((noinline));
boolean processInstruction(const char *cmd) __attribute__((noinline));
void writef(byte out, const char* fmt, ...);
//char *flstr(const char *flash_str) __attribute__((noinline));
char * itoan(int val, char *result, byte len);
#ifdef ENABLE_NETWORKING
int ethSetIP(byte ipb1, byte ipb2, byte ipb3, byte ipb4);
int ethSetMAC(byte macb3, byte macb4, byte macb5, byte macb6);
int ethSetGW(byte ipb1, byte ipb2, byte ipb3, byte ipb4);
int ethSetNetmask(byte ipb1, byte ipb2, byte ipb3, byte ipb4);
void sendODControlHead();
void sendODControlAnnouncement();
void sendODControlUpdate(int pnum);

//! Network web server instance
EthernetServer Webserver(80);

//! Telnet and UDP server instances
EthernetServer Telnet(UDP_PORT);
EthernetUDP Udp;
#endif

char instruction[BUFFERSIZE];	///< Command received

bool echo = false;		///< Flag that defines if the local echo is activated
bool debug = false;		///< Flag that defines if the board operates in debug mode
byte alarmport = 0;		///< Alarm port configured (0 if none)
byte pulseport = 0;		///< Pulse port configured (0 if none)

///! Seconds in a day
#define DAYLENGHT 86400
unsigned int days = 0;		///< Uptime days
unsigned long seconds = 0;	///< Uptime seconds (after a day, it rolls back to zero)

// {{{ owrite()
int owrite(byte out, const char *str)
{
	switch (out)
	{
#ifdef ENABLE_NETWORKING
		case TELNET:
			Telnet.print(str);
		break;

		case HTTP:
			Webserver.print(str);
		break;
#endif

		case SERIALPORT:
			Serial.print(str);
		break;

	}
}
// }}}

/*
  
*/
boolean validTypeLink (char type ){
  char t;
  byte k=0;
  do{
    t=pgm_read_byte(type_link + k);
    if (type==t) return true;
    else k++;
  }while (t != 0x00);
  
  return false;
}


	


void print_error(byte out, int code)
{
   	char buffer[16];
 	flstrn(strfmt_error,buffer,16);
	switch (out)
	{
#ifdef ENABLE_NETWORKING
	case TELNET:

		Telnet.print(buffer);
		Telnet.println(code);
		break;
#endif
	case SERIALPORT:
		Serial.print(buffer);
		Serial.println(code);
		break;
	}
}

void print_cmdok(byte out)
{
 	char buffer[16];
 	flstrn(strfmt_cmdok,buffer,16);
	switch (out)
	{
#ifdef ENABLE_NETWORKING  
	case TELNET:
		Telnet.println(buffer);
		break;
#endif
	case SERIALPORT:
		Serial.println(buffer);
		break;
	}
}



// {{{ freeMemory(): return the number of bytes currently free in RAM
extern int __bss_end;
extern int *__brkval;
int freeMemory()
{
	int free_memory;
	if ((int)__brkval == 0)
		free_memory = ((int)&free_memory) - ((int)&__bss_end);
	else
		free_memory = ((int)&free_memory) - ((int)__brkval);
	return free_memory;
}
// }}}

int freeMalloc() {
  int size = 2000; // Use 2048 with ATmega328
  byte *buf;

  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);

  return size;
}


// {{{ printVersion(): Print the board version to the standard output
void printVersion()
{
	writef(output, "%s versn %s %s %s\n", bname, VERSION, __TIME__, __DATE__);
}
// }}}

void printUptime(){
	char buffer[10];
	char format[16];
        flstrn(strfmt_std_num,format,16);
	writef(output, format, INFO, bname, flstrn(str_upsec,buffer,10), "", seconds);
	writef(output, format, INFO, bname, flstrn(str_upday,buffer,10), "", days);
}

void printMemory(){
	char buffer[10];
	char format[16];
        flstrn(strfmt_std_num,format,16);
	writef(output, format, INFO, bname, flstrn(str_mfree,buffer,10), "", freeMemory());
	writef(output, format, INFO, bname, flstrn(str_mallo,buffer,10), "", freeMemory());
	writef(output, format, INFO, bname, flstrn(str_mused,buffer,10), "", sizeof(ports));
}



// {{{ itoan(): integer to string
/** Transforms a integer into a string with lenght LEN
 \param val Integer value
 \param result String where the result will be written
 \param len Lenght of the string
 \return true if the operation was succeeded
*/
char *itoan(int val, char *result, byte len)
{
	int i = val;

	result[--len] = 0;
	while (len != 0) {
		len--;
		result[len] = ((i%10)+48);
		i /= 10;
	}
	return result;  
}
// }}}

// {{{ h2d(): hexadecimal chars to decimal
/** Transforms a 2 bytes hexadecimal string into its integer value
  Example: h2d(E,F)=239
 \param a First byte of the source string
 \param b Second byte of the source string
 \return The integer value
*/
byte h2d(char a, char b)
{
	if (a >= 65 && a <= 70)
		a = a - 55;
	else
		a = a - 48;
	if (b >= 65 && b <= 70)
		b = b - 55;
	else
		b = b - 48;
	return (a * 16 + b);
}

// }}}

/*Cambia el estado de las salidas con un tiempo activo.
  El tipo de enlace al que pertenece determina la accion.
  \
  \Entrada: ID del puerto que ha llegado a cero.
  \Salida: Vacio

*/
void timePortEnd(byte port)
{
      int i;
      
      for (i = 0; i < MAXLINKS; i++){
        if (links[i][1] == port){
          switch (links[i][2]){
            case 'r':
            case 't':
            case 's':
              setPortValue(links[i][1], LOW);            
            break;
            
            case 'l':
              
                if(ports[(links[i][0])].counter<10) ports[(links[i][0])].counter+=10;
              
                switch (ports[(links[i][0])].counter){
                      case 11:
                          if ( ports[(links[i][1])].value < 245){
                            setPortValue(links[i][1], ports[(links[i][1])].value+=5);
                            ports[(links[i][1])].counter=2;    // Recarga timer proximo incremento
                          }
                          else{
                            setPortValue(links[i][1], ports[(links[i][1])].value=255);
                          }
                      
                      break;
                      case 12:
                          if (ports[(links[i][1])].value>0)
                                setPortValue(links[i][1], ports[(links[i][1])].value=0);
                          else 
                              setPortValue(links[i][1], ports[(links[i][1])].value=255);
                              
                          ports[(links[i][0])].counter=0;
                      break;
                      
                      case 13:
                          if (ports[(links[i][1])].value > 10){
                            setPortValue(links[i][1], ports[(links[i][1])].value-=5);
                            ports[(links[i][1])].counter=2;
                          }
                          else{
                            setPortValue(links[i][1], ports[(links[i][1])].value=0);                 
                          }                       
                      break;
                      }
                break;
            
            
          }          
        }        
      }
}

/** This function triggers an event if a port value has changed.
 \param port Port number
 \param ov Old value of the port
 \param nv New value of the port
*/
void triggerPortChange(byte port, byte ov, byte nv)
{
	char buffer[20];
	char format[20];
	flstrn(strfmt_std_num,format,20);
	char pname[6];
	int i;
	
	// Variables y macros tipo de flanco Differtial UP or Differettial Down
	byte dif=0;
	#define DIFU  2
	#define DIFD  1
	#define ISDIFU  (dif==DIFU)
	#define ISDIFD  (dif==DIFD)
	#define CHANGED (!dif==0)          // Se usa por seguridad no es necesario.
	
	if (ov > nv) dif=DIFD;            // Solves differential change, Up or Down
	else if (ov < nv) dif=DIFU;
	
	if (dif == 0) return;             // Value didn't change return.
        
        
	eeprom_get_str(pname, port*EMPORTSLOT, 6);
	if (ISINPUT(port) && ISANALOG(port)) {
	  //if (ports[port].type =='a') {
	  //	Serial.print("D:");
	  //	Serial.print(pname);
	  //	Serial.print(ov,DEC);
	  //	Serial.print(" --> ");
	  //	Serial.println(nv,DEC);
	  // Si el valor supera el limite impuesto, y esta subiendo, disparar alarma
	  //i = port-DIGITALPORTS;}
	}
  
        if (ISINPUT(port) && ISDIGITAL(port))
        {
      		// Si un puerto digital ha cambiado y tiene un temporizador asignado, le
      		// asignamos al contador el valor correspondiente.
      		//if (t_len[port]>0 && t_count[port]==0 && nv==HIGH) t_count[port] = t_len[port];
      		flstrn(strfmt_std,format,20);
      		if (output!=HTTP){
      		writef(output,format,NOTICE, bname,pname,flstrn(str_value,buffer,10), (nv==1)?ON:OFF);
      		}
        }
        
        // Puertos enlazados
        
        for (i = 0; i < MAXLINKS; i++){
          if (links[i][0] == port){
                        
            // Enlades directos puede ser analogicos o digitales.
            // Directo: si A=1, B=1. Si A=0, B=0. Si A=65%, B=65%
            if (links[i][2]=='d')setPortValue(links[i][1], nv);
            
            // Tratamiento en el caso de puerto destino digital.
            else if (ISDIGITAL(links[i][1])){
              switch (links[i][2]){
                
                // Inverso: si A=1, B=0. Si A=0, B=1.

                case 'i':
                  if (ISDIFD) setPortValue(links[i][1], HIGH);
                  else if (ISDIFU) setPortValue(links[i][1], LOW);
                  /// @todo Activate inverted link for analog ports
                  //else
                  //setPortValue(links[i][1],255-nv);
                break;
                
                case 'p':
                  if (ISDIFU){
                    if (ports[links[i][1]].value == LOW)setPortValue(links[i][1], HIGH);
                    else setPortValue(links[i][1], LOW);
                  }
                break;
               
                case 'c':
                  if (CHANGED){
                    if (ports[links[i][1]].value == LOW)setPortValue(links[i][1], HIGH);
                    else setPortValue(links[i][1], LOW);
                  }
                break;
                
                case 'r':
                  if (CHANGED && ports[(links[i][0])].counter==0){
                    setPortValue(links[i][1], HIGH);
                    ports[(links[i][1])].counter=TIMEPULSE;
                  }
                break;  
                
                case 't':
                  if (ISDIFU && ports[(links[i][0])].counter==0){
                    setPortValue(links[i][1], HIGH);
                    ports[(links[i][1])].counter=TIMEPULSE;
                  }
                break;
                
                case 's':
                  if (ISDIFD){
                    setPortValue(links[i][1], HIGH);
                    ports[(links[i][1])].counter=TIMEPULSE;
                  }
                  else if(ISDIFU){
                    setPortValue(links[i][1], LOW);
                    ports[(links[i][0])].counter=0;
                  }
                break;
                
                case 'o':
                  if (CHANGED)setPortValue(links[i][1], HIGH);                  
                break;
                
                case 'f':
                  if (CHANGED)setPortValue(links[i][1], LOW);                  
                break;      
                
                default:
                  print_error(output,304);                 
                break;
              }
              
            }
            // Los enlaces con puerto destino Analogicos tienen otro tratamiento.
            else if(ISANALOG(links[i][1])){
              switch (links[i][2]){  
                       
                case 'n':
                  if (ISDIFD)
                    setPortValue(links[i][1], ports[(links[i][1])].value+=1);               
                break;
                
                case 'N':
                  if (ISDIFU){
                    setPortValue(links[i][1], ports[(links[i][1])].value+=1);
                  }             
                break;
                
                case 'l':                  
                  //  Los pulsos son contados y se guardan en el registro counter perteneciente
                  // al puerto de entrada.
                  //
                  // Los counter perteneciente a puertos de salida siempre son de tiempo y tratados por
                  // la función timePortChange.
                  //
                  //  Despues de agotar el primer tiempo se suma 10 al counter entrada (contador de pulsos) 
                  // el tratamiento del enlace lo realizara timePortChange hasta llegar a los limites o 
                  // nuevo cambio en puerto de entrada.
              
               
                  if (CHANGED && ports[(links[i][0])].counter<10 )
                  {
                   
                    ports[(links[i][0])].counter+=1;
                    
                    if (ports[(links[i][0])].counter == 4 )
                    {
                      ports[(links[i][1])].counter=0;              // Fin 
                      ports[(links[i][0])].counter=0; 
                    }     
                   
                    else
                    {
                      ports[(links[i][1])].counter=TIMEPULSE*2;                 
                    }                                   
                  }
                  else if (ISDIFU && ports[(links[i][0])].counter>10)
                  {
                    ports[(links[i][1])].counter=0;              // Fin 
                    ports[(links[i][0])].counter=0;
                  }
                  
                break;                                
                  
                 
                default:
                  print_error(output,304);
                break;
                }
            }
          }
        }
}

        



// {{{ setPortValue(): set port value
/** Set a value for the specified port
 \param i Port index (see \ref getPortId)
 \param value New value
 \return true if the value could be assigned, false if not.
*/
boolean setPortValue(char id, int value)
{
	char pname[6];
	char buffer[20];
	eeprom_get_str(pname, id*EMPORTSLOT, 6);

	if (ISINPUT(id)) {
		print_error(output,107);
		return false;
	}

	if (ISDIGITAL(id)) {
		digitalWrite(id, value);
		if (output<HTTP)
			writef(output, flstrn(strfmt_std,buffer,20), NOTICE, bname, pname, "value", value?ON:OFF,"");
	} else if (ISANALOG(id)) {
		analogWrite(id, value);		
		if (output<HTTP)
			writef(output, flstrn(strfmt_std_num,buffer,20), NOTICE, bname, pname, "value", value);
	} else if (ISDISABLED(id)) {
		// Nothing. No warning neither
	} else {
		print_error(output,103);
		return false;
	}

        if (ISVIRTUAL(id)){
                ports[id].value = value;                
        }
#ifdef ENABLE_NETWORKING
        sendODControlHead();
        sendODControlUpdate(id);
        Udp.endPacket();         
#endif
	
	return true;
}

// }}}

// {{{ resetPorts()
/** Set the type and configured characteristics to each port
*/
void resetPorts()
{
	char i;
	for(i=0;i<TOTALPORTS;i++) {
		/// @todo Alta impedancia if type = "-"
		if (ISINPUT(i)) {
			pinMode(i, INPUT);
			digitalWrite(i, HIGH);	// Activating internal pull-up resistor

			if (ISDIGITAL(i))
				ports[i].value = digitalRead(i);

			if (ISANALOG(i))
				ports[i].value = 0;
			//(analogRead(i-DIGITALPORTS) * 2.5 * ranges[i-DIGITALPORTS].fxB / 1023 ) - ranges[i-DIGITALPORTS].fxA;
		} else if (ISOUTPUT(i)) {
			digitalWrite(i, LOW);	// deactivating pull-up
			pinMode(i, OUTPUT);
		}
		if (ISPULSE(i)) {
			//pinMode(i, INPUT);
			pulseport = i;
			//noInterrupts();

		}
		//! @todo Enable wake up with attachInterrupt(0, wakeUp, LOW);
	}
}

// }}}

/** Store the specified name of the board in the EEPROM
 \param name New board name (5 Bytes)
 \return Always 0
*/
int saveBoardName(char *name)
{
	strlcpy(bname, name, 6);
	eeprom_set_str(EMBOARDOFFSET, name, 5);
	return true;
}

/// Load the configuration from the EEPROM
int loadConfig()
{
	unsigned char x;
	short dir;

	// Read the board's name (and set a default one if missing)
	eeprom_get_str(bname, EMBOARDOFFSET, 6);

	for(x=0;x<TOTALPORTS;x++) {
		dir = int (EMPORTSOFFSET + x * EMPORTSLOT);
		ports[x].type = eeprom_get_byte(dir +  EMPOSTYPEPORT);

		if (!ISVALIDCHAR(ports[x].type))
			ports[x].type = 'x';

		if (eeprom_get_byte(dir + 7) == 'a')
			alarmport = x;
	}

	for (x = 0; x < MAXLINKS; x++) {
		links[x][0] = eeprom_get_byte(EMLINKSOFFSET + x * EMLINKSLOT);
		links[x][1] = eeprom_get_byte(EMLINKSOFFSET + x * EMLINKSLOT + 1);
		links[x][2] = eeprom_get_byte(EMLINKSOFFSET + x * EMLINKSLOT + 2);

		if (links[x][0] > TOTALPORTS)
			links[x][0] = 0;
		if (links[x][1] > TOTALPORTS)
			links[x][1] = 0;
		if (!ISVALIDCHAR(links[x][2]))
			links[x][2] = 'x';
	}

	return true;
}

/// Save the configuration into the EEPROM (see \ref EEPROM Map)
void saveConfig()
{
	unsigned char x;
	short dir;

	for(x=0;x<TOTALPORTS;x++) {
		dir = int (EMPORTSOFFSET + x * EMPORTSLOT);
		eeprom_set_byte(dir + EMPOSTYPEPORT, ports[x].type);

		if ((x == alarmport) && (alarmport!=0)) {
			eeprom_set_byte(dir + 7, 'a');
		} else {
			eeprom_set_byte(dir + 7, ' ');
		}
	}
	for (x = 0; x < MAXLINKS; x++) {
		eeprom_set_Link(x);
			
	}	
}

/** Set a port's configuration (and name)
 \param id Port id
 \param cfg String with the configuration
 \param name New port name (optional)
 \return 0 if everything went right. Otherwise, -1
*/
boolean configPort(char id, char *cfg, char *name)
{
        int offsetSlot;
        offsetSlot=EMPORTSLOT * id;
        char auxType='?';
        char auxVisi='?';
	if (id == -1)
		return false;	// Ignoramos ID =-1 (puerto no encontrado)

	byte i = 0;
	if (cfg != NULL) {
		
                while (cfg[i] >='-') {
			switch (cfg[i]) {
                        
                        case 0:
			case 'x':
			case 'X':
                                auxType = 'X';
				auxVisi = 'C'; // Visible en la segunda pantalla.
                                break;
			case '-':
                                auxType = 'X';
				auxVisi = 'H'; // Ocultamos???
				break;
                        case 'h':
                              auxVisi = 'H';
                              break;
                        
                        case 's':
                              auxVisi = 'M';
                              break;
                        
                        case 'S':
                              auxVisi = 'C';
                              break;
                              

			case 'p':
				pulseport = id;
				auxType = 'P';
				break;

			case 'A':
				alarmport = id;
				break;

			case 'd':
				auxType = 'D';
				break;

			case 'a':
				auxType = 'A';
				break;

			case 'i':
				if (auxType == 'D')
					auxType = 'd';
				if (auxType == 'A')
					auxType = 'a';
				if (auxType == 'P')
					auxType = 'p';
				break; 

			case 'o':
				break;


			default:
                                print_error(output,22);
                                return false;
#ifdef MODULE_DEBUG                                
				if (debug == 1)
					writef(output,
					       "%c:invalidcfg(%d) cfg:%s \n",
					       DEBUG, id, cfg);
#endif
			}
			i++;
		}
                if (auxType != '?'){
                  eeprom_set_byte(offsetSlot + EMPOSTYPEPORT, auxType);
                  ports[id].type=auxType;
                }
                  
                if (auxVisi != '?')eeprom_set_byte(offsetSlot + EMPOSVISIBILITY, auxVisi);
                
               
                
        }
	debug_write(id);
	debug_write(cfg);
	debug_write(auxType);
        if (name != NULL){
           for (i = 0; i < 5; i++){
             if (ISVALIDCHAR(name[i]))eeprom_set_byte(offsetSlot + i, name[i]);
             else eeprom_set_byte(offsetSlot + i, 'x');
           }
			
	}

	return true;
}


/** Load the default configuration.
 Useful if the EEPROM was empty or corrupted or before loading a template.
*/
void loadDefaultConfig()
{
	byte i;
	char pname[6];
	
	// Links
	// No links in default configuration	
	for (i=0;i<MAXLINKS;i++){
		emptyLink(i);			
	}
	
	// Ports
	configPort(2, (char *)"dis", "di002");
	configPort(3, (char *)"dis", "di003");
	
	configPort(5, (char *)"dis", "di005");
	configPort(6, (char *)"dos", "do006");
	configPort(7, (char *)"dos", "do007");
	configPort(8, (char *)"dos", "do008");
	configPort(9, (char *)"dos", "do009");
        
	// Disable Ethernet and SD 
        configPort(0,  (char *)"-", "xxxrx");  // SD Select
        configPort(1,  (char *)"-", "xxxtx");  // SD Select
	configPort(4,  (char *)"-", "xxx01");  // SD Select
	configPort(10, (char *)"-", "xxx02");  
	configPort(11, (char *)"-", "xxx03");
	configPort(12, (char *)"-", "xxx04");
	configPort(13, (char *)"-", "xxx05");

	// Disable last 4 ADC
	configPort(14, (char *)"ais", "ai000");
	configPort(15, (char *)"ais", "ai001");
	configPort(16, (char *)"x",  "ai002");
	configPort(17, (char *)"x",  "ai003");
	configPort(18, (char *)"x",  "ai004");
	configPort(19, (char *)"x",  "ai005");
	
	// Virtuals ports
	for (i=0; i<MAXLINKS; i++){
		itoan(i, pname, 6);
		pname[0] = 'v';
                pname[1] = 't';
		configPort(i + (DIGITALPORTS + ANALOGPORTS), (char*)"x", pname);         
	}
	

#ifdef ENABLE_NETWORKING
	// Default network configuration
	ethSetIP(defip[0], defip[1], defip[2], defip[3]);
	ethSetMAC(mac[0], mac[1], mac[2] , mac[3] );
	ethSetGW(defip[0], defip[1], defip[2], 1);
	ethSetNetmask(255, 255, 255, 0);
#endif
	// Guarda configuración en Eeprom.
	saveConfig();
}

/// This function reads the command from the serial port
boolean readFromSerialPort(char *ins)
{
	byte b;
	byte i=0;
	// El dispositivo Arduino estara permanentemente iterando este bucle,
	// este o no conectado a traves del puerto USB. En el caso de que haya
	// conexion establecida, se tendra en cuenta.

	// Serial.available() devuelve el numero de bytes acumulados en el buffer.
	// Empleando un sistema de ancho fijo de los comandos facilitaremos en
	// gran medida el procesado
	while (Serial.available() > 0) {
		b = Serial.read();
		switch (b) {
		case -1:	// Null char
		case 0:	        // Null char
		case 10:	// LF: No deberia suceder, salvo en consola emulada
			break;

		case 13:	// Final de linea
			output = SERIALPORT;
			i = 0;
                        if (echo == 1) Serial.println();
 			return true; 
			break;
		case 8:	        // Backspace
			if (i > 0) i--;				
			ins[i] = 0;
			break;
		default:
			if (i < BUFFERSIZE) {
				if (echo == 1) Serial.print(char(b));
				ins[i] = b;
				i++;
				ins[i] = 0;
			} 
                        else 
                          print_error(output,2);			
		}
	}
	return false;
}




/// Check if any port has changed its value, and trigger an event if required.
void refreshPortStatus()
{
	byte i = 0;
	byte k = 0;
	int val;
	char pname[6];
	byte param1, param2, param3;
	
	for(i=0;i<TOTALPORTS;i++) {
		val = 0;
		// Virtual ports: special case
		if (ISVIRTUAL(i)) {
			param1 = eeprom_get_byte(EMPORTSOFFSET + i * EMPOSPARAM1);
			param2 = eeprom_get_byte(EMPORTSOFFSET + i * EMPOSPARAM2);
			param3 = eeprom_get_byte(EMPORTSOFFSET + i * EMPOSPARAM3);
			
			if (param3 == INS_INCREASE) {
				ports[i].value = ports[param1].value + ports[param2].value;			
			}
			else if (param3 == INS_AND) {
				ports[i].value = ports[param1].value * ports[param2].value;
			}
			else if (param3 == INS_OR) {
				ports[i].value = ports[param1].value + ports[param2].value;
			}
		} 
		
		else if (ISOUTPUT(i)) {
		
			if(ports[i].counter!=0){			
				if (ports[i].counter==1)timePortEnd(i);
				ports[i].counter-=1;
			}
			
			if (ISDIGITAL(i)) val = digitalRead(i);
			else val = ports[i].value;
			
			if (ports[i].value != val)
				triggerPortChange(i, ports[i].value, val);
                                ports[i].value=val;
		} 
		
		else if (ISINPUT(i)) {
		
			
			 // Tratamiento puertos digitales.
			if (ISDIGITAL(i)) {
				// varias lecturas para minimizar problemas de ruido
				val = HIGH;				
				for (k = 0; k < 3; k++) {
					if (digitalRead(i) != HIGH) {
					val = LOW;
					break;
					}					
					delay(10);
				}
				
				if ((ports[i].value == LOW && val == HIGH)) {
					triggerPortChange(i, ports[i].value, HIGH);
					ports[i].value = HIGH;
				} 
				else if ((ports[i].value == HIGH && val == LOW )) {
					triggerPortChange(i, ports[i].value, LOW);
					ports[i].value = LOW;
				}
			}
			
			// Las entradas analogicas tienen un rango distinto
			if (ISANALOG(i)) {
				delay(5);
				//LDR
				if (i-DIGITALPORTS == 0) {
					val = analogRead(i - DIGITALPORTS);
					val = (2.5 * val) ;
				}
				// TEMPERATURE
				else {
				val = analogRead(i - DIGITALPORTS);
				val = (2.5 * val * 100.0) / 1024.0;
				val = val / 10;
				}
				
				triggerPortChange(i, ports[i].value, val);
				delay(5);
				ports[i].value = val;
			}				
		}
	}
}

/// Send the list of links created to the standard output
// Formato de salida port1:port2:type
// Ejemplo di002:do008:d
void listLinks()
{
	byte i = 0;
	byte a = 0;
	char pname1[6];
	char pname2[6];
	char buffer[20];
	
	for (i = 0; i < MAXLINKS; i++) {
		// Los "slots" vacÃ­os se almacenan como enlace "0-0"
		if ((links[i][0] != 0) && (links[i][1] != 0)) {
			eeprom_get_str(pname1, links[i][0]*EMPORTSLOT, 6);
			eeprom_get_str(pname2, links[i][1]*EMPORTSLOT, 6);
			writef(output, "%s:%s:%c\n", pname1, pname2, (char*)links[i][2]);
			a++;
		}
		
	}
	if (a == 0) print_cmdok(output);
//		writef(output, "W:no links\n");
}

/** Escribe en el buffer apuntado el valor actual del puerto.
    
    Entrada:  id del puerto.
              puntero a array de salida.              
    Salida:   Valor segun tipo, on, off, valor analogico con signo y decimal simulado.
*/
void get_state(byte id, char *buffer){
       
        int i;
	if (ISDIGITAL(id)) {  			
		if (ports[id].value != LOW) strcpy(buffer,"ON");
		else strcpy(buffer,"OFF");
                  
	}
	else if (ISANALOG(id)) {
                i=ports[id].value;
        	if (i<0){
        		strcpy(buffer, "-");
        		i= ~i ;
        	}
        	else strcpy(buffer, "+");
                strlcat(buffer , itoan(i, &buffer[1], 5), 5);
                strcat(buffer , ".0000");
	}
        else strcpy(buffer,"");   
}


/** Escribe en el buffer el tipo de puerto con parametros extendidos.
    
    Entrada:  id del puerto.
              puntero a array de salida.              
    Salida:   Tipo de puerto, pestaña donde es visible y notag simulado.
*/
void get_type(byte id, char *buffer ){        
	
	buffer[1]=ISINPUT(id)?'I':'O';
	if 
		(ISDIGITAL(id)) buffer[0] = 'D';
	else if 
		(ISANALOG(id)) buffer[0] = 'A';
	else{
		buffer[0] = 'X';
		buffer[1] = 'X';
	}
	
	if ISVIRTUAL(id){
                if (buffer[0] == 'X') buffer[0] = 'V';
                else buffer[1] = 'V';		
	}
        buffer[2]=eeprom_get_byte(EMPORTSLOT * id + EMPOSVISIBILITY);
        buffer[3]='_';          // Simulación respuesta ODControl.          	
}

/// Imprime la lista de puerto compactada.
void listCompact(){
	char buffer[41];
	char auxBuf[11];
	//ai000:AIH_:-0000.2437:00000|00001::00255
	//0123456789012345789012345678901234567890
	byte i;
	byte ini=0;
	byte fin=23;
	for (i=ini ; i<fin; i++){
		eeprom_get_str(auxBuf, i*EMPORTSLOT, 6);
		strlcpy(buffer, auxBuf, sizeof(buffer));
                strlcat(buffer, ":", sizeof(buffer));
                get_type(i, auxBuf);
                strncat(buffer, auxBuf ,4);
                strlcat(buffer, ":", sizeof(buffer));
                get_state(i, auxBuf);
                strlcat(buffer, auxBuf, sizeof(buffer));
                strlcat(buffer, "\n", sizeof(buffer));
            	writef(output, buffer);	
	}
	
	
}

// {{{ listPorts(): Print the list of ports, including type and value
void listPorts()
{
	byte i;
        byte ini=0;
	byte fin=23;       //TOTALPORTS   
	char buffer[30];
        char auxBuf[11];
	
	for(i=ini;i<fin;i++) {
		
                get_type(i, auxBuf);
                strlcpy(buffer, auxBuf , 3);
                strlcat(buffer, ":", sizeof(buffer));
                eeprom_get_str(auxBuf, i*EMPORTSLOT, 6);
                strlcat(buffer, auxBuf, sizeof(buffer));
                strlcat(buffer, ":", sizeof(buffer));
                get_state(i, auxBuf);
                strlcat(buffer, auxBuf, sizeof(buffer));
                strlcat(buffer, "\n", sizeof(buffer));
                writef(output, buffer); 
	}

}

// }}}

/**Guarda en Eeprom el Link.
  \Entradas : Nº link, puerto origen, puerto destino, tipo enlace.
  \Salida ; Vacio.

*/
void eeprom_set_Link(char lnk){
		eeprom_set_byte(EMLINKSOFFSET + lnk * EMLINKSLOT + 0, links[lnk][0]);
		eeprom_set_byte(EMLINKSOFFSET + lnk * EMLINKSLOT + 1, links[lnk][1]);
		eeprom_set_byte(EMLINKSOFFSET + lnk * EMLINKSLOT + 2, links[lnk][2]);      
      
}

/** Links two ports
 \param port1 Trigger port (usually input)
 \param port2 Triggered port (usually output)
 \return the link slot if the operation succeeded, 
         or -1 (invalid type port)
         or -2 (all buffer link used)
         or -3 (link already exists)       
*/
char addLink(char port1, char port2, char type)
{
	char l;
    
	if ((port1 == -1) || (port2 == -1))return -1;

	if ((ISINPUT(port1) && ISOUTPUT(port2)) || ISVIRTUAL(port1)) {
		for (l=0;l<MAXLINKS;l++) {
                        if ((links[l][0] == port1) && (links[l][1] == port2))	// the link already exists
			{
				return -2;

			}
			if ((links[l][0] == 0) && (links[l][1] == 0))	// If the slot is empty
			{
				links[l][0] = port1;
				links[l][1] = port2;
				links[l][2] = type;
				eeprom_set_Link(l);
				return l;
			}
			
		}
	}
	return -3;
}

/** Delete the link between two ports.
 \param port1 Trigger port
 \param port2 Triggered port
 \return 0 if everything went right, otherwise -1
*/
char delLink(char port1, char port2)
{
	char l;
	if ((port1 == -1) || (port2 == -1))
		return -1;
	for (l=0;l<MAXLINKS;l++) {
		if ((links[l][0] == port1) && (links[l][1] == port2)) {
			links[l][0] = 0;
			links[l][1] = 0;
			links[l][2] = 0;
			eeprom_set_Link(l);
			return 0;
		}
	}
	return -2;		// Enlace no encontrado
}

/** \page VPorts Virtual Ports
 The virtual ports are events that can be used as regular ports, linked to
 output ports to change their values to HIGH when the events start, bringing
 it to LOW when it's ended.

 The virtual ports can be used also as internal registers, storing the data
 that we want to preserve during the execution.

 All the possible uses for this ports will be developed in the near future.
*/

/** Get the specified port index
 \param label Name of the port
 \return The port's index, or -1 if not found .
 \Maxim 127 ports.
*/
char getPortId(char *label)
{
	char id;
	char pname[6];
	for(id=0;id<TOTALPORTS;id++) {
		eeprom_get_str(pname, id*EMPORTSLOT, 6);
		if (strncmp(pname, label, 5) == 0)
			return id;
	}
	print_error(output,101);
	return -1;
}


/** Save the MAC into the EEPROM
 \param macb3 Byte 3 of MAC Address
 \param macb4 Byte 4 of MAC Address
 \param macb5 Byte 5 of MAC Address
 \param macb6 Byte 6 of MAC Address
*/
int ethSetMAC(byte macb3, byte macb4, byte macb5, byte macb6)
{
	eeprom_set_byte(EMNETCFOFFSET + 1, 25);	// First
	eeprom_set_byte(EMNETCFOFFSET + 2, 25);	// and second bytes are fixed
	eeprom_set_byte(EMNETCFOFFSET + 3, macb3);
	eeprom_set_byte(EMNETCFOFFSET + 4, macb4);
	eeprom_set_byte(EMNETCFOFFSET + 5, macb5);
	eeprom_set_byte(EMNETCFOFFSET + 6, macb6);
	return true;
}

#ifdef ENABLE_NETWORKING
/** Save the IP address into the EEPROM
 \param ipb1 Byte 1 of IP Address
 \param ipb2 Byte 2 of IP Address
 \param ipb3 Byte 3 of IP Address
 \param ipb4 Byte 4 of IP Address
*/
int ethSetIP(byte ipb1, byte ipb2, byte ipb3, byte ipb4)
{
	eeprom_set_byte(EMNETCFOFFSET + 7, ipb1);
	eeprom_set_byte(EMNETCFOFFSET + 8, ipb2);
	eeprom_set_byte(EMNETCFOFFSET + 9, ipb3);
	eeprom_set_byte(EMNETCFOFFSET + 10, ipb4);
	return true;
}

/** Save the IP address of the gateway into the EEPROM
 \param ipb1 Byte 1 of IP Address
 \param ipb2 Byte 2 of IP Address
 \param ipb3 Byte 3 of IP Address
 \param ipb4 Byte 4 of IP Address
*/
int ethSetGW(byte ipb1, byte ipb2, byte ipb3, byte ipb4)
{
	eeprom_set_byte(EMNETCFOFFSET + 11, ipb1);
	eeprom_set_byte(EMNETCFOFFSET + 12, ipb2);
	eeprom_set_byte(EMNETCFOFFSET + 13, ipb3);
	eeprom_set_byte(EMNETCFOFFSET + 14, ipb4);
	return true;
}

/** Save the network mask into the EEPROM
 \param ipb1 Byte 1 of mask
 \param ipb2 Byte 2 of mask
 \param ipb3 Byte 3 of mask
 \param ipb4 Byte 4 of mask
*/
int ethSetNetmask(byte ipb1, byte ipb2, byte ipb3, byte ipb4)
{
	eeprom_set_byte(EMNETCFOFFSET + 15, ipb1);
	eeprom_set_byte(EMNETCFOFFSET + 16, ipb2);
	eeprom_set_byte(EMNETCFOFFSET + 17, ipb3);
	eeprom_set_byte(EMNETCFOFFSET + 18, ipb4);
	return true;
}

/** Sub-procedure for dealing with Ethernet board
 \param cmd Command
 \return
*/
int ethControl(const char *cmd)
{

	if (cmd[4] == 'u' && cmd[5] == 'p') {	// Activating ethernet
		eeprom_set_byte(EMNETCFOFFSET,1);
	} else if (cmd[4] == 's' && cmd[5] == 't') {	// Status of the ethernet
//		writef(output, flstr(strfmt_ethip), INFO, bname);
//		writef(output, flstr(strfmt_ethmc), INFO, bname);
//		writef(output, flstr(strfmt_ethmk), INFO, bname);
//		writef(output, flstr(strfmt_ethgw), INFO, bname);
		return true;
	} else if (cmd[4] == 'm' && cmd[5] == 'c') {	// setting MAC address
		return ethSetMAC(h2d(cmd[7], cmd[8]), h2d(cmd[9], cmd[10]),
				 h2d(cmd[11], cmd[12]), h2d(cmd[13], cmd[14]));
	} else if (cmd[4] == 'i' && cmd[5] == 'p') {	// setting IP
		return ethSetIP(h2d(cmd[7], cmd[8]), h2d(cmd[9], cmd[10]),
				h2d(cmd[11], cmd[12]), h2d(cmd[13], cmd[14]));
	} else if (cmd[4] == 'g' && cmd[5] == 'w') {	// setting gateway
//      ethSetGW(h2d(larg[0],larg[1]), h2d(larg[2],larg[3]),\
//              h2d(larg[4],larg[5]), h2d(larg[6],larg[7]));
	} else if (cmd[4] == 'n' && cmd[5] == 'm') {	// setting netmask
//      ethSetNetmask(h2d(larg[0],larg[1]), h2d(larg[2],larg[3]),\
//              h2d(larg[4],larg[5]), h2d(larg[6],larg[7]));
	} else if (cmd[4] == 'o' && cmd[5] == 'f') {	// deactivating ethernet
		eeprom_set_byte(EMNETCFOFFSET,0);
	} else if (cmd[4] == 't' && cmd[5] == 'e') {	// testing ethernet
		// Serial.println("connecting...");
	}

	return true;
}
// }}}
#endif

/// {{{
int groupPorts(char* port1, char* port2){
	byte i;
	char port1_id = getPortId(port1);
	char port2_id = getPortId(port2);
	short pos;
	char buffer[16];
	if (port1_id == -1 || port2_id == -1) return 101;
	for(i=ANALOGPORTS+DIGITALPORTS;i<TOTALPORTS;i++){
		pos = EMPORTSOFFSET + i * EMPORTSLOT;
		if (eeprom_get_byte(pos + 18) > 30 && eeprom_get_byte(pos + 18) < 200) {
			eeprom_set_byte(pos + 16,port1_id);
			eeprom_set_byte(pos + 17,port2_id);
			eeprom_set_byte(pos + 18,INS_INCREASE);
			ports[i].type = 'a'; //Enable the virtual port
			writef(output, flstrn(strfmt_lnk,buffer,16),
				INFO, bname, port1,port2,ports[i].type);
			return true;
		}
	}
	return 501;
}
// }}}

// {{{
byte setOperation(byte portID, char* operation)
{
	short pos = EMPORTSOFFSET + portID * EMPORTSLOT+ 18;
	if ((portID > DIGITALPORTS + ANALOGPORTS) && (portID<TOTALPORTS)) {
		switch (operation[0]){
  			case 'A':
				if (operation[1]=='N') eeprom_set_byte(pos,INS_AND);
				if (operation[1]=='D') eeprom_set_byte(pos,INS_INCREASE);
				break;
			case 'O':
				if (operation[1]=='R') eeprom_set_byte(pos,INS_OR);
				break;
			default:
				return 203;
		}
		return true;
	}
	else
	{
  		return 101;
	}
	return true;
}
/// }}}

/** Print the contents of EEPROM (serial port only). See \ref EEPROM Map
*/
void printMap()
{
	int i;
	int j;
	int p;
	byte b;

	Serial.print("0");
	for (i = 0; i < EMSEGMENTS; i++)	// For each segment in EEPROM
	{
		if (i * j < 260)
			Serial.print("0");
		Serial.print(i * j, HEX);
		Serial.print(":");
		if (i * j + 20 < 260)
			Serial.print("0");
		Serial.print(i * j + 19, HEX);
		Serial.print(" | ");
		for (j = 0; j < 20; j++) {
			p = i * 20 + j;
			b = eeprom_get_byte(p);
			if ((b > 32) && (b < 127))
				Serial.print(char( b));
			else
				Serial.print(".");
			// Cada 5 Bytes un separador
			if (((j + 1) % 5) == 0)
				Serial.print(" ");
		}
		Serial.print(" | ");
		for (j = 0; j < 20; j++) {
			p = i * 20 + j;
			b = eeprom_get_byte(p);
			if (b < 16)
				Serial.print("0");
			Serial.print(b, HEX);
			// Cada 5 Bytes un separador
			if (((j + 1) % 5) == 0)
				Serial.print(" ");
		}
		Serial.println();
	}
}

/** Executes the specified command
 \param cmd The command string (max 20Bytes)
 \return true if the operation was successful, other if not
*/
boolean processInstruction(const char *cmd)
{
	char id1,id2;
	//char id2;
	int i,j;
//char buffer[10];
	char arg1[6];		// First argument (if specified)
	char arg2[6];		// Second argument (if specified)
	int code = 0;
    int incident=-1;
    char funcionout=0;
	//! @todo Change "code" to zero if invalid char
	code = (cmd[0] - 96) << 10;
	code += (cmd[1] - 96) << 5;
	code += (cmd[2] - 96);

	/* All the commands will follow a similar structure:
	 * 012345678901234 <-byte no.
	 cmd arg01 arg02
	 cfg portn di
	 lbl portn alias
	 lnk portn portm
	 set portn value
	 */
	GETARG1(cmd, arg1);
	GETARG2(cmd, arg2);

#ifdef MODULE_DEBUG
	if (debug == 1) {
		writef(output, "D:Command %s encoded as %d \n", cmd, code);
		writef(output, "D:Cmd=%s, arg1=%s, arg2=%s \n", cmd, arg1,
		       arg2);
	}
#endif

	/// Supported commands:
	switch (code) {
	case 0:
		break;

	case CMD_LST:		/// - lst: List configured ports
		listPorts();
		break;

        case CMD_LSC:		/// - List compacto.
		listCompact();
		break;

	case CMD_ECH:		/// - echo: (de)activates local echo
		echo = ! echo;
		break;

	case CMD_DEB:		/// - debug: (de)activates debug mode
		debug = ! debug;
		break;

	case CMD_DEF:		/// - default: load default configuration
		loadDefaultConfig();
		resetPorts();
		Serial.println("D:Defaults restored");		
		return 0;

	case CMD_LOA:		/// - load: load stored configuration
		if (loadConfig())incident=0;
		else incident=21;	
		break;

	case CMD_SAV:		/// - save: save configuration to EEPROM
		saveConfig();
		incident=0;
		break;

	case CMD_CFG:		/// - cfg: configures the specified port
		id1 = getPortId(arg1);
		if (configPort(id1, arg2, NULL)) 	incident=0;		
		resetPorts();
		break;

	case CMD_LBL:		/// - lbl: set an alias to a port
		id1 = getPortId(arg1);
		if (configPort(id1, NULL, arg2))	incident=0;		
		break;

	case CMD_RES:		/// - reset: clear the EEPROM
		eeprom_reset();
		incident=0;		
		break;

	case CMD_MAP:		/// - map: print the \ref EEPROM
		printMap();
		break;

	case CMD_PUT:		/// - put: changes a Byte in the \ref EEPROM
		i = atoi(arg1);
		j = atoi(arg2);
		if (i > 0 && i < EMSEGMENTS * EMPORTSLOT && j >= 0 && j <= 255) {
			eeprom_set_byte(i, j);
			incident=0;
		}
		else incident=30;		
		break;

	case CMD_VER:		/// - ver: display the current version
		printVersion();
		break;

	case CMD_SNM:		/// - snm: set a new name for the board
		saveBoardName(arg1);
		incident=0;
		break;

	case CMD_GRP: 		/// - grp: group 2 ports into a virtualport
		if(groupPorts(arg1,arg2)) incident=0;			
		else incident=i;
		break;

	case CMD_SOP:		/// - sop: Set operation to grouped port
		id1 = getPortId(arg1);
		if (true==setOperation(getPortId(arg1),arg2))
			incident=0;
		break;

	case CMD_SET:		/// - set: set a value to an output port
		id1 = getPortId(arg1);
		if (arg2[1] == 'n')
			setPortValue(id1, HIGH);
		else if (arg2[1] == 'f')
			setPortValue(id1, LOW);
		else
			setPortValue(id1, atoi(arg2));

		incident=0;
		break;

	case CMD_LNK:		/// - lnk: link two ports
		id1 = getPortId(arg1);            // Indice del puerto origen
		id2 = getPortId(arg2);            // Indice del puerto destino
               
                if (validTypeLink(cmd[16])==true){
                  funcionout=addLink(id1, id2, cmd[16]);                 
                  if (funcionout>=0)incident=0;
                  else if (funcionout==-2)incident=302;
                  else if (funcionout==-3)incident=301;
                }
                else incident=306;	
		break;

	case CMD_UNL:		/// - unl: unlink two portsValid
		id1 = getPortId(arg1);
		id2 = getPortId(arg2);
		funcionout=delLink(id1, id2);
		if (funcionout == 0) incident=0;	// Todo OK
		else if (funcionout == -2) incident=307; // Enlace no encontrado
		break;
		

	case CMD_LLN:		/// - lln: list the current links
		listLinks();
		break;

        case CMD_UPT:		/// - upt: print uptime
		printUptime();
		break;

	case CMD_MEM:		/// - mem: print available memory
		printMemory();
		break;

	case CMD_ETH:		/// - eth: ethernet functions
#ifdef ENABLE_NETWORKING
		if (ethControl(cmd))incident=0;			
		else incident=440;
#endif
		break;

	default:
		incident=1;	
		break;
	}
        
        if(incident>0){
          print_error(output,incident);
          return false;
        }
        if(incident==0)print_cmdok(output);
        return true; 
                 
        
          
}

// {{{
void updateTimeVars(){
	int i;
	unsigned long time;	///< Internal variable for measuring time
	// Leemos las marcas de tiempo y actualizamos valores
	time = millis() / 1000;
	if (seconds != (time % DAYLENGHT)) {	// Solamente actualizamos si ha cambiado
		seconds = time % DAYLENGHT;
		if (seconds == 0)
			days++;

#ifdef ENABLE_NETWORKING
		// Enviamos broadcast UDP cada 10 segundos
		if (seconds % 10 == 0){
			sendODControlAnnouncement();
		}
		// Enviamos broadcast UDP cada 10 segundos
		if (seconds % 60 == 0){
                        void sendODControlHead();
  			for (i=0;i<TOTALPORTS;i++){                                
    				if(ISDIGITAL(i)||ISANALOG(i)){
					sendODControlUpdate(i);
				}                                
			}
                        Udp.endPacket();
		}
#endif
	}
}
// }}}

/** Initializes the device.
 This function is called when the first power supply is connected.
*/
void setup()
{
	char *disabled="-";

	analogReference(EXTERNAL);

	pinMode(0,OUTPUT);
	pinMode(1,INPUT);
	digitalWrite(0, HIGH);
	if (digitalRead(1)==HIGH)
	{
		digitalWrite(0,LOW);
		if (digitalRead(1) == LOW) {
			eeprom_set_byte(EMBOARDOFFSET,255);
                        delay(5000);
		}
	}


	Serial.begin(9600);
	//Serial.println("D:init");
	// Proceso de carga de la configuraciÃ³n:


	if(eeprom_get_byte(EMBOARDOFFSET)==255)
	{
   		Serial.println("D:loaddef");
                eeprom_reset();
                loadDefaultConfig();
                resetPorts();
                saveBoardName("devkt");
                eeprom_set_byte(EMNETCFOFFSET,1); // eth up
	}


	// 2. Intentamos cargar la configuraciÃ³n almacenada
	if (loadConfig()) {
	//	Serial.println("D:loadcf");
	} else {
		//writef(SERIALPORT, flstr(strfmt_error2), ERROR, bname, 21);
		print_error(output,21);
		// 3. Si no hay ninguna vÃ¡lida, aplicamos configuraciÃ³n por defecto
		loadDefaultConfig();
	}

	Serial.print("D:mem ");
	Serial.println(freeMemory());
        
//Serial.println(freeMalloc());
#ifdef ENABLE_NETWORKING
	ip[0]=eeprom_get_byte(EMNETCFOFFSET+7);
	ip[1]=eeprom_get_byte(EMNETCFOFFSET+8);
	ip[2]=eeprom_get_byte(EMNETCFOFFSET+9);
	ip[3]=eeprom_get_byte(EMNETCFOFFSET+10);

	Ethernet.begin(mac, ip);
	Telnet.begin();
	Udp.begin(22537);
	Webserver.begin();
#endif
	Serial.println("D:OK");
	resetPorts();
}


// {{{ loop(): main
/** Permanent loop.
 This is the main function of Arduino. This function is called again when
 the last call is ended.
*/
void loop()
{
        char instruction[20];
        
        // Aparte de comprobar los comandos recibidos, miramos si
	// hay algun cambio de estado en los puertos.
	refreshPortStatus();

	updateTimeVars();

	if (readFromSerialPort(instruction))
	{
  		output = SERIALPORT;
		processInstruction(instruction);
	}
#ifdef ENABLE_NETWORKING
 	if (readFromTelnetPort(instruction))
	{
        	Serial.println("telnet_in");
  		Serial.println(instruction);
	}

	if(readFromHTTPPort(instruction))
	{
      		Serial.println("http_in ");
  //		processHTTPQuery(instruction);
  		Serial.print(instruction);
	}
#endif
	// Tiempo de espera
	delay(DELAYCYCLE);
}

// }}}


// {{{ writef(): personalized printf
void writef(byte out, const char *fmt, ...)
{
	va_list ap, ap2;
	short d;
	char c, *s;
	char i[6];


	va_start(ap, fmt);
	while (*fmt) {
		if (*fmt == '%') {
			*fmt++;
			switch (*fmt) {
			case 's':
				s = va_arg(ap, char*);
				owrite(out, s);
				break;
			case 'd':
				d = va_arg(ap, int);
				itoan(d, i, sizeof(i));
				owrite(out, i);
				break;
			case 'c':
				c = va_arg(ap, int);
				i[0]=char(c);
				i[1]=0;
				owrite(out, i);
			}
			*fmt++;

		} else {
				i[0]=char(*fmt);
				i[1]=0;
				owrite(out, i);
			*fmt++;
		}
	}
	va_end(ap);
}
// }}}

//{{{ flstrn() copies a string from flash to a buffer
// The function returns a pointer to the buffer or NULL if failed
char *flstrn(const char *flash_str, char *buffer, byte len){
	byte i=0;
	char c;
	buffer[0]=0;

	if (!flash_str)
		return NULL;

	while ((c = pgm_read_byte(flash_str++)) && i < len - 1)
		buffer[i++] = c;
	buffer[i] = 0;
	return buffer;
}
//}}}





unsigned char eeprom_get_byte(int byte_index)
{
  return eeprom_read_byte((unsigned char*) byte_index);
}

void eeprom_set_byte(int byte_index, byte value)
{
  eeprom_write_byte((unsigned char*)byte_index,value);
}

// {{{ eeprom_get_ushort(): read an unsigned short from eeprom
unsigned short eeprom_get_ushort(int byte_index)
{
	unsigned short s = 0;
	s = eeprom_get_byte(byte_index);
	s = s << 8;
	s ^= eeprom_get_byte(byte_index + 1);
	return s;
}

// }}}

// {{{ eeprom_set_ushort(): write an unsigned short into eeprom
unsigned short eeprom_set_ushort(int byte_index, unsigned short value)
{
	char sl = 0;
	char sr = 0;
	sl = value >> 8;
	sr = value;
	eeprom_set_byte(byte_index, sl);
	eeprom_set_byte(byte_index + 1, sr);
}
// }}}

// {{{ eeprom_get_str()
int eeprom_get_str(char *dst, int addr, size_t len)
{
	eeprom_read_block((void*)dst, (void*)addr, len);
	dst[5] = 0;
}
// }}}

// {{{ eeprom_set_str()
int eeprom_set_str(int addr, char *src,  size_t len)
{
	eeprom_write_block((void*)src, (void*)addr, len);
}
// }}}

void eeprom_reset()
{
  for (int i=0;i<EMSEGMENTS*EMPORTSLOT;i++) eeprom_set_byte(i,255);
}

#ifdef ENABLE_NETWORKING
/// This function reads a command from the HTTP port
int readFromHTTPPort(char *instruction){

	char b=0;
	int i = 0;
	char barfound = 0;
	char value[6];
	char pname[6] = "";
	char buffer[50];
  //boolean uristarted = false;
  // listen for incoming clients
  EthernetClient client = Webserver.available();
  if (client) {
	// HTTP TESTING
    	while (client.available()) {
 		b = client.read();

		if ((i < BUFFERSIZE) && (b >= 32 && b <= 126)) {
			instruction[i] = b;
			i++;
			instruction[i] = 0;
		}

 	 	if (b == ' ' && barfound == 1) { // First space after slash. Quitting
 	 	 	break;
 	 	}
		if (b == '/') {
   			if (barfound==0) { // First slash: following "GET " command
				i = 0;
   				barfound = 1;
   			} else { // Second slash, probably HTTP/1.1 block. Quitting.
   				break;
     			}
   		}
	}

	Serial.print("HTTP CMD:");
	Serial.println(instruction);

	if ((instruction[0]=='l' && instruction[1]=='s' && instruction[2]=='t') || i<=1) {
		// Port list
		client.println("HTTP/1.1 200 OK");
		client.println("Content-Type: text/html");
		client.println("Connnection: close");
 		client.println(); // Space between headers and body
 		client.print(flstrn(str_http_header_1,buffer,50));
 		delay(1);
  		client.print(flstrn(str_http_header_2,buffer,50));
  		delay(1);
    		client.print(flstrn(str_http_header_3,buffer,50));
    		delay(1);
      		client.print(flstrn(str_http_header_4,buffer,50));
      		delay(1);
      		client.print(flstrn(str_http_header_5,buffer,50));
      		delay(1);
            	client.print(flstrn(str_http_header_6,buffer,50));
      		delay(1);

		for(i=0;i<TOTALPORTS;i++) {
			value[0] = 0;
			eeprom_get_str(pname, i*EMPORTSLOT, 6);
			pname[sizeof(pname) - 1] = 0;

			if (ISDIGITAL(i)) {
  				if (ISINPUT(i)){
  					client.print("<li class=DI><b>");
                			client.print(pname);

    					if (ports[i].value != LOW) {
						client.print("</b><a class=ON>");
					}else{
						client.print("</b><a class=OFF>");
              				}

    				}else{
                      			client.print("<li class=DO><b>");
                                	client.print(pname);
      					if (ports[i].value != LOW) {
						client.print("</b><a class=ON href='set+");
						client.print(pname);
						client.print("+off'>on");
					}else{
						client.print("</b><a class=OFF href='set+");
						client.print(pname);
						client.print("+on'>off");
              				}

      				}
      				client.println("</a></li>");

			} else if (ISANALOG(i)) {
				client.print("<li class=AI><b>");
				itoan(ports[i].value, value, sizeof(value));
				client.print(pname);
				client.print("</b><a>");
				client.print(value);
				client.println("</a></li>");
			}

			delay(10);
		}
		client.println(flstrn(str_http_footer_1,buffer,50));
		client.print(VERSION);
		client.println(flstrn(str_http_footer_2,buffer,50));
		//client.println("</ul><div id=ftr></div></body></html>");
	} else {
		output = SERIALPORT; // Do not show any output
		if (processInstruction(instruction)==true){
     			client.println("HTTP/1.1 307 Temporary redirect"); // HTTP Code: correct (redirect)
			client.println("Location: /lst");
   		} else {
     			client.println("HTTP/1.1 500 Internal server error"); // HTTP Code: Internal server error
			client.println("Location: /lst");
     		}
  	}

	delay(10);
	client.stop();
 	// END HTTP TESTING
 	return true;
  }

  return false;
}


/// This function reads the command from a telnet port
bool readFromTelnetPort(char *instruction)
{
	char b=13;
	byte i=0;
	char pname[6] = "";
	char buffer[100];
	char pos=0;

	EthernetClient client = Telnet.available();
	if (client) {
		while (client.available()) {
 			b = client.read();
			if ((i < BUFFERSIZE) && (b >= 32 && b <= 126)) {
				instruction[i] = b;
				i++;
				instruction[i] = 0;
			}
		}

		output = TELNET;
		processInstruction(instruction);
		client.stop();
		return true;
	} else {
 		return false;

	}
}

void sendODControlHead(){
       
	IPAddress broadcast(255, 255, 255, 255);       
        Udp.beginPacket(broadcast, UDP_PORT);
	Udp.print("ODC01:"UID);      
}

void sendODControlAnnouncement(){
        sendODControlHead();
        Udp.print(":");
        Udp.print(bname);        
        Udp.print(" isalive");	
        Udp.endPacket();
}

void sendODControlUpdate(int i){
	char msg[40];
	char auxBuf[11];
	
	strlcpy(msg, " ", sizeof(msg));
        get_type(i, auxBuf);
        strncat(msg,auxBuf ,2);
        strlcat(msg, ":", sizeof(msg));
        eeprom_get_str(auxBuf, i*EMPORTSLOT, 6);
        strlcat(msg, auxBuf, sizeof(msg));
        Udp.print(msg);       
        strlcpy(msg, ":", sizeof(msg));
        get_state(i, auxBuf);
        strlcat(msg, auxBuf, sizeof(msg));
	Udp.print(msg);
}
#endif

#ifdef MODULE_DEBUG
void debug_write(char*c) {
  Serial.println(c);
}
void debug_write(int i) {
  Serial.println(i);
}
#else
void debug_write(char*c) {
  //Nothing
}
void debug_write(int i) {
  //Nothing
}
#endif
