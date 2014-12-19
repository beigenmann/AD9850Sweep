#include <elapsedMillis.h>
#include <AD9850.h>


#include <EEPROM.h>

/*

 RotEnc.pde
 
 U8glib Example (Incremental Rotary Encoder)
 
 m2tklib = Mini Interative Interface Toolkit Library
 
 Copyright (C) 2012  olikraus@gmail.com
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 */

#include <stdlib.h>
#include "U8glib.h"
#include "M2tk.h"
#include "utility/m2ghu8g.h"


// setup u8g object, please remove comment from one of the following constructor calls
// IMPORTANT NOTE: The complete list of supported devices is here: http://code.google.com/p/u8glib/wiki/device

//U8GLIB_NHD27OLED_BW u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD27OLED_2X_BW u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD27OLED_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD27OLED_2X_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD31OLED_BW u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD31OLED_2X_BW u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD31OLED_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_NHD31OLED_2X_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGS102 u8g(13, 11, 10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGM132 u8g(13, 11, 10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGM128 u8g(13, 11, 10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_ST7920_128X64_1X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_128X64_4X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_128X64_1X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_ST7920_128X64_4X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_ST7920_192X32_1X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_192X32_4X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_192X32_1X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_ST7920_192X32_4X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_ST7920_192X32_1X u8g(13, 11, 10);	// SPI Com: SCK = en = 13, MOSI = rw = 11, CS = di = 10
//U8GLIB_ST7920_192X32_4X u8g(10);		// SPI Com: SCK = en = 13, MOSI = rw = 11, CS = di = 10, HW SPI
//U8GLIB_ST7920_202X32_1X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_202X32_4X u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 17, 16);   // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, di=17,rw=16
//U8GLIB_ST7920_202X32_1X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_ST7920_202X32_4X u8g(18, 16, 17);	// SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17
//U8GLIB_LM6059 u8g(13, 11, 10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_LM6063 u8g(13, 11, 10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGXL160_BW u8g(10, 9);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGXL160_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGXL160_2X_BW u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_DOGXL160_2X_GR u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
U8GLIB_PCD8544 u8g(13, 11, 10, 9, 8);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, Reset = 8
//U8GLIB_PCF8812 u8g(13, 11, 10, 9, 8);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, Reset = 8
//U8GLIB_KS0108_128 u8g(8, 9, 10, 11, 4, 5, 6, 7, 18, 14, 15, 17, 16); 		// 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, cs1=14, cs2=15,di=17,rw=16
//U8GLIB_LC7981_160X80 u8g(8, 9, 10, 11, 4, 5, 6, 7,  18, 14, 15, 17, 16); 	// 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, cs=14 ,di=15,rw=17, reset = 16
//U8GLIB_LC7981_240X64 u8g(8, 9, 10, 11, 4, 5, 6, 7,  18, 14, 15, 17, 16); 	// 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, cs=14 ,di=15,rw=17, reset = 16
//U8GLIB_LC7981_240X128 u8g(8, 9, 10, 11, 4, 5, 6, 7,  18, 14, 15, 17, 16); 	// 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 en=18, cs=14 ,di=15,rw=17, reset = 16
//U8GLIB_ILI9325D_320x240 u8g(18,17,19,U8G_PIN_NONE,16 );  			// 8Bit Com: D0..D7: 0,1,2,3,4,5,6,7 en=wr=18, cs=17, rs=19, rd=U8G_PIN_NONE, reset = 16
//U8GLIB_SBN1661_122X32 u8g(8,9,10,11,4,5,6,7,14,15, 17, U8G_PIN_NONE, 16); 	// 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7 cs1=14, cs2=15,di=17,rw=16,reset = 16
//U8GLIB_SSD1306_128X64 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1306_128X64 u8g(10, 9);		// HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);	// HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1306_128X32 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1306_128X32 u8g(10, 9);             // HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);	// HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1309_128X64 u8g(13, 11, 10, 9);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1327_96X96_GR u8g(U8G_I2C_OPT_NONE);	// I2C
//U8GLIB_SSD1327_96X96_2X_GR u8g(U8G_I2C_OPT_NONE);	// I2C
//U8GLIB_NHD_C12864 u8g(13, 11, 10, 9, 8);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, RST = 8
//U8GLIB_NHD_C12832 u8g(13, 11, 10, 9, 8);	// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, RST = 8
//U8GLIB_T6963_240X128 u8g(8, 9, 10, 11, 4, 5, 6, 7, 14, 15, 17, 18, 16); // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7, cs=14, a0=15, wr=17, rd=18, reset=16
//U8GLIB_T6963_240X64 u8g(8, 9, 10, 11, 4, 5, 6, 7, 14, 15, 17, 18, 16); // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7, cs=14, a0=15, wr=17, rd=18, reset=16
//U8GLIB_T6963_128X64 u8g(8, 9, 10, 11, 4, 5, 6, 7, 14, 15, 17, 18, 16); // 8Bit Com: D0..D7: 8,9,10,11,4,5,6,7, cs=14, a0=15, wr=17, rd=18, reset=16



//=================================================
// Forward declaration of the toplevel element
M2_EXTERN_ALIGN(top_menu);

//=================================================
// Simple dialog: Input two values n1 and n2
AD9850 ad(5, 6, 7); // w_clk, fq_ud, d7
elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs
uint8_t startmeg = 0;
uint8_t stopmeg = 0;
uint8_t incmeg = 0;
uint32_t startnumber ;
uint32_t stopnumber ;
uint32_t incnumber ;
uint32_t _start ;
uint32_t _stop ;
uint32_t _inc;
uint32_t _s;
uint8_t _Sweep ;
uint8_t _BackLight ;
uint8_t select_Sweep;
uint8_t select_BackLight;
#define STARTF "Start Frq:"
#define STOPF "Stop  Frq:"
#define INCF "Inc  Freq:"
#define MAXMEG 40
#define BACKLIGHTPIN 12
M2_LABEL(el_labelstartf, NULL, STARTF);
M2_LABEL(el_labelstopf, NULL, STOPF);
M2_LABEL(el_labelincf, NULL, INCF);
M2_U8NUM(el_megstart, "c2", 0,MAXMEG,&startmeg);
M2_U8NUM(el_megstop, "c2", 0,MAXMEG,&stopmeg);
M2_U8NUM(el_meginc, "c2", 0,MAXMEG,&incmeg);
M2_LABEL(el_sep1, "b1", ".");                // dot is drawn on the baseline
M2_U32NUM(el_numstart, "a1c6", &startnumber);
M2_U32NUM(el_numstop, "a1c6", &stopnumber);
M2_U32NUM(el_numinc, "a1c6", &incnumber);
M2_LABEL(el_mhz1, "b1", " MHz");  
M2_BUTTON(el_ok, "", "Ok ", fn_ok);
M2_BUTTON(el_cancel, "", " Cancel ", fn_cancel);
M2_LIST(list_buttons) = {
  &el_ok, &el_cancel };
M2_HLIST(el_buttons, NULL, list_buttons);
M2_LIST(list_num_inputstart) = { 
  &el_megstart,&el_sep1, &el_numstart ,&el_mhz1};
M2_LIST(list_num_inputstop) = { 
  &el_megstop,&el_sep1, &el_numstop ,&el_mhz1};
M2_LIST(list_num_inputinc) = { 
  &el_meginc,&el_sep1, &el_numinc ,&el_mhz1};
M2_HLIST(list_elementstart, NULL, list_num_inputstart);
M2_HLIST(list_elementstop, NULL, list_num_inputstop);
M2_HLIST(list_elementinc, NULL, list_num_inputinc);

M2_LIST(liststartf) = {  
  &el_labelstartf,&list_elementstart,&el_buttons};
M2_LIST(liststopf) = {  
  &el_labelstopf,&list_elementstop,&el_buttons};
M2_LIST(listincf) = {  
  &el_labelincf,&list_elementinc,&el_buttons};
//M2_GRIDLIST(el_gridlist, "c1", list);
M2_VLIST(el_gridliststartf, NULL, liststartf);
M2_VLIST(el_gridliststopf, NULL, liststopf);
M2_VLIST(el_gridlistincf, NULL, listincf);



M2_LABEL(el_labelSweep, NULL, "Sweep: ");
M2_COMBO(el_comboSweep, NULL, &select_Sweep, 3, fn_idx_to_sweep);
M2_LABEL(el_labelBackLight, NULL, "Light: ");
M2_COMBO(el_comboBackLight, NULL, &select_BackLight, 3, fn_idx_to_sweep);
M2_LIST(listSweep) = { 
  &el_labelSweep, &el_comboSweep,
  &el_labelBackLight, &el_comboBackLight,
  &el_ok, &el_cancel
};
M2_GRIDLIST(list_elementSweep, "c2",listSweep);


M2_ROOT(el_ts_mnu1, "t1w60f8", STARTF, &el_gridliststartf);
M2_ROOT(el_ts_mnu2, "t1w60f8", STOPF, &el_gridliststopf);
M2_ROOT(el_ts_mnu3, "t1w60f8", INCF, &el_gridlistincf);
M2_ROOT(el_ts_mnu4, "t1w60f8", "Settings", &list_elementSweep);
//M2_ROOT(el_ts_mnu3, "t1w60f8", "menu 3", NULL);
M2_LIST(list_ts_mnu) = { 
  &el_ts_mnu1,
  &el_ts_mnu2,
  &el_ts_mnu3,
  &el_ts_mnu4
};
M2_VLIST(el_ts_mnu_vlist, NULL, list_ts_mnu);
M2_ALIGN(top_el_ts_mnu, "-1|1W64H64", &el_ts_mnu_vlist);

//M2tk m2(&el_gridlist, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_u8g_bf);
M2tk m2(&top_el_ts_mnu, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_u8g_bf);

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to adress + 3.
void EEPROMWritelong(int address, uint32_t value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

uint32_t EEPROMReadlong(int address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void readsetup(){
  _start = EEPROMReadlong(0);
  _stop =  EEPROMReadlong(4);
  _inc =  EEPROMReadlong(8);
  _Sweep =  EEPROM.read(12);
  if(_Sweep > 1) _Sweep = 1;
  _BackLight =  EEPROM.read(13);
  if(_BackLight > 1) _BackLight = 1;
  select_BackLight = _BackLight;
  select_Sweep = _Sweep;
}

const char *fn_idx_to_sweep(uint8_t idx)
{
  if ( idx == 0 )
    return "On";
  else if (idx == 1 )
    return "Off";
  return "Off";
}

void fn_ok(m2_el_fnarg_p fnarg) {
  if(m2.getRoot() == &el_gridliststartf){
    if(startmeg >= MAXMEG) startnumber= 0;
    _start = startnumber + startmeg * 1000000; 
    EEPROMWritelong(0,_start);
    ad.setfreq(_start);
  }
  if(m2.getRoot() == &el_gridliststopf){
    if(stopmeg >= MAXMEG) stopnumber= 0;
    _stop  = stopnumber + stopmeg * 1000000;
    EEPROMWritelong(4,_stop);
  }
  if(m2.getRoot() == &el_gridlistincf){
    if(incmeg >= MAXMEG) incnumber= 0;
    _inc = incnumber + incmeg * 1000000;
    EEPROMWritelong(8,_inc);
  }
  if(m2.getRoot() == &list_elementSweep){
    _Sweep = select_Sweep;
    _BackLight= select_BackLight;
    EEPROM.write(12, _Sweep);
    EEPROM.write(13, _BackLight);
    setBackLight();
  }
 _s = _start;

  //Serial.println((int)&el_gridliststartf);
  //Serial.println((int)element);
  m2.setRoot(&top_el_ts_mnu);

}

void fn_init(m2_el_fnarg_p fnarg) {
  // m2.setRoot(&top_el_ts_mnu);

}

void fn_cancel(m2_el_fnarg_p fnarg) {
  if(m2.getRoot() == &el_gridliststartf){
    startmeg = _start / 1000000;
    startnumber = _start - (startmeg*1000000);
  }
  if(m2.getRoot() == &el_gridliststopf){
    stopmeg = _stop / 1000000;
    stopnumber = _stop -(stopmeg * 1000000);
  }
  if(m2.getRoot() == &el_gridlistincf){
    incmeg = _inc / 1000000;
    incnumber = _inc -(incmeg * 1000000);
  }
  if(m2.getRoot() == &list_elementSweep){
    select_Sweep =_Sweep;
    select_BackLight =_BackLight;
  }

  m2.setRoot(&top_el_ts_mnu);
}
void setBackLight(){
  if(select_BackLight){
    digitalWrite(BACKLIGHTPIN, HIGH);
  }
  else{
    digitalWrite(BACKLIGHTPIN, LOW);
  }
}


uint8_t update(void) {  
  // m2_rom_void_p element = m2.getRoot();
  // Serial.println((int)element);

  if (timeElapsed > 10) {	
    if(!_Sweep){
      if(_s < _start){
        _s = _start;
      }
      else{
        _s += _inc;
        if(_s > _stop){
          _s = _start;
        }
      }
      ad.setfreq(_s);
      Serial.println((int)_s);
    }
    timeElapsed = 0;			 // reset the counter to 0 so the counting starts over...
  }
  return 0;
}
//=================================================
// Arduino Setup & Loop

void setup(void) {

  pinMode(BACKLIGHTPIN, OUTPUT);   
  digitalWrite(BACKLIGHTPIN, HIGH);
  readsetup();
  setBackLight();
  startmeg = _start / 1000000;
  if(startmeg >= MAXMEG) {
    startnumber= 0;
    startmeg = MAXMEG;
    _start  = startmeg * 10000000;
  }
  else
    startnumber = _start - (startmeg*1000000);
  stopmeg = _stop / 1000000;
  if(stopmeg >= MAXMEG) {
    stopnumber= 0;
    stopmeg = MAXMEG;
    _stop  = stopmeg * 10000000;
  }
  else
    stopnumber = _stop -(stopmeg * 1000000);

  incmeg = _inc / 1000000;
  if(incmeg >= MAXMEG) {
    stopnumber= 0;
    stopmeg = MAXMEG;
    _inc  = stopmeg * 10000000;
  }
  else
    incnumber = _inc -(incmeg * 1000000);
  select_Sweep =_Sweep;
  select_BackLight =_BackLight;
  // Connect u8glib with m2tklib
  m2_SetU8g(u8g.getU8g(), m2_u8g_box_icon);

  // Assign u8g font to index 0
  m2.setFont(0, u8g_font_6x12r);

  // define button for the select message
  pinMode(A7, INPUT);
  m2.setPin(M2_KEY_SELECT, 4);          // dogm128 shield, 2nd from top

  // The incremental rotary encoder is conected to these two pins
  m2.setPin(M2_KEY_ROT_ENC_A, 3);
  m2.setPin(M2_KEY_ROT_ENC_B, 2);
  ad.setfreq(_start);

  Serial.begin(9600);
}

void loop() {
  // check rotary encoder also inside the picture loop
  m2.checkKey();  
  // process events and redraw menu if required
  if ( m2.handleKey() != 0 || update() != 0 ) {
    u8g.firstPage();  
    do {
      // check rotary encoder also inside the picture loop
      m2.checkKey();
      // draw menu
      m2.draw();
    } 
    while( u8g.nextPage() );
  }
}






















