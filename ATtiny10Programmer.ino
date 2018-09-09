/**************************************************
 * TPI programmer for ATtiny10
 * (possibly 4,5,9 as well)
 * Make the connections as shown below.
 * 
 * To use:
 *  - Upload to arduino and power off
 *  - Connect ATtiny10 as shown
 *  - Power on and open the serial monitor
 *  - If things are working so far you should
 *    see "NVM enabled" and "ATtiny10 connected".
 *  - Input one-letter commands via serial monitor:
 *    D = dump memory. Displays all current memory
 *        on the chip
 *    E = erase chip. Erases all program memory
 *    R = read program. After sending this command
 *        you have 20 seconds to copy and paste the
 *        entire contents of a .hex file into the
 *        serial monitor and send. You can do the
 *        whole file at once. If it was successful,
 *        you should see "program received".
 *    P = write program. After reading the program
 *        with the R command, use this to write the
 *        program to the ATtiny10.
 *    V = verify. Verifies that the program was
 *        written correctly. If not, it will display
 *        the errors.
 *    F = finish. not necessary, but this disables
 *        further access until the arduino is reset
 *    S = set fuse. follow the instructions to set
 *        one of the three fuses.
 *    C = clear fuse. follow the instructions to clear
 *        one of the three fuses.
 *  - Finally, power off the arduino and remove the
 *    Attiny10
 *                                                *
 * Arduino                 ATtiny10               *
 * ----------+          +----------------         *
 * (SS#)  10 |--[R]-----| 6 (RESET#/PB3)          *
 *           |          |                         *     
 * (MOSI) 11 |--[R]--+--| 1 (TPIDATA/PB0)         *
 *           |       |  |                         *
 * (MISO) 12 |--[R]--+  |                         *
 *           |          |                         *     
 * (SCK)  13 |--[R]-----| 3 (TPICLK/PB1)          *
 * ----------+          +----------------         *
 *                                                *
 *  -[R]-  =  a few kOhm resistor                 *
 *                                                *
 *  this picture : 2011/12/08 by pcm1723          *
 *                                                *
 * thanks to pcm1723 for tpitest.pde upon which   *
 * this is based                                  *
 **************************************************/
 
#include <SPI.h>
#include "pins_arduino.h"

// define the instruction set bytes
#define SLD    0x20
#define SLDp   0x24
#define SST    0x60
#define SSTp   0x64
#define SSTPRH 0x69
#define SSTPRL 0x68
// see functions below ////////////////////////////////
// SIN  0b0aa1aaaa replace a with 6 address bits   
// SOUT 0b1aa1aaaa replace a with 6 address bits 
// SLDCS  0b1000aaaa replace a with address bits 
// SSTCS  0b1100aaaa replace a with address bits 
///////////////////////////////////////////////////////
#define SKEY   0xE0
#define NVM_PROGRAM_ENABLE 0x1289AB45CDD888FFULL // the ULL means unsigned long long

#define NVMCMD 0x33
#define NVMCSR 0x32
#define NVM_NOP 0x00
#define NVM_CHIP_ERASE 0x10
#define NVM_SECTION_ERASE 0x14
#define NVM_WORD_WRITE 0x1D

// represents the current pointer register value
unsigned short adrs = 0x0000;

// used for storing a program file
uint8_t data[1024]; //program data
unsigned int progSize = 0; //program size in bytes

// used for various purposes
long startTime;
int timeout;
uint8_t b, b1, b2, b3;
boolean idChecked;

void setup(){
  // set up serial
  Serial.begin(38400); // you can change this if you want
  // set up SPI
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  
  // enter TPI programming mode
  digitalWrite(SS, LOW); // assert RESET on tiny
  delay(1); // t_RST min = 400 ns @ Vcc = 5 V
  
  SPI.transfer(0xff); // activate TPI by emitting
  SPI.transfer(0xff); // 16 or more pulses on TPICLK
  SPI.transfer(0xff); // while holding TPIDATA to "1"
  
  writeCSS(0x02, 0x04); // TPIPCR, guard time = 8bits (default=128)
  
  send_skey(NVM_PROGRAM_ENABLE); // enable NVM interface
  // wait for NVM to be enabled
  while((readCSS(0x00) & 0x02) < 1){
    // wait
  }
  Serial.println("NVM enabled");
  
  // initialize memory pointer register  
  setPointer(0x0000);
  
  timeout = 20000;
  idChecked = false;
} // end setup()

void loop(){
  if(!idChecked){
    checkID();
    idChecked = true;
  }
  // when ready, send ready signal '.' and wait
  Serial.write('.');
  while(Serial.available() < 1){
    // wait
  }
  // the first byte is a command
  // 'R' = read program via serial onto the arduino
  // 'P' = program the ATtiny using the read program
  // 'V' = verify the program on the ATtiny
  // 'D' = dump memory to serial monitor
  // 'E' = erase chip. erases current program memory.(done automatically by 'P')
  // 'F' = finalize. Just disables NVM. Not necessary.
  // 'S' = set fuse
  // 'C' = clear fuse
  
  char comnd = Serial.read();
  if(comnd == 'D'){
    dumpMemory();
  }else if(comnd == 'R'){
    if(receiveProgram()){
      Serial.println("program received");
    }
  }else if(comnd == 'P'){
    if(progSize > 0){
      writeProgram();
    }else{
      Serial.println("no program data read");
    }
  }else if(comnd == 'E'){
    eraseChip();
  }else if(comnd == 'V'){
    if(verifyProgram()){
      Serial.println("program verified: correct");
    }
  }else if(comnd == 'F'){
    finish();
    Serial.println("everything finished. unplug and disconnect wires");
    return;
  }else if(comnd == 'S'){
    setConfig(true);
  }else if(comnd == 'C'){
    setConfig(false);
  }else{
    Serial.println("received unknown command");
  }
}

//  print the register, SRAM, config and signature memory
void dumpMemory(){
  uint8_t i;
  // initialize memory pointer register 
  setPointer(0x0000);
  
  Serial.println("Current memory state:");
  // read the memory up to 0x4400
  while(adrs < 0x4400){
    // read the byte at the current pointer address
    // and increment address
    tpi_send_byte(SLDp);
    b = tpi_receive_byte(); // get data byte
    
    // read all the memory, but only print
    // the register, SRAM, config and signature memory
    if ((0x0000 <= adrs && adrs <= 0x005F) // register/SRAM
       |(0x3F00 <= adrs && adrs <= 0x3F01) // NVM lock bits
       |(0x3F40 <= adrs && adrs <= 0x3F41) // config
       |(0x3F80 <= adrs && adrs <= 0x3F81) // calibration
       |(0x3FC0 <= adrs && adrs <= 0x3FC3) // ID
       |(0x4000 <= adrs && adrs <= 0x43FF) ) { // program
      // print +number along the top
      if ((0x00 == adrs)
          |(0x3f00 == adrs) // NVM lock bits
          |(0x3F40 == adrs) // config
          |(0x3F80 == adrs) // calibration
          |(0x3FC0 == adrs) // ID
          |(0x4000 == adrs) ) {
        outNewline(); 
        if(adrs == 0x0000){ Serial.print("registers, SRAM"); }
        if(adrs == 0x3F00){ Serial.print("NVM lock"); }
        if(adrs == 0x3F40){ Serial.print("configuration"); }
        if(adrs == 0x3F80){ Serial.print("calibration"); }
        if(adrs == 0x3FC0){ Serial.print("device ID"); }
        if(adrs == 0x4000){ Serial.print("program"); }
        outNewline();
        for (i = 0; i < 5; i++) { outChar(' '); }
        for (i = 0; i < 16; i++) {
          outChar(' '); outChar('+');
          outHex1(i);
        }
      }
      // print number on the left
      if (0 == (0x000f & adrs)) {
        outNewline();
        outHex4(adrs); // print address in hex 4 digits
        outChar(':'); // delimiter
      }
      outChar(' '); // delimiter
      outHex2(b); // print data in hex 2 digits 
    }
    adrs++; // increment memory address
    if(adrs == 0x0060){
      // skip reserved memory
      setPointer(0x3F00);
    }
  }
  Serial.println(" ");
} // end dumpMemory()

// receive and translate the contents of a hex file
// The maximum program size is 1024 bytes, so this should all fit in SRAM
boolean receiveProgram(){
  char datlength[] = "00";
  char addr[] = "0000";
  char something[] = "00";
  char chksm[] = "00";
  unsigned int currentByte = 0;
  progSize = 0;
  uint8_t linelength = 0;
  boolean fileEnd = false;
  
  // read in the data and 
  while(!fileEnd){
    startTime = millis();
    while(Serial.available() < 10){
      if(millis()-startTime > timeout){ 
        Serial.println("couldn't receive data:");
        Serial.println("timed out");
        return false; 
      }
    }
    if(Serial.read() != ':'){ // maybe it was a newline??
      if(Serial.read() != ':'){
        Serial.println("couldn't receive data:");
        Serial.println("hex file format error");
        return false; 
      }
    }
    // read data length
    datlength[0] = Serial.read();
    datlength[1] = Serial.read();
    linelength = byteval(datlength[0], datlength[1]);
    
    // read address. if "0000" currentByte = 0
    addr[0] = Serial.read();
    addr[1] = Serial.read();
    addr[2] = Serial.read();
    addr[3] = Serial.read();
    if(linelength != 0x00 && addr[0]=='0' && addr[1]=='0' && addr[2]=='0' && addr[3]=='0'){ currentByte = 0; }
    
    // read type thingy. "01" means end of file
    something[0] = Serial.read();
    something[1] = Serial.read();
    if(something[1] == '1'){
      fileEnd = true;
    }
    
    // read in the data 
    for(int k=0; k<linelength; k++){
      if(currentByte == 1024){
        Serial.println("couldn't receive data:");
        Serial.println("program is too large");
        return false;
      }
      startTime = millis();
      while(Serial.available() < 2){
        if(millis()-startTime > timeout){ 
          Serial.println("couldn't receive data:");
          Serial.println("timed out");
          return false; 
        }
      }
      data[currentByte] = byteval(Serial.read(), Serial.read());
      currentByte++;
    }
    
    // read in the checksum.
    startTime = millis();
    while(Serial.available() < 2){
      if(millis()-startTime > timeout){ 
        Serial.println("couldn't receive data:");
        Serial.println("timed out");
        return false; 
      }
    }
    chksm[0] = Serial.read();
    chksm[1] = Serial.read();
    
  }
  progSize = currentByte;
  // we need an even number of bytes for word writing
  if(progSize & 0x0001){
    data[progSize] = 0x00;
    progSize++;
  }
  
  // the program was successfully read
  return true;
}

// write the data[] to program memory
void writeProgram(){
  if(progSize < 1){
    Serial.println("program size is 0??");
    return;
  }
  
  // erase the chip
  eraseChip();
  
  setPointer(0x4000);
  adrs = 0x4000;
  writeIO(NVMCMD, NVM_WORD_WRITE);
  
  // now write all the bytes to program memory
  // write two bytes at a time (a word) and wait
  for(int k=0; k<progSize-1; k=k+2){
    tpi_send_byte(SSTp);
    tpi_send_byte(data[k]); // LSB first
    tpi_send_byte(SSTp);
    tpi_send_byte(data[k+1]); // then MSB
    while((readIO(NVMCSR) & (1<<7)) != 0x00){
      // wait for write to finish
    }
  }
  
  writeIO(NVMCMD, NVM_NOP);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  
  Serial.print("Wrote program: ");
  Serial.print(progSize, DEC);
  Serial.println(" of 1024 bytes");
}

void eraseChip(){
  // initialize memory pointer register 
  setPointer(0x4001); // need the +1 for chip erase
  
  // erase the chip
  writeIO(NVMCMD, NVM_CHIP_ERASE);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  while((readIO(NVMCSR) & (1<<7)) != 0x00){
    // wait for erasing to finish
  }
  Serial.println("chip erased");
}

boolean verifyProgram(){
  if(progSize < 1){
    Serial.println("program size is 0??");
    return false;
  }
  
  boolean correct = true;
  unsigned short ind = 0;
  setPointer(0x4000);
  while(ind < progSize){
    tpi_send_byte(SLDp);
    b = tpi_receive_byte(); // get data byte
    
    if(b != data[ind]){
      correct = false;
      Serial.println("program error:");
      Serial.print("byte ");
      outHex4(ind);
      Serial.print(" expected ");
      outHex2(data[ind]);
      Serial.print(" read ");
      outHex2(b);
      outNewline();
    }
    ind++;
  }
  return correct;
}

void setConfig(boolean val){
  // get current config byte
  setPointer(0x3F40);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  
  Serial.println("input one of these letters");
  Serial.println("c = system clock output");
  Serial.println("w = watchdog timer on");
  Serial.println("r = disable reset");
  Serial.println("x = cancel. don't change anything");
  
  while(Serial.available() < 1){
    // wait
  }
  char comnd = Serial.read();
  setPointer(0x3F40);
  writeIO(NVMCMD, (val ? NVM_WORD_WRITE : NVM_SECTION_ERASE) );
  
  if(comnd == 'c'){
    tpi_send_byte(SSTp);
	if(val){
	  tpi_send_byte(b & 0b11111011);
    }else{
      tpi_send_byte(b | 0x04);
    }
	tpi_send_byte(SSTp);
	tpi_send_byte(0xFF);
  }else if(comnd == 'w'){
    tpi_send_byte(SSTp);
	if(val){
	  tpi_send_byte(b & 0b11111101);
    }else{
      tpi_send_byte(b | 0x02);
    }
	tpi_send_byte(SSTp);
	tpi_send_byte(0xFF);
  }else if(comnd == 'r'){
    tpi_send_byte(SSTp);
	if(val){
	  tpi_send_byte(b & 0b11111110);
    }else{
      tpi_send_byte(b | 0x01);
    }
	tpi_send_byte(SSTp);
	tpi_send_byte(0xFF);
  }else if(comnd == 'x'){
    // do nothing
  }else{
    Serial.println("received unknown command. cancelling");
  }
  while((readIO(NVMCSR) & (1<<7)) != 0x00){
    // wait for write to finish
  }
  writeIO(NVMCMD, NVM_NOP);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
}


void finish(){
  writeCSS(0x00, 0x00);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  // digitalWrite(SS, HIGH); // release RESET
  delay(1); // t_RST min = 400 ns @ Vcc = 5 V
}

void checkID(){
  // check the device ID
  uint8_t id1, id2, id3;
  setPointer(0x3FC0);
  
  tpi_send_byte(SLDp);
  id1 = tpi_receive_byte();
  tpi_send_byte(SLDp);
  id2 = tpi_receive_byte();
  tpi_send_byte(SLDp);
  id3 = tpi_receive_byte();
  if(id1==0x1E && id2==0x90 && id3==0x03){
    Serial.println("ATtiny10 connected");
  }else{
    Serial.println("Unknown chip connected!!");
  }
}

/* 
* send a byte in one TPI frame (12 bits)
* (1 start + 8 data + 1 parity + 2 stop)
* using 2 SPI data bytes (2 x 8 = 16 clocks)
* (with 4 extra idle bits)
*/
void tpi_send_byte( uint8_t data ){
  // compute partiy bit
  uint8_t par = data;
  par ^= (par >> 4); // b[7:4] (+) b[3:0]
  par ^= (par >> 2); // b[3:2] (+) b[1:0]
  par ^= (par >> 1); // b[1] (+) b[0]
  
  // REMEMBER: this is in LSBfirst mode and idle is high
  // (2 idle) + (1 start bit) + (data[4:0])
  SPI.transfer(0x03 | (data << 3));
  // (data[7:5]) + (1 parity) + (2 stop bits) + (2 idle)
  SPI.transfer(0xf0 | (par << 3) | (data >> 5));
} // end tpi_send_byte()

/*
* receive TPI 12-bit format byte data
* via SPI 2 bytes (16 clocks) or 3 bytes (24 clocks)
*/
uint8_t tpi_receive_byte( void ){
  //uint8_t b1, b2, b3;
  // keep transmitting high(idle) while waiting for a start bit
  do {
    b1 = SPI.transfer(0xff);
  } while (0xff == b1);
  // get (partial) data bits
  b2 = SPI.transfer(0xff);
  // if the first byte(b1) contains less than 4 data bits
  // we need to get a third byte to get the parity and stop bits
  if (0x0f == (0x0f & b1)) { 
    b3 = SPI.transfer(0xff);
  }
  
  // now shift the bits into the right positions
  // b1 should hold only idle and start bits = 0b01111111
  while (0x7f != b1) { // data not aligned
    b2 <<= 1; // shift left data bits
    if (0x80 & b1) { // carry from 1st byte
      b2 |= 1; // set bit
    }
    b1 <<= 1;
    b1 |= 0x01; // fill with idle bit (1)
  }
  // now the data byte is stored in b2
  return( b2 );
} // end tpi_receive_byte()

// send the 64 bit NVM key
void send_skey(uint64_t nvm_key){
  tpi_send_byte(SKEY);
  while(nvm_key){
    tpi_send_byte(nvm_key & 0xFF);
    nvm_key >>= 8;
  }
} // end send_skey()

// sets the pointer address
void setPointer(unsigned short address){
  adrs = address;
  tpi_send_byte(SSTPRL);
  tpi_send_byte(address & 0xff);
  tpi_send_byte(SSTPRH);
  tpi_send_byte((address>>8) & 0xff);
}

// writes using SOUT
void writeIO(uint8_t address, uint8_t value){
  //  SOUT 0b1aa1aaaa replace a with 6 address bits 
  tpi_send_byte(0x90 | (address & 0x0F) | ((address & 0x30) << 1));
  tpi_send_byte(value);
}

// reads using SIN
uint8_t readIO(uint8_t address){
  //  SIN 0b0aa1aaaa replace a with 6 address bits 
  tpi_send_byte(0x10 | (address & 0x0F) | ((address & 0x30) << 1));
  return tpi_receive_byte();
}

// writes to CSS
void writeCSS(uint8_t address, uint8_t value){
  tpi_send_byte(0xC0 | address);
  tpi_send_byte(value);
}

// reads from CSS
uint8_t readCSS(uint8_t address){
  tpi_send_byte(0x80 | address);
  return tpi_receive_byte();
}

// converts two chars to one byte
// c1 is MS, c2 is LS
uint8_t byteval(char c1, char c2){
  uint8_t by;
  if(c1 <= '9'){
    by = c1-'0';
  }else{
    by = c1-'A'+10;
  }
  by = by << 4;
  if(c2 <= '9'){
    by += c2-'0';
  }else{
    by += c2-'A'+10;
  }
  return by;
}

/*
* The following were used by tpitest.pde
*/
void outChar( char c ){
  Serial.print(c);  
}

void outNewline( void ){
  Serial.println();  
}

 void outHex1(uint8_t n){
  Serial.print(0x0f & n, HEX);  
}

void outHex2(uint8_t n){
  outHex1(n >> 4); outHex1(n);
}

void outHex4(uint16_t n){
  outHex2(n >> 8); outHex2(n);
}

// end of file
