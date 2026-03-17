
//   4MHz/ 4  = 1MHz (machine cycle)^-1 = 0,000001s  <--> 1us

//                     Overflow                      256E-6
//   TMR1 = --------------------------------   =  ----------  = 256
//            prescaler x machine cycle            1 x 1E-6
//
//   TMR1 = 256      18000

//whole TMR1 register = 65536 - 256 = 65280
//TMR1H = (65280/ 2^8) = 255   (also calculated as 65280 >> 8) shift right is a division
//TMR1L =

// Timer2 based PWM frequency = 244.14 Hz as follows:
// PWM period = (PR2 + 1) * machine cycle * TMR2 prescaler
// (255 + 1) * 0,0001s * 16 = 4.096ms ^-1 = 244.14 Hz

// 8 bits duty cicle:
// TMR2 = PR2 + 1 (when TMR2 overflows, LOW to HIGH)/ CCPR1L:CCP1CON<5:4>


sbit PWM_OUT at RB3_bit;                                                        // Pin9: PWM control output
sbit POWER at RB5_bit;                                                          // Pin11: power on/ off indicator
sbit SENSOR_DATA at RA0_bit;                                                    // Pin17: DHT11 sensor (humidity + temperature)
sbit DATA_DIR at TRISA0_bit;                                                    // Dinamic I/O configuration

#define WAIT_TIME   1200                                                        // Empiric value, adj to avoid CheckSum error: DHT11 sensor read cycle can't be faster than 1s
#define BUFFER_SIZE 4                                                           // Store the received UART string + null terminator
#define OFFSET_TEMP 2                                                           // Temperature offset for DHT11 sensor
#define OFFSET_HUM  7                                                           // Humidity offset for DHT11 sensor

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Local functions
void PWM();
void StartSignal();
unsigned short CheckResponse();
unsigned short ReadByte();
void sensor();
void sendPWM();
void sendData(char num);
void sendGraph(char id, unsigned short val);
void comData();
void cleanBuffer();

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Global variables

unsigned short TOUT = 0, CheckSum;                                              // TOUT = time out
unsigned short T_Byte1, T_Byte2, RH_Byte1, RH_Byte2;
char TEMP[3] = " ", buffer[BUFFER_SIZE], REC, index = 0;                        // REC = received
char pwmBuffer[4] = "000";                                                      // Buffer to store PWM value string
char i, PWM_VALUE = 0x00;                                                       // PWM_VALUE: process the data received from HMI
volatile unsigned char uartDataReady = 0;                                       // Flag to indicate new UART data received
unsigned int sensorTick = 0;                                                    // Local counter to control DHT11 read cycle

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//UART interruption
void interrupt(){
    if(RCIF_bit) {                                                              // Interrupt occurred on USART receive
        if(OERR_bit) {                                                          // Check for overrun error FIRST
            CREN_bit = 0;                                                       // Reset the receiver logic
            CREN_bit = 1;
            REC = RCREG;                                                        // Clear the FIFO
            return;                                                             // Exit interrupt handler
        }

        if(FERR_bit) {                                                          // Check for framing error
            REC = RCREG;                                                        // Discard the corrupted byte
            return;
        }

        REC = RCREG;                                                            // Read the data (clears RCIF_bit when finish)

        if((REC >= '0' && REC <= '9')) {                                        // ONLY store characters that are numbers.
          if(index < BUFFER_SIZE - 1) {
             buffer[index++] = REC;                                             // Store received character
             buffer[index] = '\0';                                              // Null-terminate the string
            }
          }

        if(index == 3){                                                         // Full string received (without null terminator)
            if(strcmp(buffer, "300") == 0){                                     // "RUN" command received
               POWER = 1;
               CCP1CON = 0x0C;                                                  // Restore PWM
               uartDataReady = 1;                                               // Set flag to indicate data received
            }
                else if(strcmp(buffer, "400") == 0){                            // "STOP" command received
                   POWER = 0;
                   CCP1CON = 0x00;                                              // Kill PWM
                   PWM_OUT = 0;                                                 // Ensure 0V
                   uartDataReady = 1;
                }

                else if(strcmp(buffer, "500") == 0){                            // "EMERGENCY STOP" command received
                   POWER = 0;                                                   // Stop
                   CCP1CON = 0x00;                                              // Kill PWM
                   PWM_OUT = 0;                                                 // Ensure 0V
                   uartDataReady = 1;
                }

            else{
            strcpy(pwmBuffer, buffer);                                          // 0-100 = update pwmBuffer
            uartDataReady = 1;                                                  // Set flag to indicate data received
          }
           index = 0;                                                           // Reset buffer index for next string
        }
    }

    if(PIR1.TMR1IF){                                                            // Interrupt after 256us (sensor time out)
       PIR1.TMR1IF  = 0;                                                        // Clear the interrupt flag
       TOUT = 1;                                                                // Set the time out flag
       T1CON.TMR1ON = 0;                                                        // Stop TMR1
       SENSOR_DATA  = 1;                                                        // Set pin17 to HIGH
      }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void main() {

     CMCON       = 0x07;               // disable comparators
     OPTION_REG  = 0x86;               // RBPU = 1 (PortB pull ups disabled) | PSA = 0 (Prescaler assign. Timer0| PS<2:0> = 110 (Prescaler 1:128)
     GIE_bit     = 0x01;               // bit 7 of INTCON register (Global Interrupt Enable bit)
     PEIE_bit    = 0x01;               // bit 6 of INTCON register (Peripheral Interrupt Enable bit)

     PIE1.TMR1IE = 0x01;               // enable Timer1 interrupt

     PR2         = 0xFF;               // starts the TMR2 overflow register at maximum count: 255
     T2CON       = 0x06;               // enable TMR2 and prescaler as 1:16
     CCP1CON     = 0x0C;               // enable PWM  mode(12)
     CCPR1L      = 0x00;               // start the PWM in 0%
     
     delay_ms(1000);                   // delay before start the USART
     RCIF_bit    = 0x00;               // clear the USART interruption flag (located at PIR1 register)
     RCIE_bit    = 0x01;               // enable UART interruption for data reception (located at PIE1 register)
     SPBRG       = 0x19;               // set USART (8 bits, no parity, 1 stop bit, 9600, asynchronous)
     TXEN_bit    = 0x01;               // enable transmission (TXSTA register)
     BRGH_bit    = 0x01;               // baudrate at high speed (TXSTA register)
     SYNC_bit    = 0x00;               // asynchronous mode (TXSTA register)
     SPEN_bit    = 0x01;               // enable serial port (RCSTA register)
     CREN_bit    = 0x01;               // enable continuous reception (RCSTA register)

     TRISA       = 0x01;               // RA0 = input
     PORTA       = 0x01;               // RA0 start in HIGH, the rest in LOW
     TRISB       = 0x02;               // RB1 (RX) as input, the rest as output
     PORTB       = 0x02;               // RB1 starts in HIGH, the rest in LOW


  while(1){

    if(uartDataReady){
        PWM_VALUE = atoi(pwmBuffer);                                            // Update PWM_VALUE with the received string conv to int
        uartDataReady = 0;
        PWM();
    }
    sensorTick++;
    if(sensorTick > WAIT_TIME) {                                                // minimal time between DHT11 reading = 1s
        StartSignal();
        sensor();
        sensorTick = 0;
    }
    Delay_ms(1);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Send data to the HMI main page + update the graphs
void PWM() {
     CCPR1L = PWM_VALUE;                                                            // the PWM output is updated
     TEMP[0]= ((int)(PWM_VALUE/2.53)/100%10) + 48;                                  // Send variable data to IHM Nextion. 2.53 is only to convert into percentage at slider.
     TEMP[1]= ((int)(PWM_VALUE/2.53)/10%10) + 48;
     TEMP[2]= ((int)(PWM_VALUE/2.53)%10) + 48;
     sendPWM();

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
 void StartSignal(){
      DATA_DIR = 0;
      SENSOR_DATA  = 0;
      Delay_ms(18);    // Low for at least 18us     18
      SENSOR_DATA    = 1;

      T1CON       = 0x00;               // prescaler 1:1, and Timer1 is off initially
      PIR1.TMR1IF = 0x00;               // clear TMR INT Flag bit
      TMR1L       = 0x00;                // Inicializa o TMR1L em 0
      TMR1H       = 0xFF;                // Inicializa o TMR1H em 255
      T1CON.TMR1ON = 1;

      if(TMR1L == 22){
         DATA_DIR = 1;     // Data port is input
         T1CON.TMR1ON = 0;
         TMR1L       = 0x00;                // Inicializa o TMR1L em 0
         TMR1H       = 0xFF;                // Inicializa o TMR1H em 255
      }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
unsigned short CheckResponse(){
  TOUT = 0;
  TMR1L       = 0x00;                // Inicializa o TMR1L em 0
  TMR1H       = 0xFF;                // Inicializa o TMR1H em 255
  T1CON.TMR1ON = 1;                  // Start TMR1 while waiting for sensor response
  
  while(!SENSOR_DATA && !TOUT);       // If there's no response within 256us, the Timer1 overflows
  if (TOUT) 
     return 0;    // and exit
     
  else {
      TMR1L       = 0x00;                // Inicializa o TMR1L em 0
      TMR1H       = 0xFF;                // Inicializa o TMR1H em 255
      while(SENSOR_DATA && !TOUT);
      if (TOUT)
         return 0;
       
      else {
       T1CON.TMR1ON = 0;
       return 1;
      }
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
unsigned short ReadByte(){
  unsigned short num = 0, i;
  DATA_DIR = 1;
  
  for (i=0; i<8; i++){
       while(!SENSOR_DATA);
       TMR1L       = 0x00;                // Inicializa o TMR1L em 0
       TMR1H       = 0xFF;                // Inicializa o TMR1H em 255
       T1CON.TMR1ON = 1;  // Start TMR2 from 0 when a low to high data pulse
       while(SENSOR_DATA);       // is detected, and wait until it falls low again.
       T1CON.TMR1ON = 0;  // Stop the TMR2 when the data pulse falls low.
       
       if(TMR1L > 40) 
          num |= 1<<(7-i);  // If time > 40us, Data is 1
      }
  return num;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void sensor(){
  unsigned short check;
  static char dataSend = 0x00;                                                   // count the time to make a "delay" to send data to the graph
  check = CheckResponse();

  if (!check){
   // SENSOR TIMEOUT: Tell HMI there is an error writting in the UART: er0.val=1
    TXREG = 'e'; cleanBuffer(); TXREG = 'r'; cleanBuffer();
    TXREG = '0'; cleanBuffer(); TXREG = '.'; cleanBuffer(); TXREG = 'v'; cleanBuffer();
    TXREG = 'a'; cleanBuffer(); TXREG = 'l'; cleanBuffer(); TXREG = '='; cleanBuffer();
    TXREG = '1'; cleanBuffer(); // Set er0 to 1 (NOK)
    TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer();
    
    POWER = 0;
    CCP1CON = 0x00;                                                             // Kill PWM
    PWM_OUT = 0;                                                                // Ensure 0V
  }

  else{
    RH_Byte1 = ReadByte();
    RH_Byte2 = ReadByte();
    T_Byte1 = ReadByte();
    T_Byte2 = ReadByte();
    CheckSum = ReadByte();
    // Check for error in Data reception
    if (CheckSum == ((RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2) & 0xFF)){
      // SUCCESS: Clear error flag and write in the UART: er0.val=0
      TXREG = 'e'; cleanBuffer(); TXREG = 'r'; cleanBuffer();
      TXREG = '0'; cleanBuffer(); TXREG = '.'; cleanBuffer(); TXREG = 'v'; cleanBuffer();
      TXREG = 'a'; cleanBuffer(); TXREG = 'l'; cleanBuffer(); TXREG = '='; cleanBuffer();
      TXREG = '0'; cleanBuffer(); // Set er0 to 0 (OK)
      TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer();
      
      // --- Calibration of DTH11 sensor (Offsets) ---
      if(T_Byte1 > 2) T_Byte1 -= OFFSET_TEMP;   // Subtract 2 degrees
      else T_Byte1 = 0;              // Prevent negative rollover

      if(RH_Byte1 > 3) RH_Byte1 -= OFFSET_HUM; // Subtract 3 percent
      else RH_Byte1 = 0;             // Prevent negative rollover
      
      // Update Humidity
      TEMP[0] = RH_Byte1/10 + 48;
      TEMP[1] = RH_Byte1%10 + 48;
      TEMP[2] = RH_Byte2/10 + 48;
      sendData('1');
      sendGraph('1', RH_Byte1);                                                 // Sends raw value (e.g., 40)


      // Update Temperature
      TEMP[0] = T_Byte1/10 + 48;
      TEMP[1] = T_Byte1%10 + 48;
      TEMP[2] = T_Byte2/10 + 48;
      sendData('0');
      sendGraph('0', T_Byte1);                                                  // Sends raw value (e.g., 21)

    }

    else{
      // CHECKSUM FAIL: Treat as error
      TXREG = 'e'; cleanBuffer(); TXREG = 'r'; cleanBuffer();
      TXREG = '0'; cleanBuffer(); TXREG = '.'; cleanBuffer(); TXREG = 'v'; cleanBuffer();
      TXREG = 'a'; cleanBuffer(); TXREG = 'l'; cleanBuffer(); TXREG = '='; cleanBuffer();
      TXREG = '0'; cleanBuffer(); // Set er0 to 0 (OK)
      TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer(); TXREG = 0xFF; cleanBuffer();
    }

  }

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
 void sendPWM(){
   TXREG = 'n';                                                                 //Write in the UART: n0.val=
   cleanBuffer();                                                               //where "n0" is the % field ID in the main page
   TXREG = '0';                                                                 //Send char
   cleanBuffer();                                                               //Wait for buffer cleaning
   TXREG = '.';
   cleanBuffer();
   TXREG = 'v';
   cleanBuffer();
   TXREG = 'a';
   cleanBuffer();
   TXREG = 'l';
   cleanBuffer();
   TXREG = '=';
   cleanBuffer();
   comData();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
 void sendData(char num)                                                        //Write in the UART:  x"num".val=
{                                                                               //"num" is the field ID in the main page (x0 = Temperature, x1 = Humidity)
   TXREG = 'x';                                                                 //Send char
   cleanBuffer();                                                               //Wait for buffer cleaning
   TXREG = num;
   cleanBuffer();
   TXREG = '.';
   cleanBuffer();
   TXREG = 'v';
   cleanBuffer();
   TXREG = 'a';
   cleanBuffer();
   TXREG = 'l';
   cleanBuffer();
   TXREG = '=';
   cleanBuffer();
   comData();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
 void sendGraph(char id, unsigned short val){
   TEMP[0] = (val / 100) + 48;       // Hundreds
   TEMP[1] = ((val / 10) % 10) + 48; // Tens
   TEMP[2] = (val % 10) + 48;        // Units
   
  // Start the command: gr
   TXREG = 'g'; cleanBuffer();
   TXREG = 'r'; cleanBuffer();
   
   // Select 0 for Temp (id 6) or 1 for Hum (id 7)
   if(id == '0') { TXREG = '0';
          } else { TXREG = '1'; }
   cleanBuffer();
   
   // Finish the command: .val=
   TXREG = '.'; cleanBuffer();
   TXREG = 'v'; cleanBuffer();
   TXREG = 'a'; cleanBuffer();
   TXREG = 'l'; cleanBuffer();
   TXREG = '='; cleanBuffer();

   // Send the 3 digits as a string
   TXREG = TEMP[0]; cleanBuffer();
   TXREG = TEMP[1]; cleanBuffer();
   TXREG = TEMP[2]; cleanBuffer();

   // End of Nextion command
   TXREG = 0xFF; cleanBuffer();
   TXREG = 0xFF; cleanBuffer();
   TXREG = 0xFF; cleanBuffer();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void comData()                                                                  //common measured data (can be temperature or humidity)
{
   TXREG = TEMP[0];                                                             //Send char
   cleanBuffer();                                                               //Wait for buffer cleaning
   TXREG = TEMP[1];
   cleanBuffer();
   TXREG = TEMP[2];
   cleanBuffer();
   TXREG = 0xFF;
   cleanBuffer();
   TXREG = 0xFF;
   cleanBuffer();
   TXREG = 0xFF;
   cleanBuffer();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void cleanBuffer()                                                              //Clean the buffer
{
   while(!TRMT_bit);                                                            //while loop until the buffer is not empty
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//