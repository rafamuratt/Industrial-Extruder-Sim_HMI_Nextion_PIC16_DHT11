/* 
   MURAT-TECH HMI DIGITAL TWIN INTERFACE v.1.00
   By B.Eng. Rafa Muratt 03.2026
   MURAT-TECH CHANNEL: https://www.youtube.com/@Murat-TechChannel-EN
   MURAT-TECH HUB: https://murat-tech.eu/
   
   License
   This project is licensed under the Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
   If this project is helpfull for your application, please consider to support:
   https://www.paypal.com/donate/?hosted_button_id=8S8BJ9TT368VN
   
   PIC16F628A @4Mhz (internal oscillator) = 1us machine cycle
   Digital twin application (9600bps UART communication with the HMI Nextion Intelligent series)
   No libraries where used, nor strings in order to save memory (bare metal coding + short char commands to the HMI)
   
   The following definitions may need your tailored value:
   WAIT_TIME: empiric data, assure a minimum 1s sensor read period. If faster, the sensor will be flooded = CheckSum error. In my case, 1200 is OK
   OFFSET_TEMP and OFFSET_HUM: in comparisson with a calibrated thermo-hygrometer, you my need a "kind of calibration" to get the same readings
   
   <<< Tasks from MCU: >>>
   -- On every restart send a reset message to the HMI:
      res.val=1 (so the HMI knows a reset happen and goes to the "System Stopped" status
      
   -- Check the DHT11 sensor "alive" status and update the temperature and humidity
      It sends to the HMI: 
      er0.val=1 (sensor error) | er0.val=0 (sensor OK)
      x0.val=XXX | x1.val=XXX (x0 and x1 are temperature and humidity fields in the main page); XXX = "string" between 000 to 999
      gr0.val=XXX | gr1.val=XXX (gr0 and gr1 are variables to control the temperature and humidity graphs in the main page; XXX = "string" between 000 to 999

   -- Check the hardware emergency button (NC) on pin6
      In case the emergency is pressed, sends to the HMI:
      er1.val=1 (shut down outputs, pop up emergency error message and lock the HMI controls
      To return the operation: release the emergency button, reset the MCU and press the ACK HMI button
      
   -- Check the UART data if the following strings are received:
      "500" (Software based Emergency Stop), the HMI emergency button is pressed and lock the MCU software
      "600" (ACK received): if there was a sensor error, set the error flag to 0. If there was an emergency stop, reset the MCU (goes to addr 0x0000)
      "400" (STOP received): PWM_OUT and POWER (indicator) are switched off
      "300" (RUN received):  PWM_OUT and POWER (indicator) are switched on
      "000" to "255" (PWM data string): set the MCU PWM accordingly (data sent by the slider adjust + OK button in the HMI, page 1)
      
   -- Send the PWM callback "string" to the n0 field in the HMI page1
      n0.val=XXX (where XXX = "string" between 000 to 255)
      
   <<< Tasks from HMI: >>>
   -- Basically to hear the errors and messages sent by the MCU
   -- Its own firmware (not included here) has an interlock logic with the MCU and behaves like in the real world.
   -- It also send the string commands listed above, handle graphics and animations   */

// MCU TIMERS CALCULATION:
// 4MHz/ 4  = 1MHz (machine cycle)^-1 = 0,000001s

// ========== Timer1 check if the sensor is "alive" + read its data  ==========
//
//                     Overflow                      256E-6
//   TMR1 = --------------------------------   =  ----------  = 256
//            prescaler x machine cycle            1 x 1E-6
//
//   TMR1 = 256
//   The whole Timer1 register = 65536 - 256 = 65280
//   TMR1H = (65280/ 256) = 255
//
//   Parameters used in startSensor() and sensorAlive() functions:
//   TMR1H = 255 (0xFF)
//   TMR1L =  0  (0x00)


// =========== Timer2 for PWM frequency @ 244.14 Hz: as follows    ============
// PWM period = (PR2 + 1) * machine cycle * TMR2 prescaler
// (255 + 1) * 0,000001s * 16 = 4.096ms ^-1 = 244.14 Hz
// 8 bits duty cicle:
// TMR2 = PR2 + 1 (when TMR2 overflows, LOW to HIGH)/ CCPR1L:CCP1CON<5:4>


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//IOs and code definitions
sbit EMG at RB0_bit;                                                            // Pin6: Emergency check
sbit PWM_OUT at RB3_bit;                                                        // Pin9: PWM control output
sbit POWER at RB5_bit;                                                          // Pin11: power on/ off indicator
sbit SENSOR_DATA at RA0_bit;                                                    // Pin17: DHT11 sensor (humidity + temperature)
sbit DATA_DIR at TRISA0_bit;                                                    // Dinamic I/O configuration

#define       BUFFER_SIZE        4                                              // Store the received UART string + null terminator
#define       WAIT_TIME          1200                                           // Empiric value, adj to avoid CheckSum error: DHT11 sensor read cycle can't be faster than 1s
#define       OFFSET_TEMP        2                                              // Temperature offset for DHT11 sensor
#define       OFFSET_HUM         6                                              // Humidity offset for DHT11 sensor


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Local functions
void PWM();                                                                     // Update the PWM output + send callback to the HMI main page "n0 field"

void startSensor();                                                             // Send start pulse to sensor DHT11
void sensorHandling();                                                          // DHT11 Sensor data handling
unsigned short sensorAlive();                                                   // Check if the sensor is alive
unsigned short sensorReadByte();                                                // Build the sensor data byte

void sendPWM();                                                                 // Update the HMI with current PWM
void sendData(char val);                                                        // Update the HMI with current temperature/ humidity
void sendGraph(char id);                                                        // Update the HMI graphs with current temperature/ humidity
void errorState(char id, char val);                                             // Update the HMI with current errors

void hmi_nameParameter();                                                       // Add ".val=" to the string
void hmi_commonData();                                                          // Add a "common measured data" to the string (can be the power, temperature or humidity)
void hmi_endCommand();                                                          // Add the "end of Nextion command" to the string
void cleanBuffer();                                                             // Clean the UART buffer


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Global variables
unsigned short TOUT = 0, CheckSum;                                              // TOUT = time out
unsigned short T_Byte1, T_Byte2, RH_Byte1, RH_Byte2;
unsigned int sensorTick = 0;                                                    // Local counter to control DHT11 read cycle
char TEMP[3] = " ", buffer[BUFFER_SIZE], REC, index = 0;                        // REC = received
char pwmBuffer[4] = "000";                                                      // Buffer to store PWM value string
char i, PWM_VALUE = 0;                                                          // PWM_VALUE: process the data received from HMI
volatile unsigned char uartDataReady = 0;                                       // Flag to indicate new UART data received. Volatile forces RAM check every single loop cycle
volatile char softwareEmgState = 0, hardwareEmgState = 0, sensorError = 0;      // 1 = Software / Hardware emergency button is pressed/ DHT11 sensor with error


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void interrupt(){
    if(INTF_bit){
        // Hardware Emergency button interruption
        hardwareEmgState = 1;
        CCP1CON = 0x00;                                                         // Cut the output (Kill PWM)
        PWM_OUT = 0;                                                            // Ensure 0V
        POWER = 0;                                                              // Run indicator = OFF
        INTF_bit = 0;                                                           // Clear the flag
     }
    
    if(!hardwareEmgState){                                                      // hw emergency is released
        if(RCIF_bit){
        //UART interruption                                                     // Interrupt occurred on USART receive
            if(OERR_bit){                                                       // Check for overrun error FIRST
                CREN_bit = 0;                                                   // Reset the receiver logic
                CREN_bit = 1;
                REC = RCREG;                                                    // Clear the FIFO
                return;                                                         // Exit interrupt handler
            }

            if(FERR_bit){                                                       // Check for framing error
                REC = RCREG;                                                    // Discard the corrupted byte
                return;
            }

            REC = RCREG;                                                        // Read the data (auto clear the RCIF_bit when finish)

            if((REC >= '0' && REC <= '9')){                                     // ONLY store characters that are numbers.
              if(index < BUFFER_SIZE - 1){
                 buffer[index++] = REC;                                         // Store received character
                 buffer[index] = '\0';                                          // Null-terminate the string
                }
              }

            if(index == 3){                                                     // Full string received (without null terminator)
                if(strcmp(buffer, "300") == 0){                                 // "RUN" command received "300"
                   CCP1CON = 0x0C;                                              // Set output, restore the PWM (if disabled)
                   POWER = 1;                                                   // Run indicator = ON
                }

                else if(strcmp(buffer, "400") == 0){                            // "STOP" command received "400"
                   CCP1CON = 0x00;
                   PWM_OUT = 0;
                   POWER = 0;
                }

                else if(strcmp(buffer, "500") == 0){                            // "EMERGENCY STOP" command received "500"
                   softwareEmgState = 1;                                        // Set emergency flag
                   CCP1CON = 0x00;
                   PWM_OUT = 0;
                   POWER = 0;
                }

                else if(strcmp(buffer, "600") == 0){                            // "ACK" command received "600"
                  if(sensorError){
                    sensorError = 0;                                            // Clean the flag
                   }
                   else asm goto 0x0000;                                        // If Emergency ACK, restart uC
                }

                else strcpy(pwmBuffer, buffer);                                 // 0-100 = update pwmBuffer
                uartDataReady = 1;                                              // Set flag to indicate data received
                index = 0;                                                      // Reset buffer index for next string
            }
        }

        if(PIR1.TMR1IF){
        // SENSOR check interruption                                            // Timer1 Interrupt after 256us (sensor time out)
           PIR1.TMR1IF  = 0;                                                    // Clear the interrupt flag
           TOUT = 1;                                                            // Set the time out flag
           T1CON.TMR1ON = 0;                                                    // Stop TMR1
           SENSOR_DATA  = 1;                                                    // Set pin17 to HIGH
        }
     }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void main(){
       CMCON       = 0x07;                                                      // Disable comparators
       OPTION_REG  = 0x80;                                                      // RBPU = 1 (PortB pull ups disabled)| INTEDG = 0 (RB0/INT falling edge)
       INTE_bit    = 0x01;                                                      // Enable external interrup for emergency hardware button on pin6
       GIE_bit     = 0x01;                                                      // Bit 7 of INTCON register (Global Interrupt Enable bit)
       PEIE_bit    = 0x01;                                                      // Bit 6 of INTCON register (Peripheral Interrupt Enable bit)

       // Timer1 for sensor data
       PIE1.TMR1IE = 0x01;                                                      // Enable Timer1 interrupt

       // Timer2 for PWM
       PR2         = 0xFF;                                                      // Starts the TMR2 overflow register in the maximum count: 255
       T2CON       = 0x06;                                                      // Enable TMR2 and prescaler as 1:16 (PWM frequency = 244.14 Hz)
       CCP1CON     = 0x0C;                                                      // Enable PWM
       CCPR1L      = 0x00;                                                      // Start the PWM in 0%

       delay_ms(1000);                                                          // Delay before start the USART
       RCIF_bit    = 0x00;                                                      // Clear the USART interruption flag (located at PIR1 register)
       RCIE_bit    = 0x01;                                                      // Enable UART interruption for data reception (located at PIE1 register)
       
       SPBRG       = 0x19;                                                      // Set USART (8 bits, no parity, 1 stop bit, 9600, asynchronous)
       TXEN_bit    = 0x01;                                                      // Enable transmission (TXSTA register)
       BRGH_bit    = 0x01;                                                      // Baudrate at high speed (TXSTA register)
       SYNC_bit    = 0x00;                                                      // Asynchronous mode (TXSTA register)
       SPEN_bit    = 0x01;                                                      // Enable serial port (RCSTA register)
       CREN_bit    = 0x01;                                                      // Enable continuous reception (RCSTA register)

       TRISA       = 0x01;                                                      // RA0 = input
       PORTA       = 0x01;                                                      // RA0 start in HIGH, the rest in LOW
       TRISB       = 0x03;                                                      // RB1 (RX) as input, the rest as output
       PORTB       = 0x03;                                                      // RB1 starts in HIGH, the rest in LOW
     
       if (EMG == 0) hardwareEmgState = 1;                                      // Only start if the emergency button is released
       
       else{
           TXREG = 'r'; cleanBuffer();                                          // Send "res.val=1" (flag to indicate the PIC reset to the HMI)
           TXREG = 'e'; cleanBuffer();
           TXREG = 's'; cleanBuffer();
           hmi_nameParameter();                                                 // ".val="
           TXREG = '1'; cleanBuffer();
           hmi_endCommand();                                                    // "0xFF 0xFF 0xFF" (end of Nextion command)
       }
       
       while(1){
           if(!softwareEmgState && !hardwareEmgState){
               if(uartDataReady){
                  PWM_VALUE = atoi(pwmBuffer);                                  // Update PWM_VALUE with the received string (converted to int)
                  uartDataReady = 0;
                  PWM();
               }
               sensorTick++;
               if(sensorTick > WAIT_TIME){                                      // minimal time between DHT11 reading = 1s
                  startSensor();
                  sensorHandling();
                  sensorTick = 0;
               }
           }

           else{                                                                // Force safety
               CCP1CON = 0x00;                                                  // Cut the output (Kill PWM)
               PWM_OUT = 0;                                                     // Ensure 0V
               POWER = 0;                                                       // Run indicator = OFF
               
               while(1){                                                        // Force emergency release + reset
                 errorState('1','1');                                           // Tell HMI there is emergency error sending: "er1.val=1"
                 delay_ms(500);
               }
           }
           Delay_ms(1);
       }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Update the PWM output + send callback to the HMI main page "n0 field"
void PWM(){
     CCPR1L = PWM_VALUE;                                                        // The PWM output is updated. "2.53" is to convert one byte into percentage (for PWM slider h0).
     TEMP[0]= ((int)(PWM_VALUE/2.53)/100%10) + 48;                              // Hundreds
     TEMP[1]= ((int)(PWM_VALUE/2.53)/10%10) + 48;                               // Tens
     TEMP[2]= ((int)(PWM_VALUE/2.53)%10) + 48;                                  // Units
     sendPWM();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Send start pulse to sensor DHT11
void startSensor(){
      DATA_DIR = 0;                                                             // Set TRISA0 as output
      // Pulse generation
      SENSOR_DATA = 0;                                                          // Low on pin17 (DHT11)
      Delay_ms(18);                                                             // Keep low least 18us, the sensor detect the MCU "request signal"
      SENSOR_DATA = 1;                                                          // High

      // Timer1
      T1CON = 0x00;                                                             // Prescaler 1:1, and Timer1 is off initially
      PIR1.TMR1IF = 0x00;                                                       // Clear TMR INT Flag bit
      TMR1L = 0x00;                                                             // Set TMR1L = 0
      TMR1H = 0xFF;                                                             // Set TMR1H = 255
      T1CON.TMR1ON = 1;                                                         // Start Timer1 to keep the MCU pulse high during ca. 22us

      if(TMR1L == 22){                                                          // After 22us, reset timers (preparation to receive the sensor "response signal")
         DATA_DIR = 1;                                                          // Set TRISA0 as input
         SENSOR_DATA = 0;                                                       // Low on pin17 (DHT11)
         T1CON.TMR1ON = 0;                                                      // Stop Timer1
         TMR1L = 0x00;                                                          // Reset TMR1L = 0
         TMR1H = 0xFF;                                                          // Reset TMR1H = 255
      }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// DHT11 Sensor data handling
void sensorHandling(){
  unsigned short check;
  static char dataSend = 0x00;                                                  // Count the time to make a "delay" to send data to the graph
  check = sensorAlive();                                                        // When the sensor is OK, check = 1

  if (!check){
    errorState('0','1');                                                        // SENSOR TIMEOUT: Tell HMI there is a sensor error sending: "er0.val=1"
    CCP1CON = 0x00;                                                             // Cut the output (Kill PWM)
    PWM_OUT = 0;                                                                // Ensure 0V
    POWER = 0;                                                                  // Run indicator = OFF
    sensorError = 1;                                                            // Sensor in error state
  }

  else{
        // Get the sensor data. Note: RH_Byte2 and T_Byte2 are always "0" on DHT11 (no decimal resolution is provided)
        RH_Byte1 = sensorReadByte();
        RH_Byte2 = sensorReadByte();
        T_Byte1 = sensorReadByte();
        T_Byte2 = sensorReadByte();
        CheckSum = sensorReadByte();
        
        // Check for error in Data reception
        if ((CheckSum == (RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2) & 0xFF) && !sensorError){ // Force ACK to clear "!errorSensor"
          errorState('0','0');                                                  // SUCCESS: send to HMI: "er0.val=0"

          // Calibration of DTH11 sensor (Offsets)
          if(T_Byte1 > OFFSET_TEMP) T_Byte1 -= OFFSET_TEMP;                     // Subtract the offset
          else T_Byte1 = 0;                                                     // Prevent negative rollover

          if(RH_Byte1 > OFFSET_HUM) RH_Byte1 -= OFFSET_HUM;                     // Subtract the offset
          else RH_Byte1 = 0;                                                    // Prevent negative rollover

          // Update Humidity
          TEMP[0] = RH_Byte1/100 + 48;                                          // Hundreds
          TEMP[1] = ((RH_Byte1/10)%10) + 48;                                    // Tens
          TEMP[2] = RH_Byte1%10 + 48;                                           // Units
          sendData('1');
          sendGraph('1');

          // Update Temperature
          TEMP[0] = T_Byte1/100 + 48;
          TEMP[1] = ((T_Byte1/10)%10) + 48;
          TEMP[2] = T_Byte1%10 + 48;
          sendData('0');
          sendGraph('0');

        }

        else{
          // New CheckSum Fail: treat as error
            errorState('0', '1');                                               // Tell HMI there is a sensor error sending: "er0.val=1"
            CCP1CON = 0x00;                                                     // Cut the output (Kill PWM)
            PWM_OUT = 0;                                                        // Ensure 0V
            POWER = 0;                                                          // Run indicator = OFF
            sensorError = 1;                                                    // Sensor in error state
        }
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Check if the sensor is alive
unsigned short sensorAlive(){
      TOUT = 0;                                                                 // Reset Time Out flag
      T1CON.TMR1ON = 1;                                                         // Start TMR1 (previoulsy reseted in startSensor()

      while(!SENSOR_DATA && !TOUT);                                             // Wait for the "sensor response" on pin17 (high pulse from sensor, of ca. 80us)
      if(TOUT)                                                                  // If there's no response within 256us, the Timer1 overflows, Time Out
         return 0;                                                              // Sensor data is NOK, exit

      else{                                                                     // Reset Timer 1 to receive the sensor data
          TMR1L = 0x00;                                                         // Reset TMR1L = 0
          TMR1H = 0xFF;                                                         // Reset TMR1H = 255

          while(SENSOR_DATA && !TOUT);                                          // Wait for the "sensor data" on pin17
          if(TOUT)                                                              // If nothing comes within 256us, the Timer1 overflows, Time Out
             return 0;                                                          // Sensor data is NOK, exit

          else{
           T1CON.TMR1ON = 0;                                                    // Stop Timer1
           return 1;                                                            // Sensor data is OK, return
          }
      }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Build the sensor data byte
unsigned short sensorReadByte(){
  unsigned short num = 0, i;
  DATA_DIR = 1;                                                                 // Set TRISA0 as input

  for (i=0; i<8; i++){
       while(!SENSOR_DATA);                                                     // Wait until the first data trasition (low to high edge) comes
       
       TMR1L = 0x00;                                                            // Reset timers
       TMR1H = 0xFF;
       T1CON.TMR1ON = 1;                                                        // Start counting TMR1 from 0
       while(SENSOR_DATA);                                                      // Wait until data signal falls (high to low edge).
       T1CON.TMR1ON = 0;                                                        // Stop the TMR1

       if(TMR1L > 40)                                                           // If the pulse time > 40us, the received data is 1
          num |= 1<<(7-i);                                                      // Populate num with the corresponding sigan byte
      }
  return num;                                                                   // num returns 1 byte data
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Update the HMI with current PWM
// Write in the UART: "n0.val=", where "n0" is the "% field ID" in the main page
 void sendPWM(){
   TXREG = 'n'; cleanBuffer();
   TXREG = '0'; cleanBuffer();
   hmi_nameParameter();
   hmi_commonData();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Update the HMI with current temperature/ humidity
// Write in the UART:  "xnum.val=" where "num" is the field ID in the main page (x0 = Temperature, x1 = Humidity)
 void sendData(char val){     
   TXREG = 'x'; cleanBuffer();
   TXREG = val; cleanBuffer();
   hmi_nameParameter();
   hmi_commonData();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Update the HMI graphs with current temperature/ humidity
// Write in the UART: "grID.val=" where "ID" refers to the graph ID in the main page
 void sendGraph(char id){

   TXREG = 'g'; cleanBuffer();
   TXREG = 'r'; cleanBuffer();
   
   // Select 0 for Temp (id 25) or 1 for Hum (id 26)
   if(id == '0'){
       TXREG = '0';
   } 
   else{
       TXREG = '1';
   }
   cleanBuffer();
   hmi_nameParameter();
   hmi_commonData();
   hmi_endCommand();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Update the HMI with current errors
// Write in the UART: erID.val=1 (NOK) or erID.val=0 (OK), where "ID" is for sensor or emergengy
void errorState(char id, char val){
    TXREG = 'e'; cleanBuffer();
    TXREG = 'r'; cleanBuffer();
    TXREG = id; cleanBuffer();
    hmi_nameParameter();
    TXREG = val; cleanBuffer();
    hmi_endCommand();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Add ".val=" to the string
void hmi_nameParameter(){
    TXREG = '.'; cleanBuffer();
    TXREG = 'v'; cleanBuffer();
    TXREG = 'a'; cleanBuffer();
    TXREG = 'l'; cleanBuffer();
    TXREG = '='; cleanBuffer();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Add a "common measured data" to the string (can be the power, temperature or humidity)
void hmi_commonData(){
   TXREG = TEMP[0]; cleanBuffer();
   TXREG = TEMP[1]; cleanBuffer();
   TXREG = TEMP[2]; cleanBuffer();
   hmi_endCommand();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Add the "end of Nextion command" to the string
void hmi_endCommand(){
    TXREG = 0xFF; cleanBuffer();
    TXREG = 0xFF; cleanBuffer();
    TXREG = 0xFF; cleanBuffer();
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Clean the UART buffer
void cleanBuffer(){
   while(!TRMT_bit);                                                            //while loop as long the buffer is not empty
}




