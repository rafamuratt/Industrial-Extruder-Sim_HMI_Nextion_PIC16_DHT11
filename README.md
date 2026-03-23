**Industrial-Extruder-Sim_HMI_Nextion_PIC16_DHT11**

Bare-Metal Implementation: This project operates entirely at the register level without external dependencies.  
The firmware is fully optimized for the PIC16F628A, maintaining a lean footprint with 16% ROM remaining for future feature expansion.    

An industrial-grade HMI simulation of a *Reifenhäuser-style* plastic extruder line.   
This project integrates a PIC16F628A microcontroller with a Nextion Intelligent Series HMI to provide real-time process monitoring, motor control, and safety-critical alarm handling.  

🚀 System Overview  
This project simulates real-world industrial extrusion processes, featuring:  
* Digital Twin Interface: Replicates a Reifenhäuser extruder line with animated GIFs synchronized to motor PWM speed.  
* Environmental Monitoring: Real-time waveform plotting for pre-heater temperature and input humidity using a DHT11 sensor.  
* Safety Logic: A fail-safe hierarchy where Emergency Stops and Sensor Faults trigger an immediate hardware halt and system lockout.  
* Industrial UI Standards: Uses standardized color-coded load indicators (Green/Orange/Red) and an "Acknowledge" (ACK) workflow for error recovery.  

🛠 Hardware Stack  
* Microcontroller: PIC16F628A (@ 4MHz Internal)  
* HMI: Nextion NX8048P050-011R-Y (Intelligent Series)  
* Sensor: DHT11 (Temperature & Humidity)  
* Communication: UART @ 9600 bps  

📂 Project Structure  
/HMI_Nextion: Contains the .HMI source file and the ready-to-flash .TFT binary  
/PIC_Source: MikroC project files with compiled .hex, source code, and Fuse bits setup.  
/Photos & Screenshots: Some project photos and a folder with HMI screenshots.

⚙ Operational Flow  
* Normal Operation: System runs with animated motor feedback and live sensor plots.  
* Error Detection: Sensor data is validated via CheckSum. Disconnection or data corruption displays ### and triggers an error.  
* Emergency Stop: Immediate cessation of motor PWM; requires manual release + system ACK to reset.  
* Error Recovery: System remains in "Error" status until the physical issue is resolved and acknowledged.  
* Warning message pops up if the motor speed is over 89%

📜 License
This project is licensed under the Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).  

☕ If this project is helpfull for your application, please consider to support:  
https://www.paypal.com/donate/?hosted_button_id=8S8BJ9TT368VN

Built by rafamuratt: https://murat-tech.eu/  
Murat-Tech Channel: https://www.youtube.com/@Murat-TechChannel-EN 
