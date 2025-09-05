Here is list of component used for MRC 2025 for Apollo
1. MPU 9250 - acceleration, gyroscope, magnetometer
2. BMP 280 - temperature, pressure, altitude
3. Arduino Nano Every
4. AMS1117 -voltage regulator
5. GPS Module GY NEO 6M
6. Passive Buzzer
7. APC220
8. Duck Antenna
9. Yagi Antenna


To run full code, please test sensor and component one by one. Then, can compile the code.

For communication, firstly, have to make sure the frequency between apc220(trancmitter) and apc220(receiver) is set at the same frequency, RF data rate, Radio Output Power, UART baudrate and byte Check Parity. To set all these, please use apc220Cfg file. Please read the manual attached.
For communication, arduino IDE Serial monitor can't display two UPLOAD file at the same time. So, please download any other Serial Monitor such as CoolTerm & TeraTerm to ensure communication between Ground Station and Cansat.

For better understanding of Antenna, Antenna Toolbos in MATLAB is agood tool to understand the radiation pattern of antenna etc. To do comparison analysis of antenna, can download Airspy SDR software to visualize the signal wave. Need to use, RTL-SDR USB. Then connect the USB RTL-SDR to PC and at the end of female connector, connect the antenna you want. It will capture the frequency it receive. Have to make sure there is radio communication occured at the setting frequency.
