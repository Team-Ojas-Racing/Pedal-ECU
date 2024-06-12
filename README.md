
<h3 align="center">PEDAL ECU</h3>

## Roadmap

- [ ] function to handle pedal usage: -> take adc inputs -> check for deviation and average the inputs -> check if inputs are same for a specified time period -> output using DAC -> output using CAN
- [ ] functions for CAN communication: -> receive messages from BMS and motor controller -> decode message and send to driver interface using UART
- [ ] Receive the data serial (UART) data from Pedal ECU -> Convert to the required format -> Send to Driver Interface application
- [ ] Auto-start the Driver-Interface as soon as vehicle starts and start receiving data  

#### Additional:
- [ ] Add any more additional features that are not essential but are needed over here

