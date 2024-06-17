
<h3 align="center">PEDAL ECU</h3>

## Roadmap

- [x] Function to handle pedal usage:
  - [x] Take ADC inputs
  - [x] Check for deviation and average the inputs
  - [x] Check if inputs are the same for a specified time period (shorting for 1 second or less)
  - [x] Output using DAC
  - [ ] Output using CAN
  - [ ] Error codes for ADC calibration, potentiometer shorting, deviation check fail

- [ ] Functions for CAN communication:
  - [ ] Receive messages from BMS and motor controller
  - [x] Decode message and send to driver interface using UART

- [ ] Receive the data serial (UART) data from Pedal ECU:
  - [ ] Convert to the required format
  - [ ] Send to Driver Interface application

- [ ] Auto-start the Driver-Interface as soon as the vehicle starts and start receiving data  



