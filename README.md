# Pedal ECU 

# Targets:
1) function to handle pedal usage:
    -> take adc inputs
    -> check for deviation and average the inputs
    -> check if inputs are same for a specified time period
    -> output using DAC
    -> output using CAN
2) functions for CAN communication:
    -> receive messages from BMS and motor controller
    -> decode message and send to driver interface using UART