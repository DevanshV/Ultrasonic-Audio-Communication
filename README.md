# Ultrasonic-Audio-Communication
Unidirectional communication protocol through ultrasonic audio waves (> 20 kHz) from a master device and performing FFT on the slave (TM4C123 - ARM Cortex M4)

<a href="https://youtu.be/3LHPGdFfbrE" target="_blank">
<img src="https://user-images.githubusercontent.com/10454251/34544166-a5583cac-f0b2-11e7-85a2-6ca3a7118131.gif" width="40%" alt="Click for Music Player Demo video">
<img src="https://i.imgur.com/HijxXiJ.jpg" alt="Diagram" width="55%"/>
</a>

## Goal

Send data through audio! I wanted to set up a system to send ultrasonic audio, this was largely inspired by the "Sonic" Screwdriver in Doctor Who.

Initially, I simply wanted to send binary commands, however, I ended up adding onto to the project to send full ASCII messages.

## Setup and Required Hardware

The receiver code is written for a TivaC microcontroller. However, any device with a microphone can be used to transmit the high-frequency data. This could include a smartphone, computer or microcontroller.

## Protocol Setup

Each bit consists of 5 frames, while each frame contains 1024 samples (20ms) for which frequency is determined. As a result, it takes 100ms to transmit 1 bit. Since 5 frames are used per bit, we can avoid erroneous transmission and limit the effect of drifting due to clock inaccuracies. 

Out-of-band signaling is used to determine the **start of transmission**. The start sequence consists of alternating bits (`0b10101010`), that is transmitted at a different frequency than the rest of the data. 

The **stop condition** is a 0 byte (`0b0`). In ASCII, 0 is the NULL character, as a result, we can take advantage of this as an end condition.

## Limitations and upcoming changes

Although this was greatly resolved through synchronization and hardware optimization, during testing, it was apparent that the clocks drifted. This resulted in the frames coming out of alignment with the receiver/transmitter. The transmitter used was an Arduino board where there are known inaccuracies since an RTC was not used. As a result, accuracy may decrease over long transmissions. On the TivaC receiver, a **Ping-Pong uDMA** is used to ensure a continous flow of data, this allows us to process data, while the next frame's samples are being collected in hardware. I plan to add resynchronization every 100 bits to avoid such issues. 


