# Graphical Audio Analyzer

Principles of Computer Organization course project

## Description

Human ear is not able to precisely distinguish between sound frequencies. Moreover, the perception of sound waves with too high, or vice versa, too low frequencies weakens with ages. Nowadays sound analysis has become a crucial field of IT, and with our project we contributed to its development.

Team developed a Graphical Audio Spectrum Analyzer based on STM32. It is a real-time analyzer of the sound, depicting the frequencies on the 8x8 matrix. Each column is in charge of one group of related frequencies. The more leds are lighted in the column - the louder the frequency is.

The aim of the project was to solve the above issues, by depicting the sound on the matrix' leds. Such way of visualisation is more convenient then graphs or tables, as the matrix depicts the reslts in a natural way for people to percept.

## Conponents

- STM32 Discovery
- Electret Microphone Amplifier MAX9814
- 8x8 Matrix of Address LEDs

## Download and start

```
git clone https://github.com/linvieson/graphical-audio-analyzer.git
```

Open the CubeIDE, compile and run the program.

## Usage

As all computations are done in real time, the user will instantly view the output on the matrix. The user can speak, play music or just turn on some frequency on a frequency generator to observe the leds on the matrix changing. Each column represents one frequency (more precisely, one group of similar frequencies, because there are only 8 columns on the matrix to be used). The more leds are lighted in the column - the louder is the frequency represented by that column.

## How does it work?

There are mainly 3 steps of how the program exactly works:

- Read the input via the microphone
- Process the data, use the Fast Fourier Transform function
- Display the result on the led matrix

## Results

The result of the project is a graphical audio spectrum analyzer. The input sound is processed with the help of Fast Fourier Transform and the frequencies are displayed on the led matrix.

## Contributors

- [Alina Voronina](https://github.com/linvieson)
- [Anna-Alina Bondarets](https://github.com/alorthius)
- [Anastasiia Tkachyshyn](https://github.com/tkachyshyn)


