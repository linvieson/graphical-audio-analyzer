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

Ensure to have all the required components connected according to the scheme.

In the STM32CubeIDE environment, choose the option to create a new STM32 Project from an Existing STM32CubeMX Configuration File (.ioc) and choose the given [project.ioc](https://github.com/linvieson/graphical-audio-analyzer/blob/main/project.ioc) file. Then, in your project, **replace** the content of the files in **Core/** folder with [the given ones](https://github.com/linvieson/graphical-audio-analyzer/tree/main/Core): [main.c](https://github.com/linvieson/graphical-audio-analyzer/blob/main/Core/Src/main.c) and the [stm32f4xx_it.c](https://github.com/linvieson/graphical-audio-analyzer/blob/main/Core/Src/stm32f4xx_it.c) files in the **Core/Src/** folder.

**Create** a new files for fft permormance: [fft.c](https://github.com/linvieson/graphical-audio-analyzer/blob/main/Core/Src/fft.c) in the folder **Core/Src/**, and its header file [fft.h](https://github.com/linvieson/graphical-audio-analyzer/blob/main/Core/Inc/fft.h) in the **Core/Inc/** folder.

## Usage

As all computations are done in real time, the user will instantly view the output on the matrix. The user can speak, play music or just turn on some frequency on a frequency generator to observe the leds on the matrix changing. Each column represents one frequency (more precisely, one group of similar frequencies, because there are only 8 columns on the matrix to be used). The more leds are lighted in the column - the louder is the frequency represented by that column.

## How does it work?

There are mainly 3 steps of how the program exactly works:

- Read the input via the microphone and transform the given analogue signal to the discrete one, using the adc on the stm32 board
- Process the data, using the Fast Fourier Transform function to compute the amplitudes. Scale them so that receive the natural representation of the frequencies
- Display the result on the led matrix

## Results

The result of the project is a graphical audio spectrum analyzer. The input sound is processed with the help of Fast Fourier Transform and the frequencies are displayed on the led matrix.

## Contributors

- [Alina Voronina](https://github.com/linvieson)
- [Anna-Alina Bondarets](https://github.com/alorthius)
- [Anastasiia Tkachyshyn](https://github.com/tkachyshyn)

Menthors:

- Andrii Oksenchuk (Graduate Characterization Engineer, Renesas)
- Dmytro Ryzhenkov (Characterization Team Lead, Renesas)
