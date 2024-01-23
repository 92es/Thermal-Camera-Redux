# Raspberry PI build instructions

Graciously provided by Amish Technician from EEVBlog thermal imaging thread:

## Package installation

For anyone having difficulty getting Thermal Camera Redux working you first need to install the **'libopencv-dev'** package. 

Enter the following in a terminal on Raspberry Pi OS (or Debian derived distro on any other device or PC):

 `sudo apt update && sudo apt install libopencv-dev`


The 'build_redux_rpi' script in the src directory contains my one size fits all edits which should work on PC, all raspberry pi's, and likely most other similar SBCs:


See the [build_redux_rpi.](../src/build_redux_rpi)

There are also various startup customization flags that can be edited such as display size, C or F displays, rotation (portrait/landscape), colormap, font, etc. 


## Single thread, single core SBCs

Change these default flags to =1 for improved performance on Pi Zero 1 (W), Pi 1, computer module 1, or any other SBC with a single threaded CPU:

  `-DNO_TS=1`
  
  `-DDRAW_SINGLE_THREAD=1`

## Build

Then from the directory with the source files run the build script:

  `chmod +x build_redux_rpi`
  
  `sudo ./build_redux_rpi`

The chmod command only needs to be run once because github does not appear to preserve the execution permissions. 

## Run

If it builds successfully you should now have a new binary file called 'redux' in the same directory. Connect your camera and run redux:

  `./redux -d 0`

If this fails check file permission and ensure 'redux' is executable. Your camera might not be 'device 0' in which case specify the appropriate number for yours instead: 

  `./redux -d 1`

## Diagnose alternate platforms

A limitation to keep in mind with my script is that the resulting binary may not work if you take it to a different device from the one you build it on, if you build it on a PC and copy it to a raspberry pi, or even between some pi models. 

The flags in this script haven't resulted in a degradation in performance compared to the flags I had manually specified for each RPi before, that said there are flags I haven't tried yet. I'll update here if I discover any improvements (or if someone more knowledgeable has a suggestion that works better).

Hope this is helpful, feel free to ask if any clarification is required or if you run in to any problems.






