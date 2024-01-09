# Thermal-Camera-Redux
 Topdon TC001 (and clones) Linux thermal camera app to read and display live and offline thermal data.<br />

Ported and updated to Linux C/C++ app based on Les Wright's 21 June 2023 Python app.<br />
  - All prior licenses apply.<br />
  - https://github.com/leswright1977/PyThermalCamera - Python script <br />
  - https://github.com/92es/Thermal-Camera-Redux     - Ported C/C++ app <br />

  
  
A multi-threaded C/C++ app to read, parse, display thermal data from the Topdon TC001 Thermal camera (and clones).<br />
Rewritten with additional functionality, bug fixes, optimizations and offline post processing.<br />

## Features:
- Live mode reading from USB camera and Offline mode for post processing analysis.
- Freeze frame, snapshots and recording.
- 4 layout modes displaying image and thermal data sub-frames.
- Portrait and Landscape rotations in 90 degree increments.
- Translucent OSD settings and keybinding menus.
- Integer scaling from 1X native resolution to full screen.
- Switchable Celsius / Fahrenheit displays.
- Scalable video, fonts and graphic overlays.
- 3 user selectable fonts.
- 37 colormaps.
- 7 interpolation methods.
- Video blur and contrast settings.
- Histogram Equalization filter.
- User settable Threshold delta from average temp in degrees C or F.
- Color coded widgets depict:
  -  below (average - threshold) temps
  -  (average +/- threshold) temps
  -  above (average + threshold) temps
- Horizontal and vertical scrollable thermal plotting rulers.
  - Landscape ruler plots 256 plus 3, 5 or 7 text temps.
  - Portrait ruler plots 192 plus 3, 5 or 7 text temps.
- Thermal rulers have 5 modes with 4 clipping and 5 size settings.
  - 5 modes include:
    - Single relocatable temp.
    - Vertical and horizontal cross grid of text temps.
    - Individual horizontal or vertical plotting rulers.
    - Simulteanous horizontal and vertical plotting rulers.  
  - Cliping modes include:
    - no clipping (average baseline is proportional to Min/Avg/Max distribution)
    - outlier clipping (average baseline is centered in clippled plot)
    - below average clipping (focus on temps at or above average)
    - above average clipping (focus on temps at or below average)
  - Size settings include 1/5, 1/4, 1/3, 1/2 and full height ruler widths.
  - Horizontal and Vertical rulers can be displayed simultaenously or sepearately.
- Ability to add up to 13 user defined temp locations (great for fixed mount bench work).
- User temp and ruler display modes are mutually exclusive.
- Min, Avg and Max screen temps.
- Thermal color gradient widget with Min/Avg/Max and current focus temp indicators.
- User input can be entered via keyboard and/or mouse as well as commands redirected to stdin.
  -  Keyboard and mouse input take presendece over redirected stdin command stream.

## Build requirememts:
-  Linux, C/C++, OpenCV and pthread libraries.

Thermal Camera Redux:
	
Built with display 2560x1600, max:default scale 8:4, rotation 0, default Fahrenheit, 37 colormaps, Jet<br />

Tested on IvyBridge & Coffee Lake Debian 11 PCs with all features working<br />
Tested by Amish Technician (from eevblog) on numerous RPi models including RPi Zero 2w, 2, 3, 4 and 5<br />
    using 2023-12-05 release of Raspberry Pi OS desktop 64-bit (Debian 12 bookworm)<br />

## Camera Usage: <br />
  ./redux -d n (where 'n' is the number of the desired video camera)<br />

## Offline Usage: <br />
  ./redux -f input.raw (where input.raw is a raw dump file from ./redux)<br />

Optional flags: [-rotate n] [-scale n] [-cmap n] [-fps n] [-font n] [-clip n] [-thick n]<br />

## Mouse and Key Bindings:<br />

- a z: [In|De]crease Blur <br />
- s x: +/- threshold from avg temp that contols min/max displays and ruler plot colors<br />
- d c: Change interpolated window scale [camera native to fullscreen]<br />
- f v: [In|De]crease Contrast<br />
- g b: Cycle [for|back]wards through interpolation methods<br />
- j m: Cycle [for|back]wards through Color[m]aps<br />
- w  : Cycle through single/dual[horizontal/vertical] [w]indow layouts<br />
- 6  : Toggle between fullscreen and current scaled window size<br />
- r  : Toggle [r]ecording (.avi)<br />
- 1  : Select Font<br />
- 5  : Reset defaults<br />
- p  : Sna[p]shot (both .png and offline .raw)<br />
- h  : Cycle through overlayed screen data<br />
- t  : Toggle between Celsius and Fahrenheit <br />
- y  : Toggle Historgram Equalization filter <br />
- 8  : Rotate display 0, 90, 180, 270 degrees (Portrait and Landscape)<br />
- e  : Toggle Freeze Frame on/off<br />
- o  : Displays and cycles through 5 temp ruler modes<br />
- 3  : Ruler plot clip modes: none, outlier, below avg, above avg<br />
- 4  : Ruler thickness - 1/5, 1/4, 1/3, 1/2, full height<br />
  -   : Keypad Up/Down/Left/Right/Center(5) moves rulers<br />
  -   : Left mouse adds user temps or moves rulers<br />
  -   : Right mouse removes user temps and disables ruler mode<br />
- / : Misc stdout help information<br />
- q  : Quit<br />

## Example:  Full screen horizontal and vertical thermal ruler plots with, 4X scale and video blur settings applied.
  - Green sections of plot lines depict above (average + threshold) temps.
  - White sections of the plot lines depict (average +/- threshold) temps.
  - Red sections of the plot lines depict below (average - threshold) temps.
  - Ruler's focal point was relocated on the screen's Max temp location.<br />
    - The gradient scale's ">" marker in the upper right hand corner is Green indicating the focal point's respective temp range, above (average + threshold).<br />
    - The gradient scale's Avg temp marker "-" in the lower right hand corner is White, indicating the screen's current average temp.<br />
    - The Green lines extending from the focal points to both the Vertical and Horizontal plot lines indicate the relative temp at the current location.  Longer lines indicates a larger relative delta from the screen's Avg temp. <br />
    - All thermal ruler features update in real time from thermal data obtained by the camera and as the rulers are repositioned around the screen.

![GIT_HUB_SAMPLE](https://github.com/92es/Thermal-Camera-Redux/assets/76127081/777691ef-8e49-4cb7-9c45-f54b4627b086)

