# Thermal-Camera-Redux
 Topdon TC001 (and clones: Topdon TS001, InfiRay P2 Pro, MaAnt Super IR Cam, etc.) Linux thermal camera app to read and display live and offline thermal data.<br />

Ported and updated to Linux C/C++ app based on Les Wright's 21 June 2023 PyThermalCamera Python app.<br />
  - All prior licenses apply.<br />
  - https://github.com/leswright1977/PyThermalCamera - Python script <br />
  - https://github.com/92es/Thermal-Camera-Redux     - Ported C/C++ app <br />

  
  
A multi-threaded C/C++ app to read, parse, display thermal data from the Topdon TC001 Thermal camera (and clones).<br />
Rewritten with additional functionality, bug fixes, optimizations and offline post processing.<br />

## Features ( Version 0.9.3 ):
<img align="right" src="media/22_of_37_Colormaps.png">

- Live mode reading from USB camera and Offline mode for post processing analysis.
  - Offline mode has great advantages over .png Snapshots because the raw thermal data can be represented with any configuration combinations at a later date as if it were live data.  No longer bound to a single colormap, scale, contrast, filter, etc. included in the static .png image.  Great for post documenting thermal audits depicting differnt data perspectives.
  - Freeze Frame mode offers an additional level of control in unstable hand held scenarios.
- Freeze frame, snapshots and recording.
- Optional dynamic border layout build ( V 0.9.3 )
  - See Example 4 below.
  - Also see the [BORDER_LAYOUT notes for more information.](./doc/BORDER_LAYOUT.md)
- Optionally lock live camera's color map auto-ranging ( V 0.9.2 )
  - Numerous temperature to color mapping options.
- 4 layout modes displaying image and thermal data sub-frames.
  - Image sub-frame (Portrait or Landscape)
  - Thermal sub-frame (Portrait or Landscape)
  - Image sub-frame on left, Thermal sub-frame on right (Portrait or Landsacpe)
  - Image sub-frame on top, Thermal sub-frame on bottom (Portrait or Landscape)
- 4 display modes
  - With translucent OSD config menu
  - With translucent OSD key and mouse bindings menu
  - With out translucent OSD menu
  - With out translucent OSD menu and overlayed graphics, video image only
- Portrait and Landscape rotations in 90 degree increments.
- Translucent OSD settings and keybinding menus.
- Integer scaling from 1X native resolution to full screen.
- Switchable Celsius / Fahrenheit displays.
- Scalable video, fonts and graphic overlays.
- 3 user selectable fonts.
- 37 colormaps.
  - 22 OpenCV stock colormaps dipicted to the right.
  - 7 inverted OpenCV stock colormaps.
  - 7 generated colormaps.
  - Raw feed, no colormap.
- 7 interpolation methods.
  - Nearest
  - Linear
  - Cubic
  - Area
  - Lanczos
  - Linear Exact
  - Nearest Exact
- Video blur and contrast settings.
- Histogram Equalization filter.
- User settable Threshold delta from average temp in degrees C or F.
- Live gradient colormap scale widget.
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
    - Vertical and horizontal relocatable cross grid of text temps.
    - Individual horizontal or vertical relocatable plotting rulers.
    - Simulteanous horizontal and vertical relocatable plotting rulers.  
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
-  Linux, C/C++, OpenCV(V4L) and pthread libraries.
-  Optional build flags optimized for weak and single core / single threaded hardware. <br/>

-  NOTE 1: Building on Debian 11 and newer Debian Linux based distros have a better likihood of working due to library compatability issues.
   -  Distros like Arch, SUSE, Redhat, Fedora, BSD are not Debian based so there may be TBD issues.
-  NOTE 2: Running as a Virtual Machine Guest OS client will probably NOT work because the VM can alter the USB video feed passed to the client OS, thus destroying the thermal data.

## Raspberry Pi Build Help:
-  See the [RPi/SBC build help notes.](./doc/RPi.md)

## Revision change log:
-  Section coming soon.

Thermal Camera Redux:
	
Built with display 2560x1600, max:default scale 8:4, rotation 0, default Fahrenheit, 37 colormaps, Jet,<br />
    Multi-threaded with drag scrolling<br />

Tested on IvyBridge, SkyLake & CoffeeLake Debian 11 and 12 PCs with all features working<br />
Tested by Amish Technician (from EEVblog) on numerous RPi models including RPi Zero 2w, 2, 3, 4 and 5<br />
    using 2023-12-05 release of Raspberry Pi OS desktop 64-bit (Debian 12 bookworm)<br />

Reported working (by Cynfab) on "InfiRay P2 Pro" thermal camera <br/>
Reported working (by PMKrol, 2024/06/04) on "MaAnt Super IR Cam" thermal camera running on Ubuntu version 24.04 <br/>
Reported working (by Artag, 2024/07/15) on "Topdon TS001" thermal camera running on Debian 11 PC <br/>

## Camera Usage: <br />
  ./redux -d n (where 'n' is the number of the desired video camera)<br />

## Offline Usage: <br />
  ./redux -f input.raw (where input.raw is a raw dump file from ./redux)<br />

## Optional flags: <br />
	[-rotate n] [-scale n] [-fullscreen] [-cmap n] [-fps n] [-font n] [-clip n] [-thick n]
	[-help] [-quiet] [-snapshot [prefix]]


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
- l  : [Un]Lock camera's colormap auto ranging <br />
- i k: Cycle [for|back]wards through locked auto ranging mapping methods <br />
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

## Example 1:  Full screen horizontal and vertical thermal ruler plots with, 4X scale and video blur settings applied.
  - Green sections of plot lines depict above (average + threshold) temps.
  - White sections of the plot lines depict (average +/- threshold) temps.
  - Red sections of the plot lines depict below (average - threshold) temps.
  - Ruler's focal point was relocated on the screen's Max temp location.<br />
    - The gradient scale's ">" marker in the upper right hand corner is Green indicating the focal point's respective temp range, above (average + threshold).<br />
    - The gradient scale's Avg temp marker "-" in the lower right hand corner is White, indicating the screen's current average temp.<br />
    - The Green lines extending from the focal points to both the Vertical and Horizontal plot lines indicate the relative temp at the current location.  Longer lines indicates a larger relative delta from the screen's Avg temp. <br />
    - All thermal ruler features update in real time from thermal data obtained by the camera and as the rulers are repositioned around the screen.

![Screenshot](media/GIT_HUB_SAMPLE.png)

## Example 2: User Temp mode with 13 added users temps
  - If only the lady had this app when she ordered hot coffee from the McDonald's drive thru order window ...
  - This could be useful for fixed mount bench work.
  - Hi/Lo alarm triggers could be added to user temp points in the future for temp range monitoring and automated logging.

![Screenshot](media/GIT_HUB_USER_TEMPS.png)

## Example 3: Dual scrolling ruler mode.
  - 1/5 screen height ruler size.
  - Proportional, non-clipping display mode.

![Screenshot](media/GIT_HUB_COLD.png)

## Example 4: Optional V 0.9.3 border layout build mode
  - See the [BORDER_LAYOUT notes for more information.](./doc/BORDER_LAYOUT.md)
  - Added dynamic left and right border display areas.
    - Border widths dynamically resize based on current scale factor, font selection and (-DDISPLAY_WIDTH=width or Fullscreen aspect ratio) constraints.
  - OSD informational display in left display border.
  - Colormap gradient widget in right display border.
  - Feature requested by Amish Technician.
  - Landscape, double high layout with optional lock auto-ranging filter applied in 2nd window.

![Screenshot](media/GITHUB_BORDER_LAYOUT_2.png)
![Screenshot](media/GITHUB_PC_BORDER.png)

## Potential Future Upgrades:
- Data logging over time.
- User temp alarm triggers.
- 3D solid and mesh surface plots.

## Credits:
- LeoDJ's Kelvin conversion algorithm from post #216 on EEVblog
- Les Wright's original Python script and corresponding YT video.
- Amish Technician's gracious RPi testing, feature recommendations and valuable feedback.
- Cynfab for thermal range testing on the InfiRay P2 Pro.
- Matthias Wandel's helpful feature colaboration and RPi feedback.







  

  

