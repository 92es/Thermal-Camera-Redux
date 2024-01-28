## Optional V 0.9.3 border layout build mode
  - Added dynamic left and right border display areas.
    - Border widths dynamically resize based on current scale factor, font selection and (-DDISPLAY_WIDTH=width or Fullscreen) aspect ratio constraints.
  - OSD informational display in left display border.
  - Colormap gradient widget in right display border.
  - Feature requested by Amish Technician.

The folowing depicts possible border width resizing from 1X scale to Fullscreen.

Note that the aspect ratio changes as scale changes (to keep square pixels) until it reaches Fullscreen.

Depending on constrait permutations, the left and right border widths can shring to 0.

Selecting fullscreen will scale from the currently select scale factor, thus may appear differnt from fullscreen from other scale factors.

## 1X Scale ( DISPLAY_WIDTH border constraint )
![Screenshot](../media/GITHUB_1X_BORDER.png)

## 2X Scale ( DISPLAY_WIDTH border constraint )

![Screenshot](../media/GITHUB_2X_BORDER.png)

## Full Screen ( DISPLAY_WIDTH / DISPLAY_HEIGHT aspect ratio border constraint ):

![Screenshot](../media/GITHUB_FS_BORDER.png)
