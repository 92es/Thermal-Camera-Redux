# Work In Progress


## DEFAULT_FLAGS

-DBORDER_LAYOUT=0 
  - Border layout: [0, 1]  - enable disable dynamic border layouts ( new for V 0.9.3 )
  
-DNO_TS=0 
  -  Timestamp  [0, 1] - Compile out timestap statistics overhead

-DNO_DRAG=0
  -  Drag scroll [0, 1] - Compile to use drag scrolling or jump / click scrolling

-DFAST_DRAG=0 
  -  Low resolution drag scrolling [0, 1] - Compile to use course or fine detail drag scrolling

-DDRAW_SINGLE_THREAD=0 
  -  Reduce thread count [0, 1] - Set to 1 on single core, single thread low powered hardware

-DDEFAULT_FONT=0 
  -  Select default font [0 - 2]  -  Font 0 is the fastest, Font 2 is the best looking

-DDEFAULT_COLORMAP=4 
  - Default colormap options: [0 - 36]  </br>
  
-DROTATION=0 
  - ROTATION options:         [0, 90, 180, 270] </br>

-DDISPLAY_WIDTH=1920 
  -  Sets constraints for scaling and aspect ratio based fullscreen BORDER_LAYOUT.

-DDISPLAY_HEIGHT=1080 
  -  Sets constraints for scaling and aspect ratio based fullscreen BORDER_LAYOUT.

-DUSE_CELSIUS=1 
  - Default temp display format:   [0, 1]  0 = Fahrenheit, 1 = Celsius

-DHUD_ALPHA=0.4
  - HUD_ALPHA:            [0.0 - 1.0]    0.0 = Full Transparent, 1.0 Full Opaque, Default 0.4  </br>

-DUSE_ASSERT=0
  - debug code:  [0, 1] - Compile out debug code overhead
