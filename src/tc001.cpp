#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <signal.h>
#include <assert.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>  // Video Frame class
#include <iostream>

using namespace std;
using namespace cv;

#include "thread.h" // threads and FIFO ring buffer

#define VERSION_STR "0.9.0"
/*****************************************************************************************

  NOTE: No implied or expressed useability guarantee or warranty.  
        Use at your own risk.
        Will try to respond to valid bug reports.

  Change log from response comments:

  09-01-2024 - Initial Github Release 0.9.0
  

  Switching between "tuned-adm latency-performance" and "tuned-adm desktop" with
  unrestricted FPS rates varies stdin scripted performance by a factor of [3-5]X:

*****************************************************************************************/

#define ARRAY_COUNT(a) (int)((int)sizeof(a) / (int)sizeof(a[0]))

#define divmod(numerator, denominator, quotient, remainder) \
	*quotient  = numerator / denominator; \
	*remainder = numerator % denominator;

#if USE_ASSERT
#define ASSERT(a) assert a
#else
#define ASSERT(a)
#endif

#define NATIVE_FPS 25.0

#ifndef OFFLINE_FPS
	#define OFFLINE_FPS NATIVE_FPS
#endif

static int offline_fps = OFFLINE_FPS;

#define helpLineType		LINE_8
#define hudLineType		LINE_8

#define noHaloLineType		LINE_8
#define haloLineType		LINE_8
#define fgLineType		LINE_AA


#define USE_CLONE 1   // which is faster mat.clone() or mat.copyTo()

// seconds  0.
// 25 FPS   0.040  // Native camera frame rate
// millis   0.123
// micros   0.123456
// nanos    0.123456789

#define MILLIS_PER_SECOND 1000
#define MICROS_PER_SECOND (MILLIS_PER_SECOND * 1000)
#define NANOS_PER_SECOND  (MICROS_PER_SECOND * 1000)

// timeval  has Seconds and MICRO-seconds
// timespec has Seconds and NANO-seconds
void sleepMillis(long millis) {
        struct timespec ts;
        ts.tv_sec  = (millis / MILLIS_PER_SECOND);
        ts.tv_nsec = (millis % MILLIS_PER_SECOND) * 1000 * 1000;
        nanosleep(&ts, NULL);
}

void sleepMicros(long micros) {
        struct timespec ts;
        ts.tv_sec  = (micros / MICROS_PER_SECOND);
        ts.tv_nsec = (micros % MICROS_PER_SECOND) * 1000;
        nanosleep(&ts, NULL);
}

void sleepNanos(long nanos) {
        struct timespec ts;
        ts.tv_sec  = (nanos / NANOS_PER_SECOND);
        ts.tv_nsec = (nanos % NANOS_PER_SECOND);
        nanosleep(&ts, NULL);
}


// Returns the current time in milliseconds.
int64_t currentTimeMillis() {
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	int64_t s1 = (int64_t)(currentTime.tv_sec  * 1000);
	int64_t s2 = (int64_t)(currentTime.tv_usec / 1000);
	return s1 + s2;
}

// Returns the current time in microseconds.
int64_t currentTimeMicros(){
#if 1 // More accurate (but slower) than clock_gettime(CLOCK_MONOTONIC_COARSE)
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return (currentTime.tv_sec * (int)1e6) + currentTime.tv_usec;
#else
	// @ 4.5X faster that gettimeofday(), but may drift over time
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_COARSE, &ts);
	return ( (ts.tv_sec  * (int)1e6) +
		 (ts.tv_nsec / 1000) );
#endif
}

// Returns the current time in microseconds.
int64_t currentTimeNanos(){
#if 1 // More accurate (but slower) than clock_gettime(CLOCK_MONOTONIC_COARSE)
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return (currentTime.tv_sec * NANOS_PER_SECOND) + currentTime.tv_usec;
#else
	// @ 4.5X faster that gettimeofday(), but may drift over time
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_COARSE, &ts);
	return ( (ts.tv_sec  * NANOS_PER_SECOND) +
		  ts.tv_nsec  );
#endif
}


#define WINDOW_NAME "Thermal Camera Redux"

// Camera's native resolution
#define FIXED_TC_WIDTH  	256
#define FIXED_TC_HEIGHT 	192

#ifndef HUD_ALPHA
#define HUD_ALPHA 0.4 // 40% HUD, 60% background
#endif

// Default to Fahrenheit if not set in Makefile
#ifndef USE_CELSIUS
#define USE_CELSIUS 0  // Startup Default: 1 for Celsius, 0 for Fahrenheit
#endif

// Default to 1080P if not set in Makefile
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH   1920
#endif

#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT  1080
#endif

// Default to no rotation if not set in Makefile
#ifndef ROTATION
#define ROTATION 0
#endif

// Calculate based on display resolution and native camera resolution
#define MAX_SCALE_STEPS min( (int)(DISPLAY_WIDTH  / FIXED_TC_WIDTH), \
			     (int)(DISPLAY_HEIGHT / FIXED_TC_HEIGHT) )

static int TC_WIDTH	  = FIXED_TC_WIDTH;	// Changes based on current Portrait / Landscape rotation
static int TC_HEIGHT	  = FIXED_TC_HEIGHT;	// Changes based on current Portrait / Landscape rotation
static int TC_HALF_WIDTH  = (TC_WIDTH  / 2);	// Changes based on current Portrait / Landscape rotation
static int TC_HALF_HEIGHT = (TC_HEIGHT / 2);	// Changes based on current Portrait / Landscape rotation

static int SCALED_TC_WIDTH  = TC_WIDTH;
static int SCALED_TC_HEIGHT = TC_HEIGHT;

#define TC_MAX_TEMPS	(FIXED_TC_HEIGHT * FIXED_TC_WIDTH)
#define TC_DEF_SCALE	MAX((MAX_SCALE_STEPS / 2), 1)

static float rulerXKelvinFactor;
static float rulerYKelvinFactor;

static const char *Argv0  = "";

// Reuse Point to minimize construction/destruction overhead
#ifdef POINT
#undef POINT
#endif
#define POINT(p,a,b) p.x = (a); p.y = (b);

#ifdef SIZE
#undef SIZE
#endif
#define SIZE(s,a,b) s.width = (a); s.height = (b);

// THREAD LOAD BALANCING
float kelvinX[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Grab entire row of temps at once
float kelvinY[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Grab entire col of temps at once

Scalar *scalarX[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire row of colors at once
Scalar *scalarY[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire col of colors at once

// WINDOW_SINGLE points, eliminate continuous malloc/frees of ruler points
Point rulerX0Points[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire row of Points
Point rulerY0Points[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire col of Points

// WINDOW_DOUBLE points, eliminate continuous malloc/frees of ruler points
Point rulerX1Points[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire row of Points
Point rulerY1Points[max(FIXED_TC_WIDTH, FIXED_TC_HEIGHT)];  // Calc entire col of Points

char  rawRecFilename[256] = "now_output.raw";
FILE *rawRecFp  = 0x00;
FILE *rawReadFp = 0x00;

static int leftDragOff = 1; // Track left mouse buttion drag

#define DECODE_ROTATION(value)	(  0 <= (value) && (value) <  90) ? 0 : \
				( 90 <= (value) && (value) < 180) ? 1 : \
				(180 <= (value) && (value) < 270) ? 2 : 3;

// 0 = 0, 1 = 90, 2 = 180, 3 = 270 degrees
static int RotateDisplay = DECODE_ROTATION ( ROTATION );

static const char * ROTATION_STR =	(  0 <= ROTATION && ROTATION <  90) ?   "0" :
					( 90 <= ROTATION && ROTATION < 180) ?  "90" :
					(180 <= ROTATION && ROTATION < 270) ? "180" : "270";

static Mat  imageFrame;
static Mat  thermalFrame;
static Mat  rgbImageFrame;   // Scaled, rotated (not composited) RGB frame - COLOR_YUV2BR_YUYV
static Mat  rgbThermalFrame; // Scaled, rotated (not composited) RGB frame - COLOR_YUV2BR_YUYV

static int  MyScale;
static int  MyHalfScale;

// Support rotations by 90 degrees which flip FIXED width and height
void setHeightWidth() {
	if ((RotateDisplay % 2) == 0) {
		TC_WIDTH	= FIXED_TC_WIDTH;
		TC_HEIGHT	= FIXED_TC_HEIGHT;
	} else {
		TC_WIDTH	= FIXED_TC_HEIGHT;
		TC_HEIGHT	= FIXED_TC_WIDTH;
	}

	TC_HALF_WIDTH 	= (TC_WIDTH  / 2);
	TC_HALF_HEIGHT	= (TC_HEIGHT / 2);

	SCALED_TC_WIDTH  = MyScale * TC_WIDTH;
	SCALED_TC_HEIGHT = MyScale * TC_HEIGHT;
}

// OpenCV colors are stored in BGR, not RGB, Red and Blue are SWAPPED

static Point PointZeroZero = Point( 0, 0 );
static Scalar GREEN        = Scalar(0,255,0);
static Scalar RED          = Scalar(0,0,255);       // BGR - BlueGreenRed
static Scalar BLUE         = Scalar(255,0,0);       // BGR - BlueGreenRed
static Scalar BLACK        = Scalar(0,0,0);
static Scalar YELLOW       = Scalar(0,255,255);
static Scalar WHITE        = Scalar(255,255,255);

// GRAY improves 1X scale marker appeareance (less of a blob)
// GRAY is 127 away from RED, GREEN, BLUE, WHITE, BLACK (anti-red-green-blue)
static Scalar GRAY         = Scalar(127,127,127);

#define TempTextColor WHITE // Yellow text over yellow text HUD doesn't work well

#ifndef DEFAULT_FONT
#define DEFAULT_FONT 0		// SIMPLEX 3X faster than TRIPLEX
#endif

#define MAX_USER_FONT  3
static int UserFont = ( DEFAULT_FONT % MAX_USER_FONT );

//static int Default_Font = 
static int Default_Font = ( 2 == UserFont ) ? cv::FONT_HERSHEY_TRIPLEX :
			  ( 1 == UserFont ) ? cv::FONT_HERSHEY_DUPLEX  :
					      cv::FONT_HERSHEY_SIMPLEX;

#define COLORMAP_ROWS 256

typedef union _CMapUnion {
	ColormapTypes index;	// System Default Colormap Types
	Mat          *matrix;	// User Defined Colorap Mat(COLORMAP_ROWS, 1, CV_8UC3);
} CMapUnion;

// Support No Colormaps, Pre-Defined Colormaps and User Defined Colormaps ...
typedef enum _CMapType {
	COLORMAP_NONE,		// No colormap, camera's default 
	COLORMAP_INDEX,		// Pre-Defined colormaps
	COLORMAP_MATRIX		// User-Defined colormaps
} CMapType;

typedef struct _CMap {
	CMapUnion   cmap;
	const char *name;
	CMapType    type;	// NONE, INDEX, MATRIX
	int         bgr2rgb;
} CMap;

CMap *newCmap( ColormapTypes index, int bgr2rgb, const char *name, CMapType type) {
	CMap *ptr = (CMap *)calloc( 1, sizeof(CMap) );

	ptr->cmap.index = index;
	ptr->name       = name;
	ptr->type       = type;
	ptr->bgr2rgb    = bgr2rgb;	// Make inverted colormap with cvtColor(src, dst, COLOR_BGR2RGB);

	return (CMap *) ptr;
}

CMap *newCmap( Mat *matrix, int bgr2rgb, const char *name, CMapType type) {
	CMap *ptr = (CMap *)calloc( 1, sizeof(CMap) );

	ptr->cmap.matrix = matrix;
	ptr->name        = name;
	ptr->type        = type;
	ptr->bgr2rgb     = bgr2rgb;	

	return (CMap *) ptr;
}

#define R2B_W2B_UC -2   // Unit Circle
#define R2B_2B_L   -1   // Linear
#define R2B         0 
#define R2B_2W_L    1   // Linear
#define R2B_B2W_UC  2   // Unit Circle

Mat createRed2BlueBlackWhiteCmap(int ramp);
Mat createHSLCmap(int direction);

static Mat red2BlueCmap         = createRed2BlueBlackWhiteCmap( R2B );
static Mat red2BlueBlackLCmap   = createRed2BlueBlackWhiteCmap( R2B_2B_L );
static Mat red2BlueWhiteLCmap   = createRed2BlueBlackWhiteCmap( R2B_2W_L );
static Mat red2BlueBlackUCCmap  = createRed2BlueBlackWhiteCmap( R2B_W2B_UC );
static Mat red2BlueWhiteUCCmap  = createRed2BlueBlackWhiteCmap( R2B_B2W_UC );

static Mat HSLCmap              = createHSLCmap(-1); // HSL Hot(red)    to Cold(violet)
static Mat inverseHSLCmap       = createHSLCmap( 1); // HSL Hot(violet) to Cold(red)

// Support No Colormaps, Pre-Defined Colormaps, Inverted Colormaps, User Defined Colormaps ...
CMap *cmaps[] = {
	newCmap( (ColormapTypes)(-1),	0, "None",	COLORMAP_NONE ),   // No colormap, system's default colormap

	newCmap( COLORMAP_AUTUMN,	0, "Autumn",	COLORMAP_INDEX ),  // System default colormaps and inversions
	newCmap( COLORMAP_AUTUMN,	1, "Inv Autumn",	COLORMAP_INDEX ), 
	newCmap( COLORMAP_BONE,		0, "Bone",	COLORMAP_INDEX ),
	newCmap( COLORMAP_JET,		0, "Jet",	COLORMAP_INDEX ),
	newCmap( COLORMAP_JET,		1, "Inv Jet",	COLORMAP_INDEX ),
	newCmap( COLORMAP_WINTER,	0, "Winter",	COLORMAP_INDEX ),
	newCmap( COLORMAP_RAINBOW,	0, "Rainbow",	COLORMAP_INDEX ),
	newCmap( COLORMAP_RAINBOW,	1, "Inv Rainbow",	COLORMAP_INDEX ),
	newCmap( COLORMAP_OCEAN,	0, "Ocean",	COLORMAP_INDEX ),
	newCmap( COLORMAP_OCEAN,	1, "Inv Ocean",	COLORMAP_INDEX ),
	newCmap( COLORMAP_SUMMER,	0, "Summer",	COLORMAP_INDEX ),
	newCmap( COLORMAP_SPRING,	0, "Spring",	COLORMAP_INDEX ),
	newCmap( COLORMAP_COOL,		0, "Cool",	COLORMAP_INDEX ),
	newCmap( COLORMAP_HSV,		0, "HSV",	COLORMAP_INDEX ),
	newCmap( COLORMAP_HSV,		1, "Inv HSV",	COLORMAP_INDEX ),
	newCmap( COLORMAP_PINK,		0, "Pink",	COLORMAP_INDEX ),
	newCmap( COLORMAP_HOT,		0, "Hot",	COLORMAP_INDEX ),
	newCmap( COLORMAP_HOT,		1, "Cold",	COLORMAP_INDEX ),
	newCmap( COLORMAP_PARULA,	0, "Parula",	COLORMAP_INDEX ),
	newCmap( COLORMAP_MAGMA,	0, "Magma",	COLORMAP_INDEX ),
	newCmap( COLORMAP_INFERNO,	0, "Inferno",	COLORMAP_INDEX ),
	newCmap( COLORMAP_PLASMA,	0, "Plasma",	COLORMAP_INDEX ),
	newCmap( COLORMAP_VIRIDIS,	0, "Viridis",	COLORMAP_INDEX ),
	newCmap( COLORMAP_CIVIDIS,	0, "Cividis",	COLORMAP_INDEX ),
	newCmap( COLORMAP_TWILIGHT,	0, "Twilight",	COLORMAP_INDEX ),
	newCmap( COLORMAP_TWILIGHT_SHIFTED, 0, "Twilight Shift", COLORMAP_INDEX ),
	newCmap( COLORMAP_TURBO,	0, "Turbo",	COLORMAP_INDEX ),
	newCmap( COLORMAP_TURBO,	1, "Inv Turbo",	COLORMAP_INDEX ),
	newCmap( COLORMAP_DEEPGREEN,	0, "Deepgreen",	COLORMAP_INDEX ),

	newCmap( &HSLCmap,		0, "HSL",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &inverseHSLCmap,	0, "Inv HSL",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &red2BlueCmap,		0, "R2B",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &red2BlueBlackLCmap,	0, "C2BlackL",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &red2BlueBlackUCCmap,	0, "C2BlackUC",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &red2BlueWhiteLCmap,	0, "C2WhiteL",	COLORMAP_MATRIX ), // User defined colormap
	newCmap( &red2BlueWhiteUCCmap,	0, "C2WhiteUC",	COLORMAP_MATRIX )  // User defined colormap
};

#define MAX_CMAPS  ARRAY_COUNT( cmaps )

// Default to JET if not set in Makefile
#ifndef DEFAULT_COLORMAP
#define DEFAULT_COLORMAP (2 + COLORMAP_JET) // Jet
#endif

#define CVT_CHAN_FLAG 0

#define DEFAULT_COLORMAP_INDEX  abs(DEFAULT_COLORMAP % MAX_CMAPS) 

void applyMyColorMap( InputArray src, OutputArray dst, int cmapsIndex ) {

	ASSERT(( 0 <= cmapsIndex ));
	ASSERT(( cmapsIndex < MAX_CMAPS ));

	if        ( COLORMAP_INDEX == cmaps[ cmapsIndex ]->type ) {
		applyColorMap( src, dst, abs(cmaps[ cmapsIndex ]->cmap.index) );

		// Swap Blue and Reds on selected stock colormaps - BlueGreenRed to RedGreenBlue
		if ( cmaps[ cmapsIndex ]->bgr2rgb ) {
			cvtColor(src, dst, COLOR_BGR2RGB, CVT_CHAN_FLAG);
		}

	} else if ( COLORMAP_MATRIX == cmaps[ cmapsIndex ]->type ) {
		applyColorMap( src, dst, *cmaps[ cmapsIndex ]->cmap.matrix );
	}
	// else COLORMAP_NONE type has no maps, so nothing to apply
}


#define HSL_RED       0  // %= 360
#define HSL_YELLOW   60
#define HSL_GREEN   120
#define HSL_CYAN    180
#define HSL_BLUE    240
#define HSL_VIOLET  300

// Unit Circle
#define DEG_2_RAD(degrees)	(((degrees) * M_PI) / 180.0)
#define DEG_2_X(degrees)	sin( DEG_2_RAD( degrees) )
#define DEG_2_Y(degrees)	cos( DEG_2_RAD( degrees ) )

// Convert degrees to unit circle x, y with scaling and ceiling
#define DEG_2_UNIT_X(degrees, scale, ceiling)  MIN( scale * DEG_2_X(degrees), ceiling )
#define DEG_2_UNIT_Y(degrees, scale, ceiling)  MIN( scale * DEG_2_Y(degrees), ceiling )

void hslToRgb(float h, float s, float l, unsigned int *red, unsigned int *green, unsigned int *blue);

Mat createRed2BlueBlackWhiteCmap(int ramp) {
// printf("%s(%d)\n", __func__, __LINE__);
    Mat cmap(COLORMAP_ROWS, 1, CV_8UC3);

    unsigned char *ucPtr = &((unsigned char *)(cmap.datastart))[0];

    unsigned int red, green, blue;
    float lightness;

    float MAX_HSL   = HSL_BLUE + 7.0; // Other side of blue
    float START_HSL;
    float START_L; 

    if        (0  < ramp) {	// Ramp to white
        START_HSL = MAX_HSL - 15;
        START_L   = 0.4;
    } else if (0 == ramp) {	// No Ramp
        START_HSL = MAX_HSL;
        START_L   = 0.5;
    } else {		// Ramp to black
        START_HSL = MAX_HSL - 15; // HSL_GREEN;
        START_L   = 0.6;
    }

// TODO - FIXME - Add ramp on both hot and cold ends
//                Twilight white - dark - white
//                Red-Blue through Purple, NOT yellow/green
//                Circle of round to black with black in the middle

// COLORMAP_ROWS colors [0 - 255] in colormap
// Red(0) to Blue(240) leaves 15 entries left over (black and white)
#define PLUS_MINUS 30.0 // (RED (0-15)) and (BLUE (240+15))

    if (2 == abs(ramp)) MAX_HSL = (HSL_BLUE + PLUS_MINUS);

    for (int i = 0; i < COLORMAP_ROWS ; i++) {

//  Rainbow( Violet to Red ) and Inv Rainbow( Red - Violet )

        // degrees[RED to BLUE], saturation, lightness
        float degrees =	MAX_HSL * ((float)(255-i)/255.0); // MAX_RANGE * ([1.0 to 0.0])

        float zeroTo180 = (180.0 * i)/255.0; // Convert [0 - 255] into [0 - 180]
#define UC_SCALE 1.4 // 3.0 // 1.9 // 1.4 // 3.0
        if      (R2B_B2W_UC == ramp) {	// Unit Circle curve
		if (i < COLORMAP_ROWS/2) {
            		lightness = 1.0 - DEG_2_UNIT_X( zeroTo180, UC_SCALE, 0.5 );
		} else {
            		lightness =       DEG_2_UNIT_X( zeroTo180, UC_SCALE, 0.5 );
		}
        }
        else if (R2B_W2B_UC == ramp) {	// Unit Circle curve
		if (i < COLORMAP_ROWS/2) {
            		lightness =       DEG_2_UNIT_X( zeroTo180, UC_SCALE, 0.5 );
		} else {
            		lightness = 1.0 - DEG_2_UNIT_X( zeroTo180, UC_SCALE, 0.5 );
		}
        } else {			// Linear ramp

            // Start to ramp to [black, white, none]
            if (ramp != 0 && START_HSL < degrees && degrees <= MAX_HSL) {
                float delta = ((degrees - START_HSL) / (MAX_HSL - START_HSL)); // 0.0 to 1.0
                if        (ramp < 0) { // Ramp to Black
                    delta *= START_L;		
                    lightness = START_L - delta;	// Ramp from [ START_L to 0.0 ]
//printf("- start %.1f, delta %f, lightness %f\n", START_L, delta, lightness);
                    if (lightness < 0.0) lightness = 0.0;

                } else if (ramp > 0) { // Ramp to White
                    delta *= (1.0 - START_L);	
                    lightness = START_L + delta;	// Ramp from [ START_L to 1.0 ]
//printf("+ start %.1f, delta %f, lightness %f\n", START_L, delta, lightness);
                    if (lightness > 1.0) lightness = 1.0;
                }
            } else {
                lightness = START_L; // No ramp
            }
        }

	if (2 == abs(ramp)) degrees -= (PLUS_MINUS / 2.0); // rotate back 15 degrees

        hslToRgb( degrees, 1.0, lightness, &red, &green, &blue);

        ucPtr[i*3+0] = blue;   // Stored BGR not RGB
        ucPtr[i*3+1] = green;  // Stored BGR not RGB
        ucPtr[i*3+2] = red;    // Stored BGR not RGB
    }

    return cmap;
}

Mat createHSLCmap(int direction) {
	Mat cmap(COLORMAP_ROWS, 1, CV_8UC3);
	unsigned char *ucPtr = &((unsigned char *)(cmap.datastart))[0];

	unsigned int red, green, blue;
	float MAX_HSL = HSL_VIOLET;

	for (int i = 0; i < COLORMAP_ROWS ; i++) {
		float degrees = (direction < 0) ?
				MAX_HSL * ((float)(255-i)/255.0) :
				MAX_HSL * ((float)(    i)/255.0);
		hslToRgb( degrees, 1.0, 0.5, &red, &green, &blue);

		ucPtr[i*3+0] = blue;   // Stored BGR not RGB
		ucPtr[i*3+1] = green;  // Stored BGR not RGB
		ucPtr[i*3+2] = red;    // Stored BGR not RGB
	}
	return cmap;
}


typedef struct {
	int inter;
	const char *name;
} Inter;

Inter Inters[] = {
	{ INTER_NEAREST,	"Nearest" },
	{ INTER_LINEAR,		"Linear" },	//          Faster but lower  quality
	{ INTER_CUBIC,		"Cubic" },	// DEFAULT: Slower but higher quality
	{ INTER_AREA,		"Area" },
	{ INTER_LANCZOS4,	"Lanczos4" },
	{ INTER_LINEAR_EXACT,	"Lin Exact" },
	{ INTER_NEAREST_EXACT,	"Near Exact" },
#if 0 // These crash resize()
	{ INTER_MAX, "Max" },            // resize() Crash
	{ WARP_FILL_OUTLIERS, "Fill" },  // resize() Crash
	{ WARP_INVERSE_MAP, "Inverse" }, // resize() Crash
#endif
};

typedef enum {
	ACTIVE_OFF = 0,
	ACTIVE_ON,
	ACTIVE_ARROW,
	ACTIVE_DASH 
} ActiveState;

#define TEMP_TYPE_CMAP_SCALE 1  // Don't draw certain markers
// Temperature is 160 bytes
typedef struct {
	// 8-Byte align struct
	long  linearI;	         // linear index
	long  row, col;	         // Unscaled y, x
	long  xScaled, yScaled;	 // Pre scaled y, x thread optimization

	// Floats and ints are 4 bytes
	float       kelvin;	 // kelvin temp
	float       celsius;	 // celsius temp
	int         type;
	ActiveState active;

	const char *label;       // Display label
	char  displayLabel[64];  // thread balancing optimization
	Point markerLoc;
	Point labelLoc;
	Point markerwDLoc;
	Point labelwDLoc;
} Temperature;


bool Use_Celsius;   // GLOBAL KLUGE UNTIL REWORKED
int Use_Histogram; // GLOBAL KLUGE UNTIL REWORKED

// Inline optimization
#define CorF(cel) (float)(Use_Celsius ? cel : ( (((float)cel * 9.0)/5.0) + 32.0 ))

#if 0

// Inline optimization
#define kelvin2Celsius(kel)     ((float)(((float)(kel) / 64.0) - 273.15))
#define celsius2Kelvin(cel)      ((long)round(((float)(cel) + 273.15) * 64.0 ))
#define fahr2Kelvin(fahr)         (long)( celsius2Kelvin ( ((float)(fahr) - 32.0) * 5.0 / 9.0 ) )
#define celsius2Fahr(cel)      ((((float)cel * 9.0) / 5.0) + 32.0 )
}

#else

float kelvin2Celsius(long kelvin) { // # LeoDJ's Kelvin conversion algorithm, post #216
	return ( ((float)kelvin / 64.0) - 273.15 );
}

// Used to convert threshold to kelvin for optimized comparisons
long celsius2Kelvin(float celsius) { // # LeoDJ's Kelvin conversion algorithm, post #216
	//  celsius                  = (kelvin / 64.0) - 273.15
	//  celsius + 273.15         = (kelvin / 64.0)
	// (celsius + 273.15) * 64.0 = kelvin
	return ( round((celsius + 273.15)  *  64.0  ) );
}

// Used to convert threshold to kelvin for optimized comparisons
long fahr2Kelvin(float fahr) {
	//   f                       =  ((celsius * 9.0) / 5.0) + 32.0
	//   f - 32.0                =  ((celsius * 9.0) / 5.0)
	//  (f - 32.0) * 5.0         =   (celsius * 9.0)
	// ((f - 32.0) * 5.0) / 9.0  =    celsius
	return ( celsius2Kelvin ( (fahr - 32.0) * 5.0 / 9.0 ) );
}

float celsius2Fahr( float celsius ) {
	return ( (((float)celsius * 9.0)/5.0) + 32.0 );
}

#endif


// These colors may change - wish there was an XOR color to work across all colormaps
// These colors are used for both the rulers and Colormap Gradiant Scale markers
#define RULER_MAX_COLOR		GREEN
#define RULER_MID_COLOR		WHITE
#define RULER_MIN_COLOR		RED
#define RULER_BASELINE_COLOR	BLACK


typedef enum {
	WINDOW_IMAGE = 0,
	WINDOW_THERMAL,
	WINDOW_DOUBLE_WIDE,
	WINDOW_DOUBLE_HIGH
} WindowFormat;

typedef struct {
	int wf;
	const char *name;
} WF;

WF WFs[] = {
	{ WINDOW_IMAGE, 	"Img" },
	{ WINDOW_THERMAL,	"Therm" },
	{ WINDOW_DOUBLE_WIDE,	"Img+Therm" },
	{ WINDOW_DOUBLE_HIGH,	"Img+Therm" }
};

#define MAX_INTERS ARRAY_COUNT( Inters )
#define MAX_WFS    ARRAY_COUNT( WFs )

typedef enum {
	HUD_ON=0,
	HUD_HELP,
	HUD_OFF,
	HUD_NO_DRAWINGS,
	HUD_MAX_MOD // Rollover count, not actual HUD mode
} HUDFormat;

const char *hudFormatStr( HUDFormat hudFormat ) {
#ifdef MY_CASE
#undef MY_CASE
#endif
// Strip off "HUD_" prefix
#define MY_CASE(a) case (a): return (const char *)(#a + sizeof("HUD"));
	switch ( hudFormat ) {
		MY_CASE(HUD_ON)
		MY_CASE(HUD_HELP)
		MY_CASE(HUD_OFF)
		MY_CASE(HUD_NO_DRAWINGS)
		default: break;
	}
	return "Undefined";
}

// Globals for call optimization
static double MyFontScale;  // Other fonts scale differently than HUD fonts
static double HudFontScale; // HUD font scales differently than other fonts
static int    sX = 0, sY = 0;

typedef struct {
	const char *labelCF;
	const char *labelWF;
	char snaptime[128];
	char elapsed[128];
	int64_t recFrameCounter;
	Temperature threshold;

	int cmapCurrent;
	int rad;
	int inters;

	int sW;               // scaled Width  of SINGLE or WINDOW_DOUBLE - resize(sW,sH)
	int sH;               // scaled Height of SINGLE or WINDOW_DOUBLE - resize(sW,sH)

	int scaledSFWidth;    // scaled SINGLE Frame Width   (scale * TC_WIDTH)
	int scaledSFHeight;   // scaled SINGLE Frame Height  (scale * TC_HEIGHT)
	int windowFormat;
	int lastHelpScale = -1; // Optimization so HELP doesn't have to update every frame

	int64_t frameCounter;
	double alpha;         // see Inters[]
	int64_t startMills;
	double fps;
	time_t recordStartTime; // Seconds time(null)
	HUDFormat hud;          // HUDFormats
	bool wD;           // WindowDouble if DOUBLE_WIDTH or DOUBLE_HIGH
	bool useCelsius;
	bool recording;
	bool recordingActive = false;
	bool fullscreen;
} Controls;


#define MAX_USER_TEMPS 13
#define CENTER_OF_RULER_INDEX (MAX_USER_TEMPS - 1)
static Temperature users[ MAX_USER_TEMPS ];  // User temps XOR Ruler Temps
static Controls controls;

static Temperature *user_0  = &users[0];
static Temperature *user_1  = &users[1];
static Temperature *user_2  = &users[2];
static Temperature *user_3  = &users[3];
static Temperature *user_4  = &users[4];
static Temperature *user_5  = &users[5];
static Temperature *user_6  = &users[6];
static Temperature *user_7  = &users[7];
static Temperature *user_8  = &users[8];
static Temperature *user_9  = &users[9];
static Temperature *user_10 = &users[10];
static Temperature *user_11 = &users[11];
static Temperature *user_CENTER_OF_RULER_INDEX = &users[CENTER_OF_RULER_INDEX];

// Optimization: Remove if's from rendering routines by using function pointer
extern void drawTempMarker( Mat & frame, Temperature &temp, Scalar &dotColor );
extern void drawTempMarkerNoHalo( Mat & frame, Temperature &temp, Scalar &dotColor );
void (*drawTempMarkerPtr)( Mat &, Temperature &, Scalar & ) = &drawTempMarker;

// Loop unrolls:
#define DO_ALL_HORIZ()		DO_USER(0) DO_USER(1) DO_USER(2) DO_USER(3) DO_USER(4)  DO_USER(5)
#define DO_ALL_VERT()		DO_USER(6) DO_USER(7) DO_USER(8) DO_USER(9) DO_USER(10) DO_USER(11)
#define DO_ALL_BUT_CENTER()	DO_ALL_HORIZ()  DO_ALL_VERT()
#define DO_ALL()		DO_ALL_BUT_CENTER() DO_USER(CENTER_OF_RULER_INDEX)

typedef struct {
	Temperature min, avg, max, ch; // ch = CrossHair center of screen
	Temperature minPixel, maxPixel; // kelvin is actually the image pixel, not thermal pixel
	// avgN are derrived from minPixel and maxPixel
	Temperature avg1Pixel, avg2Pixel, avg3Pixel, avg4Pixel;
	Temperature chLevelPixel, avgLevelPixel;

	int userI;
	int userCount;

	VideoWriter videoOut;  // memset compiler errors
	Scalar *rColor;        // memset compiler errors
} ProcessedThermalFrame;

pthread_mutex_t videoOutMutex = PTHREAD_MUTEX_INITIALIZER;

typedef enum {
	RULERS_OFF = 0,
	RULERS_ONE_TEMP,
	RULERS_CROSS_HAIR,
	RULERS_HORIZONTAL,
	RULERS_VERTICAL,
	RULERS_BOTH,
	RULERS_MAX_MOD // Rollover count, not actual ruler mode
} RulerModes;

typedef enum {
	BOUND_PROPORTIONAL = 0,
	BOUND_EQUAL_OUTLIER_CLIPPING,
	BOUND_BELOW_AVG_CLIPPING,
	BOUND_ABOVE_AVG_CLIPPING,
	BOUND_MAX_MOD
} RulerBounds;

// In thickness mode 1, the plot is relative to the screen, not the ruler cross hair
#define MIN_RULER_THICKNESS  5  //  ( 1 / 5 ) * SCREEN_HEIGHT
#define MAX_RULER_THICKNESS  1  //  ( 1 / 1 ) * SCREEN_HEIGHT

static int        rulerThickness = MIN_RULER_THICKNESS;
static int        rulerBoundFlag = BOUND_PROPORTIONAL;
static int        minBound, maxBound;
static RulerModes rulersOn = RULERS_OFF; // Rotate through ruler modes
static int        rulersX  = FIXED_TC_WIDTH  / 2;
static int        rulersY  = FIXED_TC_HEIGHT / 2;

static int Rulers_Both		  = (( RULERS_BOTH == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_XOR		  = (( RULERS_VERTICAL == rulersOn || RULERS_HORIZONTAL == rulersOn) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_Horiz		  = (( RULERS_HORIZONTAL == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_Vert		  = (( RULERS_VERTICAL   == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_Both_Horiz      = (( RULERS_BOTH == rulersOn || RULERS_HORIZONTAL == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_Both_Vert       = (( RULERS_BOTH == rulersOn || RULERS_VERTICAL   == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
static int Rulers_Both_Horiz_Vert =  ( Rulers_Both_Horiz || Rulers_Both_Vert );
static int Max_Ruler_Thickness    =  ( MAX_RULER_THICKNESS == rulerThickness );
static int Horiz_Min_Bound        = ( !(Max_Ruler_Thickness && ((TC_HEIGHT - 1) == rulersY)) );
static int Horiz_Max_Bound        = ( !(Max_Ruler_Thickness && (0 == rulersY)) );
static int Vert_Min_Bound         = ( !(Max_Ruler_Thickness && (0 == rulersX)) );
static int Vert_Max_Bound         = ( !(Max_Ruler_Thickness && ((TC_WIDTH - 1) == rulersX)) );

const char *rulerModesStr( RulerModes rulerMode ) {
#ifdef MY_CASE
#undef MY_CASE
#endif
// Strip off "RULERS_" prefix
#define MY_CASE(a) case (a): return (const char *)(#a + sizeof("RULERS"));
	switch ( rulerMode ) {
		MY_CASE(RULERS_OFF)
		MY_CASE(RULERS_ONE_TEMP)
		MY_CASE(RULERS_CROSS_HAIR)
		MY_CASE(RULERS_HORIZONTAL)
		MY_CASE(RULERS_VERTICAL)
		MY_CASE(RULERS_BOTH)
		MY_CASE(RULERS_MAX_MOD)
		default: break;
	}
	return "Undefined";
}

void clearPtf(ProcessedThermalFrame *ptf) {

	ptf->rColor = &BLACK;

	ptf->userI = ptf->userCount = 0;

	ptf->minPixel.celsius      = LONG_MAX; // Prime for history calculations over time
	ptf->avg1Pixel.celsius     = LONG_MAX; // Prime for history calculations over time
	ptf->avg2Pixel.celsius     = LONG_MAX; // Prime for history calculations over time
	ptf->avg3Pixel.celsius     = LONG_MAX; // Prime for history calculations over time
	ptf->avg4Pixel.celsius     = LONG_MAX; // Prime for history calculations over time
	ptf->chLevelPixel.celsius  = LONG_MAX; // Prime for history calculations over time
	ptf->avgLevelPixel.celsius = LONG_MAX; // Prime for history calculations over time
	ptf->maxPixel.celsius      = LONG_MIN; // Prime for history calculations over time
}

extern void reScale(ProcessedThermalFrame *ptf, int value, bool flag);
extern void getTemperature(Mat *thermalFrame, Temperature &temp);

typedef struct {
	int64_t thermMicros;	// thermalDataThread's benchmarks
	int64_t imageMicros;	// imageDataThread's   benchmarks
	int64_t rotMicros;	// drawGraphics        benchmarks
	int64_t mainMicros;	// [read thru imshow]  benchmarks
	int64_t readMicros;	// [read thru imshow]  benchmarks
	int64_t imshowMicros;	// [read thru imshow]  benchmarks
	ProcessedThermalFrame *ptf; // Pseudo "this" pointer
	Mat  *rgbFrame;		// Image rendering frame
	Mat  *rgbFrameOrig;	// Image rendering frame
	Mat  *rawFrame;		// Used to dump raw offline post processing image (no rotations)
	Mat   maxCmapScale;	// max Colormap gradiant scale plane buffer
	Mat   maxRgbHud;	// max scaled HUD and HELP graphics plane buffer
	Mat   cmapScale;	// pre-rendered Colormap gradiant scale
	Mat   rgbHUD;		// pre-rendered HUD and HELP text 
	const char *source;
	const char *inputFile;

	int   FreezeFrame;
	int   lostVideo;
	int   thermalState = 0;
	int   imageState   = 0;

	int   running = 1;
	int   configurationChanged;
} ThreadData;

typedef struct {
	int64_t renderMicros = 0; // imageDataThread's   benchmarks
	int     renderState  = 0;
} RenderData;
 
static ThreadData threadData;

#define NUMBER_OF_CLIENTS  3  // mainThread, thermalDataThread, imageDataThread
#define EXIT_STATE         (int)(0xDeadBeef)

static int MyThreadStates[] = { 0, 1, 2, 3 };

static int BothPlotStates[] = { 0, 1 };   // clientCount = 2
static int BothPlotState    = 0;

static int CmapPlotStates[] = { 0, 1 };   // clientCount = 3
static int CmapPlotState    = 0;

static int B4TempPlotStates[] = { 0, 1 }; // clientCount = 3
static int B4TempPlotState    = 0;

typedef struct {
	int *states               =  MyThreadStates;
	int  maxStates            =  ARRAY_COUNT( MyThreadStates );
	int  blockingState        =  0; // initial start state
	int  clientCount          =  0; 
	int  maxClientCount       =  NUMBER_OF_CLIENTS;
    	pthread_mutex_t condMutex =  PTHREAD_MUTEX_INITIALIZER;
        pthread_cond_t  cond      =  PTHREAD_COND_INITIALIZER;
} IncWaitMutex;


static IncWaitMutex WorkerMutex;
static IncWaitMutex BothPlotMutex;
static IncWaitMutex CmapPlotMutex;
static IncWaitMutex B4TempPlotMutex;

void initIncWaitMutex( IncWaitMutex *mutex, int *states, int maxStates, int blockingState, int maxClientCount ) {
	mutex->states         = states;
	mutex->maxStates      = maxStates;
	mutex->blockingState  = blockingState;
	mutex->maxClientCount = maxClientCount;
}

void exitAllThreads() {
	// Threads will never block on EXIT_STATE
	for (int i = 0; i < WorkerMutex.maxStates; i++ ) {
		WorkerMutex.blockingState = EXIT_STATE;
		threadData.running    = 0; // Tell all threads to exit
		pthread_cond_broadcast( &WorkerMutex.cond );
	}
}

void incAndWait( IncWaitMutex *mutex, int *threadState, int blockingState ) { 
	// increment, advance, broadcast and wait

	// Thread synchronization
	pthread_mutex_lock( &mutex->condMutex );

		*threadState = blockingState; 
		mutex->clientCount++;
		// Verify client is in sync with other blocked clients
		ASSERT(( mutex->blockingState == blockingState ));
		if ( mutex->maxClientCount <= mutex->clientCount ) {
			// zero blocked clientCount and advance to next round robin blocking state
			mutex->clientCount   = 0;
			mutex->blockingState = 
				mutex->states[ ((mutex->blockingState + 1) % mutex->maxStates) ];
			// start threads racing to next state
			pthread_cond_broadcast( &mutex->cond );
		} else {
			while ( mutex->blockingState == blockingState ) {
				pthread_cond_wait( &mutex->cond, &mutex->condMutex );
			}
		}

	pthread_mutex_unlock( &mutex->condMutex );
}

#if 1
#define RERENDER_OPTIMIZATION  ( threadData.configurationChanged || ( ! threadData.inputFile && ! threadData.FreezeFrame ) )
#else
#define RERENDER_OPTIMIZATION  ( 1 )
#endif

void dumpFrameInfo(Mat *frame);

// cmapScale display controls
#define CMAP_AUTO_RANGE_DEFAULT 1
long minPixel = LONG_MAX; // pixel value from imageFrame ( NOT thermalFrame, same linearI as thermalFrame )
long maxPixel = LONG_MIN; // pixel value from imageFrame ( NOT thermalFrame, same linearI as thermalFrame )
long cmapAutoRange = CMAP_AUTO_RANGE_DEFAULT;  // Toggle color map auto ranging

void unlockColormapRange(int toggle) {
	if (toggle) {
		cmapAutoRange = !cmapAutoRange;
	}
	minPixel = LONG_MAX;
	maxPixel = LONG_MIN;
}

// Getting rid of stack overhead and using global variables saved 1%
long centerOfPixel(long loc) 
{ 
	// Visual center of scaled pixel
	// If scale is 8, then each temp is displayed as 8 x 8 pixels versus 1 x 1 pixel
	// Instead of Top/Left, [0,0], use center [(8/2),(8/2)] of the scaled pixel
#if 1
	return ( ((loc / MyScale) * MyScale) + MyHalfScale );
#else
	return (((loc / scale) * scale) + (scale / 2));
#endif
}

#define centerOfPixelNODIV(loc) ((int)( (loc * MyScale) + MyHalfScale ))

long centerOfPixelNoDiv(long loc) {
	return ( (loc * MyScale) + MyHalfScale );
}

#define centerOfPixel(loc,scale) centerOfPixel(loc)

static void userTemp(Temperature &temp, int x, int y, int scale) {
	// Handle clicks in DOUBLE_WIDE or DOUBLE_HIGH frames
	// Convert DOUBLE frame locations into single frame locations
	if (x >= (TC_WIDTH * scale)) { // Set to multiples of [0 to (TC_WIDTH -1)]
		x -= (TC_WIDTH * scale);
	} else if (x < 0) {
		x += (TC_WIDTH * scale);
	}

	if (y >= (TC_HEIGHT * scale)) { // Set to multiples of [0 to (TC_HEIGHT-1)]
		y -= (TC_HEIGHT * scale);
	} else if (y < 0) {
		y += (TC_HEIGHT * scale);
	}

	// Use center of scaled pixel
	temp.xScaled  = centerOfPixel( x, scale );  // Scaled X
	temp.yScaled  = centerOfPixel( y, scale );  // Scaled Y

	temp.col      = x / scale; // Convert back to TC_WIDTH  (1X scale)
	temp.row      = y / scale; // Convert back to TC_HEIGHT (1X scale)
	temp.active   = ACTIVE_ON;
}

void rotateXYRight90(int *x, int width, int *y) {
	int tmpX = *x;
	*x = (width-1) - *y;
	*y = tmpX;
}

static void rotateUserTemps90() {
	for (int i = 0; i < MAX_USER_TEMPS; i++) {
		if ( users[i].active ) {
			int x = users[i].col;
			int y = users[i].row;

			rotateXYRight90( &x, TC_WIDTH, &y );

			userTemp( users[i], x * MyScale, y * MyScale, MyScale );
		}
	}
}

static void clearUserTemps(ProcessedThermalFrame *ptf) {
	rulersOn   = RULERS_OFF;
	ptf->userI = ptf->userCount = 0; // Optimization for rendering

	for (int i = 0; i < MAX_USER_TEMPS; i++) {	
		users[i].active = ACTIVE_OFF;
	}
}

static void rulers(ProcessedThermalFrame *ptf, int horizontalOffset, int verticalOffset, int keyPress) {
	RulerModes tempRulersOn = rulersOn;
	clearUserTemps( ptf );

	horizontalOffset = centerOfPixelNODIV( horizontalOffset );
	verticalOffset   = centerOfPixelNODIV( verticalOffset );

	if ( tempRulersOn ) {
		// case (-105) // keypad up
		// case (-103) // keypad down
		// case (-106) // keypad left
		// case (-104) // keypad right
		// case (-99)  // keypad 5 (center)
		// waitKey(82) // up    -  SAME AS CAPITAL R
		// waitKey(84) // down
		// waitKey(81) // left
		// waitKey(83) // right
		int scaledDelta = 1 * MyScale; // Move by 1 scaled pixel resolution
		switch ( keyPress ) {
			case -99: // Keypad 5/Center key
				horizontalOffset = centerOfPixel( controls.scaledSFWidth  / 2, MyScale );
				verticalOffset   = centerOfPixel( controls.scaledSFHeight / 2, MyScale );
				break;
			case -105: // Keypad 8/Up Arrow
				verticalOffset   -= scaledDelta;
				break;
			case -103: // Keypad 2/Down Arrow
				verticalOffset   += scaledDelta;
				break;
			case -106: // Keypad 4/Left Arrow
				horizontalOffset -= scaledDelta;
				break;
			case -104: // Keypad 6/Right Arrow
				horizontalOffset += scaledDelta;
				break;
			default: break;
		}
	}

	// Handle horizontal left/right wrapping
	if      (horizontalOffset > controls.scaledSFWidth) {
		horizontalOffset -= controls.scaledSFWidth;
	} else if (horizontalOffset < 0) {
		horizontalOffset += controls.scaledSFWidth;
	}

	// Handle vertical top/bottom wrapping
	if      (verticalOffset   > controls.scaledSFHeight) {
		verticalOffset   -= controls.scaledSFHeight;
	} else if (verticalOffset   < 0) {
		verticalOffset   += controls.scaledSFHeight;
	}

	int centerX = horizontalOffset;
	int centerY = verticalOffset;

	// Stash unscaled X,Y location for later relative use with Up/Down/Left/Right keypad arrow movement
	rulersX = centerX / MyScale;
	rulersY = centerY / MyScale;

	// Center temp at intersection of horiztonal and vertical rulers
	userTemp( *user_CENTER_OF_RULER_INDEX, centerX, centerY, MyScale );

	// Scale down number of ruler temps based on MyScale 
	// and what comfortably fits in PORTRAIT width
	//     1X scale has 3 temps per row/col
	//     2X scale has 5 temps per row/col
	// [3-N]X scale has 7 temps per row/col
	int reduceBy  = (MyScale < 2) ? 4 :
		        (MyScale < 3) ? 2 : 0;

	// Take even # off horizontally and vertically
	int itemsPerRowCol = ((MAX_USER_TEMPS - 1) / 2) - reduceBy;

#undef  DO_USER
#define DO_USER(n) user_##n->active = ACTIVE_OFF;

	if ( RULERS_ONE_TEMP == tempRulersOn ) {
		DO_ALL_BUT_CENTER(); // Loop unroll
	} else {
		int originalMax = (MAX_USER_TEMPS - 1) / 2;

		// Take even # off horizontally and vertically
		int horizMax    = originalMax - reduceBy;

		if (RULERS_VERTICAL == tempRulersOn) {
			// Turn off horizontal ruler temps
			DO_ALL_HORIZ(); // Loop unroll
		} else { // Prepare horizontal ruler temps
			int delta = (MyScale * TC_WIDTH) / (itemsPerRowCol + 1);
			int x     = delta;
			int y     = centerY;
	
			for ( int i = 0; i < horizMax; i+=2, x += delta ) {
				userTemp( users[i  ], centerX + x, y, MyScale );
				userTemp( users[i+1], centerX - x, y, MyScale );
			}

			for ( int i = horizMax; i < originalMax; i++ ) {
				users[i].active = ACTIVE_OFF; 
				ASSERT((i != CENTER_OF_RULER_INDEX));
			}
		}

		// Take even # off horizontally and vertically
		int vertMax = (MAX_USER_TEMPS - 1) - reduceBy;

		if (RULERS_HORIZONTAL == tempRulersOn) {
			// Turn off vertical ruler temps
			DO_ALL_VERT(); // Loop unroll
		} else { // Prepare vertical ruler temps
			int delta = (MyScale * TC_HEIGHT) / (itemsPerRowCol + 1);
			int x     = centerX;
			int y     = delta;

			for ( int i = originalMax; i < vertMax; i += 2, y += delta ) {
				userTemp( users[i  ], x, centerY + y, MyScale );
				userTemp( users[i+1], x, centerY - y, MyScale );
			}

			for ( int i = vertMax; i < CENTER_OF_RULER_INDEX; i++ ) {
				users[i].active = ACTIVE_OFF; 
				ASSERT((i != CENTER_OF_RULER_INDEX));
			}
		}
	}

	ptf->userI     = 0;
	ptf->userCount = MAX_USER_TEMPS;
	rulersOn       = tempRulersOn;
}


static void normalizeCB( int *x, int *y ) {

	if        (*x < 0) {
		*x = 0;
	} else if (*x > (TC_WIDTH-1)*MyScale) {
		if (WINDOW_DOUBLE_WIDE == controls.windowFormat) {
			if (*x > 2*(TC_WIDTH-1)*MyScale)
				*x = (TC_WIDTH-1)*MyScale;
			else
				*x = *x % ((TC_WIDTH-1)*MyScale);
		} else {
			*x = (TC_WIDTH-1)*MyScale;
		}
	}

	if        (*y < 0) {
		*y = 0;
	} else if (*y > (TC_HEIGHT-1)*MyScale) {
		if (WINDOW_DOUBLE_HIGH == controls.windowFormat) {
			if (*y > 2*(TC_HEIGHT-1)*MyScale)
				*y = (TC_HEIGHT-1)*MyScale;
			else
				*y = *y % ((TC_HEIGHT-1)*MyScale);
		} else {
			*y = (TC_HEIGHT-1)*MyScale;
		}
	}
}

// Debounce multiple mouse events
typedef struct {
	int mouseActive = 0;
	int x           = 0;
	int y           = 0; 
} MouseRulerDebounce;

MouseRulerDebounce MRD;

// Handle 1 or more mouse events / frame
// onMouseCallback is called during the waitKeyEx() call
static void onMouseCallback( int event, int x, int y, int, void* ptr ) {
	// Complete callback as fast as possible

//printf( "%s(%d): %d(%d, %d)\n", __func__, __LINE__, event, x, y ); fflush(stdout);

	if ( leftDragOff && (EVENT_MOUSEMOVE == event) ) {
		return; // Ignore mouse hover events unless we want to implement XEyes
	}

	if        ( EVENT_LBUTTONDOWN == event ) {
		leftDragOff = 0;
	} else if ( (EVENT_MOUSEMOVE != event) && ( ! leftDragOff ) ) {
		leftDragOff = 1;
	}

	switch( event ) {
		case EVENT_MOUSEMOVE:
		case EVENT_LBUTTONDOWN: {

			if ( rulersOn && ( ! leftDragOff ) ) {
				// Debounce multiple mouse/ruler events occuring in single frame
				// N:1 reduction of calls to normalizeCB() and rulers()
				MRD.mouseActive++;
				MRD.x = x;
				MRD.y = y;
				return;
			}

			if ( EVENT_MOUSEMOVE == event ) {
				return;
			}

			// Process non-ruler click events
	
			// Grab and store user's locations of interest	
			ProcessedThermalFrame *ptf = ((ThreadData *)ptr)->ptf;

			normalizeCB( &x, &y );

			userTemp( users[ptf->userI], x, y, MyScale );

			// Let thermal data harvesting code handle temp harvesting
			ptf->userCount++;
			if ( ptf->userCount > MAX_USER_TEMPS ) {
				ptf->userCount = MAX_USER_TEMPS;
			}
			ptf->userI = (ptf->userI + 1) % MAX_USER_TEMPS;
		} break;

		case EVENT_RBUTTONDOWN: { // Turn OFF ALL user temps
			// Clear user's temp locations of interest and temp rulers
			clearUserTemps( ((ThreadData *)ptr)->ptf );
		} break;

		default: break;
	}
}

void setWindowFormat() {

	int scaledWidth  = TC_WIDTH  * MyScale;
	int scaledHeight = TC_HEIGHT * MyScale;

	// Single sub-frame layout
	controls.scaledSFWidth  = scaledWidth;
	controls.scaledSFHeight = scaledHeight;

	// Single and Double sub-fame layouts
	controls.wD = 0;
	sX = sY = 0;
	controls.sW = scaledWidth;
	controls.sH = scaledHeight;

	if	(controls.windowFormat == WINDOW_DOUBLE_WIDE) {
		controls.wD  = 1;
		sX = scaledWidth;
		controls.sW *= 2;
	}
	else if	(controls.windowFormat == WINDOW_DOUBLE_HIGH) {
		controls.wD  = 1;
		sY = scaledHeight;
		controls.sH *= 2;
	}
}

void setTempDefaults(ProcessedThermalFrame *ptf) {

	ptf->min.type = ptf->avg.type = ptf->max.type = ptf->ch.type = 0;

	ptf->minPixel.type       = ptf->maxPixel.type  = TEMP_TYPE_CMAP_SCALE;
	ptf->avg1Pixel.type      = ptf->avg2Pixel.type = TEMP_TYPE_CMAP_SCALE;
	ptf->avg3Pixel.type      = ptf->avg4Pixel.type = TEMP_TYPE_CMAP_SCALE;
	ptf->chLevelPixel.type   = TEMP_TYPE_CMAP_SCALE;
	ptf->avgLevelPixel.type  = TEMP_TYPE_CMAP_SCALE;

	ptf->min.label           = "Min: ";
	ptf->avg.label           = "Avg: "; // Isn't display outside of HUD and CmapScale
	ptf->max.label           = "Max: ";
	ptf->ch.label            = "";
	ptf->minPixel.label      = "";
	ptf->avg1Pixel.label     = "";
	ptf->avg2Pixel.label     = "";
	ptf->avg3Pixel.label     = "";
	ptf->avg4Pixel.label     = "";
	ptf->chLevelPixel.label  = "";
	ptf->avgLevelPixel.label = "";
	ptf->maxPixel.label      = "";

	ptf->min.active           = ACTIVE_ON;   // Show temp text
	ptf->avg.active           = ACTIVE_OFF;  // Only displayed in HUD
	ptf->max.active           = ACTIVE_ON;   // Show temp text

	ptf->minPixel.active      = ACTIVE_ON;   // Show temp text
	ptf->maxPixel.active      = ACTIVE_ON;   // Show temp text

	ptf->avg1Pixel.active     = ACTIVE_ON;   // Show temp text
	ptf->avg2Pixel.active     = ACTIVE_ON;   // Show temp text
	ptf->avg3Pixel.active     = ACTIVE_ON;   // Show temp text
	ptf->avg4Pixel.active     = ACTIVE_ON;   // Show temp text

	ptf->chLevelPixel.active  = ACTIVE_ARROW; // Show ">"
	ptf->avgLevelPixel.active = ACTIVE_DASH;  // Show "-"

	strcpy( ptf->chLevelPixel.displayLabel,  ">" );
	strcpy( ptf->avgLevelPixel.displayLabel, "-" );

	for (int i = 0; i < MAX_USER_TEMPS; i++) {
		users[i].label = ""; // Drawing text is expensive
	}
}

void resetFrameCounter() {
	controls.frameCounter = 0;
	controls.startMills   = currentTimeMillis();
}

void resetDefaults() {

	threadData.configurationChanged++;

	rulerThickness = MIN_RULER_THICKNESS;
	rulerBoundFlag = BOUND_PROPORTIONAL;

	resetFrameCounter();

	controls.hud          = HUD_ON;

	controls.inters       = 2; // INTER_CUBIC
	controls.useCelsius   = Use_Celsius = USE_CELSIUS;
	Use_Histogram = 0;
	controls.labelCF      = controls.useCelsius ? " C" : " F";
	controls.alpha        = 1.0;
	controls.rad          = 0; // Blur radius (0=no blur)
	controls.threshold.celsius = 2;
	controls.cmapCurrent  = DEFAULT_COLORMAP_INDEX;
	strcpy(controls.snaptime, "None");
	cmapAutoRange         = CMAP_AUTO_RANGE_DEFAULT;
	unlockColormapRange(0);

}

static int ColorScaleWidth = 12;
static int MarkerSize      = 12;

static int HudWidth	   = 166;   // 166
static int HudHeight  	   = 134;   // 134

static int HelpWidth       = TC_WIDTH;
static int HelpHeight      = TC_HEIGHT; 

#define MAX_HELP_TEXT_ROWS (19 + 1)  // Was (24 + 1)
#define MAX_HUD_TEXT_ROWS  ( 9 + 1)

//const char * LONGEST_HUD_STRING  = " Map: Twighlight Shift+Hist ";
const char * LONGEST_HUD_STRING  = "Map: Twighlight Shift+Hist";
const char * LONGEST_HELP_STRING = "L mb: Add temps, mv rulers ";

#define MAX_SCALE_FOR_FONT 5.0

static int bl_blah;
static Size longestHUDSize  = getTextSize(LONGEST_HUD_STRING,  Default_Font, MAX_SCALE_FOR_FONT, 1, &bl_blah);
static Size longestHelpSize = getTextSize(LONGEST_HELP_STRING, Default_Font, MAX_SCALE_FOR_FONT, 1, &bl_blah);
static Size hudSpaceSize    = getTextSize(" ",   Default_Font, MAX_SCALE_FOR_FONT, 1, &bl_blah);
static Size hudW_WSize      = getTextSize("G 6", Default_Font, MAX_SCALE_FOR_FONT, 1, &bl_blah);

static int  chTextHeight    = getTextSize("CF",  Default_Font, MAX_SCALE_STEPS, 2, &bl_blah).height;


void setScaleHudHelp() {
	int  baseline = 0;

	Size ts = getTextSize(LONGEST_HUD_STRING, Default_Font, HudFontScale, 1, &baseline);

	HudWidth   =  ts.width;
	HudHeight  = (ts.height + baseline) * MAX_HUD_TEXT_ROWS;

	ts = getTextSize(LONGEST_HELP_STRING, Default_Font, HudFontScale, 1, &baseline);

	HelpWidth  =  ts.width;
	HelpHeight = (ts.height + baseline - 1) * MAX_HELP_TEXT_ROWS;

	hudSpaceSize = getTextSize(" ",   Default_Font, HudFontScale, 1, &bl_blah);
	hudW_WSize   = getTextSize("G 6", Default_Font, HudFontScale, 1, &bl_blah);

	chTextHeight = getTextSize("CF", Default_Font, MyFontScale, 2, &baseline).height;
}

void setScaleControls() {

	setWindowFormat();

	// Scale Colormap Scale Widget width
	ColorScaleWidth = 3 + MyScale;  // 4 to N

	// Help needs to be redrawn based on font change
	controls.lastHelpScale = -1; // trigger Help to be redrawn
	switch ( UserFont ) {
		case 1:  Default_Font = cv::FONT_HERSHEY_DUPLEX;   break;
		case 2:  Default_Font = cv::FONT_HERSHEY_TRIPLEX;  break;
		case 0:  
		default:
			 Default_Font = cv::FONT_HERSHEY_SIMPLEX;  break;
	}

// 0.20 produces "distorted"       looking text @ 320 x 240
// 0.30 produces "barely marginal" looking text @ 320 x 240
// 0.36 produces "better"          looking font @ 640 x 480

#define MIN_SCALE       0.30
#define MAX_SCALE	0.60
#define DELTA_SCALE	(MAX_SCALE - MIN_SCALE) // Ramp from MIN_SCALE to MAX_SCALE
#define MAX_HUD_SCALE	(MIN_SCALE + ( DELTA_SCALE * 3.0  / 4.0 ))

	float oneTo5 = ( 5 < MyScale ) ? 5 :
		       ( MyScale < 1 ) ? 1 : MyScale;

	MarkerSize = 6 + (2 * (oneTo5 - 1));  // [6 - 14] - Consistent with MyFontScale growth

	// Linearaly range from MIN_SCALE to MAX_SCALE across scale values [1 through 5]
	MyFontScale = MIN_SCALE + ( DELTA_SCALE * (oneTo5 - 1.0) / 4.0 );

	// Allow other fonts to grow larger than HUD fonts and then stop growing
	if ( MAX_SCALE < MyFontScale ) {
		MyFontScale = MAX_SCALE;
	}

	HudFontScale = MyFontScale;

	// Stop growing HUD fonts earlier than other fonts
	if ( MAX_HUD_SCALE < HudFontScale ) {
		HudFontScale = MAX_HUD_SCALE;
	}

	setScaleHudHelp();

	SCALED_TC_WIDTH  = MyScale * TC_WIDTH;
	SCALED_TC_HEIGHT = MyScale * TC_HEIGHT;
}

void setDefaults(ProcessedThermalFrame *ptf) {
	clearPtf(ptf);

	// Don't reset recording, scaling, fullscreen controls
	resetDefaults();

	strcpy(controls.elapsed, "00:00:00");
	controls.recFrameCounter = 0;
	ptf->rColor              = &WHITE;
	controls.recording       = 0;
	controls.fullscreen      = 0;
	controls.lastHelpScale   = -1; // trigger Help to be redrawn
	controls.windowFormat    = WINDOW_IMAGE;
	controls.labelWF         = WFs[ controls.windowFormat ].name;
	MyScale                  = TC_DEF_SCALE;

	MyHalfScale = MyScale / 2;

	setScaleControls();

	setTempDefaults( ptf );

	reScale( ptf, 0, 0 );
}


// Calculate once/frame, use nX/frame
static int   drawMin;
static int   drawMax;
static float minF;
static float maxF;
static float avgF;
static float rulerBoundMaxKelvin;
static float rulerBoundMinKelvin;
static float thresholdF;
static float minusThresholdKelvin;
static float plusThresholdKelvin;
static float maxAvgOffsetF;
static float minAvgOffsetF;
static float anchorAvgOffsetF;
static Point hMin_src, hMin_dst;
static Point hMax_src, hMax_dst;
static Point vMin_src, vMin_dst;
static Point vMax_src, vMax_dst;
static Point hBase_src, hBase_dst;
static Point hBase_b,   hBase_c;
static Point vBase_src, vBase_dst;
static Point vBase_b,   vBase_c;
static	Point vPerp, hPerp;
static Scalar *vScalar;
static Scalar *hScalar;
static int copAnchorX;
static int copAnchorX_Offset;
static int copAnchorY;
static int copAnchorY_Offset;
static Point copAnchorXYPoint;
static int copZero;	// Both X and Y
static int copWidth;
static int copHeight;
static float arrowScaleV;
static float arrowScaleH;

float getCelsius(Mat *thermalFrame, int x, int y) {
	unsigned short *usStartPtr = &((unsigned short *)(thermalFrame->datastart))[0];

	ASSERT(( x < TC_WIDTH));
	ASSERT(( y < TC_HEIGHT));

	int linearI = (y * TC_WIDTH) + x;

	return kelvin2Celsius( usStartPtr[ linearI ] );
}

// Function removes double evaluation of a and b expression over #define min/max
float floatMin(float a, float b) { return (a < b) ? a : b; }
//float floatMax(float a, float b) { return (a < b) ? b : a; }

#define KELVIN_RULER_MIN(val)   floatMin( (val),  rulerBoundMinKelvin )
#define KELVIN_RULER_MAX(val)   floatMin( (val),  rulerBoundMaxKelvin )

float getRulerKelvinFactor( float heightWidth ) {

	// Base ruler thickness range on [un]bounded/[un]clipped Min/Max deltas from average temp
	// This produces an even or proprortional ruler plot with or without outlier clipping

	float minMaxRange = ( rulerBoundMaxKelvin + rulerBoundMinKelvin );

	if ( 0.0 == minMaxRange ) minMaxRange = 1.0; // divide by zero

	return( heightWidth / (float)rulerThickness / minMaxRange ); 
}

// Optimization: Converted to use native kelvin to eliminate numerous C/F conversion math
void getRulerXPoints( Mat *thermalFrame, float *kelvin, Point *points0, Point *points1, int x, int y, int xMax, int anchorY) {
	ProcessedThermalFrame *ptf = threadData.ptf;

	float avgKelvin    = ptf->avg.kelvin;
	// Adjust for: 
	//	PORTRAIT / LANDSCAPE (only if Full ruler thickness)
	//	Full or Fractional ruler thickness
	//		Fractional ruler thickness is always based on FIXED_TC_HEIGHT

	if ( y >= TC_HEIGHT ) {
		y = TC_HEIGHT - y;
	}

	ASSERT(( y < TC_HEIGHT));

	// Start with Row Y offset
	unsigned short *usRowPtr = &((unsigned short *)(thermalFrame->datastart))[ y * TC_WIDTH ];

	float kTmp;
	int   tmpY;
	for ( ; x < xMax; x++ ) {
		ASSERT(( x < TC_WIDTH));

		kelvin[ x ] = usRowPtr[ x ];

		tmpY = ( kelvin[ x ] < avgKelvin ) ?
	       		round(anchorY + (rulerXKelvinFactor * (KELVIN_RULER_MIN(avgKelvin - kelvin[x])))) :
	       		round(anchorY - (rulerXKelvinFactor * (KELVIN_RULER_MAX(kelvin[x] - avgKelvin))));

		POINT( points0[x], centerOfPixelNODIV( x ),
		                   centerOfPixelNODIV( tmpY ) );

		// WINDOW_DOUBLE point locations used with LINE3()
		POINT( points1[x], points0[x].x + sX,
				   points0[x].y + sY );

		if ( 0 < x ) {
			// (float) cast improves color averaging transitions
			kTmp = ( (float)(kelvin[ x - 1 ] + kelvin[ x ]) / 2.0 );
			scalarX[ x - 1 ] = ( (kTmp < minusThresholdKelvin) ? &RULER_MIN_COLOR :
					     (kTmp > plusThresholdKelvin)  ? &RULER_MAX_COLOR :
						                             &RULER_MID_COLOR );
		}
	}
}

// Optimization: Converted to use native kelvin to eliminate numerous C/F conversion math
void getRulerYPoints( Mat *thermalFrame, float *kelvin, Point *points0, Point *points1, int x, int y, int yMax, int anchorX ) {
	ProcessedThermalFrame *ptf = threadData.ptf;

	float avgKelvin    = ptf->avg.kelvin;
	// Adjust for: 
	//	PORTRAIT / LANDSCAPE (only if Full ruler thickness)
	//	Full or Fractional ruler thickness
	//		Fractional ruler thickness is always based on FIXED_TC_HEIGHT

	if ( x >= TC_WIDTH ) {
		x = TC_WIDTH - x;
	}

	ASSERT(( x < TC_WIDTH));

	// Start with Column X offset
	unsigned short *usColPtr = &((unsigned short *)(thermalFrame->datastart))[ x ];

	float kTmp;
	int   tmpX;
	for ( ; y < yMax; y++ ) {
		ASSERT(( y < TC_HEIGHT));

		kelvin[ y ] = ( usColPtr[ (y * TC_WIDTH) ] );

		tmpX = ( kelvin[ y ] < avgKelvin ) ?
			round(anchorX - (rulerYKelvinFactor * KELVIN_RULER_MIN(avgKelvin - kelvin[y]))) :
			round(anchorX + (rulerYKelvinFactor * KELVIN_RULER_MAX(kelvin[y] - avgKelvin)));

		POINT( points0[y], centerOfPixelNODIV( tmpX ),
				   centerOfPixelNODIV( y ) );

		// WINDOW_DOUBLE point locations used with LINE3()
		POINT( points1[y], points0[y].x + sX,
				   points0[y].y + sY );

		if ( 0 < y ) {
			// (float) cast improves color averaging transitions
			kTmp = ( (float)(kelvin[ y - 1 ] + kelvin[ y ]) / 2.0 );
			scalarY[ y - 1 ] = ( (kTmp < minusThresholdKelvin) ? &RULER_MIN_COLOR :
					     (kTmp > plusThresholdKelvin)  ? &RULER_MAX_COLOR :
						                             &RULER_MID_COLOR );
		}
	}
}

void getTemperature(Mat *thermalFrame, Temperature &temp) {
	unsigned short *usStartPtr = &((unsigned short *)(thermalFrame->datastart))[0];

	int x = temp.col;
	int y = temp.row;

	ASSERT(( x < TC_WIDTH));
	ASSERT(( y < TC_HEIGHT));

	temp.linearI = (y * TC_WIDTH) + x;
	temp.kelvin  = usStartPtr[ temp.linearI ];
	temp.celsius = kelvin2Celsius( temp.kelvin );
}

void scaleTemp(Temperature &temp, const char *labelCF) {

	temp.xScaled = centerOfPixelNODIV( temp.col );
	temp.yScaled = centerOfPixelNODIV( temp.row );
	
	if ( ACTIVE_ON == temp.active ) { // kluge for colormap gradiant bar indicator
		sprintf(temp.displayLabel, "%s%.1f%s", temp.label, CorF(temp.celsius), labelCF );
	}
}


// OPTIMIZATION: Precalc temp display locations prior to rendering to speed up rendering
void calcTempDisplayLocations( Temperature &temp ) {
	int x = temp.xScaled; // Prescaled Y
	int y = temp.yScaled; // Prescaled X
//printf("%d, %d\n", x, y );

	int baseline = 0;
	Size textSize = getTextSize(temp.displayLabel, Default_Font, MyFontScale, 2, &baseline);

	int xOffset = MarkerSize; // x space between marker and text

	int yOffset = textSize.height / 2; // Center text height on marker

	if ( y < textSize.height) {
		yOffset = textSize.height + 2;
	}
	else if ( y + textSize.height > controls.scaledSFHeight )
		yOffset = (- (textSize.height / 2));

	if ( x + xOffset + textSize.width > controls.scaledSFWidth ) {
		if (temp.type == TEMP_TYPE_CMAP_SCALE) {
			xOffset = (- xOffset - textSize.width);
		} else {
			int a = controls.scaledSFWidth - x;
			xOffset = - (textSize.width - a);
			if ((y + yOffset) <= controls.scaledSFHeight/2) 
				yOffset = (+ (2*textSize.height));
			else
				yOffset = (- (1*textSize.height));
		}
	}

	// 2nd frame has no HUD display
	POINT( temp.markerwDLoc, x + sX,           y + sY );
	POINT( temp.labelwDLoc,  x + sX + xOffset, y + sY + yOffset );

	if        ( controls.hud == HUD_HELP ) { // Avoid writing temps over HUD text
		if ( x < HelpWidth and y < HelpHeight )
			xOffset = xOffset + (HelpWidth - x);
	} else if ( controls.hud == HUD_ON ) { // Avoid writing temps over HUD text
		if ( x < HudWidth and y < HudHeight )
			xOffset = xOffset + (HudWidth - x);
	}

	// 1st frame may have HUD display 
	// Reuse point
	POINT( temp.markerLoc, x,           y );
	POINT( temp.labelLoc,  x + xOffset, y + yOffset );
}

// Process every 2-byte temperature pixel in the thermal frame
// Data mine Min/Avg/Max and center of frame temperatures and their
//     respective x=col, y=row locations (Avg has no location)
// Temperatures are stored in 16-bit Kelvin in row major order
// This has to be done as efficiently as possible on every thermal frame
// Can't just look at the low bytes else temps and locations will be wrong.

void processThermalFrame( ProcessedThermalFrame *ptf, Mat *thermalFrame ) {

//dumpFrameInfo(frame);

	/*******************************************************************
	 * In FreezeFrame and Offline mode, the following temps change:
	 * - User Temps / Ruler Temps
	 * - Colormap Gradiant Scale Crosshair temp
	 *
	 * The following temps don't change unless configurations change:
	 * - Min/Avg/Max
	 * - Colormap Gradiante Scale Avg temp
	 * - Colormap Gradiante Scale Min/Max and scale tick temps 
	 *
	 * Only re-harvest these temps if configs change such as:
	 * - Scale, Rotation, FullScreen, etc.
	*******************************************************************/
	// Temps that change in FreezeFrame
	// 	chLevelPixel
	// 	users[]
	// 	current 

	const char *labelCF = controls.labelCF;

	// Calculate CENTER_OF_RULER_INDEX in processThermalFrame()
	// Used later to calculate chCelsius
	if ( user_CENTER_OF_RULER_INDEX->active ) {
		getTemperature( thermalFrame, *user_CENTER_OF_RULER_INDEX);
		scaleTemp( *user_CENTER_OF_RULER_INDEX, labelCF );
		calcTempDisplayLocations( *user_CENTER_OF_RULER_INDEX );
	}

	// Parse bottom frame to harvest heat data
	// Start of thermal frame data
        unsigned short *usStartPtr = &((unsigned short *)(thermalFrame->datastart))[0];

	long kminPixel, lminPixel;
	long kmaxPixel, lmaxPixel;

	if ( RERENDER_OPTIMIZATION ) {

		kminPixel = ptf->minPixel.kelvin = LONG_MAX;
		kmaxPixel = ptf->maxPixel.kelvin = LONG_MIN;

		lminPixel = 0;
		lmaxPixel = 0;

		// Extract Center-of-ThermalFrame Crosshair Kelvin temp
		ptf->ch.linearI = (TC_HALF_HEIGHT * TC_WIDTH) + (TC_HALF_WIDTH);
		ptf->ch.kelvin  = usStartPtr[ ptf->ch.linearI ];
		ptf->ch.celsius = kelvin2Celsius( ptf->ch.kelvin );
		divmod( ptf->ch.linearI,  TC_WIDTH, &ptf->ch.row,  &ptf->ch.col );
		scaleTemp( ptf->ch,  labelCF );
		// Crosshair Temp does not have a display name
		sprintf( ptf->ch.displayLabel, "%.1f%s", CorF(ptf->ch.celsius), labelCF );
		calcTempDisplayLocations( ptf->ch );

		long lmin = 0;
		long lmax = 0;
		long kmin = ptf->min.kelvin = LONG_MAX;
		long kmax = ptf->max.kelvin = LONG_MIN;

		// Use image frame, thermal frame or both for RGB rendering, snapshot and recording
		//
		// Traverse (TC_WIDTH * TC_HEIGHT) 2-byte-per-pixel in row major order
		// Optimized loop with minimal indexes, variables and structure dives
		unsigned short *usKelvinPtr   =  usStartPtr;
		unsigned short *usMaxPtr      = &usStartPtr[TC_WIDTH * TC_HEIGHT];
       		//unsigned short *usImgPtr      = &((unsigned short *)(imageFrame.datastart))[0];
		// Find end of 2nd thermal frame
		long            ktotal        = 0; // Calculate average temps in thermal frame
		// ************* BEGIN LOOP UNROLL ***************************
		for ( ; (usKelvinPtr < usMaxPtr); usKelvinPtr += 8 /*, usImgPtr += 8 */) {
			// Linear Parsing
			// long kelvin = usKelvinPtr[0] + (usKelvinPtr[1] << 8); // LSByte + MSByte

			// perf: Most CPU intensive statement in loop
			// Calculate Average Temp
			ktotal += *  usKelvinPtr +
			       	  * (usKelvinPtr + 1) +
			       	  * (usKelvinPtr + 2) +
			       	  * (usKelvinPtr + 3) +
			       	  * (usKelvinPtr + 4) +
			       	  * (usKelvinPtr + 5) +
			       	  * (usKelvinPtr + 6) +
			       	  * (usKelvinPtr + 7); 
		
#define UNROLL_MIN(k,l,n)	if (k > *(usKelvinPtr + n)) { \
					k = *(usKelvinPtr + n); \
					l =  (usKelvinPtr + n) - usStartPtr; \
				}
			UNROLL_MIN(kmin, lmin, 0)
			UNROLL_MIN(kmin, lmin, 1)
			UNROLL_MIN(kmin, lmin, 2)
			UNROLL_MIN(kmin, lmin, 3)
			UNROLL_MIN(kmin, lmin, 4)
			UNROLL_MIN(kmin, lmin, 5)
			UNROLL_MIN(kmin, lmin, 6)
			UNROLL_MIN(kmin, lmin, 7)

#define UNROLL_MAX(k,l,n)	if (k < *(usKelvinPtr + n)) { \
					k = *(usKelvinPtr + n); \
					l =  (usKelvinPtr + n) - usStartPtr; \
				}
			UNROLL_MAX(kmax, lmax, 0)
			UNROLL_MAX(kmax, lmax, 1)
			UNROLL_MAX(kmax, lmax, 2)
			UNROLL_MAX(kmax, lmax, 3)
			UNROLL_MAX(kmax, lmax, 4)
			UNROLL_MAX(kmax, lmax, 5)
			UNROLL_MAX(kmax, lmax, 6)
			UNROLL_MAX(kmax, lmax, 7)
		}
		// ************* END LOOP UNROLL ***************************

		ptf->min.kelvin  = kmin;
		ptf->max.kelvin  = kmax;
		ptf->min.linearI = lmin;
		ptf->max.linearI = lmax;

		// Controls to [un]lock colormap auto-ranging
		ptf->minPixel.kelvin   = kminPixel; // kelvin is NOT kelvin here
		ptf->maxPixel.kelvin   = kmaxPixel; // kelvin is NOT kelvin here

		ptf->minPixel.linearI  = lminPixel;
		ptf->maxPixel.linearI  = lmaxPixel;

		// Set y=row, x=col locations
		// avg (displayed in HUD) has no location information
		divmod( ptf->min.linearI, TC_WIDTH, &ptf->min.row, &ptf->min.col );
		divmod( ptf->max.linearI, TC_WIDTH, &ptf->max.row, &ptf->max.col );

		long kAverage = ktotal / TC_MAX_TEMPS;

		if ( ptf->avg.kelvin != kAverage ) {
			threadData.configurationChanged++; // Ping HUD to update
			ptf->avg.kelvin = kAverage;
		}

		// Calculate Celsius from Kelvin
		ptf->min.celsius = kelvin2Celsius( ptf->min.kelvin );
		ptf->avg.celsius = kelvin2Celsius( ptf->avg.kelvin );
		ptf->max.celsius = kelvin2Celsius( ptf->max.kelvin );

		// linearI & kelvin does not make sense for Colormap gradient scale widet temps 
		// because they are derrived percentages

		// Display maxPixel at NE corner and minPixel at SE corner for cmapScale temps

		// TODO - FIXME - Make scale slightly shorter so Min/Max temps are not pushed inwards
		// Colormap scale has highest temps on top and lowest temps on bottom
		float heightDelta  = (TC_HEIGHT - 1);
		ptf->maxPixel.row  = heightDelta * 0.0 / 5.0;
		ptf->avg4Pixel.row = heightDelta * 1.0 / 5.0;
		ptf->avg3Pixel.row = heightDelta * 2.0 / 5.0;
		ptf->avg2Pixel.row = heightDelta * 3.0 / 5.0;
		ptf->avg1Pixel.row = heightDelta * 4.0 / 5.0;
		ptf->minPixel.row  = heightDelta * 5.0 / 5.0;

		// Colormap scale temps are vertical column aligned
		float widthDelta   = (TC_WIDTH - 1);
		ptf->maxPixel.col  = widthDelta;
		ptf->avg4Pixel.col = widthDelta;
		ptf->avg3Pixel.col = widthDelta;
		ptf->avg2Pixel.col = widthDelta;
		ptf->avg1Pixel.col = widthDelta;
		ptf->minPixel.col  = widthDelta;

		// Celsius has to be set before calling scaleTemp()
		scaleTemp( ptf->min, labelCF );
		scaleTemp( ptf->max, labelCF );

		calcTempDisplayLocations( ptf->min );
		calcTempDisplayLocations( ptf->max );

	} else {
		// Min/Avg/Max/CH and static Colormap Gradient Temps do NOT change in Offline or FreezeFrame 

		// Controls to [un]lock colormap auto-ranging
		kminPixel = ptf->minPixel.kelvin; // kelvin is NOT kelvin here
		kmaxPixel = ptf->maxPixel.kelvin; // kelvin is NOT kelvin here

		lminPixel = ptf->minPixel.linearI;
		lmaxPixel = ptf->maxPixel.linearI;
	}

	// Colormap autoranging requires camera hardware control which we don't have
	float minPixelCelsius = ptf->minPixel.celsius = ptf->min.celsius;
		                ptf->maxPixel.celsius = ptf->max.celsius;

	float minMaxRange     = (ptf->maxPixel.celsius - minPixelCelsius);

  	unsigned short *usImgPtr = &((unsigned short *)(imageFrame.datastart))[0];
	minPixel = usImgPtr[ ptf->min.linearI ];
	maxPixel = usImgPtr[ ptf->max.linearI ];

	// Track either crosshair temp or ruler crosshair temp on the colormap scale
	float chCelsius   = (rulersOn ? user_CENTER_OF_RULER_INDEX->celsius : ptf->ch.celsius);
	float chRange     = (chCelsius - minPixelCelsius);
	float chFraction  = (chRange / minMaxRange);

	float minMaxRangeKelvin   = (ptf->max.kelvin - ptf->min.kelvin);
	ptf->chLevelPixel.kelvin  = ptf->min.kelvin + (minMaxRangeKelvin * chFraction);
	ptf->chLevelPixel.linearI = (lminPixel + lmaxPixel) * chFraction;
	ptf->chLevelPixel.celsius = minPixelCelsius + chRange;
	ptf->chLevelPixel.row     = TC_HEIGHT * (1.0 - chFraction);
	ptf->chLevelPixel.col     = TC_WIDTH-1;

	// Leave off C/F for cmapScale display
	scaleTemp( ptf->chLevelPixel,  "" ); // labelCF );
	calcTempDisplayLocations( ptf->chLevelPixel );

	if ( RERENDER_OPTIMIZATION ) {

		ptf->avg1Pixel.celsius = (minPixelCelsius + (minMaxRange * 1.0 / 5.0));
		ptf->avg2Pixel.celsius = (minPixelCelsius + (minMaxRange * 2.0 / 5.0));
		ptf->avg3Pixel.celsius = (minPixelCelsius + (minMaxRange * 3.0 / 5.0));
		ptf->avg4Pixel.celsius = (minPixelCelsius + (minMaxRange * 4.0 / 5.0));

		// Colormap thermal gradiant scale temps
		float avgCelsius  = ptf->avg.celsius;
		float avgRange    = (avgCelsius - minPixelCelsius);
		float avgFraction = (avgRange / minMaxRange);

		ptf->avgLevelPixel.kelvin  = (kminPixel + kmaxPixel) * avgFraction;
		ptf->avgLevelPixel.linearI = (lminPixel + lmaxPixel) * avgFraction;
		ptf->avgLevelPixel.celsius = minPixelCelsius + avgRange;
		ptf->avgLevelPixel.row     = TC_HEIGHT * (1.0 - avgFraction);
		ptf->avgLevelPixel.col     = TC_WIDTH-1;

		// Leave off C/F for cmapScale display
		scaleTemp( ptf->minPixel,      "" ); // labelCF );
		scaleTemp( ptf->avgLevelPixel, "" ); // labelCF );
		scaleTemp( ptf->maxPixel,      "" ); // labelCF );
		scaleTemp( ptf->avg1Pixel,     "" ); // labelCF );
		scaleTemp( ptf->avg2Pixel,     "" ); // labelCF );
		scaleTemp( ptf->avg3Pixel,     "" ); // labelCF );
		scaleTemp( ptf->avg4Pixel,     "" ); // labelCF );

		calcTempDisplayLocations( ptf->minPixel );
		calcTempDisplayLocations( ptf->avgLevelPixel );
		calcTempDisplayLocations( ptf->maxPixel );
		calcTempDisplayLocations( ptf->avg1Pixel );
		calcTempDisplayLocations( ptf->avg2Pixel );
		calcTempDisplayLocations( ptf->avg3Pixel );
		calcTempDisplayLocations( ptf->avg4Pixel );
	}

	ASSERT(( ptf->min.celsius <= ptf->ch.celsius ));
	ASSERT(( ptf->min.celsius <= ptf->avg.celsius ));
	ASSERT(( ptf->avg.celsius <= ptf->max.celsius ));
	ASSERT(( ptf->ch.celsius  <= ptf->max.celsius ));
}

void reColormap(int value) {

	threadData.configurationChanged++;

	controls.cmapCurrent = (controls.cmapCurrent + value) % MAX_CMAPS;
	if (controls.cmapCurrent < 0) {
		controls.cmapCurrent = MAX_CMAPS - 1;
	}
}

#define MAX_ALPHA (10.0)
void reAlpha(double value) {

	threadData.configurationChanged++;

	controls.alpha += value;

	// controls.alpha = round(controls.alpha,1) // fix round error
	if (controls.alpha > MAX_ALPHA) {
		controls.alpha = MAX_ALPHA;
	}
	else if (controls.alpha < 0.0) { // Should lower limit be 1.0 or 0.0 ???
		controls.alpha = 0.0;
	}
}
       
extern void recording( ProcessedThermalFrame *ptf, int stop );

void resizeWindow( ProcessedThermalFrame *ptf ) {
	if ( controls.recording ) { recording(ptf, 1); } // Stop active recording

	threadData.configurationChanged++;

	resizeWindow( WINDOW_NAME, controls.sW, controls.sH );
}

void newWindow( ProcessedThermalFrame *ptf ) {
	// Add window manager's resize border with WINDOW_GUI_EXPANDED
	if ( controls.fullscreen ) {
		// This allows fullscreen
		namedWindow( WINDOW_NAME, WINDOW_GUI_EXPANDED );
		resizeWindow( ptf );
		setWindowProperty( WINDOW_NAME, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN );
	}
	else {
		// This allows resizing larger and smaller
		namedWindow( WINDOW_NAME, WINDOW_AUTOSIZE | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED );
	}

	reScale(ptf, 0, 1);
}

void toggleFullScreen( ProcessedThermalFrame *ptf ) {

	if ( controls.recording ) { recording(ptf, 1); } // Stop active recording

	threadData.configurationChanged++;

	controls.fullscreen = !controls.fullscreen;

	destroyWindow( WINDOW_NAME ); // Destroy current window and recreate resizable or fullscreen
	newWindow( ptf );

	// Callback is lost on destroy window, thus need to add again
	setMouseCallback( WINDOW_NAME, onMouseCallback, &threadData );
}

// Multi-threaded HUD stacking (and flickers) differ based on number of horizontal entries
// Re-order temp pointers in horizontal ruler to prevent HUD stacking flicker 
// based on 3, 5 or 7 entries per ruler.
void horizFlickerTomFoolery() {
	// Horizontal Tom foolery
	if ( MyScale < 3 )  {
		// 3 and 5 entries per ruler
		user_0  = &users[1];
		user_1  = &users[3];
		user_2  = &users[5];
		user_3  = &users[2];
		// user_4 not changed
		user_5  = &users[0];
		// user_6-N not changed
	} else {
		// 7 entries per ruler
		user_0  = &users[0];
		user_1  = &users[1];
		user_2  = &users[2];
		user_3  = &users[3];
		// user_4 not changed
		user_5  = &users[5];
		// user_6-N not changed
	}
}

void reScale(ProcessedThermalFrame *ptf, int value, bool resize) {
 
	if ( controls.recording ) { recording(ptf, 1); } // Stop active recording

	threadData.configurationChanged++;

	if ((value == -1) && controls.fullscreen ) {
		toggleFullScreen( ptf );  // Switch from fullscreen to resizeable window 
		return; // Only decrement in resizeable window
	}

	MyScale += value;

	if ( MyScale > MAX_SCALE_STEPS ) {
		MyScale = MAX_SCALE_STEPS;
		setScaleControls();

		if ( ! controls.fullscreen ) {
			toggleFullScreen( ptf );  // maxed out resizeable window, switch to fullscreen
		}

		MyHalfScale = MyScale / 2;
		horizFlickerTomFoolery();

		return; // Only increment in resizeable window
	}
	else if ( MyScale < 1 ) {
		MyScale = 1;
	}

	MyHalfScale = MyScale / 2;
	horizFlickerTomFoolery();

	setScaleControls();

	if ( 0 != value && controls.fullscreen) {
        // If attempthing to scale a FULLSCREEN, switch to resizeable window first
		toggleFullScreen( ptf );
		return;
	}

	if (resize && ! controls.fullscreen ) { // isPi == False:
		resizeWindow( ptf );
	}

	if ( rulersOn ) {
		// Rescale ruler offsets
		rulers(ptf, rulersX, rulersY, 0);
	}
}

void reCF() {
	controls.useCelsius = Use_Celsius = !controls.useCelsius;
	controls.labelCF = controls.useCelsius ? " C" : " F";
}

void reThreshold(int value) {

	threadData.configurationChanged++;

	controls.threshold.celsius += value;
	if ( controls.threshold.celsius < 0 )
		controls.threshold.celsius = 0;
}

void reRad(int value) {

	threadData.configurationChanged++;

	controls.rad += value;
	if ( controls.rad < 0 )
		controls.rad = 0;
}

void reInterp(int value) {

	threadData.configurationChanged++;

	controls.inters += value;
	if (controls.inters >= MAX_INTERS)
		controls.inters = 0;
	else if (controls.inters < 0)
		controls.inters = MAX_INTERS-1;
}

void windowFormat( ProcessedThermalFrame *ptf, int value ) {
	// TODO - FIXME - Save/Restore scale based on windowFormat to stay within DISPLAY_WIDTH, DISPLAY_HEIGHT

	if ( controls.recording ) { recording(ptf, 1); } // Stop active recording

	threadData.configurationChanged++;

	controls.windowFormat += value;

	if (controls.windowFormat >= MAX_WFS)
		controls.windowFormat = 0;
	else if (controls.windowFormat < 0)
		controls.windowFormat = MAX_WFS-1;

	controls.labelWF = WFs[controls.windowFormat].name;

	setWindowFormat();
	resizeWindow( ptf ); // SINGLE, DOUBLE_HIGH, DOUBLE_WIDE
}

void hud( int value ) {

	threadData.configurationChanged++;

	controls.hud = (HUDFormat)(controls.hud + value);

	if (controls.hud > (HUD_MAX_MOD-1))
		controls.hud = (HUDFormat)0;
	else if (controls.hud < 0) 
		controls.hud = (HUDFormat)(HUD_MAX_MOD-1);
}

VideoWriter rec() {
	time_t rawtime;
	struct tm * timeinfo;
	char now [128];
	char filename[256];

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	strftime (now, sizeof(now), "%Y%m%d-%H%M%S", timeinfo);

	sprintf(filename,       "%s_output.avi", now);

	sprintf(rawRecFilename, "%s_output.raw", now);

	printf("now(%s), filename(%s)\n", now,filename);

	// VideoWriter (const String &filename, int fourcc, double fps, Size frameSize, bool isColor=true)
	// static int cv::VideoWriter::fourcc(char c1, char c2, char c3, char c4)
	// int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // select desired codec (must be available at runtime)

	// https://docs.opencv.org/4.5.1/df/d94/samples_2cpp_2videowriter_basic_8cpp-example.html#a9
	VideoWriter videoOut(filename, VideoWriter::fourcc('X', 'V', 'I','D'), NATIVE_FPS, Size(controls.sW, controls.sH));

	return videoOut;
}

void recording( ProcessedThermalFrame *ptf, int forcedStop ) {

	try {
		controls.recording = (forcedStop ? 0 : ( ! controls.recording ));

		if ( controls.recording ) {
			controls.recFrameCounter = 0;
			ptf->rColor              = &RED;

			// Protect videoOut and recordingActive
			pthread_mutex_lock( &videoOutMutex );
				ptf->videoOut            = rec();
				controls.recordingActive = true;
			pthread_mutex_unlock( &videoOutMutex );
			controls.recordStartTime = time(NULL);
		} else {
			// Protect videoOut and recordingActive
			pthread_mutex_lock( &videoOutMutex );
				// Could have multiple stops because records don't work 
				// across window resizes, scales, rotations, layout changes
				if ( controls.recordingActive ) {
					controls.recordingActive = false;
					controls.recFrameCounter = 0;
					ptf->rColor              = &WHITE;

					strcpy(controls.elapsed,"00:00:00");
					ptf->videoOut.release();
					ptf->videoOut.~VideoWriter();

					if ( rawRecFp ) {
						fclose( rawRecFp );
					}	rawRecFp = 0x00;
				}
			pthread_mutex_unlock( &videoOutMutex );
		}
	} catch (...) {
	}

}

// Adding offline post raw still and raw video processing functionality

void writeRawFrame(Mat &frame, FILE *fp) {
// TODO - FIXME - HANDLE ROTATED FRAME on READ/WRITE of raw frame
// User convenience so reload doesn't require same manual rotations.
// Change head to include rotation value or change read/write code to handle flipped row/columns
// if row == 192, read and set rotate PORTRAIT / LANDSCAPE

	unsigned short  rows = frame.rows;
	unsigned short  cols = frame.cols;
	unsigned short  type = frame.type();
	unsigned short  chan = frame.channels();
        unsigned short *data = &((unsigned short *)(frame.datastart))[0];

	// Make sure this is NOT a scaled/composited frame
	ASSERT(( (FIXED_TC_HEIGHT * 2) == rows )); // 2 frames
	ASSERT((  FIXED_TC_WIDTH       == cols ));
	ASSERT((                     2 == chan ));

	// Write header
	fwrite( &rows,    sizeof(unsigned short), 1, fp );
	fwrite( &cols,    sizeof(unsigned short), 1, fp );
	fwrite( &type,    sizeof(unsigned short), 1, fp );
	fwrite( &chan,    sizeof(unsigned short), 1, fp );

	// Write row/column data
	fwrite( &data[0], sizeof(unsigned short), (rows*cols), fp );
}

FILE * writeRawFrame(Mat &frame, const char *filename, FILE *fp, int keepOpen ) {

	if ( 0 == fp ) {
		fp = fopen(filename, "wb");
	}

	if ( !fp ) {
		perror( filename );
		return fp;
	}

	writeRawFrame( frame, fp );

	if ( ! keepOpen ) {
		fclose( fp );
		fp = 0x00;
	}

	return fp;
}

void readRawFrame(Mat &frame, FILE *fp) {
	unsigned short rows, cols, type, chan, *data;

	// Read header
	fread( &rows,    sizeof(unsigned short), 1, fp );
	fread( &cols,    sizeof(unsigned short), 1, fp );
	fread( &type,    sizeof(unsigned short), 1, fp );
	fread( &chan,    sizeof(unsigned short), 1, fp );

	ASSERT(( (FIXED_TC_HEIGHT * 2) == rows )); // 2 frames
	ASSERT((  FIXED_TC_WIDTH       == cols ));
	ASSERT((                     2 == chan ));

	frame.create(rows, cols, type); // Reconfigure frame

	// Read row/column data
       	data = &((unsigned short *)(frame.datastart))[0];
	fread( &data[0], sizeof(unsigned short), (rows*cols), fp );
}

FILE * readRawFrame(Mat &frame, const char *filename, FILE *fp, int keepOpen, long *numberOfFrames) {

	if ( 0 == fp ) {
		fp = fopen(filename, "rb");
	}

	if ( ! fp ) {
		perror( filename );
		return 0x00;
	}

#define FRAME_SIZE ((4 * sizeof(unsigned short)) + (FIXED_TC_HEIGHT*2*FIXED_TC_WIDTH * sizeof(unsigned short))) 

	struct stat st;
	stat(filename, &st);

	// printf("File %s is %ld?=%ld bytes, %ld frames\n", filename, st.st_size, FRAME_SIZE, st.st_size/FRAME_SIZE);

	// Verify raw file is multiples of FRAME_SIZE
	ASSERT( ((st.st_size % FRAME_SIZE) == 0) );

	if ( (st.st_size % FRAME_SIZE) != 0 ) {
		printf("%s is invalid file size %ld, remainder %ld\n", filename, st.st_size, st.st_size % FRAME_SIZE);
		fclose( fp );
		return 0x00;
	}

	*numberOfFrames = st.st_size / FRAME_SIZE;

	readRawFrame( frame, fp );

//	printf("%ld ?= %ld\n", ftell(fp), st.st_size );

	if ( ftell(fp) == st.st_size ) { 
		// Loop single or multiple frame file
//		printf("Found end of file %s\n", filename);
		fseek( fp, 0, SEEK_SET );
	}

	if ( ! keepOpen ) {
		fclose( fp );
		fp = 0x00;
	}

	return fp;
}

void snapshot( Mat *frame ) {

	//size_t strftime (char* ptr, size_t maxsize, const char* format, const struct tm* timeptr );
	time_t rawtime;
	struct tm * timeinfo;
	char now [128];
	char filename[256];
	char rawname[256];

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	strftime (now, sizeof(now), "%Y%m%d-%H%M%S", timeinfo);
	strftime (controls.snaptime, sizeof(controls.snaptime), "%H:%M:%S", timeinfo);

	sprintf(filename, "TC001%s.png", now);
	sprintf(rawname,  "TC001%s.raw", now);

	//now(20231115-010012), filename(TC00120231115-010012.png), snaptime(01:00:12)
	printf("now(%s), filename(%s), snaptime(%s)\n", now,filename,controls.snaptime);

	imwrite(filename, *frame);
	writeRawFrame( *threadData.rawFrame, rawname, 0, 0 );
}

void rotateDisplay( ProcessedThermalFrame *ptf ) {

	if ( controls.recording ) { recording(ptf, 1); } // Stop active recording

	threadData.configurationChanged++;

	RotateDisplay = (RotateDisplay + 1) % 4;  // Increment rotation by 90 degrees
	setHeightWidth();                         // Flip TC_WIDTH and TC_HEIGHT
	setWindowFormat();                     // Change window width and height
	resizeWindow(ptf);

	// Rotate rulersX and rulersY so Up/Down/Left/Right will work across rotations
	rotateXYRight90( &rulersX, TC_WIDTH, &rulersY );

	// Rotate all active user and ruler temps
	rotateUserTemps90();
}


void printKeyBindings() {
  printf("\n");
  printf("Version: %s\n", VERSION_STR);
  printf("\n");
  printf("%s:\n\tPorted and updated C/C++ app based on Les Wright's 21 June 2023 Python app\n", WINDOW_NAME);
  printf("\tAll prior licenses apply.\n");
  printf("\t\thttps://github.com/leswright1977/PyThermalCamera - Python script\n");
  printf("\t\thttps://github.com/92es/Thermal-Camera-Redux     - Ported/Updated C/C++ app\n");
  printf("\n");
  printf("A multi-threaded C/C++ app to read, parse, display thermal data from the Topdon TC001 Thermal camera\n");
  printf("Rewritten with additional functionality, bug fixes, optimizations and offline post processing\n");
  printf("Built with display %dx%d, max:default scale %d:%d, rotation %s, default %s, %d colormaps, %s\n", 
	  DISPLAY_WIDTH, DISPLAY_HEIGHT, MAX_SCALE_STEPS, TC_DEF_SCALE, ROTATION_STR,
	  USE_CELSIUS?"Celsius":"Fahrenheit", MAX_CMAPS, cmaps[controls.cmapCurrent]->name
       	);
  printf("\n");
  printf("Tested on IvyBridge & Coffee Lake Debian 11 PCs with all features working\n");
  printf("Tested by Amish Technician on numerous RPi models including RPi Zero 2w, 2, 3, 4 and 5\n");
  printf("    using 2023-12-05 release of Raspberry Pi OS desktop 64-bit (Debian 12 bookworm)\n");
  printf("\n");
  printf( "Camera Usage: \n\t%s -d n (where 'n' is the number of the desired video camera)\n\n", Argv0 );
  printf( "Offline Usage: \n\t%s -f input.raw (where input.raw is a raw dump file from %s)\n\n", Argv0, Argv0 );
  printf( "Optional flags: [-rotate n] [-scale n] [-cmap n] [-fps n] [-font n] [-clip n] [-thick n]\n\n");
  printf("Key Bindings:\n");
  printf("\n");
  printf("a z: [In|De]crease Blur\n");
  printf("s x: +/- threshold from avg temp that contols min/max displays and ruler plot colors\n");
  printf("d c: Change interpolated window scale [camera native to fullscreen]\n");
  printf("f v: [In|De]crease Contrast\n");
  printf("g b: Cycle [for|back]wards through interpolation methods\n");
  printf("j m: Cycle [for|back]wards through Color[m]aps\n");
  printf("w  : Cycle through single/dual[horizontal/vertical] [w]indow layouts\n");
  printf("6  : Toggle between fullscreen and current scaled window size\n");
  //printf("r  : Toggle [r]ecording (both .avi and offline .raw)\n");
  printf("r  : Toggle [r]ecording (.avi)\n");
  printf("1  : Font\n");
  printf("5  : Reset defaults\n");
  printf("p  : Sna[p]shot (both .png and offline .raw)\n");
  printf("h  : Cycle through overlayed screen data\n");
  printf("t  : Toggle between Celsius and Fahrenheit\n");
  printf("y  : Toggle Historgram filter (for gray scales)\n");
// Need internal camera control to lock camera from rescaling
//  printf("l  : Lock/Unlock colormap auto ranging\n");
  printf("8  : Rotate display 0, 90, 180, 270 degrees (Portrait and Landscape)\n");
  printf("e  : Toggle Freeze Frame on/off\n");
  printf("o  : Displays and cycles through %d temp ruler modes\n", RULERS_MAX_MOD-1);
  printf("3  : Ruler plot clip modes: none, outlier, below avg, above avg\n");
  printf("4  : Ruler thickness - 1/5, 1/4, 1/3, 1/2, full height\n");
  printf("   : Keypad Up/Down/Left/Right/Center(5) moves rulers\n");
  printf("   : Left mouse adds user temps or moves rulers\n");
  printf("   : Right mouse removes user temps and disables ruler mode\n");
  printf("/  : Misc stdout help information\n");
  printf("q  : Quit\n\n");
}

void processKeypress(int c, ProcessedThermalFrame *ptf, Mat *frame ) {
// waitKey(82) // up
// waitKey(84) // down
// waitKey(81) // left
// waitKey(83) // right
// waitKey(-105) keypad up
// waitKey(-103) keypad down
// waitKey(-106) keypad left
// waitKey(-104) keypad right
// waitKey(-99) keypad 5 (center)

//printf("keyPress(%d)\n", c);

	switch ( c ) {
		case 'a': reRad( 1); break; // Blur
		case 'z': reRad(-1); break;

		case 's': reThreshold( 1); break; // Threshold
		case 'x': reThreshold(-1); break;

		case 'd': 
			  threadData.configurationChanged++;
			  reScale(ptf,  1, 1); break; // Scale
		case 'c': 
			  threadData.configurationChanged++;
			  reScale(ptf, -1, 1); break;

		case 'w': 
			  threadData.configurationChanged++;
			  windowFormat(ptf, 1); break; // Window format

		case 'f': reAlpha( 0.1); break;  // Alpha
		case 'v': reAlpha(-0.1); break;

		case 'g': reInterp( 1); break;  // Interpolation
		case 'b': reInterp(-1); break;

		case 'j': reColormap( 1); break; // Colormap
		case 'm': reColormap(-1); break;

		case 'y': 
			  threadData.configurationChanged++;
			  Use_Histogram = !Use_Histogram; break; // Temp filter

		case 't': 
			  threadData.configurationChanged++;
			  reCF(); break;           // Temp format

		case 'h': hud(1); break;  // Display onscreen elements

// Remove capital letters because RPi's have shift key mapping issues
//		case 'G': 
		case '6':
			  threadData.configurationChanged++;
			  toggleFullScreen(ptf); break; // Fullscreen

		case 'r': 
			  threadData.configurationChanged++;
			  recording(ptf, 0); 
			  break; // Recording

// Remove capital letters because RPi's have shift key mapping issues
//		case 'R': 
		case '5': 
			  threadData.configurationChanged++;
			  resetDefaults(); 
			  if ( rulersOn ) {
			  	// Reset rulers to center of screen
				rulers(ptf, FIXED_TC_WIDTH/2, FIXED_TC_HEIGHT/2, 0); 
			  } else {
			  	// Reset ruler anchors to center of screen
			 	rulersX = FIXED_TC_WIDTH/2; 
				rulersY = FIXED_TC_HEIGHT/2;
			  }
			  break;    // Reset Defaults

		case 'p': snapshot(frame); break;  // Snapshot

// Need internal camera control to lock camera from rescaling
//		case 'l': unlockColormapRange(1); break;

		case 'o': // Cycle [1 - (MAX_MOD-1)], not [0 - (MAX_MOD-1)]
			  threadData.configurationChanged++;
			  rulersOn = (RulerModes)((rulersOn + 1) % RULERS_MAX_MOD);
			  if (rulersOn == RULERS_OFF)  {
				  rulersOn = (RulerModes)(RULERS_OFF+1);
			  }
			  if ( rulersOn ) {
			  	// Keep rulers at current anchors 
				rulers(ptf, rulersX, rulersY, 0); 
			  }
			  break;

		case '/':
		case '?': system("v4l2-ctl --all\n");
		  	  printf("\n");
		 	  system("v4l2-ctl --list-devices\n");
		  	  printKeyBindings();
			  break;
		case -105: // keypad up
		case -103: // keypad down
		case -106: // keypad left
		case -104: // keypad right
		case -99:  // keypad 5 (center)
//		case 81:
// 		case 82: // Same as 'R' ???
//		case 83:
//		case 84:
			  if (rulersOn)	rulers(ptf, rulersX, rulersY, c);
			  break;

		case '1': 
			  threadData.configurationChanged++;
			  UserFont = ( UserFont + 1 ) % MAX_USER_FONT;
			  setScaleControls();
			  break;
		case '3': 
			  threadData.configurationChanged++;
			  rulerBoundFlag = (rulerBoundFlag + 1) % BOUND_MAX_MOD;
			  break;
		case '4': 
			  threadData.configurationChanged++;
			  rulerThickness--;
			  if (rulerThickness < MAX_RULER_THICKNESS) rulerThickness = MIN_RULER_THICKNESS;
			  break;

		case '8': 
			  controls.lastHelpScale = -1; // trigger Help to be redrawn
			  threadData.configurationChanged++;
			  rotateDisplay(ptf);
			  // Rotate rulers around current rotated anchors
			  if ( rulersOn ) {
				rulers(ptf, rulersX, rulersY, 0); 
			  }
			  break;

		case 'e': 
			  threadData.configurationChanged++;
			  resetFrameCounter();
			  threadData.FreezeFrame = ! threadData.FreezeFrame; 
			  break;

		default: break;
	}
}


#if 0
void dumpTypes() {
	// RGB=0: Temp=4600: width=256, height=384, nframes=-1
	//   cap.set(CAP_PROP_CONVERT_RGB, 0.0); // This breaks playback
	//
	//     Setting CAP_PROP_CONVERT_RGB to 1.0 changes type to 16, CV_8UC3 (3 bytes)
	//       end-start(294912) channels(3) elemSize(3) total(98304) cols(256) rows(384) type(16) depth(0)
	//
	//     Setting CAP_PROP_CONVERT_RGB to 0.0 changes type to  8, CV_8UC2 (2 bytes)
	//       end-start(196608) channels(2) elemSize(2) total(98304) cols(256) rows(384) type(8) depth(0)
	//       ALSO BREAKS PLAYBACK
	//
	// 0 8 16 24
	// 1 9 17 25
	// 1 9 17 25
	// 2 10 18 26
	// 5 13 21 29
	// 4 12 20 28

	printf("%d %d %d %d\n", CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4);
	printf("%d %d %d %d\n", CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4);
	printf("%d %d %d %d\n", CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4);
	printf("%d %d %d %d\n", CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4);
	printf("%d %d %d %d\n", CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4);
}

void dumpFrameInfo(Mat *frame) {
// frame.total() returns the total number of array elements.
// The method returns the number of array elements 
// (a number of pixels if the array represents an image). 
// total() = 98304 = 2 * 256 * 192

// NOTE: frame.data pointer to (uchar *data)
// https://docs.opencv.org/4.5.1/d3/d63/classcv_1_1Mat.html#aa4d317d43fb0cba9c2503f3c61b866c8
// type() CV_32FC1, CV_8UC1, CV_64F, CV_32F or ...

// CONVERT_RGB = 1.0
// end-start(294912) channels(3) elemSize(3:1) total(98304) cols(256) rows(384) type(16) depth(0)
// CONVERT_RGB = 0.0
// end-start(196608) channels(2) elemSize(2:1) total(98304) cols(256) rows(384) type(8) depth(0)

#if 1
    printf( "end-start(%d) channels(%d) elemSize(%lu:%lu) total(%lu) cols(%d) rows(%d) type(%d) depth(%d)\n", 
	(int)(frame->dataend -  frame->datastart),
	    frame->channels(),
	    frame->elemSize(), 
	    frame->elemSize1(), 
	    frame->total(), 
	    frame->cols, 
	    frame->rows,
	    frame->type(),
	    frame->depth()
	    );
    fflush(stdout); fflush(stderr);
#endif
}
#endif


// Optimization: Used when drag scrolling rulers ...
void drawTempMarkerNoHalo( Mat & frame, Temperature &temp, Scalar &dotColor ) {
	// WHITE halo is move visible than a black halo

	int notCmapScale = (temp.type != TEMP_TYPE_CMAP_SCALE);

	if ( notCmapScale ) {

		drawMarker(frame, temp.markerLoc, dotColor, MARKER_CROSS, MarkerSize, 1, LINE_8);

		if ( controls.wD ) { // DOUBLE_WIDE or DOUBLE_HIGH display
			drawMarker(frame, temp.markerwDLoc, dotColor, MARKER_CROSS, MarkerSize, 1, LINE_8);

			putText(frame, temp.displayLabel, temp.labelwDLoc, Default_Font, MyFontScale, TempTextColor, 1, noHaloLineType);
		}
	}

	// Display labels are pre-generated at data harvest time
	putText(frame, temp.displayLabel, temp.labelLoc, Default_Font, MyFontScale, 
			(notCmapScale ? TempTextColor : dotColor), 1, noHaloLineType);
}


void drawTempMarker( Mat & frame, Temperature &temp, Scalar &dotColor ) 
{
	// WHITE halo is move visible than a black halo
	int notCmapScale = (temp.type != TEMP_TYPE_CMAP_SCALE);

	if ( notCmapScale ) {

		// GRAY works better @ 1X scale, less of a blob
		drawMarker(frame, temp.markerLoc, GRAY,     MARKER_CROSS, MarkerSize, 2, LINE_8);
		drawMarker(frame, temp.markerLoc, dotColor, MARKER_CROSS, MarkerSize, 1, LINE_8);

		if ( controls.wD ) { // DOUBLE_WIDE or DOUBLE_HIGH display
		        // GRAY works better @ 1X scale, less of a blob
			drawMarker(frame, temp.markerwDLoc, GRAY,     MARKER_CROSS, MarkerSize, 2, LINE_8);
			drawMarker(frame, temp.markerwDLoc, dotColor, MARKER_CROSS, MarkerSize, 1, LINE_8);

// Draw halo in LINE_8 for speed
// Draw foreground in LINE_AA (anti-aliased)
			putText(frame, temp.displayLabel, temp.labelwDLoc, Default_Font, MyFontScale, BLACK, 2, haloLineType);
			putText(frame, temp.displayLabel, temp.labelwDLoc, Default_Font, MyFontScale, TempTextColor, 1, fgLineType);
		}
	}

// Draw halo in LINE_8 for speed
// Draw foreground in LINE_AA (anti-aliased)

	// Display labels are pre-generated at data harvest time
	putText(frame, temp.displayLabel, temp.labelLoc, Default_Font, MyFontScale, BLACK, 2, haloLineType);
	putText(frame, temp.displayLabel, temp.labelLoc, Default_Font, MyFontScale, 
			(notCmapScale ? TempTextColor : dotColor), 1, fgLineType);
}

char *tempStr(char *buf, const char *prefix, Temperature &temp, const char *labelCF) {
        sprintf( buf, "%s%.1f%s", prefix, CorF(temp.celsius), labelCF );
        return buf;
}

char *tempStr(char *buf, const char *prefix, float delta, const char *labelCF) {
	// Used for [0-N] threshold displays, versus [32-N] Fahrenheit
        sprintf( buf, "%s%.1f%s", prefix, delta, labelCF );
        return buf;
}


unsigned int hueToRgb(float p, float q, float t) {

	if (t < 0.0) t += 1.0;
	if (t > 1.0) t -= 1.0;

	float ret;

	if      ((t * 6.0) < 1.0)  ret = p + (q - p) * 6.0 * t;
	else if ((t * 2.0) < 1.0)  ret = q;
	else if ((t * 3.0) < 2.0)  ret = p + (q - p) * ((2.0/3.0) - t) * 6.0;
	else                       ret = p;

// printf("ret %f => %f\n", ret, 255.0 * ret );

	return ret * 255.0; // returns [0-255]
}

void hslToRgb(float h, float s, float l,
	unsigned int *red, unsigned int *green, unsigned int *blue) {

// printf("%.2f %.2f %.2f\n", h, s, l);

	h /= 360.0; // Convert degrees [0 - 360] to [0 - 1]

	if (s == 0.0) {
		*red = *green = *blue = l; // achromatic
	} else {
		float q = (l < 0.5) ? (l * (1.0 + s)) : ((l + s) - (l * s));
		float p = ((2.0 * l) - q);

		float h1 = h + (1.0/3.0);
		float h2 = h; 
		float h3 = h - (1.0/3.0);

		*red   = hueToRgb( p, q, h1 ); // returns [0, 255]
		*green = hueToRgb( p, q, h2 ); // returns [0, 255]
		*blue  = hueToRgb( p, q, h3 ); // returns [0, 255]
	}

//printf("\t\t\th=%f s=%f l=%f => %d %d %d\n", h,s,l, *red, *green, *blue);
}


void drawCmapScale( Mat &cmapScale, int roiJumpWidth ) {

	// NOTE: Image pixels are NOT stored in Kelvin
	// NEED to find and track min/max pixel values in Image frame, not thermal frame 
	// NEED to hide min/max pixel values in Image frame to prevent auto-scaling
	// Min/Max pixel values are a range NOT starting at 0x00
	// Start with CU_8UC2 Mat
	// Divide range up by the scaled height of the frame
	// Store those pixels into the Mat (1 value per row)
	// Optionally convert Mat to current colormap (if there is one)
	// Convert Mat to COLOR_YUV2BGR_YUYV
	// Copy to video frame to be rendered

	// colormaps - 
	//     RGB(red, green, blue) 
	//     CMYK(cyan, magenta, yellow, black) 
	//     HSV(hue, saturation, value)

	int scaledSFHeight = controls.scaledSFHeight;

	double range  = maxPixel - minPixel;
	double factor = (range / (float)scaledSFHeight);

	unsigned short *datastart = &((unsigned short *)(cmapScale.datastart))[ 0 ];

	// Set cmapScale pixel values one row at a time ...
	unsigned short rowPixel, *usRowPtr;
	for ( int i = 0; i < scaledSFHeight; i++ ) {
		// Same pixel value per row

		// Colormap scale is a range [minPixel to maxPixel] starting at minPixel, NOT 0x00
		// Transisiton from maxPixel on top to minPixel on bottom ...
		rowPixel = maxPixel - ((float)i * factor); 

		// Calculate linear offset (row, column)
	        usRowPtr = &(datastart)[ (i * roiJumpWidth) ];

#if 0
		for ( int j = 0; j < ColorScaleWidth; j++ ) {
			usRowPtr[ j ] = rowPixel;
		}
#else
		// ColorScaleWidth = 3 + scale => minimum 4 to maximum (3 + maxScale)
		usRowPtr[ 0 ] = rowPixel;
		usRowPtr[ 1 ] = rowPixel;
		usRowPtr[ 2 ] = rowPixel;
		usRowPtr[ 3 ] = rowPixel;

		// Handle remainder
		for ( int j = 4; j < ColorScaleWidth; j++ ) {
			usRowPtr[ j ] = rowPixel;
		}
#endif
	}

	cvtColor(cmapScale, cmapScale, COLOR_YUV2BGR_YUYV, CVT_CHAN_FLAG);  // Same as video frame
	applyMyColorMap( cmapScale, cmapScale, controls.cmapCurrent );
}

#define L_X	( hudSpaceSize.width / 2 )		// Left X margin

static Point helpPoint;
void drawHelp( Mat &rgbHUD ) {

	controls.lastHelpScale = MyScale;

	char buf[256];

	float yOffset = 1.4 * (float) hudSpaceSize.height;
	float yDelta  = (0.1 + ((float)HelpHeight / (float)MAX_HELP_TEXT_ROWS));

	int x1 =  L_X;
	int x2 = ( x1 + hudW_WSize.width ); // Vertically align ":"

	#define Y(row)  round(yOffset + ((float)(row) * yDelta))

	POINT( helpPoint, HelpWidth, HelpHeight );

	// display black box for our data
	rectangle( rgbHUD, PointZeroZero, helpPoint, BLACK, -1 );

#ifdef PT
#undef PT
#endif

#define RT(r, s2, xxx ) \
	POINT( helpPoint, xxx, Y(r) ); \
	putText(rgbHUD, s2, helpPoint, Default_Font, HudFontScale, YELLOW, 1, helpLineType);

	// Manually align ":" since we are NOT using fixed sized fonts
#define PT(r, s1, s2) \
	POINT( helpPoint, x1, Y(r) ); \
	putText(rgbHUD, s1, helpPoint, Default_Font, HudFontScale, WHITE, 1, helpLineType); \
	RT(r, s2, x2)

		PT(  0, "a z", ": Blur filter" );
		PT(  1, "s x", ": Threshold degrees" );
		sprintf( buf,": Window scale [1-%d]", MAX_SCALE_STEPS );
		PT(  2, "d c", buf );
		PT(  3, "f v", ": Contrast" );
		sprintf( buf,": Inerpolation (%d)", MAX_INTERS );
		PT(  4, "g b", buf );
		sprintf( buf,": Colormaps (%d)", MAX_CMAPS );
		PT(  5, "j m", buf );
		PT(  6, "w", ": Window layouts (4)" );
		PT(  7, "5", ": Reset, 6 : Fullscreen" );
		PT(  8, "r", ": Record (.avi)" );
		PT(  9, "p", ": Snapshot (.png, .raw)" );
		PT( 10, "h", ": OSD modes, 1 : Font" );
		PT( 11, "t", ": C/F,  y : Histogram" );
		PT( 12, "8", ": Rotate, e Freeze" );
		sprintf( buf,": Ruler modes (%d)", RULERS_MAX_MOD-1 );
		PT( 13, "o", buf );
		PT( 14, "", "  3 : Clip, 4 : Size" );
		PT( 15, "/", ": stdout, q : Quit" );
		RT( 16, "Keypad arrows : Move rulers", x1 );
		RT( 17, "L mb: Add temps, mv rulers", x1 );
		RT( 18, "R mb: Del temps & rulers", x1 );
}

static Point hudPoint;
void drawHUD(ProcessedThermalFrame *ptf, Mat &rgbHUD, const char *src, Scalar srcColor) 
{
	char buf[256];
	const char *labelCF = controls.labelCF;

	float yOffset = 1.4 * (float) hudSpaceSize.height;
	float yDelta  = (float)HudHeight / (float)MAX_HUD_TEXT_ROWS;

#ifdef Y
#undef Y
#endif
#define Y(row)  round(yOffset + ((float)row * yDelta))

	// display black box for our data
	POINT( hudPoint, HudWidth, HudHeight );
	rectangle(rgbHUD, PointZeroZero, hudPoint, BLACK, -1);

	// put text in the box
	POINT( hudPoint, L_X, Y(0) );
	putText(rgbHUD, tempStr(buf,"Avg Temp: ", ptf->avg, labelCF), hudPoint,
	Default_Font, HudFontScale, WHITE, 1, hudLineType);

	// Threshold is a [0-N] delta in degrees, not a temp
	POINT( hudPoint, L_X, Y(1) );
	putText(rgbHUD, tempStr(buf,"Threshold: ", controls.threshold.celsius,labelCF), hudPoint,
	Default_Font, HudFontScale, WHITE, 1, hudLineType);

	//sprintf(buf, "Colormap: %s", cmaps[controls.cmapCurrent].name);
	sprintf(buf, "Map: %s%s", cmaps[controls.cmapCurrent]->name, Use_Histogram?"+Hist":"");
	POINT( hudPoint, L_X, Y(2) );
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW, 1, hudLineType);

	sprintf(buf, "Blur: %d  Intr: %s", controls.rad, Inters[controls.inters].name);
	POINT( hudPoint, L_X, Y(3) );
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW, 1, hudLineType);

 	//sprintf(buf, "Scale: %d  Src: %s", MyScale, src );
 	sprintf(buf, "Scale: %d  Src: ", MyScale ); // Add Source error color

	// Get width offset to display src in srcColor
	int baseline;
	Size ts = getTextSize(buf, Default_Font, HudFontScale, 1, &baseline);

	POINT( hudPoint, L_X, Y(4) ); 
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW,   1, hudLineType);
	POINT( hudPoint, L_X+ts.width, Y(4) );
	putText(rgbHUD, src, hudPoint, Default_Font, HudFontScale, srcColor, 1, hudLineType);

	sprintf(buf, "Contrast: %.1f%s", controls.alpha, threadData.FreezeFrame ? " Freeze" : "" );
	POINT( hudPoint, L_X, Y(5) ); 
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW, 1, hudLineType);

	sprintf(buf, "Snapshot: %s", controls.snaptime);
	POINT( hudPoint, L_X, Y(6) ); 
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW, 1, hudLineType);

      	sprintf(buf, "Recording: %s", controls.elapsed);
	POINT( hudPoint, L_X, Y(7) ); 
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, *ptf->rColor, 1, hudLineType);

	sprintf(buf, "FPS: %.1f  %s", controls.fps, controls.labelWF);
	POINT( hudPoint, L_X, Y(8) ); 
	putText(rgbHUD, buf, hudPoint, Default_Font, HudFontScale, YELLOW, 1, hudLineType);
}


//  Calculate p3 and p4 on the fly
#define LINE2( mat, p1, p2, p3, p4, color, thickness, sX, sY ) \
{ \
	line( mat, p1, p2, color, thickness ); \
	if ( controls.wD ) { \
		POINT( p3, p1.x + sX, p1.y + sY ); \
		POINT( p4, p2.x + sX, p2.y + sY ); \
		line( mat, p3, p4, color, thickness ); \
	} \
}

#define ARROW_LINE2( mat, p1, p2, p3, p4, color, thickness, sX, sY, arrow ) \
{ \
	arrowedLine( mat, p1, p2, color, thickness, LINE_8, 0, arrow ); \
	if ( controls.wD ) { \
		POINT( p3, p1.x + sX, p1.y + sY ); \
		POINT( p4, p2.x + sX, p2.y + sY ); \
		arrowedLine( mat, p3, p4, color, thickness, LINE_8, 0, arrow ); \
	} \
}

// Remove 1X flicker efforts
// Draw all reference BASELINE_COLOR black lines in parallel (black on black doesn't flicer) (bottom of stack)
// 	incAndWait
// Draw all min MIN_COLOR red lines in serial  (red on red doesn't flicker)
// Draw all max MAX_COLOR green lines in serial(green on green doesn't flicker) 
// In other thread, draw perp lines which don't overlap
// 	incAndWait
// Draw all plots
//

static	Point m3, m4;      // Eliminate automatic variable overhead
static	Point h_p3, h_p4;  // Eliminate automatic variable overhead
static	Point v_p3, v_p4;  // Eliminate automatic variable overhead
void drawRulerPlot( Mat &frame, Point *sf1, Point *sf2, Scalar **scalar, int max, int horizCaller ) {
//	int64_t t1 = currentTimeNanos();

	// renderDataThread() and drawMinMaxRulerLines() can't display these without a
	// rare flicker race condition with the parallel plot graph lines
	if ( horizCaller ) {
		// Draw horizontal black temp reference lines
		if ( Rulers_Both_Horiz )
		{
			// Draw 2 black lines with void in middle for the colored cross hair 
			LINE2( frame, hBase_src, hBase_b,   h_p3, h_p4, RULER_BASELINE_COLOR, 1, sX, sY ); // horizontal reference line
			LINE2( frame, hBase_c,   hBase_dst, h_p3, h_p4, RULER_BASELINE_COLOR, 1, sX, sY ); // horizontal reference line
		} 
	} else {
		// Draw vertical black temp reference lines
		if ( Rulers_Both_Vert )
		{
			// Draw 2 black lines with void in middle for the colored cross hair 
			LINE2( frame, vBase_src, vBase_b,   v_p3, v_p4, RULER_BASELINE_COLOR, 1, sX, sY ); // vertical reference line
			LINE2( frame, vBase_c,   vBase_dst, v_p3, v_p4, RULER_BASELINE_COLOR, 1, sX, sY ); // vertical reference line
		}
	}

	if ( Rulers_Both ) {
		// Wait for black reference lines to be drawn
		int myState;
		incAndWait( &BothPlotMutex, &myState, BothPlotState );
	}


	if ( horizCaller ) {
		// draw all MIN_COLOR before MAX_COLOR
		if ( Rulers_Both_Horiz ) {
			// XXXX_Min_Bound controls MAX_RULER_THICKNESS flickering over Min/Max lines
			if ( minBound && Horiz_Min_Bound ) {
				LINE2( frame, hMin_src, hMin_dst, m3, m4, RULER_MIN_COLOR, 1, sX, sY ); // horizontal reference line
			}

		}
		if ( Rulers_Both_Vert ) {
			// XXXX_Min_Bound controls MAX_RULER_THICKNESS flickering over Min/Max lines
			if ( minBound && Vert_Min_Bound ) {
				LINE2( frame, vMin_src, vMin_dst, m3, m4, RULER_MIN_COLOR, 1, sX, sY ); // vertical reference line
			}
		}

		// draw all MAX_COLOR after MIN_COLOR
		if ( Rulers_Both_Horiz ) {
			if ( maxBound && Horiz_Max_Bound ) {
				LINE2( frame, hMax_src, hMax_dst, m3, m4, RULER_MAX_COLOR, 1, sX, sY ); // horizontal reference line
			}
		}
		if ( Rulers_Both_Vert ) {
			if ( maxBound && Vert_Max_Bound ) {
				LINE2( frame, vMax_src, vMax_dst, m3, m4, RULER_MAX_COLOR, 1, sX, sY ); // vertical reference line
			}
		}

	} else {
		// Draw All Perps
		if ( Rulers_Both_Horiz ) {
			// Draw perpendicular vertical cross hair line on single horizontal ruler
			ARROW_LINE2( frame, vPerp, rulerX0Points[ rulersX ], m3, m4, *vScalar, 1, sX, sY, arrowScaleV );
		}
		if ( Rulers_Both_Vert ) {
			// Draw perpendicular horizontal cross hair line on single vertical ruler
			ARROW_LINE2( frame, hPerp, rulerY0Points[ rulersY ], m3, m4, *hScalar, 1, sX, sY, arrowScaleH);
		}
	}


	if ( Rulers_Both ) {
		// Wait for min/max and perp lines to be drawn
		int myState;
		incAndWait( &BothPlotMutex, &myState, (BothPlotState + 1) % ARRAY_COUNT(BothPlotStates) );
	}

	int minusOneMax = max - 1; // Compenstate for loop accessing [i + 1]

	if (   (horizCaller && Rulers_Both_Horiz) ||
	     (! horizCaller && Rulers_Both_Vert)  ) {

		// Fomerly LINE3() ...
		if ( controls.wD ) { // Take if out of loop body
			for ( int i = 0; i < minusOneMax; i++ ) {
				line( frame, sf1[i], sf1[i + 1], *scalar[i], 1 );
				line( frame, sf2[i], sf2[i + 1], *scalar[i], 1 );
			}
		} else {
			for ( int i = 0; i < minusOneMax; i++ ) {
				line( frame, sf1[i], sf1[i + 1], *scalar[i], 1 );
			}
		}
	}

	if ( Rulers_Both ) {
		// Prevent flicker between plot and temp text points
		int myState;
		incAndWait( &BothPlotMutex, &myState, (BothPlotState + 2) % ARRAY_COUNT(BothPlotStates) );
	}

	BothPlotState = BothPlotMutex.blockingState; // Set for next stage

//	printf("\t%s(%d) %ld nanos\n", __func__, __LINE__, (currentTimeNanos() - t1)); fflush(stdout); fflush(stderr);
}


// Optimization: When dragging rulers, don't draw text and marker halos
// Calls drawTempMarker() or drawTempMarkerNoHalo() via function pointer
// Eliminate array dereferencing by using Temperature pointers
#undef  DO_USER
#define DO_USER(n)   if ( user_##n->active ) { drawTempMarkerPtr( frame, *user_##n, GREEN ); } 


// Called only once per frame
void drawOneThirdOfTheTemps() {

	ProcessedThermalFrame *ptf = threadData.ptf;
	Mat frame                  = *threadData.rgbFrame;

	{
		// Don't draw any text temps until lines and plots are rendered
		int myState;
		incAndWait( &B4TempPlotMutex, &myState, B4TempPlotState );
	}

#if 0
{
	// NO_HALO is [13-14]X faster
	// SIMPLEX is 3X faster than TRIPLEX
	Default_Font = cv::FONT_HERSHEY_SIMPLEX; // Normal size sans-serif font
	#define MAX_TEST 100
	int64_t t1,t2,t3;
	for (int j = 0; j < 2; j++) {
		t1 = currentTimeMillis();
		drawTempMarkerPtr = &drawTempMarker;
		for (int i = 0; i < MAX_TEST; i++) { 
			DO_ALL()
		}
		t2 = currentTimeMillis(); 
		drawTempMarkerPtr = &drawTempMarkerNoHalo;
		for (int i = 0; i < MAX_TEST; i++) { 
			DO_ALL()
		}
		t3 = currentTimeMillis();
		int64_t d1 = t2-t1;
		int64_t d2 = t3-t2;
		if (0 < d1) {
			printf("%s: DO_USER(%ld), NO_HALO(%ld) delta(%.3f)\n", 
			((0==j) ? "SIMPLEX" : "TRIPLEX"), 
			d1, d2, (double)(0 == d2) ? 0.0 : ((float)d1/(float)d2) );
		}
		Default_Font = cv::FONT_HERSHEY_TRIPLEX; // Normal size serif font (more complex than COMPLEX)
	}
	printf("\n");

	// These are after removing Anti-Aliasing from halos
	// SIMPLEX: DO_USER(27), NO_HALO(3) delta(9.000)
	// TRIPLEX: DO_USER(81), NO_HALO(10) delta(8.100)

	// SIMPLEX: DO_USER(27), NO_HALO(3) delta(9.000)
	// TRIPLEX: DO_USER(82), NO_HALO(10) delta(8.200)

	// SIMPLEX: DO_USER(28), NO_HALO(3) delta(9.333)
	// TRIPLEX: DO_USER(82), NO_HALO(10) delta(8.200)

	// Restore state
	drawTempMarkerPtr = &drawTempMarker;
	Default_Font = cv::FONT_HERSHEY_SIMPLEX;	// Normal size sans-serif font
}
#endif

	// Horizontal are [0=5], vertical are [6-11], center is [12]
	// Users are [0-12]
	// Split users, horizontal and vertical evenly amoung 3 threads
	//
	// Prevent horizontal flicker when stacked against HUD/HELP
	// 	Left 3 and 2nd and 3rd right ( rollover ) of horizontal
	// See horizFlickerTomFoolery() - hidden sauce
	// 	0, 1, 2, 3, 4  or
	// 	1, 3, 5, 2, 4
	// Entire horizontal row for reduced scale 1 and 2
	DO_USER(0) DO_USER(1) DO_USER(2) DO_USER(3) DO_USER(4)

	{
		// Prevent cmap Temps from flashing
		int myState;
		incAndWait( &CmapPlotMutex, &myState, CmapPlotState );
	}

	drawTempMarker( frame, ptf->maxPixel,  WHITE );
	drawTempMarker( frame, ptf->avg1Pixel, WHITE );
}

// This function is called in parallel (vertical and horizontal) to divide up rendering
void drawUserAndRulerTemps( int horizontal ) {

	ProcessedThermalFrame *ptf = threadData.ptf;
	Mat frame                  = *threadData.rgbFrame;
	Rect roi;

	{
		// Don't draw any text temps until lines and plots are rendered
		int myState;
		incAndWait( &B4TempPlotMutex, &myState, B4TempPlotState );
	}

//        int64_t t1 = currentTimeMicros();

	// Draw half of user or ruler temps per parallel thread

	// Horizontal are [0=5], vertical are [6-11], center is [12]
	// Users are [0-12]
	// Split users, horizontal and vertical evenly amoung 3 threads
	//
	// Prevent horizontal stack flickering against HUD/HELP
	// 	First Right 2 of horizontal + 2 vertical
	if ( horizontal ) {

		if ( RULERS_OFF == rulersOn ) {
			// Only draw once, not for both horizontal and vertical calls
			// Draw crosshair and label over first frame
			drawTempMarker( frame, ptf->ch, BLACK );
		}

		// ONLY render once / frame
		// Keep center marker from flickering on single rulers
		DO_USER( CENTER_OF_RULER_INDEX );

		// Draw 1st right horizontal and 2 vertical
		// See horizFlickerTomFoolery() - hidden sauce
		DO_USER(5) DO_USER(6) DO_USER(7)

		{
			// Prevent cmap Temps from flashing
			int myState;
			incAndWait( &CmapPlotMutex, &myState, CmapPlotState );
		}

		drawTempMarker( frame, ptf->avg2Pixel, WHITE );
		drawTempMarker( frame, ptf->avg3Pixel, WHITE );

		{
			// Wait for all cmap gradient temps to be drawn to prevent flicker
			int myState;
			incAndWait( &BothPlotMutex, &myState, BothPlotState );
		}

		// Draw ">" crosshair temp indicator on colormap gradient scale
		float kTmp = ptf->chLevelPixel.kelvin;

		drawTempMarker( frame, ptf->chLevelPixel,
				((kTmp < minusThresholdKelvin) ? RULER_MIN_COLOR :
				 (kTmp > plusThresholdKelvin)  ? RULER_MAX_COLOR : RULER_MID_COLOR) );

		try {
			// Display either OSD Help or HUD, not both
			if ( HUD_HELP == controls.hud ) {
				roi.x = roi.y = 0;
				// NOTE: Help is taller than 1X scale PORTRAIT Help
				roi.width    = min( HelpWidth,  SCALED_TC_WIDTH  );
				roi.height   = min( HelpHeight, SCALED_TC_HEIGHT );
				Mat frameROI = frame( roi ); // Grab pointer to section of image under HUD

				addWeighted( frameROI, 1-HUD_ALPHA, threadData.rgbHUD, HUD_ALPHA, 0.0, frameROI );

			} else if ( HUD_ON == controls.hud ) {
				// Make HUD translucent
				// Alpha blended HUD is CPU intensive, use smallest rectangle possible
				roi.x = roi.y = 0;
				roi.width    = min( HudWidth,  SCALED_TC_WIDTH  );
				roi.height   = min( HudHeight, SCALED_TC_HEIGHT );
				Mat frameROI = frame( roi ); // Grab pointer to section of image under HUD

				// Alpha blend with HUD
				// Copy blended result back to same section of output frame
				addWeighted( frameROI, 1-HUD_ALPHA, threadData.rgbHUD, HUD_ALPHA, 0.0, frameROI );
			}
		} catch (...) {
			controls.lastHelpScale = -1; // trigger Help to be redrawn
			printf("%s(%d) - HUD exception (%d)\n", __func__, __LINE__, horizontal); 
			fflush(stdout); fflush(stderr);
		}

	} else {
		// Top and bottom 2 of vertical
		DO_USER(8) DO_USER(9) DO_USER(10) DO_USER(11)

		{
			// Prevent cmap Temps from flashing
			int myState;
			incAndWait( &CmapPlotMutex, &myState, CmapPlotState );
		}

		drawTempMarker( frame, ptf->avg4Pixel, WHITE );
		drawTempMarker( frame, ptf->minPixel,  WHITE );
	
		// Add crosshair temp marker on Colormap Scale to mark crosshair and avg temps
		//     | | 
		//    >| | Crosshair temp indicator
		//     | | 
		//     | | 
		//     | | 
		//    -| | Average temp indicator
		//     | | 
		//     | | 
		//     | | 
		//

		{
			// Wait for all cmap gradient temps to be drawn to prevent flicker
			int myState;
			incAndWait( &BothPlotMutex, &myState, BothPlotState );
		}

		// Draw "-" average temp indicator on colormap gradient scale
		drawTempMarker( frame, ptf->avgLevelPixel, RULER_MID_COLOR );

	//	try {
			{ // Draw opaque Colormap Gradient Scale
				// Reversed from Mat cmapScale(width,height)
				roi.x      = (controls.scaledSFWidth - ColorScaleWidth);
				roi.y      = 0;
				roi.width  = ColorScaleWidth;
				roi.height = controls.scaledSFHeight; 

				threadData.cmapScale.copyTo( frame( roi ) );  // MUST BE SAME PIXEL FORAMT !!!
			}
	//	} catch (...) {
	//		printf("%s(%d) - cmapScale exception\n", __func__, __LINE__); 
	//		fflush(stdout); fflush(stderr);
	//	}
	
	}

//	printf("horiz(%d) - %s(%d) tenth ms %ld\n", horizontal, __func__, __LINE__, (currentTimeMicros() - t1) / 100 );
}

extern void exitAllThreads();

static int rotateFlags[] = {
	-1, 
	ROTATE_90_CLOCKWISE,
	ROTATE_180,
	ROTATE_90_COUNTERCLOCKWISE 
};

void *stdinDataThread( void *ptr ) {
	ThreadData *td = (ThreadData *)ptr;

        int64_t t1 = currentTimeMicros();

	//setvbuf(stdin, NULL, _IONBF, 0); // Set stdin unbuffered
	//setvbuf(stdin, NULL, _IOFBF, 1); // Set stdin unbuffered

	int stdinCharacter = -1;
	char buf[256];

	while ( td->running ) {

		// Using read blocks and writeKeyEvent mutexes 
		buf[0] = 0;
		//int tmp = fgetc(stdin); // Block on stdin
#if PRINT_STDIN
		memset( buf, 0, sizeof(buf) );
#endif
		if ( fgets( buf, sizeof(buf), stdin ) ) {
#if PRINT_STDIN
	printf("%s(%d) str(%s) c(%c) [%d][%d][%d][%d][%d]\n", 
		__func__, __LINE__, buf, buf[0], 
		buf[0], buf[1], buf[2], buf[3], buf[4] );

#endif
			int tmp0 = buf[0];
			if ('\n' != tmp0) {
				int tmp1 = buf[1];
				int tmp2 = buf[2];
				// Map 3-char terminal keypad value to OpenCV keypad value
				if (27 == tmp0 && 91 == tmp1) {
					if      (65 == tmp2) stdinCharacter = -105;
					else if (66 == tmp2) stdinCharacter = -103;
					else if (67 == tmp2) stdinCharacter = -104;
					else if (68 == tmp2) stdinCharacter = -106;
					else if (69 == tmp2) stdinCharacter = -99;
				} else {
					stdinCharacter = tmp0;
				}
				writeKeyEvent( stdinCharacter );
			}
		} else {
			if ( feof(stdin) ) {
				printf("\n\n%s(%d) - %sstdin EOF detected%s - exiting\n", 
					__func__, __LINE__, RED_STR(), RESET_STR() ); 
				fflush(stdout); fflush(stderr);
				break;
			}
			printf("a");
			sleepMillis(100);
		}
// Exended ASCII ???
// keypad arrows ESC(27) + 91 + [65,66,67,68,69]
// 65 up arrow
// 66 down arrow
// 67 right arrow
// 68 left arrow
// 69 center(5)
//	case -105: // keypad up
//	case -103: // keypad down
//	case -106: // keypad left
//	case -104: // keypad right
//	case -99:  // keypad 5 (center)

		//printf("'%c'   -    \"%d\"\n", stdinCharacter, stdinCharacter);
		//printf("\"%d\"\n", stdinCharacter);
#if PRINT_STDIN
		memset( buf, 0, sizeof(buf) );
#endif
	}

        double seconds = (currentTimeMicros() - t1) / 1e6;

	threadData.running = 0; // Tell all threads to exit

	printf("%s(%d) - %sduration %.3f seconds%s\n\n\n",
		__func__, __LINE__, GREEN_STR(), seconds, RESET_STR() ); fflush(stdout);

	return ptr;
}


#if 1
#define TS(a) a 
#else
#define TS(a) /* compile out (T)iming (S)tat call overhead */
#endif

// Optimization and make thread safe by user passing in pre-allocated hist scratch buffer
void histogramWrapper( Mat &src, Mat &dst )
{
//printf("> %s(%d) %d x %d\n", __func__, __LINE__, src.rows, src.cols ); fflush(stdout); fflush(stderr);

	Mat hist( src.rows, src.cols, CV_8UC1 ); // equalizeHist() only works on 8UC1 matrix
	int max = src.rows * src.cols;

	// max 33022 min 32768 delta 254

	// Perf Results: 
	// Point 8X  Loop Unroll 13.42% vs Array Index 41.85%
	// Point 16X Loop Unroll 11.64% vs Array Index 42.72%

	// Gprof Results:
	// gprof array (not unrolled) 0.27ms/call
	// gprof ptr unroll @ 2X faster than array unroll 0.08ms/call vs 0.14ms/call

	// *************** BEGIN POINTER LOOP UNROLL VERSION ******************
	unsigned short *srcPtr  = &((unsigned short *)(src.datastart))[0];
	unsigned short *maxPtr  = &((unsigned short *)(src.datastart))[max];
	unsigned char  *histPtr = &( (unsigned char *)(hist.datastart))[0];
	for ( ; srcPtr < maxPtr; srcPtr += 8, histPtr += 8) { // Copy CV_8UC2 into CV_8UC1 to make monochrome
		* histPtr    = (unsigned char)(* srcPtr    - 32768);
		*(histPtr+1) = (unsigned char)(*(srcPtr+1) - 32768);
		*(histPtr+2) = (unsigned char)(*(srcPtr+2) - 32768);
		*(histPtr+3) = (unsigned char)(*(srcPtr+3) - 32768);
		*(histPtr+4) = (unsigned char)(*(srcPtr+4) - 32768);
		*(histPtr+5) = (unsigned char)(*(srcPtr+5) - 32768);
		*(histPtr+6) = (unsigned char)(*(srcPtr+6) - 32768);
		*(histPtr+7) = (unsigned char)(*(srcPtr+7) - 32768);
	}
	// *************** END POINTER LOOP UNROLL VERSION ******************

	equalizeHist( hist, hist ); // Source 8-bit single channel image. 

	// *************** BEGIN POINTER LOOP UNROLL VERSION ******************
	unsigned short *dstPtr  = &((unsigned short *)(dst.datastart))[0];
	                maxPtr  = &((unsigned short *)(dst.datastart))[max];
	                histPtr = &( (unsigned char *)(hist.datastart))[0];
	for ( ; dstPtr < maxPtr; dstPtr += 8, histPtr += 8) { // Copy CV_8UC2 into CV_8UC1 to make monochrome
		* dstPtr    = (unsigned short)(* histPtr    + 32768);
		*(dstPtr+1) = (unsigned short)(*(histPtr+1) + 32768);
		*(dstPtr+2) = (unsigned short)(*(histPtr+2) + 32768);
		*(dstPtr+3) = (unsigned short)(*(histPtr+3) + 32768);
		*(dstPtr+4) = (unsigned short)(*(histPtr+4) + 32768);
		*(dstPtr+5) = (unsigned short)(*(histPtr+5) + 32768);
		*(dstPtr+6) = (unsigned short)(*(histPtr+6) + 32768);
		*(dstPtr+7) = (unsigned short)(*(histPtr+7) + 32768);
	}
	// *************** END POINTER LOOP UNROLL VERSION  ******************
//printf("< %s(%d) %d x %d\n", __func__, __LINE__, src.rows, src.cols ); fflush(stdout); fflush(stderr);
}


void *imageDataThread( void *ptr ) {

	nice( -20 );

	ThreadData *td = (ThreadData *)ptr;
	ProcessedThermalFrame *ptf = td->ptf;

	Rect subFrameROI = Rect(0, 0, FIXED_TC_WIDTH, FIXED_TC_HEIGHT);

	TS( int64_t imageMicros; )
	TS( int64_t tmp; )

	Size  size; // Reuse automatic variable
	Point point;
	Mat   source, *sourcePtr;

	while ( td->running ) {

		incAndWait(&WorkerMutex, &td->imageState, 0); /**************** BLOCKING - Tier 0 */
		if ( ! td->running ) break; // after keypress

		TS( imageMicros = currentTimeMicros(); )

		imageFrame = Mat( *(threadData.rawFrame), subFrameROI ).clone();
		if ( RotateDisplay ) { rotate( imageFrame, imageFrame, rotateFlags[ RotateDisplay ] ); }

		if ( WINDOW_THERMAL != controls.windowFormat ) {
			if ( Use_Histogram ) {
				histogramWrapper( imageFrame, imageFrame );
			}

			// Convert the composited image copy to RGB
			// cv::cvtColor (InputArray src, OutputArray dst, int code, int dstCn=0)
			cvtColor ( imageFrame, rgbImageFrame, COLOR_YUV2BGR_YUYV, CVT_CHAN_FLAG );

			// Set contrast
			// void cv::convertScaleAbs (InputArray src, OutputArray dst, double alpha=1, double beta=0)
			// dst(I)=saturate\_cast<uchar>(|src(I)∗alpha+beta|)
			if ( 1.0 != controls.alpha ) {
				// Optimization:  Only set alpha/contrast if it is NOT 1.0
				convertScaleAbs(rgbImageFrame, rgbImageFrame, controls.alpha); // Contrast
			}
		}

		ASSERT(( TC_WIDTH  == imageFrame.cols ));
		ASSERT(( TC_HEIGHT == imageFrame.rows ));

		TS( tmp = currentTimeMicros(); )

		TS( threadData.rotMicros += ( tmp - imageMicros ); ) // track relative benchmarks
		TS( td->imageMicros      += ( tmp - imageMicros ); ) // track relative benchmarks

		incAndWait(&WorkerMutex, &td->imageState, 1); /**************** BLOCKING - Tier 1 */

		TS( imageMicros = currentTimeMicros(); )
//printf("\t%s(%d) - Running %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

		// Offline and FreezeFrame optimizations 
		if ( RERENDER_OPTIMIZATION ) {

			// Use image frame and/or thermal frame for RGB rendering, snapshot and recording

			// Make window layout modes from copy
			switch ( controls.windowFormat ) {
				case WINDOW_IMAGE:
					sourcePtr = &rgbImageFrame;
					break;
				case WINDOW_THERMAL:
					sourcePtr = &rgbThermalFrame;
					break;
				case WINDOW_DOUBLE_WIDE:
					cv::hconcat( rgbImageFrame, rgbThermalFrame, source );
					sourcePtr = &source;
					break;
				case WINDOW_DOUBLE_HIGH:
				default:
					cv::vconcat( rgbImageFrame, rgbThermalFrame, source );
					sourcePtr = &source;
					break;
			}

			/************************************************************************************
			 * At this point we may have WINDOW_DOUBLE modes implying a larger matrix 
			 * along with the subsequent call to resize makes it less desirable to push more 
			 * image processing back into tier 0.
			 * Doing so would increase more pixel processing in subsequent calls
			************************************************************************************/

			// User selected interpolation incuding bicubic, upscale
			// /usr/include/opencv4/opencv2/imgproc.hpp
			// void cv::resize(cv::InputArray, cv::OutputArray, cv::Size, double, double, int)
		
			SIZE( size, controls.sW, controls.sH );
			resize(*sourcePtr, *td->rgbFrameOrig, size,
				     0.0, 0.0, // optional 0.0, 0.0 args is REQUIRED for Interpolation options to work
				     Inters[controls.inters].inter // INTER_CUBIC default
				); // Scale up from native camera resolution

			// /usr/include/opencv4/opencv2/imgproc.hpp
			// void cv::blur(cv::InputArray, cv::OutputArray, cv::Size, cv::Point, int)

			if ( controls.rad > 0 ) { // Optionally Blur to smooth scaling pixelization and diagonal jaggies
				POINT( point, controls.rad, controls.rad );

				blur( *td->rgbFrameOrig, *td->rgbFrameOrig, point );
			}

			//  Colormaps do not support ALPHA/TRANSPARENTCIES
			//  what():  OpenCV(4.5.1) ../modules/imgproc/src/colormap.cpp:736: error: (-5:Bad argument) 
			//  cv::ColorMap only supports source images of type CV_8UC1 or CV_8UC3 in function 'operator()'

			applyMyColorMap( *td->rgbFrameOrig, *td->rgbFrameOrig, controls.cmapCurrent );
		}

#if USE_CLONE
		*td->rgbFrame = td->rgbFrameOrig->clone();
#else
		td->rgbFrameOrig->copyTo( *td->rgbFrame );
#endif

		TS( td->imageMicros += ( currentTimeMicros() - imageMicros ); ) // track relative benchmarks
      		incAndWait(&WorkerMutex, &td->imageState, 2); /**************** BLOCKING - Tier 2 */

		TS( imageMicros = currentTimeMicros(); )

		// Do not display graphics over video
		if ( controls.hud != HUD_NO_DRAWINGS )
		{
			if ( Rulers_Both_Horiz_Vert ) 
			{
		//		try {
					drawRulerPlot( *threadData.rgbFrame, 
						rulerY0Points, rulerY1Points, scalarY, TC_HEIGHT, 0 );
		//		} catch (...) {
		//			printf("\t%s(%d) - drawRulers exception\n", __func__, __LINE__);
		//			fflush(stdout); fflush(stderr);
		//		}
			}
		//	try {
				drawUserAndRulerTemps( 0 );
		//	} catch (...) {
		//		printf("\t%s(%d) - drawUserAndRulerTemps exception\n", __func__, __LINE__);
		//		fflush(stdout); fflush(stderr);
		//	}
		}

		if ( controls.hud != HUD_NO_DRAWINGS ) {
	//		try {
				// Display floating min temp (while keeping text visible @ display boundaries)
				if ( drawMin ) {
					drawTempMarker( *threadData.rgbFrame, ptf->min, BLUE );
				}
	//		} catch (...) {
	//		}
		}

		TS( td->imageMicros += ( currentTimeMicros() - imageMicros ); ) // track relative benchmarks

//printf("\t%s(%d) - Halted %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

		incAndWait(&WorkerMutex, &td->imageState, 3); /**************** BLOCKING - Tier 3 */

		TS( imageMicros = currentTimeMicros(); )

		// Active Recording doesn't handle scale/resize/rotate changes, so ...
		// the active recording gets stopped when these configuration changes happen.
		if ( controls.recording ) {
			try {
				time_t now          = time(NULL);
				time_t delta        = now - controls.recordStartTime;
				struct tm *timeinfo = gmtime (&delta);

				strftime (controls.elapsed, sizeof(controls.elapsed), "%H:%M:%S", timeinfo);

				pthread_mutex_lock( &videoOutMutex );
					// Protect videoOut and recordingActive
					if ( controls.recordingActive ) {
						ptf->videoOut.write( *threadData.rgbFrame );
					}
				pthread_mutex_unlock( &videoOutMutex );

				controls.recFrameCounter++;

				if ( 0 == (controls.recFrameCounter % 6) ) {
// TODO - FIXME - This increment maybe a race condition with the main threads clearing to 0x00
					// Signal HUD to update at least 4X / second to update elapsed timer
					threadData.configurationChanged++; 
				}

				// Raw video file grows very quickly, maybe drop a few frames ???
#if 0 // Disabling this feature until streaming compression is added
				if ((controls.recFrameCounter % 10) == 0) {
					rawRecFp = writeRawFrame( rawFrame, rawRecFilename, rawRecFp, 1 );
				}
#endif
			} catch (...) {
			}
		}

		TS( td->imageMicros += ( currentTimeMicros() - imageMicros ); ) // track relative benchmarks

	}  // end while(1)

	printf("%s(%d) - exiting\n", __func__, __LINE__); fflush(stdout);

	exitAllThreads();

	return ptr;
}


void *thermalDataThread( void *ptr ) {

	nice( -10 );

	// NOTE: Misc extra processing has been moved to this shortest parallel 
	//       worker thread for THREAD LOAD BALANCING reasons

	ThreadData *td = (ThreadData *)ptr;
	ProcessedThermalFrame *ptf = td->ptf;
	Rect roi;
	roi.x = roi.y = 0; // x and y are always ZERO

	// Reuse scratch memory without having to copy

	Rect subFrameROI = Rect(0, FIXED_TC_HEIGHT, FIXED_TC_WIDTH, FIXED_TC_HEIGHT);

       	TS( int64_t thermMicros; )

	Mat *thermalFramePtr; // Parallel Histogram processing
	Mat  copy;

	int lastX = -1, lastY = -1;

	while ( td->running ) 
	{
		incAndWait(&WorkerMutex, &td->thermalState, 0); /**************** BLOCKING - Tier 0 */
		if ( ! td->running ) break; // after keypress

        	TS( thermMicros = currentTimeMicros(); )

		thermalFrame = Mat( *(threadData.rawFrame), subFrameROI ).clone();
		if ( RotateDisplay ) { rotate( thermalFrame, thermalFrame, rotateFlags[ RotateDisplay ] ); }

		if ( WINDOW_IMAGE != controls.windowFormat ) {
			if ( Use_Histogram ) {
				// Apply historgram to copy, not original
				// Changing thermalFrame will break subsequent call to processThermalFrame()
#if USE_CLONE
				copy = thermalFrame.clone();
#else
				thermalFrame.copyTo( copy );
#endif
				histogramWrapper( thermalFrame, copy );
				thermalFramePtr = &copy;
			} else {
				thermalFramePtr = &thermalFrame;
			}

			// Convert the composited image copy to RGB
			// cv::cvtColor (InputArray src, OutputArray dst, int code, int dstCn=0)
			cvtColor ( *thermalFramePtr, rgbThermalFrame, COLOR_YUV2BGR_YUYV, CVT_CHAN_FLAG );

			// Set contrast
			// void cv::convertScaleAbs (InputArray src, OutputArray dst, double alpha=1, double beta=0)
			// dst(I)=saturate\_cast<uchar>(|src(I)∗alpha+beta|)
			if ( 1.0 != controls.alpha ) {
				// Optimization:  Only set alpha/contrast if it is NOT 1.0
				convertScaleAbs(rgbThermalFrame, rgbThermalFrame, controls.alpha); // Contrast
			}
		}

		ASSERT(( TC_WIDTH  == thermalFrame.cols ));
		ASSERT(( TC_HEIGHT == thermalFrame.rows ));

		TS( td->thermMicros += ( currentTimeMicros() - thermMicros ); ) // track relative benchmarks

		incAndWait(&WorkerMutex, &td->thermalState, 1); /**************** BLOCKING - Tier 1 */

        	TS( thermMicros = currentTimeMicros(); )

	//	try {
			processThermalFrame( ptf, &thermalFrame );
	//	} catch (...) {
	//		printf("\t%s(%d) - processThermalFrame exception\n", __func__, __LINE__);
	//		fflush(stdout); fflush(stderr);
	//	}

		if ( controls.hud != HUD_NO_DRAWINGS )
		{
			// HAS TO BE DONE AFTER Min/Max/Avg has been calculated !!!
			// THREAD LOAD BALANCING - Grab ruler row and column temps and points in parallel

			minF       = ptf->minPixel.celsius;
			maxF       = ptf->maxPixel.celsius;
			avgF       = ptf->avg.celsius;

			// BOUND_RULER_MAX, BOUND_RULER_MIN
			switch ( rulerBoundFlag ) {
				case BOUND_PROPORTIONAL:
					// Proportional plots - Baseline offset from middle
					// No outlier clipping
					rulerBoundMinKelvin = (ptf->avg.kelvin - ptf->min.kelvin); // No Bounds
					rulerBoundMaxKelvin = (ptf->max.kelvin - ptf->avg.kelvin); // No Bounds
					maxBound = minBound = 1;
					break;

				case BOUND_EQUAL_OUTLIER_CLIPPING:
					// Equal plots - Baseline in the middle
					// Outlier clipping
					rulerBoundMinKelvin = rulerBoundMaxKelvin = 
						MIN( (ptf->avg.kelvin - ptf->min.kelvin),
						     (ptf->max.kelvin - ptf->avg.kelvin) );
					maxBound = minBound = 1;
					break;

				case BOUND_BELOW_AVG_CLIPPING:
					// Below average clipping
					rulerBoundMaxKelvin = ptf->max.kelvin - ptf->avg.kelvin;
					rulerBoundMinKelvin = 0;
					maxBound = 1;
					minBound = 0;
					break;

				case BOUND_ABOVE_AVG_CLIPPING:
					// Above average clipping
					rulerBoundMaxKelvin = 0;
					rulerBoundMinKelvin = ptf->avg.kelvin - ptf->min.kelvin;
					maxBound = 0;
					minBound = 1;

				default: break;
			}

			float kelvinFactor = getRulerKelvinFactor( FIXED_TC_HEIGHT - 1 );

			minAvgOffsetF = kelvinFactor * rulerBoundMinKelvin; // Should be non-negative
			maxAvgOffsetF = kelvinFactor * rulerBoundMaxKelvin; // Should be non-negative

			thresholdF = controls.threshold.celsius; // Threshold is a delta C or F

			drawMin    = ( CorF( minF ) < (CorF( avgF ) - thresholdF) );
			drawMax    = ( CorF( maxF ) > (CorF( avgF ) + thresholdF) );

			// Convert threshold in Celsius or Fahrenheit to relative threshold in Kelvin
			float kRange = ( ptf->max.kelvin - ptf->min.kelvin );

			//printf("kRange %f, cRange %f, fRange %f\n", kRange, cRange, fRange );

			// Eliminate @ 450 kelvin2Celsius() calls / frame from getRuler[X|Y]Points()
			// Eliminate @ 450 CorF()           calls / frame from getRuler[X|Y]Points()
                        if ( Use_Celsius ) {
				// Interpret threshold as Celsius
				float cRange = ( ptf->max.celsius - ptf->min.celsius );
				float kDelta = ( ( thresholdF * kRange ) / cRange );

				minusThresholdKelvin = ( ptf->avg.kelvin - kDelta );
				plusThresholdKelvin  = ( ptf->avg.kelvin + kDelta );
			} else {
				// Interpret threshold as Fahrenheit
				// Range has to be calculated it like destination units (Apples - Apples)
				float fRange = celsius2Fahr( ptf->max.celsius ) - 
					       celsius2Fahr( ptf->min.celsius );
				float kDelta = ( ( thresholdF * kRange ) / fRange );

				minusThresholdKelvin = ( ptf->avg.kelvin - kDelta );
				plusThresholdKelvin  = ( ptf->avg.kelvin + kDelta );
			}

			int anchorY, anchorX;

			if ( Max_Ruler_Thickness ) {
				float minMaxRange = rulerBoundMaxKelvin + rulerBoundMinKelvin;
				anchorY = ( TC_HEIGHT - 1 ) * rulerBoundMaxKelvin / minMaxRange;
				anchorX = ( TC_WIDTH  - 1 ) * rulerBoundMinKelvin / minMaxRange;
			} else {
				anchorX = rulersX;
				anchorY = rulersY;
			}

			if ( Rulers_Both_Horiz ) {
				if ( RERENDER_OPTIMIZATION || (lastY != rulersY) ) {
					rulerXKelvinFactor = getRulerKelvinFactor(
							( Max_Ruler_Thickness ) ?
							( TC_HEIGHT       - 1 ) :  /* reversed from getRulerYPoints() */
							( FIXED_TC_HEIGHT - 1 ) ); /* same as getRulerYPoints() */
					lastY = rulersY;
					getRulerXPoints( &thermalFrame, kelvinX,
						rulerX0Points, rulerX1Points, 0, rulersY, TC_WIDTH, anchorY );
										/* x=0 to TC_WIDTH */
				}
			}

			if ( Rulers_Both_Vert ) {
				if ( RERENDER_OPTIMIZATION || (lastX != rulersX) ) {
					rulerYKelvinFactor = getRulerKelvinFactor(
							( Max_Ruler_Thickness ) ?
							( TC_WIDTH        - 1 ) :  /* reversed from getRulerXPoints() */
							( FIXED_TC_HEIGHT - 1 ) ); /* same as getRulerXPoints() */
					lastX = rulersX;
					getRulerYPoints( &thermalFrame, kelvinY,
						rulerY0Points, rulerY1Points, rulersX, 0, TC_HEIGHT, anchorX );
										/* y=0 to TC_HEIGHT */
				}
			}

			if ( Rulers_Both_Horiz_Vert ) {
				// CALCULATE drawMinMaxRulerLines() globals here to be used by multiple threads ...

				float centerKelvin = user_CENTER_OF_RULER_INDEX->kelvin;

				anchorAvgOffsetF = kelvinFactor * ( centerKelvin - ptf->avg.kelvin );

				copAnchorX        = centerOfPixelNODIV( rulersX );
				copAnchorY        = centerOfPixelNODIV( rulersY );

				copAnchorX_Offset = centerOfPixelNODIV( round( rulersX + anchorAvgOffsetF ) );
				copAnchorY_Offset = centerOfPixelNODIV( round( rulersY - anchorAvgOffsetF ) );

				POINT( copAnchorXYPoint, copAnchorX, copAnchorY );
		
				copZero	   = centerOfPixelNODIV( 0 );	// both x and y
				copWidth   = centerOfPixelNoDiv( ( TC_WIDTH  - 1 ) );
				copHeight  = centerOfPixelNoDiv( ( TC_HEIGHT - 1 ) );

				{ // Precalculate ruler line points
					int x1, x2, y1, y2;

					// Fullscreen unlocks ruler from plot
					if ( Max_Ruler_Thickness ) { 
						y1 = copHeight;
						y2 = copZero;
						x1 = copZero;
						x2 = copWidth;
					} else {
						y1 = centerOfPixelNoDiv( round( rulersY + minAvgOffsetF ) );
						y2 = centerOfPixelNoDiv( round( rulersY - maxAvgOffsetF ) );
						x1 = centerOfPixelNoDiv( round( rulersX - minAvgOffsetF ) );
						x2 = centerOfPixelNoDiv( round( rulersX + maxAvgOffsetF ) );
					}

					POINT( hMin_src, copZero,  y1 );
					POINT( hMin_dst, copWidth, y1 );

					POINT( hMax_src, copZero,  y2 );
					POINT( hMax_dst, copWidth, y2 );

					POINT( vMin_src, x1, copZero );
					POINT( vMin_dst, x1, copHeight );

					POINT( vMax_src, x2, copZero );
					POINT( vMax_dst, x2, copHeight );
				}

				// Black baseline start, middle(b & c), end points
				POINT( hBase_src, copZero,    copAnchorY );
				POINT( hBase_dst, copWidth,   copAnchorY );
				POINT( vBase_src, copAnchorX, copZero );
				POINT( vBase_dst, copAnchorX, copHeight );

// Center flicker control on 1X scale
#define plus_FLICKER    + MarkerSize 
#define minus_FLICKER   - MarkerSize 
#define PLUS_DEBUG     // + MarkerSize

				// Create hole where perpendicular center indicators would overwrite baselines
				// Not doing so could cause flickers
				// This only applies to when both rulers are active at the same time
				// Compare scaled Y to scaled Y to determine where the hole is
				// Check if the plot line is above or below the center marker
				// and adjust _b and _c accordingly to NOT overwrite arrows
				//
				// NOTE: All 4 permutations are required to handle all corner cases
				// 		AA AB BB BA
				// with PORTRATE/LANDSCALE and PROPORTIONAL/FULL thickness rullers
				// Prevent center point flicker @ 1X and 2X scale
				if ( rulerX0Points[ rulersX ].y < copAnchorY )
				{
					POINT( vPerp, copAnchorX, (copAnchorY minus_FLICKER ) );

					POINT( vBase_b, ( rulerX0Points[ rulersX ].x PLUS_DEBUG ),
							( rulerX0Points[ rulersX ].y minus_FLICKER ) );
					POINT( vBase_c, ( copAnchorX PLUS_DEBUG ), ( copAnchorY plus_FLICKER ) );
				} else {
					POINT( vPerp, copAnchorX, (copAnchorY plus_FLICKER ) );

					POINT( vBase_b, ( copAnchorX PLUS_DEBUG ), ( copAnchorY minus_FLICKER ) );
					POINT( vBase_c, ( rulerX0Points[ rulersX ].x PLUS_DEBUG ),
							( rulerX0Points[ rulersX ].y plus_FLICKER ) );
				}

				if ( Rulers_Vert ) {
					POINT( vBase_b, ( copAnchorX PLUS_DEBUG ), ( copAnchorY minus_FLICKER ) );
					POINT( vBase_c, ( copAnchorX PLUS_DEBUG ), ( copAnchorY plus_FLICKER  ) );
				}

				if ( rulerY0Points[ rulersY ].x < copAnchorX )
				{
					POINT( hPerp, (copAnchorX minus_FLICKER), copAnchorY );

					POINT( hBase_b, ( rulerY0Points[ rulersY ].x minus_FLICKER ),
							( rulerY0Points[ rulersY ].y PLUS_DEBUG ) );
					POINT( hBase_c, ( copAnchorX plus_FLICKER ), ( copAnchorY PLUS_DEBUG ) );
				} else {
					POINT( hPerp, (copAnchorX plus_FLICKER ), copAnchorY );

					POINT( hBase_b, ( copAnchorX minus_FLICKER ), ( copAnchorY PLUS_DEBUG ) );
					POINT( hBase_c, ( rulerY0Points[ rulersY ].x plus_FLICKER),
							( rulerY0Points[ rulersY ].y PLUS_DEBUG ) );
				}

				if ( Rulers_Horiz ) {
					POINT( hBase_b, ( copAnchorX minus_FLICKER ), ( copAnchorY PLUS_DEBUG ) );
					POINT( hBase_c, ( copAnchorX plus_FLICKER  ), ( copAnchorY PLUS_DEBUG ) );
				}

				// Scale size of arrow head based on center perpendicular indicator line length
				// to counteract OpenCV's growth algorithm which GROWS TOO FAST
				// Perpendicular to horizontal, use width length
				arrowScaleV = (1.0 - (fabs( copAnchorX -
						rulerX0Points[ rulersX ].x) / SCALED_TC_WIDTH)) * 0.025;

				// Perpendicular to vertical, use height length
				arrowScaleH = (1.0 - (fabs( copAnchorY -
						rulerY0Points[ rulersY ].y) / SCALED_TC_HEIGHT))  * 0.025;

// if ( RULERS_BOTH == rulersOn ) { assert(( kelvinX[ rulersX ] == kelvinY[ rulersY ] )); }

				float kTmp;
				kTmp = kelvinX[ rulersX ];
				vScalar = ((kTmp < minusThresholdKelvin) ? &RULER_MIN_COLOR :
				           (kTmp > plusThresholdKelvin)  ? &RULER_MAX_COLOR :
								           &RULER_MID_COLOR );

				kTmp = kelvinY[ rulersY ];
				hScalar = ((kTmp < minusThresholdKelvin) ? &RULER_MIN_COLOR :
					   (kTmp > plusThresholdKelvin)  ? &RULER_MAX_COLOR :
								           &RULER_MID_COLOR );
			}

			if ( rulersOn )
			{
				// Offset center of ruler label
				if ( user_CENTER_OF_RULER_INDEX->active ) {
			                int ht     = 1.5 * chTextHeight;
					int delta  = ((user_CENTER_OF_RULER_INDEX->labelLoc.y <= ((SCALED_TC_HEIGHT/2) + ht)) ? ht : - ht);

					user_CENTER_OF_RULER_INDEX->labelLoc.y   += delta; 
					user_CENTER_OF_RULER_INDEX->labelwDLoc.y += delta; 
				}
			}

		}

		// THREAD LOAD BALANCING - Prepare scaled pre-processed rendering areas

		if ( RERENDER_OPTIMIZATION ) {
			// THREAD LOAD BALANCING - Pre-render Colormap Scale into its own RGB image
			// cmapScale will change based on scaledSFHeight as well as CV_TYPE, thus needs to be remade
		//	try { 
				roi.width     = ColorScaleWidth;
			       	roi.height    = controls.scaledSFHeight;
				td->cmapScale = td->maxCmapScale( roi ); // Reuse memory
				drawCmapScale( td->cmapScale, td->maxCmapScale.cols ); 
		//	} catch (...) {
		//		printf("\t%s(%d) - cmapScale exception\n", __func__, __LINE__);
		//		fflush(stdout); fflush(stderr);
		//	}
		}
		
		TS( td->thermMicros += ( currentTimeMicros() - thermMicros ); ) // track relative benchmarks

        	incAndWait(&WorkerMutex, &td->thermalState, 2); /**************** BLOCKING - Tier 2 */

        	TS( thermMicros = currentTimeMicros(); )

		// Do not display graphics over video
		if ( controls.hud != HUD_NO_DRAWINGS )
		{
			if ( Rulers_Both_Horiz_Vert ) 
			{
			//	try {
					drawRulerPlot( *threadData.rgbFrame, 
						rulerX0Points, rulerX1Points, scalarX, TC_WIDTH, 1 );
			//	} catch (...) {
			//		printf("\t%s(%d) - drawRulersLines exception\n", __func__, __LINE__);
			//		fflush(stdout); fflush(stderr);
			//	}
			}
	//		try {
				drawUserAndRulerTemps( 1 );
	//		} catch (...) {
	//			printf("\t%s(%d) - drawUserAndRulerTemps exception\n", __func__, __LINE__);
	//			fflush(stdout); fflush(stderr);
	//		}
		}

		if ( controls.hud != HUD_NO_DRAWINGS ) {
		//	try {
				// Display floating max temp (while keeping text visible @ display boundaries)
				if ( drawMax ) {
					drawTempMarker( *threadData.rgbFrame, ptf->max, RED );
				}
		//	} catch (...) {
		//	}
		}

		TS( td->thermMicros += ( currentTimeMicros() - thermMicros ); ) // track relative benchmarks

//printf("\t%s(%d) - Halted %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

		incAndWait(&WorkerMutex, &td->thermalState, 3); /**************** BLOCKING - Tier 3 */

	} // end while( 1 )

	printf("%s(%d) - exiting\n", __func__, __LINE__); fflush(stdout);

	return ptr;
}


int openCamera(VideoCapture &cap, char *camera, char *appName) {

  	cap = VideoCapture(camera, CAP_V4L); // CAP_V4L dictates frame buffer size and format 

	printf("%s(%d): Opening cameara %s\n", __func__,__LINE__, camera); fflush(stdout);
 
	// V4L - Video for Linux
	// RGB needed for thermal data
	// Convert to COLOR_YUV2BGR_YUYV for playback
	cap.set(CAP_PROP_CONVERT_RGB, 0.0); 
	cap.set(CAP_PROP_MONOCHROME,  1.0); 
	// TODO-FIXME - Investigate CAP_PROP_FORMAT and -1 for raw
	// Can it be set CV_16U, 1 channel of unsigned short

	// Check if camera opened successfully
	if (!cap.isOpened()) {
		printf( "\nError opening video stream(%s)\n\tCalling: v4l2-ctl --list-devices\n\n",
			camera); fflush(stdout);
		system( "v4l2-ctl --list-devices\n" );
		printf( "Usage: \n\t%s -d n (where 'n' is the number of the desired video camera)\n\n", appName );
		return -1;
	}

	printf("%s(%d): Opened camera %s\n", __func__,__LINE__, camera); fflush(stdout);
 
// gamma(300.00) sharp(50.00) temp(4600.00) hue(0.00) gain(-1.00) contrast(50.00) bright(0.00) exposure(166.00) saturation(64)
// V4L2
// fps(25.00) foc(-1.00) br(-1.00) fmt(-1.00) gamma(300.00) sharp(50.00) temp(4600.00) hue(0.00) gain(-1.00) contrast(50.00) bright(0.00) exposure(166.00) saturation(64.00)

	//cap.set(CAP_PROP_HUE, 0);
	//cap.set(CAP_PROP_CONTRAST, 50);
	//cap.set(CAP_PROP_SATURATION, 64);

// Need 4.8 OpenCV for hardware acceleration
//	cap.get(cv::CAP_PROP_HW_ACCELERATION),
	cout << "Backend: " << cap.getBackendName() << endl;
	printf("fps(%.2f) mode(%.2f) foc(%.2f) br(%.2f) fmt(%.2f) gamma(%.2f) sharp(%.2f) temp(%.2f) hue(%.2f) gain(%.2f) contrast(%.2f) bright(%.2f) exposure(%.2f) saturation(%.2f)\n", 
		cap.get(CAP_PROP_FPS),
		cap.get(CAP_PROP_MODE),
		cap.get(CAP_PROP_FOCUS),
		cap.get(CAP_PROP_BITRATE),
		cap.get(CAP_PROP_CODEC_PIXEL_FORMAT),
		cap.get(CAP_PROP_GAMMA),
		cap.get(CAP_PROP_SHARPNESS),
		cap.get(CAP_PROP_TEMPERATURE),
		cap.get(CAP_PROP_HUE),
		cap.get(CAP_PROP_GAIN),
		cap.get(CAP_PROP_CONTRAST),
		cap.get(CAP_PROP_BRIGHTNESS),
		cap.get(CAP_PROP_EXPOSURE),
		cap.get(CAP_PROP_SATURATION)
		);
	return 1;
}

// C/C++ implementation of capturing live stream from a TC001 thermal camera 
// Display video stream with annotations, record video stream, snapshot of video stream
//
// From: v4l2-ctl --all
//
//	Pixel Format      : 'YUYV' (YUYV 4:2:2)
//	Colorspace        : sRGB
//	Transfer Function : Rec. 709
//	YCbCr/HSV Encoding: ITU-R 601
//	Width/Height      : 256/384
//	Bytes per Line    : 512      #    512 =       256 * 2 Bytes/pixel
//	Size Image        : 196608   # 196608 = 384 * 256 * 2 Bites/pixel
//	Frames per second: 25.000 (25/1)

double perfMax    = -1; // Keep track of longest thread duration
double waitMicros = -1; // Keep thread wait times

int parseArgs( int argc, char *argv[], char *camera, VideoCapture &cap ) {
	int inputNotFound = -1;
	int i = 1;
	for ( ; i < argc; i++ ) {
		if ( ! strcmp( argv[i], "-scale") ) {
			MyScale               = abs( atoi( argv[ i + 1 ] ) ) % MAX_SCALE_STEPS;
			i++;
		} else if ( ! strcmp( argv[i], "-font") ) {
			UserFont  = abs( atoi( argv[ i + 1 ] ) ) % MAX_USER_FONT;
			i++;
		} else if ( ! strcmp( argv[i], "-cmap") ) {
			controls.cmapCurrent  = abs( atoi( argv[ i + 1 ] ) ) % MAX_CMAPS;
			i++;
		} else if ( ! strcmp( argv[i], "-fps") ) {
			offline_fps  = abs( atoi( argv[ i + 1 ] ) );
			i++;
		} else if ( ! strcmp( argv[i], "-clip") ) {
			rulerBoundFlag  = abs( atoi( argv[ i + 1 ] ) ) % BOUND_MAX_MOD;
			i++;
		} else if ( ! strcmp( argv[i], "-thick") ) {
			rulerThickness  = 5 - (abs( atoi( argv[ i + 1 ] ) ) % MIN_RULER_THICKNESS);
			i++;
		} else if ( ! strcmp( argv[i], "-rulers") ) {
			rulersOn  = (RulerModes)(abs( atoi( argv[ i + 1 ] ) ) % (int)RULERS_MAX_MOD);// Rollover count, not actual ruler mode
			i++;
		} else if ( ! strcmp( argv[i], "-rotate") ) {
			RotateDisplay = abs( atoi( argv[ i + 1 ] ) );
			RotateDisplay = DECODE_ROTATION( RotateDisplay );
			if ( 3 < RotateDisplay ) {
				RotateDisplay = 0;
			}
			i++;
		} else if ( ! strcmp( argv[i], "-f"    ) ||
			    ! strcmp( argv[i], "-file" ) ) {
			threadData.source     = "File";
			threadData.inputFile  = argv[i + 1];
			printf("%s(%d): Opening file %s\n", __func__,__LINE__, threadData.inputFile); fflush(stdout);
			inputNotFound = 0;
			i++;
		} else if ( ! strcmp( argv[i], "-d"      ) ||
			    ! strcmp( argv[i], "-device" ) ) {
			threadData.source    = "Camera";
			sprintf( camera, "/dev/video%d", abs( atoi(argv[i + 1]) ) ); // Default is camera 0
			threadData.inputFile = 0;
			printf("%s(%d): Opening camera %s\n", __func__,__LINE__, camera ); fflush(stdout);
			if ( openCamera( cap, camera, argv[0] ) < 0 ) {
				return -1;
			}
			inputNotFound = 0;
			i++;
////printf("device(%s)\n", camera);
		} else {
			printf("Unknown argv[%d] (%s)\n", i, argv[i] );
			return -1;
		}
	}
	return inputNotFound;
}

int mainPrivate (int argc, char *argv[]) {

	nice( -5 );

	setHeightWidth(); // Configure rotation

	ProcessedThermalFrame ptf;

	// Mat frame;
	// cv::Mat::Mat(int rows, int cols, int type)
	Mat rawFrame(FIXED_TC_HEIGHT * 2, FIXED_TC_WIDTH, CV_8UC2);   // Used for FreezeFrame and to write .raw files
	Mat tFrame_0(FIXED_TC_HEIGHT * 2, FIXED_TC_WIDTH, CV_8UC2);   // Used to facilitate camera connection recovery
	Mat tFrame_1(FIXED_TC_HEIGHT * 2, FIXED_TC_WIDTH, CV_8UC2);   // Used to facilitate camera connection recovery
	Mat rgbFrameOrig; // Scaled, rotated (not composited) RGB frame - COLOR_YUV2BR_YUYV
	Mat rgbFrame;     // Scaled, rotated  and composited  RGB frame - COLOR_YUV2BR_YUYV

	int maxDim = max( MAX_SCALE_STEPS * FIXED_TC_WIDTH,
		          MAX_SCALE_STEPS * FIXED_TC_HEIGHT );

	// Avoid reallocating matrix memory, alloc largest and then use ROI() sub-rectangle
	// Valgrind complains about maxCmapScale constructor so calling zeros()
	threadData.maxCmapScale   = Mat::zeros( maxDim, maxDim, CV_8UC2 ); // Scalable Colormap Gradient Scale

	// This graphics plane is XOR shared between HUD and HELP
	maxDim = max( max( longestHUDSize.width,  longestHUDSize.height ), 
	              max( longestHelpSize.width, longestHelpSize.height ) );
	threadData.maxRgbHud      = Mat::zeros( maxDim, maxDim, CV_8UC3 ); // Scalable HUD

	threadData.source         = "";
	threadData.inputFile      = 0; // Flag !0 for file, 0 for camera
	threadData.FreezeFrame    = 0;
	threadData.ptf            = &ptf;
	threadData.rgbFrame       = &rgbFrame;
	threadData.rgbFrameOrig   = &rgbFrameOrig;
	threadData.rawFrame       = &rawFrame;
	threadData.lostVideo      = 0;
	threadData.running        = 1;

	setDefaults( &ptf );

	// (code that uses SSE4.2, AVX/AVX2, and other instructions on the platforms that support it)
	cv::setUseOptimized( true ); // Make sure hardware optimization is enabled

	//VideoCapture cap(camera, CAP_V4L); // CAP_V4L dictates frame buffer size and format 
	VideoCapture cap; // CAP_V4L dictates frame buffer size and format 

	// Create a VideoCapture object and use camera to capture the video
	// or use a raw intput file in offline mode
	char camera[128];
	if ( parseArgs( argc, argv, camera, cap ) ) {
		printKeyBindings();
		return -1;
	}

	printKeyBindings();

	newWindow( &ptf ); // Should not start in fullscreen

	setMouseCallback( WINDOW_NAME, onMouseCallback, &threadData );

	initIncWaitMutex( &WorkerMutex,     MyThreadStates,   ARRAY_COUNT(MyThreadStates),   0, NUMBER_OF_CLIENTS );
	initIncWaitMutex( &BothPlotMutex,   BothPlotStates,   ARRAY_COUNT(BothPlotStates),   0, 2);
	initIncWaitMutex( &CmapPlotMutex,   CmapPlotStates,   ARRAY_COUNT(CmapPlotStates),   0, 3);
	initIncWaitMutex( &B4TempPlotMutex, B4TempPlotStates, ARRAY_COUNT(B4TempPlotStates), 0, 3);

	long numberOfRawFrames = 0;
	int  released          = 0;

	cv::setUseOptimized( true ); // Make sure hardware optimization is enabled

	// Read keyboard input from launching terminal or command file piped into stdin
	pthread_t stdinThread;   pthread_create( &stdinThread,   NULL, stdinDataThread,   (void*) &threadData );
	pthread_t imageThread;   pthread_create( &imageThread,   NULL, imageDataThread,   (void*) &threadData );
	pthread_t thermalThread; pthread_create( &thermalThread, NULL, thermalDataThread, (void*) &threadData );

	RenderData  rdMain;
	RenderData *rd = &rdMain;
	Rect        roi;
	roi.x = roi.y = 0; // x and y are always ZERO

	// Main loop: process frames, stdin, mouse and keyboard events
#define FRAME_MICROS (MICROS_PER_SECOND / offline_fps)
	int64_t loopMicros;
	int64_t frame_micros = FRAME_MICROS; // Calc once, use many
	int lastGoodFrame = 0;	

	while ( threadData.running ) {

		int64_t readMicros = loopMicros = currentTimeMicros();

		if ( ! threadData.FreezeFrame ) {
			if        ( ! threadData.inputFile ) {
				// Try to read from camera
				// Jump into FreezeFrame mode on camera read issues
				// When coming back out of FreezeFrame, try to re-establish camera connection

				if ( threadData.lostVideo ) {
					released = 0;
					if ( openCamera(cap, camera, argv[0]) < 0 ) {
						printf("Open camera(%s) failed, switching to freeze frame\n", camera);
						threadData.lostVideo   = 1;
						threadData.FreezeFrame = 1;
					} else {
						// TODO - FIXME - Regress long disconnects and mouse actions
						// onMouseCallback was lost during long dissconnect
						//setMouseCallback( WINDOW_NAME, onMouseCallback, &threadData );
						threadData.lostVideo = 0;
					}
				}

				// Wait for user to fix camera connection and exit FreeseFrame
				if ( ! threadData.lostVideo && ! threadData.FreezeFrame ) {
					// Capture frame-by-frame from video camera
					// Don't overwrite last good frame
					// Avoid expensive deep copy clone() by double buffering
				
					int readError;
					if (0 == lastGoodFrame) {
						cap >> tFrame_1;
						readError     = tFrame_1.empty();
						lastGoodFrame = readError ?        0 :        1;
						rawFrame      = readError ? tFrame_0 : tFrame_1;
					} else {
						cap >> tFrame_0;
						readError     = tFrame_0.empty();
						lastGoodFrame = readError ?        1 :        0;
						rawFrame      = readError ? tFrame_1 : tFrame_0;
					}

					if ( readError ) {
						printf("Frame is empty, swithing to freeze frame\n");
						if ( ! released ) {
							cap.release();
							released = 1;
						}
						threadData.lostVideo   = 1;
						threadData.FreezeFrame = 1;
					}
				}

				// Display source error in HUD display if camera connection was lost
				threadData.source = threadData.lostVideo ? "ERROR" : "Camera";

			} else if ( 1 != numberOfRawFrames ) {
				// Do not repetitiously re-read a single raw frame snapshot
				rawReadFp = readRawFrame(rawFrame, threadData.inputFile, rawReadFp, 1, &numberOfRawFrames);
				if (0 == rawReadFp || 0 == numberOfRawFrames) { 
					break;
				}
			}

			// If the frame is empty, break immediately
			if ( rawFrame.empty() ) {
				printf("Frame is empty, exiting\n");
				break;
			}
		}

		int64_t mainMicros = currentTimeMicros();

		threadData.readMicros += ( mainMicros - readMicros ); // track relative benchmarks

		// Assignment is NOT a copy constructor

		// Grab individual Image and Thermal frames from double high single frame

		// Break into individual sub-frames in worker threads
		// Layout code will recomposite 1 or both sub-frames in 1:4 layout formats
		// Support camera with USB port facing up/down/left or right
		// Portrait rotation has most overhead so do in parallel
		// 	0.06 ms -  No rotation - 1X
		// 	0.24 ms -  90 rotation - 4X - PORTRAIT
		// 	0.12 ms - 180 rotation - 2X
		// 	0.18 ms - 270 rotation - 3X - PORTRAIT


//dumpFrameInfo(&rawFrame);
// dumpTypes();

		// image and thermal frames may get rotated in sync with TC_WIDTH and TC_HEIGHT

		// rawFrame does not get rotated
		// end-start(196608) channels(2) elemSize(2:1) total(98304) cols(256) rows(384) type(8) depth(0)
		ASSERT(( rawFrame.data == rawFrame.datastart ));
		ASSERT(( ((unsigned long)rawFrame.dataend - (unsigned long)rawFrame.datastart) == (unsigned long)(2*rawFrame.total()) ));
		ASSERT(( ((unsigned long)rawFrame.cols * (unsigned long)rawFrame.rows) == (unsigned long)rawFrame.total() ));
		ASSERT((                     2 == rawFrame.channels() ));
		ASSERT((                     2 == rawFrame.elemSize() ));
		ASSERT((                     1 == rawFrame.elemSize1() ));
		ASSERT(((size_t)(FIXED_TC_WIDTH*FIXED_TC_HEIGHT*2) == rawFrame.total() ));
		ASSERT((  FIXED_TC_WIDTH       == rawFrame.cols ));
		ASSERT(( (FIXED_TC_HEIGHT * 2) == rawFrame.rows )); // 2 frames
		ASSERT((  CV_8UC2              == rawFrame.type() )); // 2 channels of uint8
		ASSERT((                     0 == rawFrame.depth() ));

		/***************************************************************************************
		 * Order of operations:
		 *     Capture frame in non-RGB CV_8UC2 2-bytes/pixel format
		 *         Frame consistes of 2 sub-frames, top(image) and bottom(heat data)
		 *     processThermalFrame() to collect thermal data
		 *     cvtColor() - convert image frame to RGB COLOR_YUV2BR_YUYV for rendering
		 *     cvtScaleAbs() - set Alpha/Contrast
		 *     Scale/interpolate/resize frame (expensive slow call)
		 *     Add blur if set
		 *     Apply currently selected colormap
		 *     Render data and drawings
		 *     Show frame
		 *     Optionally record frame
		 *     Handle key and mouse -> changes config / snapshot
		 *     Wash, rinse, repeat
		 *
		 *     Per Frame Thread Model: 
		 *
		 *  init config
		 *  open camera or offline raw file
		 *  launch stdin and worker threads
		 *  loop [ main thread -> | 2 worker + main threads[sync'd tiers] | -> main thread   ]
		 *       [ read frame     |  process sub-frame(s) across 4 tiers  |    show image    ]
		 *       [ re-use frame                                                print stats   ]
		 *       [                                                             record        ]
		 *       [                [   !!!!! CONSTANT CONFIG REGION !!!!!  ]    key & mouse   ]
		 *       [                                                             config change ]
		 *       [                                                             snapshot      ]
		 *       [                                                             repeat/break  ]
		 *       [                        stdin thread until EOF               shutdown      ]
		 *
		***************************************************************************************/

		// Prepare RGB frame in one thread while harvesting thermal data the other thread in parallel
		// Added extra tiers to worker threads

		// Blocked on  0      Wait for startup threads to sync and/or wait for next frame to process
		// - running   0 -> 1 Split, rotate, histogram, contrast, image and thermal sub frames
		// Blocked on  1      Wait for all threads to finish tier 0 workloads
		// - running   1 -> 2 Composite scaled RGB layout Frame and Process Thermnal Frame data
		// Blocked on  2      Wait for all threads to finish tier 1 workloads
		// - running   2 -> 3 Composite pre-rended offscreen graphic collection into Scaled RGB layout Frame
		// Blocked on  3      Wait for all threads to finish tier 2 workloads
		//             3 -> 0 Frame ready, show/record frame, process key, mouse, config changes, snapshot
		//                    Grab next frame when appropriate or [re]use last good frame
		//                    Wash, rinse, repeat

		// Make this section of the main thread a sync'd peer worker thread to minimize context switches
		{
			// Sync all threads including this main thread at the next frame
			incAndWait(&WorkerMutex, &rd->renderState, 0); /**************** BLOCKING - Tier 0 */
			if ( ! threadData.running ) break; // after keypress

			/* split and rotate imageFrame and thermalFrame */

			TS( int64_t renderMicros = currentTimeMicros(); )

			// Calc FPS
			controls.frameCounter++;

			int64_t now    = currentTimeMillis();
			double seconds = (double)(now - controls.startMills) / (double)1000.0;
			controls.fps = (seconds < 1) ? 1 : (double)controls.frameCounter / seconds;
			if ( seconds > (10 * 60.0) ) {
				// Periodically reset counters
				resetFrameCounter();
			}

			TS( rd->renderMicros += ( currentTimeMicros() - renderMicros ); ) // track relative benchmarks

			// Sync all threads including this main thread for sub frame processing
			incAndWait(&WorkerMutex, &rd->renderState, 1); /**************** BLOCKING - Tier 1 */

			TS( renderMicros = currentTimeMicros(); )

			// Thermal sub-frame is ready for read at this sync point

			if ( 0 < ptf.userCount ) {
				const char *labelCF = controls.labelCF;

				// Calculate user/ruler temps which will change with location changes
				int userCount = min( ptf.userCount, CENTER_OF_RULER_INDEX );

				// Calculate CENTER_OF_RULER_INDEX in processThermalFrame
	
				for ( int i = 0; i < userCount; i++ ) {
					if ( users[i].active ) {
						getTemperature( &thermalFrame, users[i] );
						scaleTemp( users[i], labelCF );
						calcTempDisplayLocations( users[i] );
					}
				}
			}

			// THREAD LOAD BALANCING - Prepare pre-processed scaled offscreen rendering areas

			// THREAD LOAD BALANCING - Pre-render HUD into its own RBG image
			if ( HUD_ON == controls.hud ) {
				controls.lastHelpScale = -1; // trigger Help to be redrawn
				roi.width  = min( HudWidth,  SCALED_TC_WIDTH  );
				roi.height = min( HudHeight, SCALED_TC_HEIGHT );
				threadData.rgbHUD = threadData.maxRgbHud( roi ); // Reuse memory
				// Update every 25 frames or when config changes to eliminate config change display lag
				if ( threadData.configurationChanged || (0 == (controls.frameCounter % (int)NATIVE_FPS)) ) {
					// Text rendering is expensive because it is rendered twice for background shadow
					drawHUD( &ptf, threadData.rgbHUD, threadData.source, (threadData.lostVideo ? RED: YELLOW) );
				}
			}

			// THREAD LOAD BALANCING - Pre-render HELP into its own RBG image
			else if ( HUD_HELP == controls.hud ) {
				// OPTIMIZATION: Only needs to be drawn once per scale change and HUD_HELP display
				if ( MyScale != controls.lastHelpScale ) {
					roi.width  = min( HelpWidth,  SCALED_TC_WIDTH );
					roi.height = min( HelpHeight, SCALED_TC_HEIGHT );
					threadData.rgbHUD = threadData.maxRgbHud( roi ); // Reuse memory
					drawHelp( threadData.rgbHUD );
				}
			}

			TS( rd->renderMicros += ( currentTimeMicros() - renderMicros ); ) // track relative benchmarks

			// Sync all threads including this main thread for drawing graphics onto scaled rgb layout frame
			incAndWait(&WorkerMutex, &rd->renderState, 2); /**************** BLOCKING - Tier 2 */

			TS( renderMicros = currentTimeMicros(); )

			if ( HUD_NO_DRAWINGS != controls.hud ) {
				drawOneThirdOfTheTemps();
			}

			TS( rd->renderMicros += ( currentTimeMicros() - renderMicros ); ) // track relative benchmarks

			// Sync all threads including this main thread at rendering completion prior to showing
			incAndWait(&WorkerMutex, &rd->renderState, 3); /**************** BLOCKING - Tier 3 */

			threadData.configurationChanged = 0x00;
		}

		// Worker threads should have finished processing sub-frames at this point.
		// imageDataThread maybe recording the rgbFrame while main thread is showing the rgbFrame
		// thermalDataThread and imageDataThead are then on their way to blocking on state 0
		// while this main thread is finishing with the current frame and providing the next frame.
		// Their thread states should be 0, or 3 transitioning to 0

//printf("%s(%d) - Parallel worker threads finished %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout); fflush(stderr);

		TS( int64_t imshowMicros = currentTimeMicros(); )

		// Show the composited frame while imageDataThread maybe recording same composited frame
		imshow( WINDOW_NAME, rgbFrame );

		TS( threadData.imshowMicros += ( currentTimeMicros() - imshowMicros ); ) // track relative benchmarks

		~threadData.cmapScale;
		~threadData.rgbHUD;

		threadData.mainMicros += currentTimeMicros() - mainMicros; // track linear read and processing

#define N_FRAMES (20.0 * 25.0) // Stat log average timings every N_FRAMES

		if (0 == (controls.frameCounter % (long)N_FRAMES)) {

			perfMax    = rdMain.renderMicros + threadData.thermMicros + threadData.imageMicros;
			waitMicros = threadData.mainMicros - 
				( max( threadData.thermMicros, threadData.imageMicros ) + threadData.imshowMicros);

			printf(                // mls read rotation [ render | therm | image ] + show main wait
"main:thrm:img (%.2f : %.2f : %.2f) scale %d fps %.1f read %.2f rot %.2f : [%.2f | %.2f | %.2f] + %.2f %s%.2f%s %s%.2f%s %s %s\n",
				(double)rdMain.renderMicros / perfMax,
				(double)threadData.thermMicros / perfMax,
				(double)threadData.imageMicros / perfMax,
				MyScale, controls.fps, 
				((double)threadData.readMicros   / 1000.0 / N_FRAMES), // mls read rotation [ render
				((double)threadData.rotMicros    / 1000.0 / N_FRAMES), // rotation [ render
				((double)rdMain.renderMicros     / 1000.0 / N_FRAMES), // [ render |       ]
				((double)threadData.thermMicros  / 1000.0 / N_FRAMES), //          | therm | 
				((double)threadData.imageMicros  / 1000.0 / N_FRAMES), //                  | image ]
				((double)threadData.imshowMicros / 1000.0 / N_FRAMES), // + show
				BLUE_STR(),
				((double)threadData.mainMicros / 1000.0 / N_FRAMES),   // + show  main
				RESET_STR(),
				RED_STR(),
				((double)waitMicros / 1000.0 / N_FRAMES),
				RESET_STR(),
				rulerModesStr(rulersOn), 
				hudFormatStr(controls.hud));

			threadData.thermMicros = threadData.imageMicros = threadData.rotMicros    = 0;
			threadData.mainMicros  = threadData.readMicros  = threadData.imshowMicros = 0;
			rdMain.renderMicros = 0;
		}

		// Read key from OpenCV (give it priority over stdin)
		// To put waitKeyEx(1) in it's own thread, 
		//     onMouseCallback's data needs to be mutex protected and synch'd
		//     Would have to have an integrated mouse and keyboard event queue
		//     Even then, there are opencv ISUEES that cause LOCKUP !!!
#if 0
		int c = -1; // 686 FPS without waiting 1ms, doesn't render graphics
#else
		// Use following sleepMicros() to throttle FPS
		int c = (char)waitKeyEx( 1 );
#endif

		// multiple onMouseCallbacks can happen while in waitKeyEx()
		// ONLY PROCESS LAST ONE to save CPU cycles !!!
		if ( 0 < MRD.mouseActive ) {
			// Debounce multiple onMouseCallback results in a single frame
			if ( rulersOn ) {
				normalizeCB( &MRD.x, &MRD.y );
				rulers( &ptf, MRD.x / MyScale,
				 	      MRD.y / MyScale, 0 );
			}
			MRD.mouseActive = 0;
		}

		if (-1 == c) {
			// Read key from stdin (and possibly other sources)
			c = readKeyEvent( &c );
		}

		if ((-1 != c) && ('q' != c)) {
			// Configuration changes happen here in state machine !!!
			processKeypress( c, &ptf, &rgbFrame );
		}

		// Optimization: Don't draw halos while drag scrolling rulers
		drawTempMarkerPtr	= ( rulersOn && (! leftDragOff) ) ? &drawTempMarkerNoHalo : &drawTempMarker;

		// Optimization: Capture state change varaiables ASAP after processKeyPress()
		//               and before worker threads are run
		Rulers_Both		= (( RULERS_BOTH == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_XOR		= (( RULERS_VERTICAL == rulersOn || RULERS_HORIZONTAL == rulersOn) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_Horiz		= (( RULERS_HORIZONTAL == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_Vert		= (( RULERS_VERTICAL   == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_Both_Horiz	= (( RULERS_BOTH == rulersOn || RULERS_HORIZONTAL == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_Both_Vert	= (( RULERS_BOTH == rulersOn || RULERS_VERTICAL   == rulersOn ) && ( HUD_NO_DRAWINGS != controls.hud ));
		Rulers_Both_Horiz_Vert	=  ( Rulers_Both_Horiz || Rulers_Both_Vert );
		Max_Ruler_Thickness	=  ( MAX_RULER_THICKNESS == rulerThickness );
		Horiz_Min_Bound		= ( !(Max_Ruler_Thickness && ((TC_HEIGHT - 1) == rulersY)) );
		Horiz_Max_Bound		= ( !(Max_Ruler_Thickness && (0 == rulersY)) );
		Vert_Min_Bound		= ( !(Max_Ruler_Thickness && (0 == rulersX)) );
		Vert_Max_Bound		= ( !(Max_Ruler_Thickness && ((TC_WIDTH - 1) == rulersX)) );

                BothPlotState		= BothPlotMutex.blockingState;   // Set PlotState to next blocking state
                CmapPlotState		= CmapPlotMutex.blockingState;   // Set PlotState to next blocking state
                B4TempPlotState		= B4TempPlotMutex.blockingState; // Set PlotState to next blocking state

#if 1
		if ( threadData.inputFile || threadData.FreezeFrame ) {
#define SLEEP_CALC_OVERHEAD 100 
			// watiKeyEx( milliseconds ) is too unpredictable (jitter) for accurate timing purposes
			// Use sleepMicros to achieve faster steady state FPS setting
			// Sleep (FPS microseconds - elapsed microseconds) - sleep calc overhead
			loopMicros = frame_micros - (long)(currentTimeMicros() - loopMicros) - SLEEP_CALC_OVERHEAD;
			if ( 0 < loopMicros ) {
				sleepMicros( loopMicros );
			}
		}
#endif

		if ('q' == c) {
			break;
		}

	} // End for loop

// SHUTDOWN:

	// Tell threads to exit
	exitAllThreads();
 
	if ( ! released && ! threadData.inputFile ) {
		// When everything done, release the video capture and write object
		cap.release();
	}

	if ( rawReadFp ) {
		fclose( rawReadFp );
		rawReadFp = 0x00;
	}

	// Closes all the frames
	destroyAllWindows();

	for (int i = 0; i < ARRAY_COUNT(cmaps); i++) {
		cmaps[i]->cmap.matrix = 0x00;
		free( cmaps[i] );
		cmaps[i] = 0x00;
	}

	~rawFrame;
	~rgbFrame;
	~tFrame_0;
	~tFrame_1;

	~red2BlueCmap;
	~red2BlueBlackLCmap;
	~red2BlueWhiteLCmap;
	~red2BlueBlackUCCmap;
	~red2BlueWhiteUCCmap;
	~HSLCmap;
	~inverseHSLCmap;

	return 0;
}

int main (int argc, char *argv[]) {

	nice( -1 );

	Argv0 = argv[0];

	int ret =  mainPrivate( argc, argv );

	printf("Good bye !!!\n");

	return ret;
}
