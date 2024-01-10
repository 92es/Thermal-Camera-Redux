#include <stdio.h>

/****************************************************************************
 * Create stdin test driver key command file for
 *   - Regression testing
 *   - Gprof, Perf and Valgrind testing
 *   - Repeatable user scripts
****************************************************************************/

#define N_CMAPS 37

// keypad arrows ESC(27) + 91 + [65,66,67,68,69]
// 65 up arrow
// 66 down arrow
// 67 right arrow
// 68 left arrow
// 69 center(5)
//      case -105: // keypad up
//      case -103: // keypad down
//      case -106: // keypad left
//      case -104: // keypad right
//      case -99:  // keypad 5 (center)

void esc() { putchar(27); putchar(91); }

void horiz(int n, int dir)  { for ( int i = 0; i < n; i++ ) { esc(); putchar( (dir > 0) ? 67 : 68 ); puts(""); } }
void vert(int n, int dir)   { for ( int i = 0; i < n; i++ ) { esc(); putchar( (dir > 0) ? 65 : 66 ); puts(""); } }
void horizT(int n, int dir) { for ( int i = 0; i < n; i++ ) { esc(); putchar( (dir > 0) ? 67 : 68 ); puts("\nt"); } }
void vertT(int n, int dir)  { for ( int i = 0; i < n; i++ ) { esc(); putchar( (dir > 0) ? 65 : 66 ); puts("\nt"); } }
void home(int n)            { for ( int i = 0; i < n; i++ ) { esc(); putchar( 69 ); puts(""); } }

void ne(int n, int dir)     { for ( int i = 0; i < n; i++ ) { horiz(1,dir); vert( 1,dir);    } }
void nw(int n, int dir)     { for ( int i = 0; i < n; i++ ) { vert( 1,dir); horiz(1,dir*-1); } }

void cmaps(int n, int dir)  { for ( int i = 0; i < n; i++ ) { puts( (dir > 0) ? "j" : "m" ); } }
void scale(int n, int dir)  { for ( int i = 0; i < n; i++ ) { puts( (dir > 0) ? "d" : "c" ); } }
void cont(int n, int dir)   { for ( int i = 0; i < n; i++ ) { puts( (dir > 0) ? "f" : "v" ); } }
void blur(int n, int dir)   { for ( int i = 0; i < n; i++ ) { puts( (dir > 0) ? "a" : "z" ); } }
void interp(int n, int dir) { for ( int i = 0; i < n; i++ ) { puts( (dir > 0) ? "g" : "b" ); } }
void rotate(int n)          { for ( int i = 0; i < n; i++ ) { puts( "8" ); } }
void layout(int n)          { for ( int i = 0; i < n; i++ ) { puts( "w" ); } }
void rulers(int n)          { for ( int i = 0; i < n; i++ ) { puts( "o" ); } }
void reset()                { puts("5");}
void hud()                  { puts("h");}
void hist()                 { puts("y");}
void temp()                 { puts("t");}

// TODO - FIXME - Add all test commands, Fullscreen, FreezeFrame, etc.

int main( int argc, char *argv[] ) {
    rulers(5); // Set both horizontal and vertical rulers
    for (int ro = 0; ro < 4; ro++ ) {
        for (int lo = 0; lo < 4; lo++ ) {
            for (int i = -1; i <= 1; i+= 2) {
		home(1);
                cmaps(N_CMAPS/4, i);
		scale(1, i);
		horiz(256, i);
		scale(1, i*-1);
		home(1);
		ne(100,i);
                cmaps(N_CMAPS/4, i);
		scale(1, i);
		vert(192, i);
		scale(1, i);
		nw(100,i);
		blur(100, 1);
		blur(100, -1);
		hud();
		horizT(256, i);
		cont(10, -1);
		hist();
		cont(110, 1);
		vertT(192, i);
		cont(100, -1);
		hud();
		hist();
		interp(10, 1);
		interp(10, -1);
		reset();
            }
            layout(1);
        }
        rotate(1);
    }
}
