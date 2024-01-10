#include <limits.h>

#ifndef MY_THREADS
#define MY_THREADS

extern "C" {

#define BLACK_STR()   "\e[1;30m"
#define RED_STR()     "\e[1;31m"
#define GREEN_STR()   "\e[1;32m"
#define YELLOW_STR()  "\e[1;33m"
#define BLUE_STR()    "\e[1;34m"
#define MAGENTA_STR() "\e[1;35m"
#define CYAN_STR()    "\e[1;36m"
#define WHITE_STR()   "\e[1;37m"
#define RESET_STR()   "\e[0m"

extern int readKeyEvent( int *keyEvent );
extern void writeKeyEvent( int keyEvent );

}

#endif
