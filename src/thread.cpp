#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>

#include "thread.h"

extern "C" {

pthread_mutex_t IndexMutex  = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t FillMutex   = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  FillCond    = PTHREAD_COND_INITIALIZER;

pthread_mutex_t DrainMutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  DrainCond   = PTHREAD_COND_INITIALIZER;

int    Fill_Index  = 1;
int    Drain_Index = 0;

#define QUEUE_SIZE 20
#define NEXT(a)           ( ((a) + 1) % QUEUE_SIZE )

int KeyEventQueue[QUEUE_SIZE];

extern int drainIsBlocked(int *drainIndex, int *filIndex);
extern int fillIsBlocked(int *drainIndex, int *filIndex);
extern void waitOnDrain(int *drainIndex, int *fillIndex);
extern void waitOnFill(int *drainIndex, int *fillIndex);

extern void signalFill_SpaceReady();
extern void signalDrain_DataReady();

int readKeyEventIndex( int *drainIndex, int *fillIndex, int *keyEvent ) {
	pthread_mutex_lock( &IndexMutex );

		int blocked = ( NEXT( Drain_Index ) == Fill_Index );

		if ( ! blocked ) {
			Drain_Index = NEXT( Drain_Index );
			*keyEvent = KeyEventQueue[ Drain_Index ];
			KeyEventQueue[ Drain_Index ] = 0x00;

			if (drainIndex) *drainIndex = Drain_Index; // return IndexMutex protected copy 
			if (fillIndex)  *fillIndex  = Fill_Index;  // return IndexMutex protected copy 
		}

	pthread_mutex_unlock( &IndexMutex );

	if ( blocked ) {
		return -1;
	} else {
		signalFill_SpaceReady();
		//printf(">< %s(%d) - space ready\n", __func__, *keyEvent); ; fflush(stdout);
		return *keyEvent;
	}
}

int readKeyEvent( int *keyEvent ) {
	return readKeyEventIndex( 0, 0, keyEvent );
}

// Blocks on drain thread
void writeKeyEventIndex( int *drainIndex, int *fillIndex, int keyEvent ) {

	// Can have multiple writers and 1 reader
	int blocked, drain=-1, fill=-1;
	do {
		pthread_mutex_lock( &IndexMutex );

		blocked = ( NEXT( Fill_Index ) == Drain_Index );

		if ( ! blocked ) {
			KeyEventQueue[ Fill_Index ] = keyEvent;
			Fill_Index  = NEXT( Fill_Index );

			drain = Drain_Index; // return IndexMutex protected copy 
			fill  = Fill_Index;  // return IndexMutex protected copy 

			if (drainIndex) *drainIndex = drain; // return IndexMutex protected copy 
			if (fillIndex)  *fillIndex  = fill;  // return IndexMutex protected copy 
		}

		pthread_mutex_unlock( &IndexMutex );


		if ( blocked ) {
			//printf("%s> %s(%s) waitOnDrain()%s\n", RED_STR(), __func__, who, RESET_STR() ); fflush(stdout);
			waitOnDrain( 0, 0 );
			//printf("%s< %s(%s) waitOnDrain()%s\n", RED_STR(), __func__, who, RESET_STR() ); fflush(stdout);
		} else {
			signalDrain_DataReady();
			//printf(">< %s(%s, %d) - data ready\n", __func__, who, keyEvent); fflush(stdout);
		}

	} while ( blocked );
}

void writeKeyEvent( int keyEvent ) {
	writeKeyEventIndex( 0, 0, keyEvent );
}

int drainIsBlocked(int *drainIndex, int *fillIndex) {
	pthread_mutex_lock( &IndexMutex );
		int blocked = ( NEXT( Drain_Index ) == Fill_Index );
		if (drainIndex) *drainIndex = Drain_Index; // return IndexMutex protected copy 
		if (fillIndex)  *fillIndex  = Fill_Index;  // return IndexMutex protected copy 
	pthread_mutex_unlock( &IndexMutex );
	return blocked;
}

int fillIsBlocked(int *drainIndex, int *fillIndex) {
	pthread_mutex_lock( &IndexMutex );
		int blocked = ( NEXT( Fill_Index ) == Drain_Index );
		if (drainIndex) *drainIndex = Drain_Index; // return IndexMutex protected copy 
		if (fillIndex)  *fillIndex  = Fill_Index;  // return IndexMutex protected copy 
	pthread_mutex_unlock( &IndexMutex );
	return blocked;
}

void waitOnDrain(int *drainIndex, int *fillIndex) {
	pthread_mutex_lock( &FillMutex );

//		char baseChar = who[0];
//		const char *threadColor = islower(baseChar) ? BLUE_STR() : GREEN_STR();

		// Wait for drainTread to remove some entries from the RingBuffer
		int drain, fill; // drainIndex and/or fillIndex maybe null
		while ( fillIsBlocked( &drain, &fill ) ) { // wait loop broken by advanceFill()
//printf("%sFill:  waitOnDrain - d:f %d:%d%s\n", threadColor, drain, fill, RESET_STR()); fflush(stdout);
			pthread_cond_wait( &FillCond, &FillMutex );
		}

		if (drainIndex) *drainIndex = drain; // return IndexMutex protected copy
		if (fillIndex)  *fillIndex  = fill;  // return IndexMutex protected copy

	pthread_mutex_unlock( &FillMutex );
}

void waitOnFill(int *drainIndex, int *fillIndex) {
	pthread_mutex_lock( &DrainMutex );

		// Wait for fillThread to add some entries to the RingBuffer
		int drain, fill; // drainIndex and/or fillIndex maybe null
		while ( drainIsBlocked( &drain, &fill ) ) { // wait loop broken by advanceDrain()
//printf("%sDrain: waitOnFill - d:f %d:%d%s\n", YELLOW_STR(), drain, fill, RESET_STR()); fflush(stdout);
			pthread_cond_wait( &DrainCond, &DrainMutex ); 
		}

		if (drainIndex) *drainIndex = drain; // return IndexMutex protected copy
		if (fillIndex)  *fillIndex  = fill;  // return IndexMutex protected copy

	pthread_mutex_unlock( &DrainMutex );
}

void signalFill_SpaceReady() {
	pthread_mutex_lock( &FillMutex );
		pthread_cond_signal( &FillCond );
	pthread_mutex_unlock( &FillMutex );
}

void signalDrain_DataReady() {
	pthread_mutex_lock( &DrainMutex );
		pthread_cond_signal( &DrainCond );
	pthread_mutex_unlock( &DrainMutex );
}

}
