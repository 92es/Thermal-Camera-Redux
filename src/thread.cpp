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

extern int fillIsBlocked();
extern void waitOnDrain();

extern void signalFill_SpaceReady();
extern void signalDrain_DataReady();

int readKeyEvent( int *keyEvent ) {
	pthread_mutex_lock( &IndexMutex );

		int blocked = ( NEXT( Drain_Index ) == Fill_Index );

		if ( ! blocked ) {
			Drain_Index = NEXT( Drain_Index );
			*keyEvent = KeyEventQueue[ Drain_Index ];
			KeyEventQueue[ Drain_Index ] = 0x00;
		}

	pthread_mutex_unlock( &IndexMutex );

	if ( blocked ) {
		return -1;
	} else {
		signalFill_SpaceReady();
		return *keyEvent;
	}
}

// Blocks on drain thread
void writeKeyEvent( int keyEvent ) {

	// Can have multiple writers and 1 reader
	int blocked;
	do {
		pthread_mutex_lock( &IndexMutex );

		blocked = ( NEXT( Fill_Index ) == Drain_Index );

		if ( ! blocked ) {
			KeyEventQueue[ Fill_Index ] = keyEvent;
			Fill_Index  = NEXT( Fill_Index );
		}

		pthread_mutex_unlock( &IndexMutex );

		if ( blocked ) {
			waitOnDrain();
		} else {
			signalDrain_DataReady();
		}

	} while ( blocked );
}

int fillIsBlocked() {
	pthread_mutex_lock( &IndexMutex );
		int blocked = ( NEXT( Fill_Index ) == Drain_Index );
	pthread_mutex_unlock( &IndexMutex );
	return blocked;
}

void waitOnDrain() {
	pthread_mutex_lock( &FillMutex );
		// Wait for drainTread to remove some entries from the RingBuffer
		while ( fillIsBlocked() ) { // wait loop broken by advanceFill()
			pthread_cond_wait( &FillCond, &FillMutex );
		}
	pthread_mutex_unlock( &FillMutex );
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
