#ifndef _IMAGE_3_CPP
#define _IMAGE_3_CPP

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
#if BORDER_FRAME
                                                ptf->videoOut.write( borderFrame );
#else
                                                ptf->videoOut.write( *threadData.rgbFrame );
#endif
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

                TS( threadData.imageMicros += ( currentTimeMicros() - imageMicros ); ) // track relative benchmarks

#endif
