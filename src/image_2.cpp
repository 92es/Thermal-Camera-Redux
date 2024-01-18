#ifndef _IMAGE_2_CPP
#define _IMAGE_2_CPP

                TS( imageMicros = currentTimeMicros(); )

                // Do not display graphics over video
                if ( controls.hud != HUD_ONLY_VIDEO )
                {
                        if ( Rulers_Both_Horiz_Vert )
                        {
                        //      try {
                                        drawRulerPlot( *threadData.rgbFrame,
                                                rulerY0Points, rulerY1Points, scalarY, TC_HEIGHT, 0 );
                        //      } catch (...) {
                        //              printf("\t%s(%d) - drawRulers exception\n", __func__, __LINE__);
                        //              fflush(stdout); fflush(stderr);
                        //      }
                        }

                        drawUserAndRulerTemps( 0 );
                }

                TS( tmp = currentTimeMicros(); )

                TS( threadData.vertMicros  += ( tmp - imageMicros ); ) // track relative benchmarks
                TS( threadData.imageMicros += ( tmp - imageMicros ); ) // track relative benchmarks

//printf("\t%s(%d) - Halted %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

#endif
