#ifndef _THERM_2_CPP
#define _THERM_2_CPP

                TS( thermMicros = currentTimeMicros(); )

                // Do not display graphics over video
                if ( controls.hud != HUD_ONLY_VIDEO )
                {
                        if ( Rulers_Both_Horiz_Vert ) 
                        {
                        //      try {
                                        drawRulerPlot( *threadData.rgbFrame, 
                                                rulerX0Points, rulerX1Points, scalarX, TC_WIDTH, 1 );
                        //      } catch (...) {
                        //              printf("\t%s(%d) - drawRulersLines exception\n", __func__, __LINE__);
                        //              fflush(stdout); fflush(stderr);
                        //      }
                        }

                        drawUserAndRulerTemps( 1 );
                }

                TS( int64_t tmp = currentTimeMicros(); )

                TS( threadData.horizMicros += ( tmp - thermMicros ); ) // track relative benchmarks
                TS( threadData.thermMicros += ( tmp - thermMicros ); ) // track relative benchmarks

//printf("\t%s(%d) - Halted %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

#endif
