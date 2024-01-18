#ifndef _THERM_1_CPP
#define _THERM_1_CPP

                TS( thermMicros = currentTimeMicros(); )

        //      try {
                        processThermalFrame( ptf, &thermalFrame );
        //      } catch (...) {
        //              printf("\t%s(%d) - processThermalFrame exception\n", __func__, __LINE__);
        //              fflush(stdout); fflush(stderr);
        //      }

                if ( controls.hud != HUD_ONLY_VIDEO )
                {
                        // HAS TO BE DONE AFTER Min/Max/Avg has been calculated !!!
                        // THREAD LOAD BALANCING - Grab ruler row and column temps and points in parallel

                        minF       = ptf->minPixel.celsius;
                        maxF       = ptf->maxPixel.celsius;
                        avgF       = ptf->avg.celsius;
                        float thresholdF = controls.threshold.celsius; // Threshold is a delta C or F

                        drawMin    = ( CorF( minF ) < (CorF( avgF ) - thresholdF) );
                        drawMax    = ( CorF( maxF ) > (CorF( avgF ) + thresholdF) );

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

			drawHorizMin  = minBound && Horiz_Min_Bound && Rulers_Both_Horiz;
			drawHorizMax  = maxBound && Horiz_Max_Bound && Rulers_Both_Horiz;

			drawVertMin   = minBound && Vert_Min_Bound  && Rulers_Both_Vert;
			drawVertMax   = maxBound && Vert_Max_Bound  && Rulers_Both_Vert;

                        float kelvinFactor = getRulerKelvinFactor( FIXED_TC_HEIGHT - 1 );

                        minAvgOffsetF = kelvinFactor * rulerBoundMinKelvin; // Should be non-negative
                        maxAvgOffsetF = kelvinFactor * rulerBoundMaxKelvin; // Should be non-negative

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
                                // Range has to be calculated in like destination units (Apples - Apples)
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

                                copZero    = centerOfPixelNODIV( 0 );   // both x and y
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
                                //              AA AB BB BA
                                // with PORTRATE/LANDSCALE and PROPORTIONAL/FULL thickness rullers
                                // Prevent center point flicker @ 1X and 2X scale


#if FAST_DRAG   // Speed optimization for weak hardware
        #define IF_NOT_FAST_DRAG        if ( ! Rulers_And_Drag ) /* No Perp Arrows during FAST_DRAG */
        #define OR_RULERS_AND_DRAG      || Rulers_And_Drag       /* No Perp Arrows during FAST_DRAG */
#else
        #define IF_NOT_FAST_DRAG
        #define OR_RULERS_AND_DRAG
#endif

                                IF_NOT_FAST_DRAG
                                {
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
                                }

                                if ( Rulers_Vert  OR_RULERS_AND_DRAG )
                                {
                                        POINT( vBase_b, ( copAnchorX PLUS_DEBUG ), ( copAnchorY minus_FLICKER ) );
                                        POINT( vBase_c, ( copAnchorX PLUS_DEBUG ), ( copAnchorY plus_FLICKER  ) );
                                }

                                IF_NOT_FAST_DRAG
                                {
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
                                }


                                if ( Rulers_Horiz   OR_RULERS_AND_DRAG )
                                {
                                        POINT( hBase_b, ( copAnchorX minus_FLICKER ), ( copAnchorY PLUS_DEBUG ) );
                                        POINT( hBase_c, ( copAnchorX plus_FLICKER  ), ( copAnchorY PLUS_DEBUG ) );
                                }

                                IF_NOT_FAST_DRAG
                                {
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

                // THREAD LOAD BALANCING - Prepare scaled, pre-processed offscreen rendering areas

                if ( RERENDER_OPTIMIZATION ) {
                        // THREAD LOAD BALANCING - Pre-render Colormap Scale into its own RGB image
                        // cmapScale will change based on scaledSFHeight as well as CV_TYPE, thus needs to be remade
                //      try { 
                                thermROI.width     = ColorScaleWidth;
                                thermROI.height    = controls.scaledSFHeight;
                                threadData.cmapScale = threadData.maxCmapScale( thermROI ); // Reuse memory
                                drawCmapScale( threadData.cmapScale, threadData.maxCmapScale.cols ); 
                //      } catch (...) {
                //              printf("\t%s(%d) - cmapScale exception\n", __func__, __LINE__);
                //              fflush(stdout); fflush(stderr);
                //      }
                }
                
                TS( threadData.thermMicros += ( currentTimeMicros() - thermMicros ); ) // track relative benchmarks

#endif
