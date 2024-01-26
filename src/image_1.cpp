#ifndef _IMAGE_1_CPP
#define _IMAGE_1_CPP

                TS( imageMicros = currentTimeMicros(); )
//printf("\t%s(%d) - Running %ld\n", __func__, __LINE__, currentTimeMicros()); fflush(stdout);

		int useFrameOrig = 0;
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
                                        cv::hconcat( rgbImageFrame, rgbThermalFrame, *threadData.rgbFrameOrig );
                                        sourcePtr = threadData.rgbFrameOrig;
					useFrameOrig = 1;
                                        break;
                                case WINDOW_DOUBLE_HIGH:
                                default:
                                        cv::vconcat( rgbImageFrame, rgbThermalFrame, *threadData.rgbFrameOrig );
                                        sourcePtr = threadData.rgbFrameOrig;
					useFrameOrig = 1;
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

			// Optimization: Cornercase for 1X scale.  Interpolation of 1:1 pixels does not make sense
			if ( 1 < MyScale ) {
				SIZE( size, controls.sW, controls.sH );
				resize(*sourcePtr, *threadData.rgbFrameOrig, size,
                       	              0.0, 0.0, // optional 0.0, 0.0 args is REQUIRED for Interpolation options to work
                                     Inters[controls.inters].inter // INTER_CUBIC default
                                ); // Scale up from native camera resolution
				useFrameOrig = 1;
			}

                        // /usr/include/opencv4/opencv2/imgproc.hpp
                        // void cv::blur(cv::InputArray, cv::OutputArray, cv::Size, cv::Point, int)

                        if ( controls.rad > 0 ) { // Optionally Blur to smooth scaling pixelization and diagonal jaggies
                                POINT( point, controls.rad, controls.rad );

                                blur( ( useFrameOrig ? *threadData.rgbFrameOrig : *sourcePtr ),
						*threadData.rgbFrameOrig, point );
				useFrameOrig = 1;
                        }

                        //  Colormaps do not support ALPHA/TRANSPARENTCIES
                        //  what():  OpenCV(4.5.1) ../modules/imgproc/src/colormap.cpp:736: error: (-5:Bad argument)
                        //  cv::ColorMap only supports source images of type CV_8UC1 or CV_8UC3 in function 'operator()'

			if ( 0x00 == controls.cmapCurrent ) { /* COLORMAP_NONE */
				if ( ! useFrameOrig ) {
					// Do not copy WINDOW_DOUBLE layouts twice
					*threadData.rgbFrameOrig = sourcePtr->clone();
				}
			} else {
				applyMyColorMap( ( useFrameOrig ? *threadData.rgbFrameOrig : *sourcePtr ),
					*threadData.rgbFrameOrig, controls.cmapCurrent );
			}
                }

                if ( controls.hud == HUD_ONLY_VIDEO ) {
                        // Skip the clone/copy if there will be no graphic overlays
                        *threadData.rgbFrame = *threadData.rgbFrameOrig;
                } else {
#if USE_CLONE
                        *threadData.rgbFrame = threadData.rgbFrameOrig->clone();
#else
                        threadData.rgbFrameOrig->copyTo( *threadData.rgbFrame );
#endif
                }

                TS( threadData.imageMicros += ( currentTimeMicros() - imageMicros ); ) // track relative benchmarks

#endif
