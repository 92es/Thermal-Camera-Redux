#ifndef _THERM_0_CPP
#define _THERM_0_CPP

                TS( thermMicros = currentTimeMicros(); )

                thermalFrame = Mat( *(threadData.rawFrame), thermFrameROI ).clone();
                if ( RotateDisplay ) { rotate( thermalFrame, thermalFrame, rotateFlags[ RotateDisplay ] ); }

                if ( WINDOW_IMAGE != controls.windowFormat ) {

                        if ( Use_Histogram ) {
                                // Write historgram to copy, not original
                                // Changing thermalFrame will break subsequent call to processThermalFrame()
				// Only realloate copy when absolutely necessary
				// Don't use .size() which creates/destroys intermediate Size object
			        if (	copy.rows != thermalFrame.rows ||
					copy.cols != thermalFrame.cols ) {
					// Don't need deep copy, just allocate same dst size matrix
					copy = Mat( thermalFrame.rows, thermalFrame.cols, CV_8UC2 );
				} 

				// Write Histrogram Equalization filter into copy 
                                histogramWrapper( thermalFrame, copy, 1 );
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

                TS( threadData.thermMicros += ( currentTimeMicros() - thermMicros ); ) // track relative benchmarks

#endif	
