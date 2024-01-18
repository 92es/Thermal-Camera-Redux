#ifndef _IMAGE_0_CPP
#define _IMAGE_0_CPP

                TS( imageMicros = currentTimeMicros(); )

                imageFrame = Mat( *(threadData.rawFrame), imageFrameROI ).clone();
                if ( RotateDisplay ) { rotate( imageFrame, imageFrame, rotateFlags[ RotateDisplay ] ); }

                if ( WINDOW_THERMAL != controls.windowFormat ) {

                        if ( Use_Histogram ) {
                                histogramWrapper( imageFrame, imageFrame, 0 );
                        }

                        // Convert the composited image copy to RGB
                        // cv::cvtColor (InputArray src, OutputArray dst, int code, int dstCn=0)
                        cvtColor ( imageFrame, rgbImageFrame, COLOR_YUV2BGR_YUYV, CVT_CHAN_FLAG );

                        // Set contrast
                        // void cv::convertScaleAbs (InputArray src, OutputArray dst, double alpha=1, double beta=0)
                        // dst(I)=saturate\_cast<uchar>(|src(I)∗alpha+beta|)
                        if ( 1.0 != controls.alpha ) {
                                // Optimization:  Only set alpha/contrast if it is NOT 1.0
                                convertScaleAbs(rgbImageFrame, rgbImageFrame, controls.alpha); // Contrast
                        }
                }

                ASSERT(( TC_WIDTH  == imageFrame.cols ));
                ASSERT(( TC_HEIGHT == imageFrame.rows ));

                TS( tmp = currentTimeMicros(); )

                TS( threadData.rotMicros   += ( tmp - imageMicros ); ) // track relative benchmarks
                TS( threadData.imageMicros += ( tmp - imageMicros ); ) // track relative benchmarks

#endif
