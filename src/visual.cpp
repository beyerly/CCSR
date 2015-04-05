#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include "ccsr.h"
#include "lcdDisp.h"
#include "utils.h"
#include "servoCtrl.h"
#include "visual.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

extern ccsrStateType ccsrState;

extern "C" {

   // Set CCSR's active target color range in HSV values. CCSR will track this color if enabled 
void setTargetColorRange(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV) {
   ccsrState.targetColor_iLowH  =  iLowH;
   ccsrState.targetColor_iHighH =  iHighH;
   ccsrState.targetColor_iLowS  =  iLowS;
   ccsrState.targetColor_iHighS =  iHighS;
   ccsrState.targetColor_iLowV  =  iLowV;
   ccsrState.targetColor_iHighV =  iHighV;
}

// Set CCSR's active target volume. CCSR will assume it arrived at the active target object if the 
// volume (area) of the tracked color blob is more than this value
void setTargetColorVolume(int vol) {
   ccsrState.targetColorVolume = vol;
}

// pthread process processing images from USB camera
void *visual () {

   VideoCapture cap(0); //capture the video from webcam

   // We capture 640x480 image by default
   if ( !cap.isOpened() )  // if not success, exit program
   {
         cout << "Cannot open the web cam" << endl;
   }

   int roiHeight = ROI_HEIGHT;
   int roiWidth = ROI_WIDTH;
   int roiX = IMAGE_WIDTH/2 - ROI_WIDTH/2;
   int roiY = IMAGE_HEIGHT/2 - ROI_HEIGHT/2;
   // Adjust ROI from center to spot covering the grabber.
   roiX = roiX - 65 ;
   roiY = roiY + 130;
   

   double fps = cap.get(CV_CAP_PROP_FPS);
//   cout << "fps: " << fps << endl;
//   cout << "width: " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
//   cout << "set: " << cap.set(CV_CAP_PROP_FPS, 6) << endl;

   int iLowH  = ccsrState.targetColor_iLowH;
   int iHighH = ccsrState.targetColor_iHighH;

   int iLowS  = ccsrState.targetColor_iLowS;
   int iHighS = ccsrState.targetColor_iHighS;

   int iLowV  = ccsrState.targetColor_iLowV;
   int iHighV = ccsrState.targetColor_iHighV;

   //Capture a temporary image from the camera
   Mat imgTmp;
   cap.read(imgTmp);

   //Create a black image with the size as the camera output
   Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 
       Mat imgOriginal;
       Mat imgHSV;
       Mat imgThresholded;
       Moments oMoments;

   while (true)
   {

  iLowH  = ccsrState.targetColor_iLowH;
  iHighH = ccsrState.targetColor_iHighH;

  iLowS  = ccsrState.targetColor_iLowS;
  iHighS = ccsrState.targetColor_iHighS;

  iLowV  = ccsrState.targetColor_iLowV;
  iHighV = ccsrState.targetColor_iHighV;

 
       bool bSuccess = cap.read(imgOriginal); // read a new frame from video

       if (!bSuccess) //if not success, break loop
       {
   	    cout << "Cannot read a frame from video stream" << endl;
   	    break;
       }


       cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 

       // If asked to analyze an object held up by arm, calculate and capture avarage color values of object.
       // Object is assumed to be held by arm exactly at square region of interrest (roi) in middle of image
       // This is a one-shot operation. analyzeObject in actions.c sets ccsrState.analyzeObject to 1
       // visual handshakes by resetting:
       if(ccsrState.analyzeObject) {
          Mat roi(imgHSV, Rect(roiX,roiY,roiWidth,roiHeight)); // extract small roi of image
	  Scalar meanRoi = mean(roi);                          // calculate mean HSV color value of roi
	  ccsrState.analyzedObjectH = meanRoi.val[0];          
	  ccsrState.analyzedObjectS = meanRoi.val[1];
	  ccsrState.analyzedObjectV = meanRoi.val[2];
          ccsrState.analyzeObject = 0; 
          cout << "analysed HSV:" << ccsrState.analyzedObjectH << " " << ccsrState.analyzedObjectS << " " << ccsrState.analyzedObjectV << endl;
       }

       inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
       //morphological opening (removes small objects from the foreground)
       erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
       dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

       //morphological closing (removes small holes from the foreground)
       dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
       erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

       //Calculate the moments of the thresholded image
       oMoments = moments(imgThresholded);

       // If requested, save analysed images to disk, for display on web interfacee
       // This is a one-shot operation. telemetry.c sets ccsrState.camCapture to 1, and
       // visual handshakes by resetting.
       if (ccsrState.camCapture) {
          rectangle( imgOriginal,
 		 Point(roiX, roiY),                           // upper left edge
 		 Point(roiX + roiWidth, roiY + roiHeight),    // lower right edge
 		 Scalar( 0, 255, 255 ),                       // yellow
 		 1,                                           // thickness 1 pow of pixels
 		 8 ); 
          rectangle( imgThresholded,
 		 Point(roiX, roiY),                           // upper left edge
 		 Point(roiX + roiWidth, roiY + roiHeight),    // lower right edge
 		 Scalar( 0, 255, 255 ),                       // yellow
 		 1,                                           // thickness 1 pow of pixels
 		 8 ); 
          imwrite(CAM_CAPTURE_RAW, imgOriginal);
          imwrite(CAM_CAPTURE_THRESHOLD, imgThresholded);
          ccsrState.camCapture = 0;
       }


       double dM01 = oMoments.m01;
       double dM10 = oMoments.m10;
       double dArea = oMoments.m00;

//       cout << " area " << dArea << endl;

       // if the area < MIN_TRACKED_OBJECT_VOLUME, I consider that the there are no object in the image and it's 
       // because of the noise, the area is not zero 
       if (dArea > MIN_TRACKED_OBJECT_VOLUME)
       {
          ccsrState.objectTracked = 1;
 	  //calculate the position of the ball
 	  int posX = dM10 / dArea;
 	  int posY = dM01 / dArea;

          ccsrState.targetVisualObject_X = posX;
          ccsrState.targetVisualObject_Y = posY;
          ccsrState.targetVisualObject_Vol = dArea;

//         cout << posX << " " << posY << " " << dArea << endl;

        }
	else {
          ccsrState.objectTracked = 0;
	}
//      usleep(10000);
    }

}

}  // extern 'C'
