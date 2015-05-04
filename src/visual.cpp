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

   // Recognize target object in imgHSV based on target color range set in ccsrState.targetColor_*. 
   // If found, set ccsrState.targetVisualObject_[X, Y, Area] to coordinates 
   // in image and volume of object, and set ccsrState.objectTracked = 1. imgThresholded is updated for visualization on webinterface. 
   int objRecogColorThreshold(Mat imgHSV, Mat imgThresholded){
      Moments oMoments;
      int iLowH  = ccsrState.targetColor_iLowH;
      int iHighH = ccsrState.targetColor_iHighH;

      int iLowS  = ccsrState.targetColor_iLowS;
      int iHighS = ccsrState.targetColor_iHighS;

      int iLowV  = ccsrState.targetColor_iLowV;
      int iHighV = ccsrState.targetColor_iHighV;

      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

      //morphological opening (removes small objects from the foreground)
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      //morphological closing (removes small holes from the foreground)
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      //Calculate the moments of the thresholded image
      oMoments = moments(imgThresholded);

      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      //       cout << " area " << dArea << endl;

      // if the area < MIN_TRACKED_OBJECT_VOLUME, I consider that the there are no object in the image and it's 
      // because of the noise, the area is not zero 
      if (dArea > MIN_TRACKED_OBJECT_VOLUME)
      {
         ccsrState.objectTracked = 1;
         //calculate the position of the target object
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
   }



   /**
   * Helper function to find a cosine of angle between vectors
   * from pt0->pt1 and pt0->pt2
    unused
   */
   static double angle(Point pt1, Point pt2, Point pt0)
   {
      double dx1 = pt1.x - pt0.x;
      double dy1 = pt1.y - pt0.y;
      double dx2 = pt2.x - pt0.x;
      double dy2 = pt2.y - pt0.y;
      return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
   }

   
   // Display shape label in the source image 'im'. 
   void tagShape(Mat& im, const string label, vector<Point>& contour)
   {
      int fontface = FONT_HERSHEY_SIMPLEX;
      double scale = 0.4;
      int thickness = 1;
      int baseline = 0;

      Size text = getTextSize(label, fontface, scale, thickness, &baseline);
      Rect r = boundingRect(contour);

      Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
      rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
      putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
   }

   // Recognize target object in src based on a triangular shape with a target color range as set in ccsrState.targetColor_*. 
   // If found, set ccsrState.targetVisualObject_[X, Y, Area] to triangle shape cordinates  
   // in image and volume of object, and set ccsrState.objectTracked = 1. bw is updated for visualization on webinterface. 
   // imgHSV is used for color detection.
   int objRecogShapeDetection(Mat src, Mat imgHSV, Mat bw) {

      int roiHeight = COLORPROBE_ROI_HEIGHT;
      int roiWidth = COLORPROBE_ROI_WIDTH;
      int roiX;
      int roiY;
      char label[10];
      int tgtVolume;

      // todo: determine volume of shape
      tgtVolume = 0;

      // Convert to grayscale
      Mat gray;
      cvtColor(src, gray, CV_BGR2GRAY);

      // Use Canny instead of threshold to catch squares with gradient shading
      Canny(gray, bw, 100, 300, 3);

      // Find contours
      vector<vector<Point> > contours;
      findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      vector<Point> approx;

      for (int i = 0; i < contours.size(); i++)
      {
         // Approximate contour with accuracy proportional
         // to the contour perimeter
         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

         // Skip small or non-convex objects 
         if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
            continue;

         if (approx.size() == 3)
         {
            // We have a triangle, now probe color.
            Rect r = boundingRect(contour[i]);

            Point center(r.x + r.width/2, r.y r.height/2);
            roiX = center.x - ROI_WIDTH/2;
            roiY = center.y - ROI_HEIGHT/2;
            // extract small roi of image, inside the triangle
            Mat roi(imgHSV, Rect(roiX,roiY,roiWidth,roiHeight));
            // calculate mean HSV color value of roi
            Scalar meanRoi = mean(roi);                          
            // Check if triangle color is in range of target color
            if (isTargetColor(meanRoi.val[0], meanRoi.val[1], meanRoi.val[2]){
               // Get color name string
               colorName = lookupColor(meanRoi.val[0], meanRoi.val[1], meanRoi.val[2]);
               if(colorName!=0){
                  sprintf(label, "BCN_%s", colorName);
               }
               else{
                  sprintf(label, "BCN_x");
               }
               // Tag shape in source image
               tagShape(src, label, contours[i]);    
               ccsrState.targetVisualObject_X = center.x;
               ccsrState.targetVisualObject_Y = center.y;
               ccsrState.targetVisualObject_Vol = tgtVolume;
               // Let system know object is found and tracked!
               ccsrState.objectTracked = 1;
               break;   // Jump out of iteration through contours, we're done
            }
            else{
               ccsrState.objectTracked = 0;
            }
         }
         else{
            // No triangle
            ccsrState.objectTracked = 0;
         }
      }
   }




   // pthread process processing images from USB camera
   void *visual () {

      VideoCapture cap(0); //capture the video from webcam
      int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontScale = 2;
      int thickness = 3;
      char* textList[NUM_DISPLAY_STRINGS];
      int i;

      // We capture 640x480 image by default
      if ( !cap.isOpened() )  // if not success, exit program
      {
         cout << "Cannot open the web cam" << endl;
      }

      // Allocate memory for in-display telemetry
      for(i=0;i<NUM_DISPLAY_STRINGS;i++) {
         textList[i] = malloc(MAX_DISP_STRING_LEN*sizeof(char));
      }

      // Define region of interest: predefined spot in between CCSR's grabber.
      // We'll use this to identify what CCSR is holding
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


      //Capture a temporary image from the camera
      //Mat imgTmp;
      //cap.read(imgTmp);

      //Create a black image with the size as the camera output
///      Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );

      Mat imgOriginal;
      Mat imgHSV;
      Mat imgThresholded;

      while (true)
      {
         bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
         {
            cout << "Cannot read a frame from video stream" << endl;
            break;
         }


         cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

         // FInd the X/Y location in the image of target object. If the object is found, ccsrState.objectTracked is set to '1'
         // We have 2 modes: find general object of target color or a triangular shape of target color. The triangles are used for
         // visual beacon detection and triangulation.
         switch(ccsrState.objectRecognitionMode){
         case OBJREC_COLORTHRESHOLD:
            // should we tag object in imgOriginal here too??
            objRecogColorThreshold(imgHSV, imgThresholded);
            break;
         case OBJREC_SHAPEDETECTION:
            objRecogShapeDetection(imgOriginal, imgHSV, imgThresholded);
            break;
         }

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

         // If requested, save analysed images to disk, for display on web interfacee
         // This is a one-shot operation. telemetry.c sets ccsrState.camCapture to 1, and
         // visual handshakes by resetting.
         if (ccsrState.camCapture) {
            // Draw ROI rectangles
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
            // Create and draw In-picture telemetry data
            // Draw text list as column on left side of picture
            sprintf(textList[0], "hdng %d",ccsrState.heading);
            sprintf(textList[1], "batt %d",ccsrState.batteryPercent);
            sprintf(textList[2], "tgtX %d",ccsrState.ccsrState.targetVisualObject_X);
            sprintf(textList[3], "tgtY %d",ccsrState.ccsrState.targetVisualObject_Y);
            sprintf(textList[4], "tgtV %d",ccsrState.ccsrState.targetVisualObject_Vol);
            sprintf(textList[5], "trck %d",ccsrState.trackTargetColorOn);
            Point textOrg(10, 10);
            for(i=0;i<NUM_DISPLAY_STRINGS;i++) {
               putText(imgThresholded, textList[i], textOrg, fontFace, fontScale,
                  Scalar::all(255), thickness, 8);
               textOrg.y += 10;
            }
            // Write images to disk, will be read by web interface
            imwrite(CAM_CAPTURE_RAW, imgOriginal);
            imwrite(CAM_CAPTURE_THRESHOLD, imgThresholded);
            ccsrState.camCapture = 0;  // Handshake back to tememetry.c
         }
         // Simple handshake with other processes: pass back req to ack that visual has been run at least once after req was asserted. 
         // Note, only one pipe now, may need more req/ack lines?
         ccsrState.visual_ack = ccsrState.visual_req;
      }
   }

}  // extern 'C'
