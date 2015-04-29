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

// extern ccsrStateType ccsrState;

extern "C" {

#define NUM_COLORS 16
#define ROI_HEIGHT 50
#define ROI_WIDTH  50

   char *colorName;
   int  analyzedObjectH;          
   int  analyzedObjectS;
   int  analyzedObjectV;


   int roiHeight = ROI_HEIGHT;
   int roiWidth = ROI_WIDTH;
   int roiX;
   int roiY;



   typedef struct colorType {
      int iLowH;
      int iHighH;

      int iLowS;
      int iHighS;

      int iLowV;
      int iHighV;

      char name[30];
   } colorType;


   // Static list of known colors by HSV value range
   colorType colors[NUM_COLORS];

   // Populate list of known colors by HSV range. This is obviously an approximation, we'll simply
   // call all HSV values representing any shade of green "green". More detail can be added. 
   // Note: Is there a HSV2string function? string s = Color.FromArgb(255, 143, 143, 143).Name seems to be
   // from .NET only.
   void initColors(){
      colors[0].iLowH  = 100;
      colors[0].iHighH = 120;
      colors[0].iLowS  = 100;
      colors[0].iHighS = 208;
      colors[0].iLowV  = 89;
      colors[0].iHighV = 255;
      strcpy(colors[0].name, "blue");

      colors[1].iLowH  = 75;
      colors[1].iHighH = 99;
      colors[1].iLowS  = 100;
      colors[1].iHighS = 200;
      colors[1].iLowV  = 89;
      colors[1].iHighV = 255;
      strcpy(colors[1].name, "green");

   }

   // Return pointer string representing approximate color name of HSV value. Return 0 if no match found
   char* lookupColor(int H, int S, int V){

      int i;

      for (i=0;i<NUM_COLORS;i++) {
         if((H>=colors[i].iLowH)  &&
            (H<=colors[i].iHighH) &&
            (S>=colors[i].iLowS)  &&
            (S<=colors[i].iHighS) &&
            (V>=colors[i].iLowV)  &&
            (V<=colors[i].iHighV)){
               return colors[i].name;
         }
      }
      return 0;
   }



   /**
   * Helper function to find a cosine of angle between vectors
   * from pt0->pt1 and pt0->pt2
   */
   static double angle(Point pt1, Point pt2, Point pt0)
   {
      double dx1 = pt1.x - pt0.x;
      double dy1 = pt1.y - pt0.y;
      double dx2 = pt2.x - pt0.x;
      double dy2 = pt2.y - pt0.y;
      return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
   }

   /**
   * Helper function to display text in the center of a contour
   */
   void setLabel(Mat& im, const string label, vector<Point>& contour)
   {
      int fontface = FONT_HERSHEY_SIMPLEX;
      double scale = 0.4;
      int thickness = 1;
      int baseline = 0;

      Size text = getTextSize(label, fontface, scale, thickness, &baseline);
      Rect r = boundingRect(contour);

      Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
      rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
      putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
   }


   int main() {

      initColors();

      Mat src = imread("test.png");
      if (src.empty())
         return -1;


      // Convert to grayscale
      Mat gray;
      Mat imgHSV;
      cvtColor(src, gray, CV_BGR2GRAY);
      cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

      // Use Canny instead of threshold to catch squares with gradient shading
      Mat bw;
      Canny(gray, bw, 0, 50, 5);


      // Find contours
      vector<vector<Point> > contours;
      findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      vector<Point> approx;
      Mat dst = src.clone();

      for (int i = 0; i < contours.size(); i++)
      {
         // Approximate contour with accuracy proportional
         // to the contour perimeter
         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

         // Skip small or non-convex objects 
         if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
            continue;

         Rect r = boundingRect(contour[i]);

         Point center(r.x + r.width/2, r.y r.height/2);
         roiX = center.x - ROI_WIDTH/2;
         roiY = center.y - ROI_HEIGHT/2;

         Mat roi(imgHSV, Rect(roiX,roiY,roiWidth,roiHeight)); // extract small roi of image
         Scalar meanRoi = mean(roi);                          // calculate mean HSV color value of roi
         analyzedObjectH = meanRoi.val[0];          
         analyzedObjectS = meanRoi.val[1];
         analyzedObjectV = meanRoi.val[2];

         // Lookup color name and say it outloud
         colorName = lookupColor(analyzedObjectH,
            analyzedObjectS,
            analyzedObjectV);
         if(colorName!=0){
            printf("c; %s\n", colorName);
         }
         else{
            printf("c: unknown\n");
         }

         if (approx.size() == 3)
         {
            setLabel(dst, "TRI", contours[i]);    // Triangles
         }
         /*
         else if (approx.size() >= 4 && approx.size() <= 6)
         {
         // Number of vertices of polygonal curve
         int vtc = approx.size();

         // Get the cosines of all corners
         std::vector<double> cos;
         for (int j = 2; j < vtc+1; j++)
         cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

         // Sort ascending the cosine values
         std::sort(cos.begin(), cos.end());

         // Get the lowest and the highest cosine
         double mincos = cos.front();
         double maxcos = cos.back();

         // Use the degrees obtained above and the number of vertices
         // to determine the shape of the contour
         if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
         setLabel(dst, "RECT", contours[i]);
         else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
         setLabel(dst, "PENTA", contours[i]);
         else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
         setLabel(dst, "HEXA", contours[i]);
         }
         else
         {
         // Detect and label circles
         double area = cv::contourArea(contours[i]);
         cv::Rect r = cv::boundingRect(contours[i]);
         int radius = r.width / 2;

         if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
         std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
         setLabel(dst, "CIR", contours[i]);
         }
         */
      }

      imshow("canny", gray);
      imshow("src", src);
      imshow("dst", dst);
      waitKey(0);
      return 0;
   }


}  // extern 'C'
