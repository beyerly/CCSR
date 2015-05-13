//############################################################################################
// 
// CCSR Robot project: http://letsmakerobots.com/robot/project/ccsr
//
// Draw telemetry graphs using opencv, save in files for display in CCSR web interface webIFCCSR
//            
// date: April 2015
//
//
//############################################################################################

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include "ccsr.h"
#include "graph.h"
#include "irSensors.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

extern ccsrStateType ccsrState;

extern "C" {

// Draw a graph of the the currently stored sonar profile, write file to disk for display in the CCSR web interface
void drawSonarProfileGraph(){
   char x_label[10];
   int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
   double fontScale = 0.2;
   int thickness = 1;
   int i, x;

   // Create white background image
   Mat imgSonarProfile(Size(GRAPH_WIDTH,GRAPH_HEIGHT),CV_8UC3);  
   imgSonarProfile = Scalar(255,255,255);

   // Draw Y axis on the left
   line(imgSonarProfile,
        Point( 0, 0 ),
        Point( 0, GRAPH_HEIGHT ),
        Scalar( 0, 0, 0 ),
        GRAPH_AXIS_THICKNESS,
        GRAPH_AXIS_LINETYPE);
   // Draw X axis on bottom
   line(imgSonarProfile,
        Point( 0, GRAPH_EFF_HEIGHT),
        Point( GRAPH_WIDTH, GRAPH_EFF_HEIGHT ),
        Scalar( 0, 0, 0 ),
        GRAPH_AXIS_THICKNESS,
        GRAPH_AXIS_LINETYPE);

   // Draw Y axis grid
   for(i=0;i<GRAPH_YGRID_SIZE;i++){
      line(imgSonarProfile,
        Point(0 , i*GRAPH_EFF_HEIGHT/GRAPH_YGRID_SIZE  ),
        Point(GRAPH_GRID_NOTCH_SIZE , i*GRAPH_EFF_HEIGHT/GRAPH_YGRID_SIZE  ),
        Scalar( 0, 0, 0 ),
        GRAPH_AXIS_THICKNESS,
        GRAPH_AXIS_LINETYPE);
   }
   // Draw X axis grid and add degrees at labels
   for(i=0;i<GRAPH_XGRID_SIZE;i++){
      sprintf(x_label,"%d", i*360/GRAPH_YGRID_SIZE);
      line(imgSonarProfile,
        Point(i*GRAPH_WIDTH/GRAPH_XGRID_SIZE , GRAPH_EFF_HEIGHT),
        Point(i*GRAPH_WIDTH/GRAPH_XGRID_SIZE , GRAPH_EFF_HEIGHT-GRAPH_GRID_NOTCH_SIZE),
        Scalar( 0, 0, 0 ),
        GRAPH_AXIS_THICKNESS,
        GRAPH_AXIS_LINETYPE);
        putText(imgSonarProfile, x_label, Point(i*GRAPH_WIDTH/GRAPH_XGRID_SIZE , GRAPH_HEIGHT-3), fontFace, fontScale,
                     Scalar::all(0), thickness, 8);
   }

   // Draw vertical line representing sonar depth for each valid profile entry
   for (x=0;x<=360;x++){
      if(ccsrState.profileValid[x]){
         line(imgSonarProfile,
            Point(x*GRAPH_WIDTH/360 , GRAPH_HEIGHT-10),
            Point(x*GRAPH_WIDTH/360 , GRAPH_HEIGHT-10 - GRAPH_HEIGHT*ccsrState.sonarDistProfile[x]/MAX_SONAR_RANGE),
            Scalar( 0, 255, 0 ),
            GRAPH_AXIS_THICKNESS,
            GRAPH_AXIS_LINETYPE);
      }
   }
   // Write image to disk
  imwrite(GRAPH_SONARPROFILE, imgSonarProfile);
}      


}  // extern 'C'


