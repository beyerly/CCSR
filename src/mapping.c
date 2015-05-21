/**
 * section: Tree
 * synopsis: Navigates a tree to print element names
 * purpose: Parse a file to a tree, use xmlDocGetRootElement() to
 *          get the root element, then walk the document and print
 *          all the element name in document order.
 * usage: tree1 filename_or_URL
 * test: tree1 test2.xml > tree1.tmp && diff tree1.tmp $(srcdir)/tree1.res
 * author: Dodji Seketeli
 * copy: see Copyright for the status of this software.
 */
#include <stdio.h>
#include <string.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "ccsr.h"
#include "mapping.h"

extern ccsrStateType ccsrState;

xmlDoc *doc = NULL;
xmlNode *root_element = NULL;


void fitToMap(int* X, int* Y){
   if(*X<0){
      *X=0;
   }
   else if (*X>MAP_WIDTH) {
      *X=MAP_WIDTH;
   }
   if(*Y<0){
      *Y=0;
   }
   else if (*Y>MAP_HEIGHT) {
      *Y=MAP_HEIGHT;
   }
}

// Draw a locator at the current CCSR position in the CCSR map 
void drawLocationInSVGMap()
{

   xmlNodePtr newnode;
   xmlAttrPtr newattr;

   char locator[30];
   char pivot[30];
   char point[10];
   char color[10];
   int i;
   int tX[3], tY[3];
   int X, Y;
   
   strcpy(point, "");
    strcpy(locator, "");
  
   // Translate cartesian to image X,Y:
   X = (int) round(ccsrState.locationX);
   Y = (int) round(MAP_HEIGHT - ccsrState.locationY);

   if(ccsrState.locationAccurate){
      strcpy(color, "purple");
   }
   else {
      strcpy(color, "red");
   }
   
   tX[0] = X;
   tY[0] = Y - LOCATOR_SIZE/2;
   tX[1] = X - LOCATOR_SIZE/2;
   tY[1] = Y + LOCATOR_SIZE/2;
   tX[2] = X + LOCATOR_SIZE/2;
   tY[2] = Y + LOCATOR_SIZE/2;
   
   for(i=0;i<3;i++){
      fitToMap(&tX[i], &tY[i]);
      sprintf(point, " %d %d", tX[i], tY[i]);  // @@@
      strcat(locator, point);
   }
   
   newnode = xmlNewTextChild (root_element, NULL, "polygon", NULL);
   newattr = xmlNewProp (newnode, "points", locator);
   newattr = xmlNewProp (newnode, "stroke", color);
   newattr = xmlNewProp (newnode, "fill", color);
   newattr = xmlNewProp (newnode, "stroke-width", "0");
   sprintf(pivot, "rotate(%d %d %d)", ccsrState.heading, X, Y);  
   newattr = xmlNewProp (newnode, "transform", pivot);
   sprintf(pivot, "h:%d,(%d, %d)", ccsrState.heading, (int) round(ccsrState.locationX), (int) round(ccsrState.locationY));  
   newnode = xmlNewTextChild (root_element, NULL, "text", pivot);
   sprintf(point, "%d", X);  
   newattr = xmlNewProp (newnode, "x", point);
   sprintf(point, "%d", Y);  
   newattr = xmlNewProp (newnode, "y", point);
   newattr = xmlNewProp (newnode, "font-size", "5");
   newattr = xmlNewProp (newnode, "fill", "black");

}



// Draw a locator at the current CCSR position in the CCSR map 
void writeSVGMap()
{
   xmlSaveFormatFileEnc(SVG_MAP_SLAM_NAME , doc, "UTF-8", 1);
}


// Parse through map.svg file and extract all visual beacon names and locations, store in ccsrState.
void parseSVGBeacons(xmlNode * a_node)
{
   xmlNode *cur_node = NULL;
   xmlChar *id;
   xmlChar *name;
   xmlChar *bcnX;
   xmlChar *bcnY;
   int beaconID;
   beaconID = 0;
   for (cur_node = a_node->xmlChildrenNode; cur_node; cur_node = cur_node->next) {
      if (cur_node->type == XML_ELEMENT_NODE) {
	 id = xmlGetProp(cur_node, "id");
         if(!xmlStrcmp(id, (const xmlChar *)"bcn")){
            name = xmlGetProp(cur_node, "name");
            bcnX = xmlGetProp(cur_node, "x");
            bcnY = xmlGetProp(cur_node, "y");
            if(name && bcnX && bcnY){
	    ccsrState.beaconListName[beaconID] =  (char*) malloc(10*sizeof(char));
            strcpy(ccsrState.beaconListName[beaconID], (char*) name);
	    ccsrState.beaconListX[beaconID] = atoi((char *) bcnX);
            ccsrState.beaconListY[beaconID] = MAP_HEIGHT - atoi((char *) bcnY);
            printf("parseSVGBeacons: found beacon %s at x:%d y:%d\n", (char*) name, ccsrState.beaconListX[beaconID], ccsrState.beaconListY[beaconID]);
            beaconID = beaconID+1;
	    }
	    else{
	       printf("parseSVGBeacons: SVG beacon formatting error\n");
	    }
         }
         if(beaconID>NUM_BEACONS){
            printf("parseSVGBeacons: exceeding max numner of beacons in SVG file\n");
            exit(0);
         }
      }
   }
   xmlFree(id);
   xmlFree(name);
   xmlFree(bcnX);
   xmlFree(bcnY);
}


// Parse SVG map
// Create an XML root node, to which we can add object such as CCSR location, triangulation lines and SLAM-discovered obstacles.
// Also parse SVG for visual beacons, and populate the beaconlist with names and locations.
int parseSVGMap()
{
    printf("parsing SVG map %s\n", SVG_MAP_NAME);

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
//     LIBXML_TEST_VERSION

    /*parse the file and get the DOM */
    doc = xmlReadFile(SVG_MAP_NAME, NULL, 0);

    if (doc == NULL) {
        printf("error: could not parse file %s\n", SVG_MAP_NAME);
    }

    /*Get the root element node */
    root_element = xmlDocGetRootElement(doc);

    parseSVGBeacons(root_element);

    /*free the document */
    // xmlFreeDoc(doc);

    /*
     *Free the global variables that may
     *have been allocated by the parser.
     */
    // xmlCleanupParser();

    return 0;
}
// #else
// #endif
