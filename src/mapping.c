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
   for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
      if (cur_node->type == XML_ELEMENT_NODE) {
         id = xmlGetProp(cur_node, "id");
         if(xmlStrcmp(id, (const xmlChar *)"bcn")){
            name = xmlGetProp(cur_node, "name");
            bcnX = xmlGetProp(cur_node, "bcnX");
            bcnY = xmlGetProp(cur_node, "bcnY");
            ccsrState.beaconListName[beaconID] =  (char*) malloc(10*sizeof(char));
            strcpy(ccsrState.beaconListName[beaconID], (char*) name);
            ccsrState.beaconListX[beaconID]=atoi((char *) bcnX);
            ccsrState.beaconListY[beaconID]=atoi((char *) bcnY);
            printf("parseSVGBeacons: found beacon %s\n", (char*) name);

         }
         beaconID = beaconID+1;
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
