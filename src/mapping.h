 
#define SVG_MAP_NAME "scripts/webIFCCSR/images/map.svg"
#define SVG_MAP_SLAM_NAME "scripts/webIFCCSR/images/map_SLAM.svg"
#define MAP_WIDTH 100
#define MAP_HEIGHT 100



int parseSVGMap(char* svgFileName);
void parseSVGBeacons(xmlNode * a_node);
void writeSVGMap();
void drawLocationInSVGMap();
