 
#define SVG_MAP_NAME "scripts/webIFCCSR/images/map.svg"
#define SVG_MAP_SLAM_NAME "scripts/webIFCCSR/images/map_SLAM.svg"
#define MAP_WIDTH 100   // pixels
#define MAP_HEIGHT 100  // pixels
#define MAP_WIDTH_SCALE 40    // Map scale in cm/pix. This means full edge-to-edge map spans MAP_WIDTH*MAP_WIDTH_SCALE cm
#define MAP_HEIGHT_SCALE 40   // Map scale in cm/pix. This means full edge-to-edge map spans MAP_HEIGHT*MAP_HEIGHT_SCALE cm
#define LOCATOR_SIZE 10
#define ODOMETER_INTERVAL 500000  // 0.5sec


int parseSVGMap();
void writeSVGMap();
void drawLocationInSVGMap();
