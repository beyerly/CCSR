#ifdef __cplusplus
extern "C" {
#endif

// ROI is region of interest: square box center-image that is assumed to be contained inside an object
// to be analyzed for color content. May need to tweak this based on object size and CCSR manouvering
// precision.
#define ROI_HEIGHT 50
#define ROI_WIDTH  50
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
 
void *visual ();

#ifdef __cplusplus
}
#endif

