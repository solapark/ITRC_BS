#ifndef _COMMON_H_
#define _COMMON_H_

// Define for libararies
//#define _USE_MATH_DEFINES

// Platform
#define WINDOW 1

// C/C++
#include <conio.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#if WINDOW
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>  
#include <mutex>
#pragma comment(lib, "ws2_32.lib")
#endif

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/video/tracking.hpp>

#include "camToCar.h"		//send data
#include "timeStamp.h"		//time stamp 
#include "pixel2Gps.h"		//gps
#include "gps2Pixel.h"		//gps
#include "gpsVelocity.h"	//velocity
#include "colorDetector.h"	//color detection for auto car

//autoCar Detection
//#include "yolo_v2_class.hpp"

using namespace cv;
using namespace std;
using namespace ml;

typedef       void					Void;
typedef       bool					Bool;
typedef       char					TChar;
typedef       signed char			SChar;
typedef       unsigned char			UChar;
typedef       short					Short;
typedef       unsigned short		UShort;
typedef       int					Int;
typedef       unsigned int			UInt;
typedef       double				Double;
typedef       float					Float;

// Type of input (enable for an input only)
#define STATIC_IMAGE				1			  
#define VIDEO						0			  
#define CAMERA						0
#define IP_CAM_NUM					174

// Image size
#define SIZE_HOR					640
#define SIZE_VER					360

/*
ROI
*/
#define STATIC_ROI					1
/*
Perspective transformation
*/
#define IS_USE_PER_TRANS			0
/*
CarSnukt detector
*/
#define MVO_VIRTUAL_ID_START				2000 //autoCar = 1000~1999, MVO = 2000~2999
#define MOV_VIRTUAL_ID_END					2999
#define AUTO_CAR_VIRTUAL_ID_START			1000
#define AUTO_CAR_VIRTUAL_ID_END				1000

// Flags
#define BGM_FIRST_BUILD				1					// use the 1st frame for background. 
#define WAIT_BGM_BUILD				1					// wait until bgm is built.
#define BGM_BUILD_WATITING_FRAME	INITAIL_BGM_DT*5	//waiting time for bgm built.
#define SHADOW_REMOVAL				1
#define TRANSFORM_CENTER_POINT		0
#define TRANSFORM_CRITICAL_POINT	0
#define DETECT_HARD_ID				0
#define DETECT_DIRECTION			0
#define LOAD_STORE_BG				0
#define REOPEN_CAM_WHEN_TIME_OVER	1					//Reopen cam when processing time > TIME_LIMIT
#define TIME_LIMIT					3000
#define CHECK_INSIDE_ROI			0
#define PIXEL2GPS					1

//sola
#define GHOST_REMOVE				1				//Assume BGM_DYNAMIC=1.
#define SEND_DATA					1

#define PREDICTIVE_TRACK			0
#define KALMAN_TRACK				0

// Debug
#define DEBUG_FINAL					1
#define DEBUG_CRITICAL_POINT		0
#define DEBUG_TRACKING				0
#define DEBUG_NONZ_SEG				0
#define DEBUG_NONZ_SEG_TWICE		0
#define DEBUG_SHADOW_DET			0
#define DEBUG_MVO_CLASSSIFY			0
#define DEBUG_BKG_UPDATE			0
#define DEBUG_TARGET_LINE			0
#define DEBUG_SEND_DATA				0

//sola
#define DEBUG_SUB					0
#define DEBUG_RAW_SEG_SAL_N_SIZE	0
#define DEBUG_TRACK_DIFF			0
#define DEBUG_RUNNING_TIME			1
#define DEBUG_GPS					0
#define	DEBUG_AUTO_CAR_DETECTION	0

#define DEBUG_IMG_IDX				0
#define TARGET_IMG_IDX				173

#define SAVE_NEW_BG					0

#define EXCLUDE_SMALL_MVOS_IN_TRK	1		//in tracking process, don't consider small MVOs

#define PERS_VIEW					0


#if IP_CAM_NUM == 173
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.173/video1"
#define VIDEO_FILE					""
//#define VIDEO_FILE					"data/1.avi"
#define DATASET_DIR					"data/173/1/173_2017112310475_"
//#define DATASET_DIR					"data/174/2/174_2017112310576_"
//#define DATASET_DIR					"data/174/3/174_20171123104048_"
//#define DATASET_DIR					"data/174/4/174_20171123105146_"


#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%s%d%s"
#define FILE_EXT					".jpg"

#define FIRST_IMG_IDX				1066
#define LAST_IMG_IDX				1850
//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				909
//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				697
//#define FIRST_IMG_IDX				727
//#define LAST_IMG_IDX				1636
#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 1);
const Point2i ROI_BR(450, 1);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#endif

#if PIXEL2GPS
const Point2f pixel0(718, 545);
const Point2f pixel1(329, 415);
const Point2f pixel2(506, 181);
const Point2f pixel3(706, 140);

const Point2d mapDouble0(36.9691973, 127.8717136);
const Point2d mapDouble1(36.9691541, 127.8717977);
const Point2d mapDouble2(36.9688922, 127.8717741);
const Point2d mapDouble3(36.9687236, 127.8716605);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			1

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.1				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						300				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				1				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
#define LOW_H						36		/179.0*360.0
#define HIGH_H						51		/179.0*360.0
#define LOW_S						47		/255.0
#define HIGH_S						169		/255.0
#define LOW_V						101		/255.0
#define HIGH_V						255		/255.0
//#define LOW_H						0
//#define HIGH_H						160
//#define LOW_S						0
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			10			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200

#endif

#if IP_CAM_NUM == 174
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.174/video1"
#define VIDEO_FILE					""
//#define VIDEO_FILE					"data/1.avi"
//#define DATASET_DIR					"data/174/1/174_2017111715310_"
//#define DATASET_DIR					"data/174/2/174_2017112310576_"
#define DATASET_DIR					"data/174/3/174_20171123104048_"
//#define DATASET_DIR					"data/174/4/174_20171123105146_"


//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%s%d%s"
#define FILE_EXT					".jpg"

//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				171
//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				909
#define FIRST_IMG_IDX				0
#define LAST_IMG_IDX				697
//#define FIRST_IMG_IDX				727
//#define LAST_IMG_IDX				1636

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(200, 80);
const Point2i ROI_BR(370, 80);
const Point2i ROI_TR(450, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(285, 528);
const Point2f pixel1(595, 207);
const Point2f pixel2(585, 332);
const Point2f pixel3(405, 350);

const Point2d mapDouble0(36.9696571, 127.8717345);
const Point2d mapDouble1(36.9704718, 127.8719885);
const Point2d mapDouble2(36.9698856, 127.8718563);
const Point2d mapDouble3(36.9698512, 127.8717732);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			1

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.1				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						100				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
#define LOW_H						25		/179.0*360.0
#define HIGH_H						56		/179.0*360.0
#define LOW_S						24		/255.0
#define HIGH_S						108		/255.0
#define LOW_V						236		/255.0
#define HIGH_V						255		/255.0
//#define LOW_H						0
//#define HIGH_H						160
//#define LOW_S						0
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			10			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200

#endif

#if IP_CAM_NUM == 175
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.175/video1"
#define VIDEO_FILE					""
//#define VIDEO_FILE					"data/1.avi"
//#define DATASET_DIR					"data/175/1/175_2017112310359_"
//#define DATASET_DIR					"data/175/2/175_20171123105121_"
#define DATASET_DIR					"data/175/3/175_20171123105740_"


//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%s%d%s"
#define FILE_EXT					".jpg"

//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				351
//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				415
#define FIRST_IMG_IDX				825
#define LAST_IMG_IDX				1359

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 270);
const Point2i ROI_BR(460, 1);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(596, 444);
const Point2f pixel1(330, 573);
const Point2f pixel2(816, 149);
const Point2f pixel3(963, 517);

const Point2d mapDouble0(36.9718700, 127.8712666);
const Point2d mapDouble1(36.9719497, 127.8711135);
const Point2d mapDouble2(36.9718047, 127.8718005);
const Point2d mapDouble3(36.9717033, 127.8712464);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			1

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.1				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						100				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
#define LOW_H						34		/179.0*360.0
#define HIGH_H						60		/179.0*360.0
#define LOW_S						49		/255.0
#define HIGH_S						255		/255.0
#define LOW_V						126		/255.0
#define HIGH_V						255		/255.0
//#define LOW_H						0
//#define HIGH_H						160
//#define LOW_S						0
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			40			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200
#endif

#if IP_CAM_NUM == 176
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.176/video1"
#define VIDEO_FILE					""
//#define VIDEO_FILE					"data/1.avi"
//#define DATASET_DIR					"data/176/1/176_20171123103424_"
//#define DATASET_DIR					"data/176/2/176_20171123104944_"
//#define DATASET_DIR					"data/176/3/176_20171123105044_"
#define DATASET_DIR					"data/176/4/176_20171123105829_"


//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%s%d%s"
#define FILE_EXT					".jpg"

//#define FIRST_IMG_IDX				603
//#define LAST_IMG_IDX				1170
//#define FIRST_IMG_IDX				0
//#define LAST_IMG_IDX				882
//#define FIRST_IMG_IDX				345
//#define LAST_IMG_IDX				940
#define FIRST_IMG_IDX				0
#define LAST_IMG_IDX				652

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 250);
const Point2i ROI_BR(SIZE_HOR - 1, 30);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(440, 547);
const Point2f pixel1(408, 478);
const Point2f pixel2(874, 386);
const Point2f pixel3(1101, 576);

const Point2d mapDouble0(36.9722769, 127.8704997);
const Point2d mapDouble1(36.9723460, 127.8704575);
const Point2d mapDouble2(36.9724826, 127.8707809);
const Point2d mapDouble3(36.9722611, 127.8708721);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			1

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.1				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						100				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
#define LOW_H						34		/179.0*360.0
#define HIGH_H						60		/179.0*360.0
#define LOW_S						49		/255.0
#define HIGH_S						255		/255.0
#define LOW_V						126		/255.0
#define HIGH_V						255		/255.0
//#define LOW_H						0
//#define HIGH_H						160
//#define LOW_S						0
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			30			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200

#endif

#if IP_CAM_NUM == 177
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.177/video1"
#define VIDEO_FILE					""
//#define VIDEO_FILE					"data/1.avi"
//#define DATASET_DIR					"data/177/177_20171123104137_"
#define DATASET_DIR					"data/177/177_20171123105318_"
//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%s%d%s"
#define FILE_EXT					".jpg"

#define FIRST_IMG_IDX				3000
#define LAST_IMG_IDX				3476

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 70);
const Point2i ROI_BR(SIZE_HOR - 1, 70);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(531, 558);
const Point2f pixel1(846, 321);
const Point2f pixel2(940, 448);
const Point2f pixel3(1215, 392);

const Point2d mapDouble0(36.9682701, 127.8715543);
const Point2d mapDouble1(36.9683786, 127.8713520);
const Point2d mapDouble2(36.9683281, 127.8715161);
const Point2d mapDouble3(36.9684011, 127.8714794);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			1

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.1				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						100				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
#define LOW_H						12		/179.0*360.0
#define HIGH_H						61		/179.0*360.0
#define LOW_S						59		/255.0
#define HIGH_S						255		/255.0
#define LOW_V						0		/255.0
#define HIGH_V						255		/255.0
//#define LOW_H						0
//#define HIGH_H						160
//#define LOW_S						0
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			2500			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200
#endif

#if IP_CAM_NUM == 181
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.181/video1"
#define VIDEO_FILE					"data/182_170929.avi"
//#define VIDEO_FILE					"data/1.avi"
#define DATASET_DIR					"data/182_175_"
//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%0.4d"
#define FILE_EXT					".jpg"
#define FIRST_IMG_IDX				0 
#define LAST_IMG_IDX				998

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 20);
const Point2i ROI_BR(SIZE_HOR - 1, 20);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(575, 380);
const Point2f pixel1(486, 223);
const Point2f pixel2(786, 232);
const Point2f pixel3(1164, 301);

const Point2d mapDouble0(36.9667088, 127.8716052);
const Point2d mapDouble1(36.9665775, 127.8715658);
const Point2d mapDouble2(36.9666157, 127.8714914);
const Point2d mapDouble3(36.9667029, 127.8714487);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			0

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.5				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						150				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
//#define LOW_H						25
//#define HIGH_H						70
//#define LOW_S						26
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define LOW_H						0
#define HIGH_H						160
#define LOW_S						0
#define HIGH_S						255
#define LOW_V						0
#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			1			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200
#endif

#if IP_CAM_NUM == 182
// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.182/video1"
#define VIDEO_FILE					"data/182_170929.avi"
//#define VIDEO_FILE					"data/1.avi"
#define DATASET_DIR					"data/182_175_"
//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bus_bg.jpg"
#define FILE_FORMAT					"%0.4d"
#define FILE_EXT					".jpg"
#define FIRST_IMG_IDX				0 
#define LAST_IMG_IDX				998

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

const Point2i ROI_BL(1, 35);
const Point2i ROI_BR(SIZE_HOR - 1, 30);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER - 1);
const Point2i ROI_TL(1, SIZE_VER - 1);
///////////////////////////////////////////////////////////////
//////////	Perspective transform static setup	///////////////
///////////////////////////////////////////////////////////////

// Test sequence 1 (real serveillance camera)
//const Point2f Trans_BL(298, 58);
//const Point2f Trans_BR(399, 75);
//const Point2f Trans_TR(300, 265);
//const Point2f Trans_TL(2, 150);

#if PIXEL2GPS
const Point2f pixel0(547, 234);
const Point2f pixel1(265, 204);
const Point2f pixel2(84, 166);
const Point2f pixel3(34, 344);

const Point2d mapDouble0(36.9670609, 127.8717418);
const Point2d mapDouble1(36.9670859, 127.8716417);
const Point2d mapDouble2(36.9671451, 127.8715260);
const Point2d mapDouble3(36.9669830, 127.8716165);

//For GPS transforamtion
#define LAT_SAME_DIGIT						3
#define LON_SAME_DIGIT						4
#define LAT_WHOLE_DIGIT						9
#define LON_WHOLE_DIGIT						10
#define LAT_PRECISION						7
#define LON_PRECISION						7

//to calculate velocity
#define VELOCITY_LIMIT						5500			

#endif
#endif

#define VEL_PRECISION						2
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;

#define AUTO_CAR_DETECTION			0

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.5				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						150				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Thresholds and gains
#define TL_MIN						0.05				  // suppression noise level threshold after the background subtraction step (min)
//#define TL_MIN						0.03				  // suppression noise level threshold after the background subtraction step (min)
#define TL_MAX						3.00              // suppression noise level threshold after the background subtraction step (max)
#define ALPHA						0.4               // the low threshold for detecting shadow as presented in the paper
#define BETA						1.0               // the high threshold for detecting shadow as presented in the paper
#define SHT							550               // the minimum salient property needed for an object before applying the shadow suppression step
#define OT							30                // the threshold non - zero pixels for an object

//***
#define SAL							25				  // the object should be large enough for MVO and GHOST detection
//#define SAL							30				  // the object should be large enough for MVO and GHOST detection
#define SIZE						5				  // the minimum size of the segmented object

//***
#define LARGE_OBJ_DENS				25				  // the minimum pixel density which indicates a large object
#define LARGE_OBJECT_SIZE			5				  // the minimum size in both horizontal and vertical directions of a large object

#define MVT_MIN						0.002			  // the threshold to differentiate the immediate MV from noise (min)
#define MVT_MAX						3.00			  // the threshold to differentiate the immediate MV from noise (max)

#define BBMIN						4                 // the minimum size of the bounding box
#define SEGMIN						3                // the minimum distance between two separated segments
//#define SEGMIN						7                // the minimum distance between two separated segments

#define GFTHR_LOW					220				  // the lower bound after applying a Gaussian filter for shadow objects
#define GFTHR_HIGH					255				  // the upper bound after applying a Gaussian filter for shadow objects
#define ROI_VEC						5				  // outer-inner ROI gap
#define VECTOR_UNIT_LENGTH			30
#define MOVING_THRESH				5

// Moving object classification
#define ANN							0
#define RANTREE						0
#define SIZE_SALIENCY				1

#define STORE_MVO					0				 
#define TRAIN_MODEL					0

//detecting autoCar
//#define LOW_H						25
//#define HIGH_H						70
//#define LOW_S						26
//#define HIGH_S						255
//#define LOW_V						0
//#define HIGH_V						255
#define LOW_H						0
#define HIGH_H						160
#define LOW_S						0
#define HIGH_S						255
#define LOW_V						0
#define HIGH_V						255
#define REFINE						0
#define NUM_COLOR_PIXEL_THR			1			


// Tracking for MVO & autoCar
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define LIVE_AUTO_CAR_SIZE			1
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200
#endif


#endif // !_COMMON_H_
