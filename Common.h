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

//send data
#include "camToCar.h"
//time stamp
#include "timeStamp.h"

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
#define STATIC_IMAGE				0			  
#define VIDEO						0			  
#define CAMERA						1  

// Dataset configurations
#define CAM_ID						"rtsp://admin:1234@222.116.156.182/video1"
#define VIDEO_FILE					"data/182_170929.avi"
#define DATASET_DIR					"../../../Datasets/intersection/image_"
//#define BG_FILE						"../../../Datasets/intersection/image_3279.jpg"
#define BG_FILE						"data/bg_182_170929.jpg"
#define FILE_FORMAT					"%0.4d"
#define FILE_EXT					".jpg"
#define FIRST_IMG_IDX				0 
#define LAST_IMG_IDX				4999 

// Image size
#define SIZE_HOR					640
#define SIZE_VER					360
//#define SIZE_HOR					480
//#define SIZE_VER					270



/*
ROI
*/
#define STATIC_ROI					1

/*
Perspective transformation
*/
#define IS_USE_PER_TRANS			0

#if STATIC_ROI
///////////////////////////////////////////////////////////////
//////////			ROI static setup			///////////////
///////////////////////////////////////////////////////////////

//const Point2i ROI_BL(1, 22);
//const Point2i ROI_BR(479, 22);
//const Point2i ROI_TR(479, 269);
//const Point2i ROI_TL(1, 269);
const Point2i ROI_BL(1, 22);
const Point2i ROI_BR(SIZE_HOR - 1, 22);
const Point2i ROI_TR(SIZE_HOR - 1, SIZE_VER-1);
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
const uint32_t Trans_W = 500;
const uint32_t Trans_H = 450;



/*
CarSnukt detector
*/
#define MVO_VIRTUAL_ID_START				2000 //autoBus = 1000~1999, MVO = 2000~2999
#define MOV_VIRTUAL_ID_END					2999

//For GPS transforamtion
#define LON_OFFSET					(int64_t) 1278700000
#define LAT_OFFSET					(int64_t) 369720000

// For background update algorithm
#define BGM_DYNAMIC					1				// 1: a dynamic background model (BGM) is used, otherwise a statistical BGM is used
#define BGM_WB						1.5				// The weight for the update of the current background model 
#define BGM_N						3				// The number of background image candidates
#define BGM_DT						150				// The BGM update interval (frames)
#define INITAIL_BGM_DT				10				// The BGM update interval (frames)
#define BGM_KNOWLEDGE				0				// 1: use the knowledge-base BGM, default 0
#define BGM_STABLE_CNT				BGM_N + 3		

// Flags
#define BGM_FIRST_BUILD				1					// use the 1st frame for background. 
#define WAIT_BGM_BUILD				1					// wait until bgm is built.
#define BGM_BUILD_WATITING_FRAME	INITAIL_BGM_DT*5	//waiting time for bgm built.
#define SHADOW_REMOVAL				0
#define TRANSFORM_CENTER_POINT		0
#define TRANSFORM_CRITICAL_POINT	0
#define DETECT_HARD_ID				0
#define DETECT_DIRECTION			0
#define LOAD_STORE_BG				0
//sola
#define GHOST_REMOVE				0				//Assume BGM_DYNAMIC=1.
#define SEND_DATA					1


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

// Tracking for MVO
#define HIS_POS_SIZE				15
#define LIVE_OBJECT_SIZE			100
#define HISTOGRAM_BIN_SIZE			256
#define UNKNOW_HARD_ID				111

#define DISTANCE_THRES				200
#define SIZE_THRES					200
#define HISTOGRAM_THRES				200

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

//sola
#define DEBUG_SUB					0
#define DEBUG_RAW_SEG_SAL_N_SIZE	0
#define DEBUG_TRACK_DIFF			0
#define DEBUG_RUNNING_TIME			0

#define DEBUG_IMG_IDX				0
#define TARGET_IMG_IDX				0

#define SAVE_NEW_BG					0

#define EXCLUDE_SMALL_MVOS_IN_TRK	1		//in tracking process, don't consider small MVOs

#define PERS_VIEW					0

#endif // !_COMMON_H_
