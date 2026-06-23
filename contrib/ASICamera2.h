// Minimal ZWO ASI Camera SDK 2 header for anyloop asi_source plugin.
// Covers the subset of the public API needed for single-camera video capture.
// Struct layout matches ASI SDK v1.39/v1.41.
#pragma once
#include <stdint.h>

typedef enum {
	ASI_SUCCESS                   =  0,
	ASI_ERROR_INVALID_INDEX,
	ASI_ERROR_INVALID_ID,
	ASI_ERROR_INVALID_CONTROL_TYPE,
	ASI_ERROR_CAMERA_CLOSED,
	ASI_ERROR_CAMERA_REMOVED,
	ASI_ERROR_INVALID_PATH,
	ASI_ERROR_INVALID_FILEFORMAT,
	ASI_ERROR_INVALID_SIZE,
	ASI_ERROR_INVALID_IMGTYPE,
	ASI_ERROR_OUTOF_BOUNDARY,
	ASI_ERROR_TIMEOUT,
	ASI_ERROR_INVALID_SEQUENCE,
	ASI_ERROR_BUFFER_TOO_SMALL,
	ASI_ERROR_VIDEO_MODE_ACTIVE,
	ASI_ERROR_EXPOSURE_IN_PROGRESS,
	ASI_ERROR_GENERAL_ERROR,
	ASI_ERROR_INVALID_MODE,
	ASI_ERROR_END                 = -1
} ASI_ERROR_CODE;

typedef enum {
	ASI_FALSE = 0,
	ASI_TRUE
} ASI_BOOL;

typedef enum {
	ASI_IMG_RAW8  = 0,
	ASI_IMG_RGB24 = 1,
	ASI_IMG_RAW16 = 2,
	ASI_IMG_Y8    = 3,
	ASI_IMG_END   = -1
} ASI_IMG_TYPE;

typedef enum {
	ASI_BAYER_RG = 0,
	ASI_BAYER_BG,
	ASI_BAYER_GR,
	ASI_BAYER_GB
} ASI_BAYER_PATTERN;

typedef enum {
	ASI_GAIN              =  0,
	ASI_EXPOSURE          =  1,
	ASI_GAMMA             =  2,
	ASI_WB_R              =  3,
	ASI_WB_B              =  4,
	ASI_OFFSET            =  5,
	ASI_BANDWIDTHOVERLOAD =  6,
	ASI_OVERCLOCK         =  7,
	ASI_TEMPERATURE       =  8,
	ASI_FLIP              =  9,
	ASI_AUTO_MAX_GAIN     = 10,
	ASI_AUTO_MAX_EXP      = 11,
	ASI_AUTO_TARGET_BRIGHTNESS = 12,
	ASI_HARDWARE_BIN      = 13,
	ASI_HIGH_SPEED_MODE   = 14,
	ASI_COOLER_POWER_PERC = 15,
	ASI_TARGET_TEMP       = 16,
	ASI_COOLER_ON         = 17,
	ASI_MONO_BIN          = 18,
	ASI_FAN_ON            = 19,
	ASI_PATTERN_ADJUST    = 20,
	ASI_ANTI_DEW_HEATER   = 21,
	ASI_CONTROL_END       = -1
} ASI_CONTROL_TYPE;

typedef struct {
	char            Name[64];
	int             CameraID;
	long            MaxHeight;
	long            MaxWidth;
	ASI_BOOL        IsColorCam;
	ASI_BAYER_PATTERN BayerPattern;
	int             SupportedBins[16];
	ASI_IMG_TYPE    SupportedVideoFormat[8];
	double          PixelSize;
	ASI_BOOL        MechanicalShutter;
	ASI_BOOL        ST4Port;
	ASI_BOOL        IsCoolerCam;
	ASI_BOOL        IsUSB3Host;
	ASI_BOOL        IsUSB3Camera;
	float           ElecPerADU;
	int             BitDepth;
	ASI_BOOL        IsTriggerCam;
	char            Unused[16];
} ASI_CAMERA_INFO;

int            ASIGetNumOfConnectedCameras(void);
ASI_ERROR_CODE ASIGetCameraProperty(ASI_CAMERA_INFO *pASICameraInfo, int iCameraIndex);
ASI_ERROR_CODE ASIOpenCamera(int iCameraID);
ASI_ERROR_CODE ASIInitCamera(int iCameraID);
ASI_ERROR_CODE ASICloseCamera(int iCameraID);
ASI_ERROR_CODE ASISetROIFormat(int iCameraID, int iWidth, int iHeight, int iBin, ASI_IMG_TYPE Img_type);
ASI_ERROR_CODE ASISetStartPos(int iCameraID, int iStartX, int iStartY);
ASI_ERROR_CODE ASISetControlValue(int iCameraID, ASI_CONTROL_TYPE ControlType, long lValue, ASI_BOOL bAuto);
ASI_ERROR_CODE ASIStartVideoCapture(int iCameraID);
ASI_ERROR_CODE ASIStopVideoCapture(int iCameraID);
ASI_ERROR_CODE ASIGetVideoData(int iCameraID, unsigned char *pBuffer, long lBufferSize, int iWaitms);
