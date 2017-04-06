/********************************************************************
Projection time:	2016/11/03   10:55
File Name: 	projectorCamera.h
@author:	<hzzhouzhimin@corp.netease.com>
Copyright (c) 2017, NetEase Inc. All rights reserved.

note:       projectorCamera system processing file
*********************************************************************/


#ifndef PROJECTOR_CAMERA
#define PROJECTOR_CAMERA

#include "Htime.h"
#include <Math.h>
#include "sendEvent.h"
#include <numeric>
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include "role.h"
#include "inTheAirGesture.h"
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
//LOG DEFINITIONS
#define LOG_TAG_FLOW "ProjectorCamera/ProcessingFlow"
#define LOGF(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_FLOW, __VA_ARGS__))

#define LOG_TAG "ProjectorCamera/DebugInformation"
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))

#define LOG_TAG_CALB "ProjectorCamera/CalibrationInformation"
#define LOGC(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_CALB, __VA_ARGS__))

#define LOG_TAG_WARN "ProjectorCamera/WarningInformation"
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG_WARN, __VA_ARGS__))

#define SCREEN_WIDTH_MM  744
#define SCREEN_HEIGHT_MM 434
#define CAMERA_HEIGHT_MM 800
#define SCREEN_WIDTH_PX  1280
#define SCREEN_HEIGHT_PX 720
#define SALVER_WIDTH_MM 190
#define DEPTH_SURFACE 650


struct StereoProjection
{
	vector<Point2f> cameraVertex;
	vector<Point2f> projectorVertex;
	vector<Point3f> proVertex3D;
	vector<float>   vertexDepth;
	vector<TouchHand> curtHands;
	vector<TouchHand> lastHands;
	Point2f center;
	int lastId,first;
	bool valid,boatDemo;
	StereoProjection() :valid(false),lastId(0),first(0){};
};

struct ObjectInfo{
    Mat object;
    Rect rec;
    float maxdepth,cArea;
    vector<float> angles;
    vector<Point> contour;
};

class ProjectorCamera
{
public:
	ProjectorCamera();
	~ProjectorCamera();

	void init();
	void calibration();
	void calibrationFixed();
	void processing(Mat& colorSrc, Mat& depthSrc);
	bool isHistContinue(Mat&);
	void getBgDepth();
	void getDynamicBgDepth();
	void getFgDepth();
	void getContour();
	void findFinger();
	void findOnDeskObject();
	void findInAirObject();
	void findInAirGesture();
	void refineRect(Rect& rec, Mat image);
	void refineFingerPosition(int);
	void transAxisCameraToPro();
	void fillDepthImageHole(Mat &);
	void convetFloatToChar(Mat& src, Mat& dst);
	void fingerTipDepthDistribution(Point2f&,float&,float&);
	int  intersectionPointsWithBoard(ObjectInfo&);
	Rect palmRectangle(ObjectInfo&);
	void palmInfor(ObjectInfo&, TouchHand&);
	float contourAverageDepth(vector<Point>&);
	float caclAngle(TouchHand&,int);
	float caclAngleForInAirGesture(Point& left, Point& center, Point& right);
	string  intTostring(int);
    Point2f fingerDirection(Point2f, Point2f, Point2f);
	void calibDepToPro(vector<Point2f>&, vector<float>&, vector<Point3f>&);
	void calibDepToPro(vector<Point2f>&, vector<float>&, vector<Point2f>&);
	Point2f calibDepToPro(Point2f p_cam, float depth);
	void calibCamToPro(vector<Point2f>&, vector<float>&, vector<Point3f>&);
	void calibCamToPro(vector<Point2f>&, vector<float>&, vector<Point2f>&);
	Point2f calibCamToPro(Point2f p_cam, float depth);
	Point2f homogCamToPro(Point2f);	
	void refineVerticals(vector<Point3f>&);
	void clockwiseContour(vector<Point2f>&);
	void clockwiseContour(vector<Point>&);

	//zhangbo
    bool isHandGraspOrOpen(double& handRatio, int& approxCurveSize, int& hullsize, int& littleangle, int& distance_x,int& distance_y,Point& centerpoint);
    int handWaveState(vector<Point>& pointsInLastFrames);
	Point CalculateSumofPoints(Point pointA, Point pointB);
    void findConvexityDefects(vector<Point>& contour, vector<int>& hull, vector<ConvexityDefect>& convexDefects);
    void inAirGestureManage(bool& openstate, int& handwave);


	//images 
	Mat colorImg, depthImg, irImage, depthImg_old, binaryCopy;
	Mat averaImg, foreground, foreground_store, foreground_copy, averaIR;		
	vector<Mat> bgdepths;
	Mat hist, homo;

	//rectangle
	Rect screenRoi,filterRoi;

	//parameters
	bool calibrated, initBg, setViewDepth;
	size_t  frameId, initFrames;	

	float nearSurFaceThresh,nearCameraThresh;
	double maxdepth;
	int histMinPixs;

	//calibration
	int gridstart_x, gridstart_y,gridwidth;
	int offset_x,offset_y,offset_init_x,offset_init_y;

	//android state 
	int fd, appState;

	//stereo projection result
	StereoProjection stereoProjectDesk,stereoProjectHover;	

	//objects in foreground image
	vector<ObjectInfo> objects;

    //procamera calibration
    string cablibParaFilePath;
    Mat colToProR, colToProT;
    Mat depToColR, depToColT;
    Mat camera_KK, camera_dis;
    Mat project_KK, project_dis;
    Mat depth_KK, depth_dis; 

    //homography
    Mat depToColHomo;
    Mat colToProHomo;
    Mat depToProHomo;

    //state model role
    Role fingerTouchRole;

    //zhangbo
    //in air gesture variables
	int ncount_totalimg, ncount_grasp;
	int ncount_totalimg_open, ncount_open;
	vector<Point>pointsInLastFrames;
	//InAirGestureInfo inairgesture;
	vector<bool> handgraspstates_temp;//hand grasp
    vector<int> handwavestates;//hand wave
    bool graspstate_temp;

	int lastframe;
	int lastframe_grasp;
	int lastframe_wave;
	
    bool m_graspstate;
    bool m_openstate;
    int m_wavestate;

	//output info-->for android
	bool graspstate_finnal;
	bool openstate_finnal;
	int wavestate_finnal;
	
	//dynamic background updating 
	vector<bool> ifBackground;
    int pixelInd;
    int fgNum, bgNum;
    vector<bool> isFingers;

    //for test
    vector<TouchPoint> touchPointsAll;
    int countid;
};
#endif