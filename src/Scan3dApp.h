#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"
#include "cv.h"
#include "ofxLine2d.h"
#include "ofxLine3d.h"
#include "ofxRay3d.h"
#include "ofxPlane.h"
#include <sstream>

enum {CAMERA_CALIBRATION, SETUP, CAPTURE, PROCESSING,RECONSTRUCTION,POINTS3D};


enum Setup{TOP_SECTION, BOTTOM_SECTION, T_TL, T_TR, T_BL, T_BR, B_TL, B_TR, B_BL, B_BR,ESTIMATE_CAMERA, WAITING};


enum CameraCalibration{CAM_CAL_PROCESSING, CAM_CAL_LOADING, CAM_CAL_WAITING};
enum Points3d{POINTS3D_PROCESSING, POINTS3D_WAITING};


enum { COLOR, GRAYSCALE, MINIMAGE, MAXIMAGE, SHADOWTHRESHIMAGE, DIFF, THRESH, EDGE, CORNER};
enum { UP, DOWN, LEFT, RIGHT, VERTICAL, HORIZONTAL, BOTH, LOG};
enum {VIDEO,IMAGE_SEQUENCE,NONE};



class Scan3dApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void camCalUpdate();
		void setupUpdate();
		void captureUpdate();
		void processingUpdate();
		void points3dUpdate();

		void draw();
		void camCalDraw();
		void setupDraw();
		void captureDraw();
		void processingDraw();
		void visualizationDraw();

		void loadSettings();
		void saveSettings();
		void clearSettings();
		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawSectionRectangles();
		void drawMarkerPoints();
		ofxLine2d computeLineFromZeroCrossings(ofxCvGrayscaleImage img, ofRectangle roi);
		bool isPointInRegion(ofPoint pt, ofRectangle roi);
		ofPoint getNearestCorner(ofxCvGrayscaleImage img, int windowSize, int x, int y);
		void computeExtrinsicMatrix(ofPoint *objectPoints, ofPoint *imagePoints, CvMat* cameraMatrix, CvMat* distCoeffs);
		void convertOfPointsToCvMat(ofPoint *pts, int dimensions, int size, CvMat* output);
		ofxRay3d pixelToRay(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint imagePt);
		ofPoint pt3DToPixel(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint pt3D);
		//ofPoint rayPlaneIntersection(ofPoint planePt, ofVec3f planeNormal, ofPoint rayOrigin, ofVec3f rayDirection);
		ofxLine3d projectLineOntoPlane(ofxLine2d line, ofxPlane plane, const CvMat* intrinsicMat, const CvMat* extrinsicMat);

		ofxCvGrayscaleImage computeGradientImage(ofxCvGrayscaleImage &input, int direction);
		ofxPlane getPlaneFromFrameIndex(float fi);

		float getFrameFromColor(ofColor color);
		void writePointsToFile(int numPoints);

		bool isPlaneAtFrameIndex(float fi);

		int sobelHorizontal[3][3];
    	int sobelVertical[3][3];

    	int laplacianOfGaussian[3][3];

		ofVideoPlayer vid;

		ofxXmlSettings settings;
		string inputVideoFile;

		//Calibration points for the planes the object rests on
		ofPoint verticalPlaneImagePts[4];
		ofPoint horizontalPlaneImagePts[4];

		ofPoint verticalPlaneObjectPts[4];
		ofPoint horizontalPlaneObjectPts[4];
    
        //Images
        ofxCvColorImage colorFrame;
        ofxCvGrayscaleImage grayscaleFrame;

        int frameIndex;
        int numFrames;

        int displayState;
        int programState;
        int setupSubState;
        int camCalSubstate;
        int points3dSubstate;
        int inputType;

        int width,height;

        ofxCvGrayscaleImage minImg;
        ofxCvGrayscaleImage maxImg;
        ofxCvColorImage maxColorImg;
        ofxCvGrayscaleImage shadowThreshImg;
	    ofxCvColorImage temporalImg;
	    ofxCvGrayscaleImage diffFrame;
	    ofxCvGrayscaleImage previousDiffFrame;
	    ofxCvGrayscaleImage enterFrame;
	    ofxCvGrayscaleImage exitFrame;

        ofImage bufferOfImage;
        ofxCvColorImage bufferOfxCvColorImage;
        ofxCvGrayscaleImage bufferOfxCvGrayscaleImage;

        vector<ofxCvGrayscaleImage> frames;
        vector<ofxPlane> planes;

        unsigned int frameBufferSize;
        int zeroCrossingThreshold;

        ofRectangle topSection;
		ofRectangle bottomSection;
		ofRectangle fullSection;
		
		ofColor topSectionColor;
		ofColor bottomSectionColor;

		bool settingTopSection;
		bool settingBottomSection;

		int messageBarHeight;

		string messageBarText;
		string messageBarSubText;

		ofTrueTypeFont messageBarFont;
		ofTrueTypeFont messageBarSubTextFont;

		ofxLine2d topLine;
		ofxLine2d bottomLine;
		ofxLine2d testLine;

		string filename;

		vector<ofPoint> points;
		vector<ofColor> colors;

		ofDirectory dir;

		// Calibration Variables (Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)

		ofDirectory camCalDir;

		unsigned int camCalFrame;
		int boardXCount, boardYCount;
		unsigned int numBoards;
		int boardPatternSize;
		float boardSquareSize; //in mm
		int successes;
		CvSize boardNumInternalCornersCV;

		CvMat* image_points;
		CvMat* object_points;
		CvMat* point_counts;
		CvMat* intrinsic_matrix;
		CvMat* extrinsic_matrix;
		CvMat* distortion_coeffs;

		CvPoint2D32f* corners;
		int corner_count;
		IplImage* mapx;
		IplImage* mapy;

		string intrinsicFilename;
		string distortionFilename;

		ofPoint camPos;
		CvMat* camRotMat;
		CvMat* invIntrinsic;

		ofxPlane vertPlane;
		ofxPlane horizPlane;

		bool paused;


        
};
