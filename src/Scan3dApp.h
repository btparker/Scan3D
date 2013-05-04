#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"
#include "cv.h"
#include "ofxLine.h"

enum {CAMERA_CALIBRATION, SETUP, CAPTURE, PROCESSING,RECONSTRUCTION,VISUALIZATION};


enum Setup{TOP_SECTION, BOTTOM_SECTION, T_TL, T_TR, T_BL, T_BR, B_TL, B_TR, B_BL, B_BR, WAITING};


enum CameraCalibration{TL, TR, BL, BR, CAM_CAL_WAITING};


enum { COLOR, GRAYSCALE, MINIMAGE, MAXIMAGE, SHADOWTHRESHIMAGE, DIFF, THRESH, EDGE, CORNER};
enum { UP, DOWN, LEFT, RIGHT, VERTICAL, HORIZONTAL, BOTH};
enum {VIDEO,IMAGE_SEQUENCE,NONE};



class Scan3dApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void camCalUpdate();
		void setupUpdate();
		void captureUpdate();
		void processingUpdate();
		void visualizationUpdate();

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
		ofxLine computeLineFromZeroCrossings(ofxCvGrayscaleImage img, ofRectangle roi);
		bool isPointInRegion(ofPoint pt, ofRectangle roi);
		ofPoint getNearestCorner(ofxCvGrayscaleImage img, int windowSize, int x, int y);

		ofVideoPlayer vid;

		ofxXmlSettings settings;
		string inputVideoFile;

		//Calibration points for the planes the object rests on
		ofPoint verticalPlanePts[4];
		ofPoint horizontalPlanePts[4];
    
        //Images
        ofxCvColorImage colorFrame;
        ofxCvGrayscaleImage grayscaleFrame;

        int frameIndex;

        int displayState;
        int programState;
        int setupSubState;
        int camCalSubstate;
        int inputType;

        int width,height;

        ofxCvGrayscaleImage minImg;
        ofxCvGrayscaleImage maxImg;
        ofxCvColorImage maxColorImg;
        ofxCvGrayscaleImage shadowThreshImg;
	    ofxCvColorImage temporalImg;
	    ofxCvGrayscaleImage diffFrame;
	    ofxCvGrayscaleImage previousDiffFrame;
	    ofxCvGrayscaleImage zeroCrossingFrame;

        ofImage bufferOfImage;
        ofxCvColorImage bufferOfxCvColorImage;
        ofxCvGrayscaleImage bufferOfxCvGrayscaleImage;

        vector<ofxCvGrayscaleImage> frames;

        int frameBufferSize;
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

		ofxLine topLine;
		ofxLine bottomLine;
		ofxLine testLine;

		string filename;

		vector<int> columnIndices;

		ofDirectory dir;

		// Calibration Variables (Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)

		ofDirectory camCalDir;

		int camCalFrame;
		int boardXCount, boardYCount;
		int numBoards;
		int boardPatternSize;
		float boardSquareSize; //in mm
		int successes;
		CvSize boardNumInternalCornersCV;

		CvMat* image_points;
		CvMat* object_points;
		CvMat* point_counts;
		CvMat* intrinsic_matrix;
		CvMat* distortion_coeffs;

		CvPoint2D32f* corners;
		int corner_count;
		IplImage* mapx;
		IplImage* mapy;




        
};
