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

enum {CAMERA_CALIBRATION, PROJECTOR_CALIBRATION, SETUP, CAPTURE, PROCESSING,RECONSTRUCTION,POINTS3D};


enum Setup{TOP_SECTION, BOTTOM_SECTION, T_TL, T_TR, T_BL, T_BR, B_TL, B_TR, B_BL, B_BR,ESTIMATE_CAMERA, WAITING};


enum CameraCalibration{CAM_CAL_PROCESSING, CAM_CAL_LOADING, CAM_CAL_WAITING};
enum ProjectorCalibration{PROJ_CAL_PROCESSING, PROJ_CAL_LOADING, PROJ_CAL_WAITING,BR,TR,TL,BL, BIN, GRAY};
enum Points3d{POINTS3D_PROCESSING, POINTS3D_WAITING};


enum { COLOR, GRAYSCALE, MINIMAGE, MAXIMAGE, SHADOWTHRESHIMAGE, DIFF, THRESH, EDGE, CORNER, BINCODE, CTEMPORAL, RTEMPORAL};
enum { UP, DOWN, LEFT, RIGHT, VERTICAL, HORIZONTAL, BOTH, LOG};
enum {VIDEO,IMAGE_SEQUENCE,NONE};



class Scan3dApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void camCalUpdate();
		void projCalUpdate();
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
		void clearSetupSettings();
		void clearProjectorSettings();
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
		void computeCameraExtrinsicMatrix(ofPoint *objectPoints, ofPoint *imagePoints, CvMat* cameraMatrix, CvMat* distCoeffs);
		void convertOfPointsToCvMat(ofPoint *pts, int dimensions, int size, CvMat* output);
		ofxRay3d camPixelToRay(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint imagePt);
		ofPoint pt3DToPixel(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint pt3D);
		//ofPoint rayPlaneIntersection(ofPoint planePt, ofVec3f planeNormal, ofPoint rayOrigin, ofVec3f rayDirection);
		ofxLine3d projectLineOntoPlane(ofxLine2d line, ofxPlane plane, const CvMat* intrinsicMat, const CvMat* extrinsicMat);

		ofxCvGrayscaleImage computeGradientImage(ofxCvGrayscaleImage &input, int direction);
		ofxPlane getPlaneFromFrameIndex(float fi);

		float getFrameFromColor(ofColor color);
		void writePointsToFile(int numPoints);

		bool isPlaneAtFrameIndex(float fi);

		void drawPointCloud();

		void assertPoint(ofPoint pt);

		ofxCvGrayscaleImage computeBinCodeImage(int w, int h, int power, bool inverse, int type);
		ofxCvGrayscaleImage computeGrayCodeImage(int w, int h, int power, bool inverse, int type);

		void setCameraAndProjector();

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

		ofPoint projPlaneObjectPts[4];
		ofPoint projPlaneImagePts[4];

		//Images
		ofxCvColorImage colorFrame;
		ofxCvGrayscaleImage grayscaleFrame;

		int frameIndex;
		int colIndex;
		int rowIndex;
		int numFrames;

		int displayState;
		int programState;
		int setupSubState;
		int camCalSubstate;
		int projCalSubstate;
		int points3dSubstate;
		int inputType;

		int width,height;
		int projWidth,projHeight;
		int projType;

		float screenScale;



		ofxCvGrayscaleImage minImg;
		ofxCvGrayscaleImage maxImg;
		ofxCvColorImage maxColorImg;
		ofxCvGrayscaleImage shadowThreshImg;
		ofxCvColorImage temporalImg;
		ofxCvGrayscaleImage diffFrame;
		ofxCvGrayscaleImage invDiffFrame;
		ofxCvGrayscaleImage noiseImg;
		ofxCvGrayscaleImage codeImg;

		ofImage bufferOfImage;
		ofxCvColorImage bufferOfxCvColorImage;
		ofxCvGrayscaleImage bufferOfxCvGrayscaleImage;

		vector<ofxCvGrayscaleImage> frames;
		vector<ofxCvGrayscaleImage> colFrames;
		vector<ofxCvGrayscaleImage> invColFrames;
		vector<ofxCvGrayscaleImage> rowFrames;
		vector<ofxCvGrayscaleImage> invRowFrames;
		vector<ofxCvGrayscaleImage> codeFrames;
		vector<ofxPlane> planes;

		unsigned int frameBufferSize;
		int maxThreshold;
		int minThreshold;

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
		int camBoardXCount, camBoardYCount;
		unsigned int camNumBoards;
		int camBoardPatternSize;
		float camBoardSquareSize; //in mm
		int camSuccesses;
		CvSize camBoardNumInternalCornersCV;

		CvMat* cam_image_points;
		CvMat* cam_object_points;
		CvMat* cam_point_counts;
		CvMat* cam_intrinsic_matrix;
		CvMat* cam_extrinsic_matrix;
		CvMat* cam_distortion_coeffs;

		CvPoint2D32f* cam_corners;
		int cam_corner_count;
		IplImage* cammapx;
		IplImage* cammapy;
		
		IplImage* rowMapping16Img;
		IplImage* colMapping16Img;

		int numColMapFrames;
		int numRowMapFrames;
		
		ofxCvColorImage rowMappingColorImg;
		ofxCvColorImage colMappingColorImg;

		string camIntrinsicFilename;
		string camDistortionFilename;

		ofPoint camPos;
		CvMat* camRotMat;
		CvMat* camInvIntrinsic;


		ofDirectory projCalDir;

		unsigned int projCalFrame;
		int projBoardXCount, projBoardYCount;
		unsigned int projNumBoards;
		int projBoardPatternSize;
		float projBoardSquareSize; //in mm
		int projSuccesses;
		CvSize projBoardNumInternalCornersCV;

		CvMat* proj_image_points;
		CvMat* proj_object_points;
		CvMat* proj_point_counts;
		CvMat* proj_intrinsic_matrix;
		CvMat* proj_extrinsic_matrix;
		CvMat* proj_distortion_coeffs;

		CvPoint2D32f* proj_corners;
		int proj_corner_count;
		IplImage* projmapx;
		IplImage* projmapy;

		string projIntrinsicFilename;
		string projDistortionFilename;

		ofPoint projPos;
		CvMat* projRotMat;
		CvMat* projInvIntrinsic;




		ofxPlane vertPlane;
		ofxPlane horizPlane;
		ofxPlane negYnegZPlane;

		bool paused;

		ofEasyCam easyCam;

		bool bDrawPointCloud;
		ofMesh planePts;

		ofVec2f cam_focal_length;
		ofVec2f cam_principal_point;

		ofVec2f proj_focal_length;
		ofVec2f proj_principal_point;
        
};
