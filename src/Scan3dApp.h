#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"
#include "cv.h"
#include "ofxLine.h"

enum {SETUP, CAPTURE, PROCESSING,RECONSTRUCTION,VISUALIZATION};
enum { COLOR, GRAYSCALE, MINIMAGE, MAXIMAGE, SHADOWTHRESHIMAGE, DIFF, THRESH, EDGE, CORNER};
enum { UP, DOWN, LEFT, RIGHT, VERTICAL, HORIZONTAL, BOTH};
enum {VIDEO,NONE};



class Scan3dApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void loadSettings();
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
		ofxLine computeLineEquationFromZeroCrossings(ofxCvGrayscaleImage img, ofRectangle roi);

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
        int inputType;

        int width,height;


        ofxCvGrayscaleImage minImg;
        ofxCvGrayscaleImage maxImg;
        ofxCvColorImage maxColorImg;
        ofxCvGrayscaleImage shadowThreshImg;
	    ofxCvColorImage temporalImg;

        ofImage bufferOfImage;
        ofxCvColorImage bufferOfxCvColorImage;
        ofxCvGrayscaleImage bufferOfxCvGrayscaleImage;

        vector<ofxCvGrayscaleImage> frames;
        vector<ofxCvGrayscaleImage> diffFrames;
        vector<ofxCvGrayscaleImage> zeroCrossingFrames;

        int frameBufferSize;
        int zeroCrossingThreshold;

        ofRectangle topSection;
		ofRectangle bottomSection;
		
		ofColor topSectionColor;
		ofColor bottomSectionColor;

		bool settingTopSection;
		bool settingBottomSection;

		int messageBarHeight;

		string messageBarText;
		string messageBarSubText;

		ofTrueTypeFont messageBarFont;
		ofTrueTypeFont messageBarSubTextFont;

};
