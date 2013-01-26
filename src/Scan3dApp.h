#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"

enum { COLOR, GRAYSCALE, DIFF, THRESH, EDGE, CORNER};
enum { UP, DOWN, LEFT, RIGHT, VERTICAL, HORIZONTAL, BOTH};
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

		ofVideoPlayer vid;

		ofxCvGrayscaleImage computeGradientImage(ofxCvGrayscaleImage &input, int direction);

		ofxXmlSettings settings;
		string inputVideoFile;

		//Calibration points for the planes the object rests on
		ofPoint verticalPlanePts[4];
		ofPoint horizontalPlanePts[4];
    
        //Images
        ofxCvColorImage currentColorFrame;
        ofxCvColorImage previousColorFrame;
        ofxCvGrayscaleImage currentGrayscaleFrame;
		ofxCvGrayscaleImage previousGrayscaleFrame;		
        ofxCvGrayscaleImage diffFrame;
        //ofxCvGrayscaleImage threshImages;
    	//ofxCvGrayscaleImage edgeImage;
        
        int frameIndex;

        int displayState;

        int sobelHorizontal[3][3];
    	int sobelVertical[3][3];

		//ofxCvGrayscaleImage cornerMap;
};
