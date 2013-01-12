#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"

class testApp : public ofBaseApp{

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

		ofxXmlSettings settings;

		//Calibration points for the planes the object rests on
		ofPoint verticalPlanePts[4];
		ofPoint horizontalPlanePts[4];
    
        //Image Vectors
        vector <ofImage> colorImages;
        vector <ofImage> gsImages;
        vector <ofImage> diffImages;
    
        int frameIndex;

		
};
