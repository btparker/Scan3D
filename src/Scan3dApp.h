#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"
#include "cv.h"
#include <sstream>



class Scan3dApp : public ofBaseApp{

	public:
		void setup();
		void update();
		
		void draw();
	

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);

		void convertOfPointsToCvMat(vector<ofPoint> pts, int dimensions, CvMat* output);
		
		void writePointsToFile(vector<ofPoint> pts,vector<ofPoint> colors, string filename);

		ofEasyCam easyCam;

		string messageBarText;
		string messageBarSubText;

		int messageBarHeight;

		ofFont messageBarFont;
		ofFont messageBarSubTextFont;
	
        
};
