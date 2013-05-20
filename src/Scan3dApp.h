#pragma once

#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "ofxOpenCv.h"
#include "cv.h"
#include <sstream>
using namespace cv;


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

		void loadMesh(string filename, ofMesh* mesh);

		void setCamera();

		void convertOfPointsToCvMat(vector<ofPoint> points, int dimensions, CvMat* output);
		
		void writeMeshToFile(const ofMesh* mesh, string filename);

		void transformMesh(Mat mat, ofMesh* mesh);

		ofEasyCam easyCam;

		string messageBarText;
		string messageBarSubText;

		int messageBarHeight;

		ofTrueTypeFont messageBarFont;
		ofTrueTypeFont messageBarSubTextFont;

		ofMesh mesh0;
		ofMesh mesh1;
		ofMesh resultMesh;
	
        
};
