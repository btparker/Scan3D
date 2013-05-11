#ifndef _OFX_LINE_3D
#define _OFX_LINE_3D

#include "ofMain.h"

class ofxLine3d{
	public:
		ofVec3f dir;
		ofPoint pt;

		bool initialized;

		ofxLine3d();
		ofxLine3d(float vx,float vy, float vz, float x0, float y0, float z0);
		ofxLine3d(ofVec3f dir, ofPoint pt);
		bool isInit();
		void set(float vx,float vy, float vz, float x0, float y0, float z0);
		//ofPoint intersection(ofxLine3d line);
		//ool isParallelTo(ofxLine3d line);
		//void drawLineInRegion(ofRectangle roi);
		//bool isLineInRegion(ofRectangle roi);
		//bool drawLineInRegion(ofRectangle roi, bool drawLine);
};

#endif