#ifndef _OFX_LINE_2D
#define _OFX_LINE_2D

#include "ofMain.h"
#include "ofxLine3d.h"
#include "ofxPlane.h"

class ofxLine2d{
	public:
		ofVec2f dir;
		ofPoint pt;

		bool initialized;

		ofxLine2d();
		ofxLine2d(float vx,float vy, float x0, float y0);
		ofxLine2d(ofVec2f dir, ofPoint pt);
		bool isInit();
		void set(float vx,float vy, float x0, float y0);
		ofPoint intersection(ofxLine2d line);
		bool isParallelTo(ofxLine2d line);
		void drawLineInRegion(ofRectangle roi);
		bool isLineInRegion(ofRectangle roi);
		bool drawLineInRegion(ofRectangle roi, bool drawLine);
};

#endif