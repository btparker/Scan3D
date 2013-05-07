#ifndef _OFX_LINE_2d
#define _OFX_LINE_2d

#include "ofMain.h"

class ofxLine2d{
	public:
		ofVec2f dir;
		ofPoint pt;

		ofxLine2d();
		ofxLine2d(float vx,float vy, float x0, float y0);
		void set(float vx,float vy, float x0, float y0);
		ofPoint intersection(ofxLine2d line);
		bool isParallelTo(ofxLine2d line);
		void drawLineInRegion(ofRectangle roi);
		bool isLineInRegion(ofRectangle roi);
		bool drawLineInRegion(ofRectangle roi, bool drawLine);
};

#endif