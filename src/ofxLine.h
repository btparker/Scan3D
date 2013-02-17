#ifndef _OFX_LINE
#define _OFX_LINE

#include "ofMain.h"

class ofxLine{
	public:
		ofVec2f dir;
		ofPoint pt;

		ofxLine();
		ofxLine(float vx,float vy, float x0, float y0);
		void set(float vx,float vy, float x0, float y0);
		ofPoint intersection(ofxLine line);
		bool isParallelTo(ofxLine line);
		void drawLineInRegion(ofRectangle roi);
		bool isLineInRegion(ofRectangle roi);
		bool drawLineInRegion(ofRectangle roi, bool drawLine);
};

#endif