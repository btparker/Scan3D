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
};

#endif