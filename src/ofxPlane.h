#ifndef _OFX_PLANE
#define _OFX_PLANE

#include "ofMain.h"

class ofxPlane{
	public:
		ofVec3f normal;
		ofPoint pt;
		float d;

		ofxPlane();
		ofxPlane(ofPoint pt, ofVec3f normal);
		void setNormal(float vx,float vy, float vz);
		void setNormal(ofVec3f normal);
		void setPoint(float x,float y, float z);
		void setPoint(ofPoint pt);
		void computeD();
};

#endif