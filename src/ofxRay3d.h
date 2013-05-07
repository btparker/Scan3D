#ifndef _OFX_RAY_3D
#define _OFX_RAY_3D

#include "ofMain.h"
#include "ofxPlane.h"

class ofxRay3d{
	public:
		ofVec3f dir;
		ofPoint origin;

		ofxRay3d();
		ofxRay3d(ofPoint origin, ofVec3f dir);
		void setDir(float vx,float vy, float vz);
		void setDir(ofVec3f dir);
		void setOrigin(float x,float y, float z);
		void setOrigin(ofPoint origin);
		ofPoint intersect(ofxPlane plane);
};

#endif