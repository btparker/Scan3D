#ifndef _OFX_PLANE
#define _OFX_PLANE

#include "ofMain.h"
#include "ofxLine3d.h"
#include "ofxOpenCv.h"

class ofxPlane{
	public:
		ofVec3f normal;
		ofPoint pt;
		float d;

		bool initialized;

		ofxPlane();
		ofxPlane(ofPoint pt, ofVec3f normal);
		ofxPlane(ofxLine3d line0, ofxLine3d line1);

		void setNormal(float vx,float vy, float vz);
		void setNormal(ofVec3f normal);
		void setPoint(float x,float y, float z);
		void setPoint(ofPoint pt);
		void computeD();
		ofxPlane interpolate(ofxPlane plane, float value);
		bool isInit();

		ofPoint getPointAt(int i, float x, float y, float z);

		ofxPlane bestFitPlaneEquation(int n, ofPoint pts[]);
		ofxPlane(int n, ofPoint pts[]);

		ofVec4f getEqtnParams();

};

#endif