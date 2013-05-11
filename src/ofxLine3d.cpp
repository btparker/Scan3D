#include "ofxLine3d.h"

ofxLine3d::ofxLine3d(){
	initialized = false;
}


//--------------------------------------------------------------
/**
	Initializes a line with values

	@param vx The x component of the direction vector
	@param vy The y component of the direction vector
	@param x0 The x component of a point on the line
	@param y0 The y component of a point on the line
*/	
ofxLine3d::ofxLine3d(float vx,float vy, float vz, float x0, float y0, float z0){
	set(vx,vy,vz, x0, y0, z0);
	initialized = true;

}


ofxLine3d::ofxLine3d(ofVec3f dir, ofPoint pt){
	set(dir.x, dir.y, dir.z, pt.x, pt.y, pt.z);
	initialized = true;
}

bool ofxLine3d::isInit(){
	return initialized;
}


//--------------------------------------------------------------
/**
	Sets the values of this line

	@param vx The x component of the direction vector
	@param vy The y component of the direction vector
	@param x0 The x component of a point on the line
	@param y0 The y component of a point on the line
*/		
void ofxLine3d::set(float vx,float vy, float vz, float x0, float y0, float z0){
	dir.x = vx;
	dir.y = vy;
	dir.z = vz;

	pt.x = x0;
	pt.y = y0; 
	pt.z = z0; 
	initialized = true;
}
