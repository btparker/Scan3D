#include "ofxRay3d.h"

ofxRay3d::ofxRay3d(){
}


//--------------------------------------------------------------
/**
	Initializes a ray with values
*/	
ofxRay3d::ofxRay3d(ofPoint origin, ofVec3f dir){
	setDir(dir.x,dir.y,dir.z);
	setOrigin(origin.x,origin.y,origin.z);
}


//--------------------------------------------------------------
/**
	Sets the values of this ray direction
*/		
void ofxRay3d::setDir(float vx,float vy, float vz){
	dir.x = vx;
	dir.y = vy;
	dir.z = vz;
}

void ofxRay3d::setDir(ofVec3f dir){
	setDir(dir.x,dir.y,dir.z);
}

void ofxRay3d::setOrigin(float x,float y, float z){
	origin.x = x;
	origin.y = y;
	origin.z = z;
}

void ofxRay3d::setOrigin(ofPoint origin){
	setOrigin(origin.x,origin.y,origin.z);
}
//ofPoint intersect(ofxPlane plane);
