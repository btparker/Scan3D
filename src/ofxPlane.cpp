#include "ofxPlane.h"

ofxPlane::ofxPlane(){
}


//--------------------------------------------------------------
/**
	Initializes a plane with values
*/	
ofxPlane::ofxPlane(ofPoint pt, ofVec3f normal){
	setNormal(normal.x,normal.y,normal.z);
	setPoint(pt.x,pt.y,pt.z);
	computeD();
}

ofxPlane::ofxPlane(ofPoint* pts){
	//best fit plane
}

void ofxPlane::computeD(){
	d = pt.x*normal.x+pt.y*normal.y+pt.z*normal.z;
}


//--------------------------------------------------------------
/**
	Sets the values of this plane normal
*/		
void ofxPlane::setNormal(float nx,float ny, float nz){
	normal.x = nx;
	normal.y = ny;
	normal.z = nz;
	computeD();
}

void ofxPlane::setNormal(ofVec3f normal){
	setNormal(normal.x,normal.y,normal.z);
}

void ofxPlane::setPoint(float x,float y, float z){
	pt.x = x;
	pt.y = y;
	pt.z = z;
	computeD();
}

void ofxPlane::setPoint(ofPoint pt){
	setPoint(pt.x,pt.y,pt.z);
}


ofxPlane ofxPlane::interpolate(ofxPlane plane, float value){
	ofVec3f interNormal = normal.interpolate(plane.normal,value);
	ofPoint interPoint = pt.interpolate(plane.pt,value);

	interNormal.normalize();

	return ofxPlane(interPoint,interNormal);
}

