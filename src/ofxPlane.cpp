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

void ofxPlane::computeD(){
	d = pt.x*normal.x+pt.y*normal.y+pt.z*normal.z;
}

ofxPlane::ofxPlane(ofxLine3d line0, ofxLine3d line1){
	
	// Computing 'good enough' fit plane
	ofPoint pt0 = line0.pt+line0.dir;
	ofPoint pt1 = line1.pt+line1.dir;
	ofPoint pt2 = line0.pt+line1.dir;

	ofPoint centroid = pt0+pt1+pt2;
	centroid /= 3;

	ofPoint pt01 = pt1-pt0;
	ofPoint pt02 = pt2-pt0;

	ofVec3f normal = pt01.cross(pt02);
	normal.normalize();

	setNormal(normal.x,normal.y,normal.z);
	setPoint(centroid.x,centroid.y,centroid.z);
	computeD();
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

