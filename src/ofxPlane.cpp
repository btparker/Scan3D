#include "ofxPlane.h"


ofxPlane::ofxPlane(){
	initialized = false;
}


//--------------------------------------------------------------
/**
	Initializes a plane with values
*/	
ofxPlane::ofxPlane(ofPoint pt, ofVec3f normal){
	setNormal(normal.x,normal.y,normal.z);
	setPoint(pt.x,pt.y,pt.z);
	computeD();
	initialized = true;
}

void ofxPlane::computeD(){
	d = -(pt.x*normal.x+pt.y*normal.y+pt.z*normal.z);
}

ofxPlane::ofxPlane(ofxLine3d line0, ofxLine3d line1){
	if(!line0.isInit() || !line1.isInit()){
		initialized = false;
	}
	else{
		// Computing 'good enough' fit plane
		ofPoint pt0 = line0.pt;
		ofPoint pt1 = line1.pt;
		ofPoint pt2 = (line0.pt+line0.dir);

		ofVec3f pt01 = pt1-pt0;
		ofVec3f pt02 = pt2-pt0;

		ofVec3f normalVec = pt01.cross(pt02);
		normalVec.normalize();

		setNormal(normalVec.x,normalVec.y,normalVec.z);
		cout << "Line Crossed Plane normal " << endl;
		cout << normal << endl;
		setPoint(pt2.x,pt2.y,pt2.z);
		computeD();
		initialized = true;
	}
	
	
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
	if(!plane.isInit()){
		return ofxPlane();
	}
	ofVec3f interNormal = normal.interpolate(plane.normal,value);
	ofPoint interPoint = pt.interpolate(plane.pt,value);

	interNormal.normalize();

	return ofxPlane(interPoint,interNormal);
}

bool ofxPlane::isInit(){
	return initialized;
}

ofPoint ofxPlane::getPointAt(int i, float x, float y, float z){
	ofPoint pointAt;
	switch(i){
		case 0:
			x = -1*(normal.y*y+normal.z*z+d)/normal.x;
			break;
		case 1:
			y = -1*(normal.x*x+normal.z*z+d)/normal.y;
			break;
		case 2:
			z = -1*(normal.x*x+normal.y*y+d)/normal.z;
			break;
			
	}
	
	pointAt.set(x,y,z);
	return pointAt;
}

