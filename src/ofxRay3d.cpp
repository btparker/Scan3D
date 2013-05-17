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

ofPoint ofxRay3d::intersect(ofxPlane plane){
	ofVec3f p0(origin);
	float t = -1*(p0.dot(plane.normal)+plane.d)/(dir.dot(plane.normal));
	ofPoint p = p0+t*dir;
	return p;
}

//http://gamedev.stackexchange.com/questions/9738/points-on-lines-where-the-two-lines-are-the-closest-together
ofPoint ofxRay3d::intersect(ofxRay3d ray){
	

    ofVec3f d0 = dir;
    ofVec3f d1 = ray.dir;

    float a = d0.dot(d0);
    float b = d0.dot(d1);
    float e = d1.dot(d1);

    float d = a*e - b*b;
    if (d != 0) // If the two lines are not parallel.
    {
        ofVec3f r = ofVec3f(origin) - ofVec3f(ray.origin);
        float c = d0.dot(r);
        float f = d1.dot(r);

        float s = (b*f - c*e) / d;
        float t = (a*f - b*c) / d;

        ofPoint pt0, pt1;

        pt0 = pointOnRay(s);
        pt1 = ray.pointOnRay(t);

        return (pt0+pt1)/2;
    }
    else
    {
        printf("Lines were parallel.\n");
        return ofPoint();
    }
	

}

ofPoint ofxRay3d::pointOnRay(float t){
	return origin+(ofPoint)t*dir;
}
