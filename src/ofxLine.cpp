#include "ofxLine.h"

ofxLine::ofxLine(){
}

ofxLine::ofxLine(float vx,float vy, float x0, float y0){
	dir.x = vx;
	dir.y = vy;

	pt.x = x0;
	pt.y = y0;
}
		
void ofxLine::set(float vx,float vy, float x0, float y0){
	dir.x = vx;
	dir.y = vy;

	pt.x = x0;
	pt.y = y0;
}

ofPoint ofxLine::intersection(ofxLine line){

	if(isParallelTo(line)){
		ofLogError() << "Attempted line intersection of parallel lines";
	}

	ofPoint intersectionPt;

	float a0 = (pt.y + dir.y) - pt.y;
	float b0 = pt.x - (pt.x + dir.x);
	float c0 = b0 * pt.y + a0 * pt.x;

	float a1 = (line.pt.y + line.dir.y) - line.pt.y;
	float b1 = line.pt.x - (line.pt.x + line.dir.x);
	float c1 = b1 * line.pt.y + a1 * line.pt.x;

	float det = a0*b1-a1*b0;

	if(det == 0){
		return intersectionPt;
	}


	intersectionPt.x = (b1*c0-b0*c1)/det;
	intersectionPt.y = (a0*c1-a1*c0)/det;

	return intersectionPt;
}

bool ofxLine::isParallelTo(ofxLine line){
	

	float a0 = (pt.y + dir.y) - pt.y;
	float b0 = pt.x - (pt.x + dir.x);
	float c0 = b0 * pt.y + a0 * pt.x;

	float a1 = (line.pt.y + line.dir.y) - line.pt.y;
	float b1 = line.pt.x - (line.pt.x + line.dir.x);
	float c1 = b1 * line.pt.y + a1 * line.pt.x;

	float det = a0*b1-a1*b0;

	return det == 0;
}
