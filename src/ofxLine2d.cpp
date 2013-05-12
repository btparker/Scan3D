#include "ofxLine2d.h"

ofxLine2d::ofxLine2d(){
	initialized = false;
}

ofxLine2d::ofxLine2d(ofVec2f dir, ofPoint pt){
	set(dir.x, dir.y, pt.x, pt.y);
	initialized = true;

}


//--------------------------------------------------------------
/**
	Initializes a line with values

	@param vx The x component of the direction vector
	@param vy The y component of the direction vector
	@param x0 The x component of a point on the line
	@param y0 The y component of a point on the line
*/	
ofxLine2d::ofxLine2d(float vx,float vy, float x0, float y0){
	set(vx,vy, x0, y0);
	initialized = true;

}

bool ofxLine2d::isInit(){
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
void ofxLine2d::set(float vx,float vy, float x0, float y0){
	dir.x = vx;
	dir.y = vy;

	pt.x = x0;
	pt.y = y0; 
	pt.z = 0;

	dir.normalize();
	initialized = true;
}

//--------------------------------------------------------------
/**
	Returns the point of intersection between this line and another

	@param line The other line to check against
	@returns ofPoint The point of intersection (or throws an error if lines are parallel)
*/
ofPoint ofxLine2d::intersection(ofxLine2d line){
	ofPoint intersectionPt;

	if(!initialized || !line.isInit()){
		ofLogError() << "Attempted line intersection of uninitialized line(s)";
		intersectionPt.x = -1;
		intersectionPt.y = -1;
		return intersectionPt;
	}

	if(isParallelTo(line)){
		ofLogError() << "Attempted line intersection of parallel lines";
		intersectionPt.x = -1;
		intersectionPt.y = -1;
		return intersectionPt;
	}

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

//--------------------------------------------------------------
/**
	Checks to see if another line is parallel to this one

	@param line The other line to check against
	@returns bool Whether or not the line is parralel to this one 
*/
bool ofxLine2d::isParallelTo(ofxLine2d line){

	

	float a0 = (pt.y + dir.y) - pt.y;
	float b0 = pt.x - (pt.x + dir.x);
	float c0 = b0 * pt.y + a0 * pt.x;

	float a1 = (line.pt.y + line.dir.y) - line.pt.y;
	float b1 = line.pt.x - (line.pt.x + line.dir.x);
	float c1 = b1 * line.pt.y + a1 * line.pt.x;

	float det = a0*b1-a1*b0;

	return det == 0;
}


//--------------------------------------------------------------
/**
    Computes if line is in and potentially draws line from region

    @param roi The region of interest to draw the line in
    @param boolean drawLine Whether to draw line or not
    @returns bool If the line does not cross the region, 
    	returns false and does not draw. Otherwise, returns true
*/
bool ofxLine2d::drawLineInRegion(ofRectangle roi, bool drawLine){
	if(!initialized){
		return false;
	}
	ofxLine2d top(1, 0, 0, roi.y);
	ofxLine2d bottom(1, 0, 0, roi.y + roi.height);
	ofxLine2d left(0, 1, roi.x, 0);
	ofxLine2d right(0, 1, roi.x + roi.width, 0);

	/*

	if(!top.isParallelTo(bottom)){
		ofLogFatalError() << "Top not parralel to bottom";
	}

	if(!left.isParallelTo(right)){
		ofLogFatalError() << "Left not parralel to right";
	}

	if(top.intersection(left).x != roi.x || top.intersection(left).y != roi.y){
		ofLogFatalError() << "Top/Left not equal to roi tl corner";
	}

	if(right.intersection(bottom).x != (roi.x + roi.width)|| right.intersection(bottom).y != (roi.y + roi.height)){
		ofLogFatalError() << "Bottom/Right not equal to roi br corner";
	}
	*/

	ofPoint linePt0;
	ofPoint linePt1;

	bool isPt0Set = false;
	bool isPt1Set = false;

	bool topCrossed = false;
	bool leftCrossed = false;
	bool bottomCrossed = false;
	bool rightCrossed = false;

	if(isParallelTo(top)){
		linePt0 = intersection(left);
		linePt1 = intersection(right);
	}
	else if(isParallelTo(left)){
		linePt0 = intersection(top);
		linePt1 = intersection(bottom);
	}
	else{
		ofPoint tI = intersection(top);
		ofPoint bI = intersection(bottom);
		ofPoint lI = intersection(left);
		ofPoint rI = intersection(right);

		if(tI.x <= (roi.x+roi.width) && tI.x >= (roi.x)){
			topCrossed = true;
			if(!isPt0Set){
				linePt0 = tI;
				isPt0Set = true;
			}
			else{
				linePt1 = tI;
				isPt1Set = true;
			}
		}

		if(lI.y <= (roi.y+roi.height) && lI.y >= (roi.y)){
			leftCrossed = true;
			if(!isPt0Set){
				linePt0 = lI;
				isPt0Set = true;
			}
			else{
				linePt1 = lI;
				isPt1Set = true;
			}
		}

		if(bI.x <= (roi.x+roi.width) && bI.x >= (roi.x)){
			bottomCrossed = true;
			if(!isPt0Set){
				linePt0 = bI;
				isPt0Set = true;
			}
			else{
				linePt1 = bI;
				isPt1Set = true;
			}
		}

		if(rI.y <= (roi.y+roi.height) && rI.y >= (roi.y)){
			rightCrossed = true;
			if(!isPt0Set){
				linePt0 = rI;
				isPt0Set = true;
			}
			else{
				linePt1 = rI;
				isPt1Set = true;
			}
		}


	}

	bool returnValue = isPt0Set && isPt1Set;
	if(drawLine && returnValue){
		ofLine(linePt0,linePt1);
	}

	return returnValue;
}

//--------------------------------------------------------------
/**
    Determines if line crosses region

    @param roi The region of interest to draw the line in
    @returns bool If the line does not cross the region, 
    	returns false and does not draw. Otherwise, returns true
*/
bool ofxLine2d::isLineInRegion(ofRectangle roi){
	return drawLineInRegion(roi, false);
}

//--------------------------------------------------------------
/**
    Draws line, cropped to region

    @param roi The region of interest to draw the line in
*/
void ofxLine2d::drawLineInRegion(ofRectangle roi){
	drawLineInRegion(roi, true);
}

