#include "ofxLine.h"

ofxLine::ofxLine(){
}


//--------------------------------------------------------------
/**
	Initializes a line with values

	@param vx The x component of the direction vector
	@param vy The y component of the direction vector
	@param x0 The x component of a point on the line
	@param y0 The y component of a point on the line
*/	
ofxLine::ofxLine(float vx,float vy, float x0, float y0){
	set(vx,vy, x0, y0);
}


//--------------------------------------------------------------
/**
	Sets the values of this line

	@param vx The x component of the direction vector
	@param vy The y component of the direction vector
	@param x0 The x component of a point on the line
	@param y0 The y component of a point on the line
*/		
void ofxLine::set(float vx,float vy, float x0, float y0){
	dir.x = vx;
	dir.y = vy;

	pt.x = x0;
	pt.y = y0;
}

//--------------------------------------------------------------
/**
	Returns the point of intersection between this line and another

	@param line The other line to check against
	@returns ofPoint The point of intersection (or throws an error if lines are parallel)
*/
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

//--------------------------------------------------------------
/**
	Checks to see if another line is parallel to this one

	@param line The other line to check against
	@returns bool Whether or not the line is parralel to this one 
*/
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


//--------------------------------------------------------------
/**
    Draws line in a subregion

    @param roi The region of interest to draw the line in
    @returns bool If the line does not cross the region, 
    	returns false and does not draw. Otherwise, returns true
*/
bool ofxLine::drawLineInRegion(ofRectangle roi){
	ofxLine top(1, 0, 0, roi.y);
	ofxLine bottom(1, 0, 0, roi.y + roi.height);
	ofxLine left(0, 1, roi.x, 0);
	ofxLine right(0, 1, roi.x + roi.width, 0);

	ofPoint linePt0;
	ofPoint linePt1;

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
			linePt0 = tI;
		}
		else if(lI.y <= (roi.y+roi.height) && lI.y >= (roi.y)){
			linePt0 = lI;
		}
		else{
			return false;
		}

		if(bI.x <= (roi.x+roi.width) && bI.x >= (roi.x)){
			linePt0 = bI;
		}
		else if(rI.y <= (roi.y+roi.height) && rI.y >= (roi.y)){
			linePt0 = rI;
		}
		else{
			return false;
		}


	}

	ofLine(linePt0,linePt1);

	return true;
}
