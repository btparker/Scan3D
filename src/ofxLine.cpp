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