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
	d = (pt.x*normal.x+pt.y*normal.y+pt.z*normal.z);
}

ofxPlane::ofxPlane(ofxLine3d line0, ofxLine3d line1){
	if(!line0.isInit() || !line1.isInit()){
		initialized = false;
	}
	else{
		ofPoint pts[4];
		pts[0] = line0.pt;
		pts[1] = line1.pt;
		pts[2] = (line0.pt+line0.dir);
		pts[3] = (line1.pt+line1.dir);
		
		ofxPlane resPlane = bestFitPlaneEquation(4, pts);
		
		setNormal(resPlane.normal);
		setPoint(resPlane.pt);
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

ofxPlane::ofxPlane(int n, ofPoint pts[]){
	ofxPlane resPlane = bestFitPlaneEquation(n, pts);
		
	setNormal(resPlane.normal);
	setPoint(resPlane.pt);
	computeD();
	initialized = true;
}

ofxPlane ofxPlane::bestFitPlaneEquation(int n, ofPoint pts[]){
    float sum_i_x = 0;
    float sum_i_xx = 0;
    float sum_i_xy = 0;
    float sum_i_yy = 0;
    float sum_i_y = 0;
    float sum_i_xz = 0;
    float sum_i_yz = 0;
    float sum_i_z = 0;

    for(int i = 0; i < n; i++){
        sum_i_x     +=  pts[i].x;
        sum_i_xx    +=  pts[i].x * pts[i].x;
        sum_i_xy    +=  pts[i].x * pts[i].y;
        sum_i_yy    +=  pts[i].y * pts[i].y;
        sum_i_y     +=  pts[i].y;
        sum_i_xz    +=  pts[i].x * pts[i].z;
        sum_i_yz    +=  pts[i].y * pts[i].z;
        sum_i_z     +=  pts[i].z;
    }

    /*
    If you have n data points (x[i], y[i], z[i]), compute the 3x3 symmetric matrix A whose entries are:

    sum_i x[i]*x[i],    sum_i x[i]*y[i],    sum_i x[i]
    sum_i x[i]*y[i],    sum_i y[i]*y[i],    sum_i y[i]
    sum_i x[i],         sum_i y[i],         n

    */
    CvMat* A = cvCreateMat( 3, 3, CV_32FC1 );
    CvMat* invA = cvCreateMat( 3, 3, CV_32FC1 );
    CV_MAT_ELEM(*A,float,0,0) = sum_i_xx;   CV_MAT_ELEM(*A,float,0,1) = sum_i_xy;   CV_MAT_ELEM(*A,float,0,2) = sum_i_x;    
    CV_MAT_ELEM(*A,float,1,0) = sum_i_xy;   CV_MAT_ELEM(*A,float,1,1) = sum_i_yy;   CV_MAT_ELEM(*A,float,1,2) = sum_i_y;    
    CV_MAT_ELEM(*A,float,2,0) = sum_i_x;    CV_MAT_ELEM(*A,float,2,1) = sum_i_y;    CV_MAT_ELEM(*A,float,2,2) = (float)n;   


    /*
    Also compute the 3 element vector b:

    {sum_i x[i]*z[i],   sum_i y[i]*z[i],    sum_i z[i]}

    */

    CvMat* b = cvCreateMat( 3,1, CV_32FC1 );
    CV_MAT_ELEM(*b,float,0,0) = sum_i_xz;   CV_MAT_ELEM(*b,float,1,0) = sum_i_yz;   CV_MAT_ELEM(*b,float,2,0) = sum_i_z;    

    CvMat* x  = cvCreateMat(3,1,CV_32FC1);

    cvInv(A,invA);

    cvMatMul(invA,b,x);

    //cvSolve(&A, &b, &x, CV_LU);    // solve (Ax=b) for x

    float planeA = CV_MAT_ELEM(*x,float,0,0);
    float planeB = CV_MAT_ELEM(*x,float,1,0);
    float planeC = -1;
    float planeD = -1*CV_MAT_ELEM(*x,float,2,0);

    float z = (-planeD/planeC);

    ofPoint pointRes = ofPoint(0,0,z);

    ofxPlane planeRes;

    ofVec3f normalRes = ofVec3f(planeA,planeB,planeC);
    normalRes.normalize();



    /*
    Then solve Ax = b for the given A and b. The three components of the solution vector are the 
    coefficients to the least-square fit plane {a,b,c}.

    Note that this is the "ordinary least squares" fit, which is appropriate only when z is expected to 
    be a linear function of x and y. If you are looking more generally for a "best fit plane" in 3-space, 
    you may want to learn about "geometric" least squares.

    Note also that this will fail if your points are in a line, as your example points are.


    */


    cvReleaseMat(&A);
    cvReleaseMat(&invA);
    cvReleaseMat(&b);
    cvReleaseMat(&x);
    return ofxPlane(pointRes,normalRes);

}

ofVec4f ofxPlane::getEqtnParams(){
	return ofVec4f(normal.x,normal.y,normal.z,d);
}