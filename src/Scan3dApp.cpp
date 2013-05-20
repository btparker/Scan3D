#include "Scan3dApp.h"
using namespace cv;

//--------------------------------------------------------------
void Scan3dApp::setup(){


    ofBackground(0);
    ofSetWindowTitle("Merging Point Clouds");
    ofSetFrameRate(30);
    ofSetWindowShape(1024,768);


    /*
        Initializing the text for the bottom message bar
    */
    messageBarText = "Hello";
    messageBarSubText = "World";
    messageBarHeight = 70;
    messageBarFont.loadFont("OpenSans-Semibold.ttf", 20);
    messageBarSubTextFont.loadFont("OpenSans-Light.ttf", 15);

   
}

//--------------------------------------------------------------
void Scan3dApp::update(){

}

void Scan3dApp::convertOfPointsToCvMat(vector<ofPoint> pts, int dimensions, CvMat* output){
    if(pts.size() == 0){
        return;
    }
    for(int i = 0; i < pts.size(); i++){
        for(int j = 0; j < dimensions; j++){
            float value = 0;
            switch(j){
                case 0:
                    value = pts[i].x;
                    break;
                case 1:
                    value = pts[i].y;
                    break;
                case 2:
                    value = pts[i].z;
                    break;
            }
            CV_MAT_ELEM( *output, float, i, j ) = value;
        }
    }
}


void Scan3dApp::writePointsToFile(vector<ofPoint> points,vector<ofColor> colors, string filename){
    ofFile outputFile(filename.c_str(), ofFile::WriteOnly);
    outputFile <<"ply\n";
    outputFile <<"format ascii 1.0\n";
    outputFile <<"element vertex ";
    outputFile << ofToString(points.size());
    outputFile << "\n";
    outputFile <<"property float32 x\n";
    outputFile <<"property float32 y\n";
    outputFile <<"property float32 z\n";
    outputFile <<"property uchar red\n";
    outputFile <<"property uchar green\n";
    outputFile <<"property uchar blue\n";
    outputFile <<"element face 0\n";
    outputFile <<"property list uint8 int32 vertex_indices\n";
    outputFile <<"end_header\n";
    
    for(int i = 0; i < points.size(); i++){
        int r = colors[i].r;
        int g = colors[i].g;
        int b = colors[i].b;


        ostringstream buff;
        buff << fixed << points[i].x;
        buff << " ";
        buff << fixed << points[i].y;
        buff << " ";
        buff << fixed << points[i].z;
        buff << " ";
        buff << setw(0) << r;
        buff << " ";
        buff << setw(0) << g;
        buff << " ";
        buff << setw(0) << b;
        buff << " ";
        outputFile << buff.str();
        outputFile <<" \n";
    }
    
    outputFile.close();
}

//--------------------------------------------------------------
void Scan3dApp::draw(){
    ofBackground(60);
    ofSetColor(255);

    messageBarFont.drawString(messageBarText,10,ofGetHeight()+messageBarHeight/2-5);
    messageBarSubTextFont.drawString(messageBarSubText,10,ofGetHeight()+messageBarHeight-15);

    easyCam.begin();
    ofDrawGrid();
    /*
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    //ofScale(1, -1, -1);
    //ofTranslate(camPos.x,camPos.y,camPos.z); // center the points a bit

    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
    */
    easyCam.end();

 
}


void Scan3dApp::setCamera(){
    ofNode lookat;

    lookat.setPosition(ofPoint(0,0,0));

    ofVec3f easyCamPos = ofVec3f(5,5,5);
    
    easyCam.setPosition(easyCamPos);
    easyCam.setTarget(lookat);
}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 32: //SPACEBAR
            
            break;
        case 49: // 1
            
            break;
        case 50: // 2
            
            break;
        case 51: // 3
            
            break;
        case 'q':
            std::exit(1);
            break;
    }    
}

//--------------------------------------------------------------
void Scan3dApp::keyReleased(int key){

}

//--------------------------------------------------------------
void Scan3dApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void Scan3dApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void Scan3dApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void Scan3dApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void Scan3dApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void Scan3dApp::dragEvent(ofDragInfo dragInfo){ 

}
