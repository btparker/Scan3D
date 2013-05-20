#include "Scan3dApp.h"


//--------------------------------------------------------------
void Scan3dApp::setup(){


    ofBackground(0);
    ofSetWindowTitle("Merging Point Clouds");
    ofSetFrameRate(15);
    ofSetWindowShape(1024,768);


    /*
        Initializing the text for the bottom message bar
    */
    messageBarText = "Hello";
    messageBarSubText = "World";
    messageBarHeight = 70;
    messageBarFont.loadFont("OpenSans-Semibold.ttf", 20);
    messageBarSubTextFont.loadFont("OpenSans-Light.ttf", 15);

    setCamera();

    mesh0.setMode(OF_PRIMITIVE_POINTS);
    mesh1.setMode(OF_PRIMITIVE_POINTS);
    loadMesh("cube.ply", &mesh0);
    loadMesh("cube_trans.ply", &mesh1);

    float a[4][4] = {
                        {1, 0, 0, 5},
                        {0, 1, 0, 5},
                        {0, 0, 1, 5},
                        {0, 0, 0, 1}
                    };

    Mat A = Mat(4, 4, CV_32FC1, a);

    transformMesh(A,&mesh1);
}

void Scan3dApp::transformMesh(Mat mat, ofMesh* mesh){
    Mat vertMat = Mat(4, 1, CV_32FC1);
    Mat resVert = Mat(4, 1, CV_32FC1);
    vertMat.at<float>(3,0) = 1.0;

    int meshSize = mesh->getVertices().size();

    for(int i = 0; i < meshSize; i++){
        ofVec3f vertex = mesh->getVertex(0);
        vertMat.at<float>(0,0) = vertex.x;
        vertMat.at<float>(1,0) = vertex.y;
        vertMat.at<float>(2,0) = vertex.z;

        resVert = mat*vertMat;

        vertex.x = resVert.at<float>(0,0);
        vertex.y = resVert.at<float>(1,0);
        vertex.z = resVert.at<float>(2,0);

        mesh->removeVertex(0);
        mesh->addVertex(vertex);


    }
}

//--------------------------------------------------------------
void Scan3dApp::update(){

}

void Scan3dApp::convertOfPointsToCvMat(vector<ofPoint> points, int dimensions, CvMat* output){
    if(points.size() == 0){
        return;
    }
    for(int i = 0; i < points.size(); i++){
        for(int j = 0; j < dimensions; j++){
            float value = 0;
            switch(j){
                case 0:
                    value = points[i].x;
                    break;
                case 1:
                    value = points[i].y;
                    break;
                case 2:
                    value = points[i].z;
                    break;
            }
            CV_MAT_ELEM( *output, float, i, j ) = value;
        }
    }
}

void Scan3dApp::loadMesh(string filename, ofMesh* mesh){
    ofBuffer buffer = ofBufferFromFile(ofToDataPath(filename)); // reading into the buffer
    bool pointsLine = false;
    int i = 0;
    int numPoints = 1;
    while(i < numPoints){
        string line = buffer.getNextLine();
        if(pointsLine){
            
            float x,y,z;
            int r,g,b;
            sscanf((line).c_str(), "%f %f %f %d %d %d", &x,&y,&z,&r,&g,&b);
            mesh->addColor(ofColor(r,g,b));
            mesh->addVertex(ofVec3f(x,y,z));
            
            i++;
        }
        else if(line.find("element vertex") != string::npos){
            sscanf((line).c_str(), "element vertex %d", &numPoints);
            continue;
        }
        else if(line == "end_header"){
            pointsLine = true;
            continue;
        }
    }
   
}

void Scan3dApp::writeMeshToFile(const ofMesh* mesh, string filename){
    ofFile outputFile(ofToDataPath(filename).c_str(), ofFile::WriteOnly);
    outputFile <<"ply\n";
    outputFile <<"format ascii 1.0\n";
    outputFile <<"element vertex ";
    outputFile << ofToString(mesh->getVertices().size());
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
    
    for(int i = 0; i < mesh->getVertices().size(); i++){
        int r = mesh->getColors()[i].r*255;
        int g = mesh->getColors()[i].g*255;
        int b = mesh->getColors()[i].b*255;


        ostringstream buff;
        buff << fixed << mesh->getVertices()[i].x;
        buff << " ";
        buff << fixed << mesh->getVertices()[i].y;
        buff << " ";
        buff << fixed << mesh->getVertices()[i].z;
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
    ofScale(100,100,100);

    messageBarFont.drawString(messageBarText,10,ofGetHeight()+messageBarHeight/2-5);
    messageBarSubTextFont.drawString(messageBarSubText,10,ofGetHeight()+messageBarHeight-15);

    easyCam.begin();
    //ofDrawGrid();
    
    glPointSize(10);
    ofPushMatrix();
    glEnable(GL_DEPTH_TEST);
    mesh0.drawVertices();
    mesh1.drawVertices();
    resultMesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
    
    easyCam.end();

 
}


void Scan3dApp::setCamera(){
    ofNode lookat;

    lookat.setPosition(ofPoint(0,0,0));

    ofVec3f easyCamPos = ofVec3f(1,1,1);
    
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
        case 's':
            writeMeshToFile(&mesh0,"dafuq.ply");
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
