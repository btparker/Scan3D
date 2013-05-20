#include "Scan3dApp.h"


//--------------------------------------------------------------
void Scan3dApp::setup(){


    ofBackground(0);
    ofSetWindowTitle("Merging Point Clouds");
    ofSetFrameRate(15);
    ofSetWindowShape(1024,768);
    displayState = DEFAULT;

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
    resultMesh.setMode(OF_PRIMITIVE_POINTS);
    transLine.setMode(OF_PRIMITIVE_LINES);
    loadMesh("cube.ply", &mesh0);
    loadMesh("cube.ply", &resultMesh);
    loadMesh("cube_trans.ply", &mesh1);

    int meshSize = mesh0.getVertices().size();
    vector<int> correspondences;
    
    for(int i = 0; i < meshSize; i++){
        correspondences.push_back(i);
    }

    float a[3][4] = {
                        {1, 0, 0, 10},
                        {0, 1, 0, 5},
                        {0, 0, 1, 12}
                    };

    Mat A = Mat(3, 4, CV_32FC1, a);

    cout << "A:\n" << A << endl;
 
    transformMesh(A,&mesh1);
    Mat trans = Mat(3, 4, CV_32FC1);

    align(&mesh0,&mesh1, correspondences, &trans);

    cout << "Trans:\n" << trans << endl;

    transformMesh(trans,&resultMesh);

    
    transLine.addColor(ofColor(0,0,255));
    transLine.addVertex(ofVec3f(mesh0.getCentroid()));
    transLine.addColor(ofColor(0,0,255));
    transLine.addVertex(ofVec3f(mesh1.getCentroid()));
    
}

void Scan3dApp::align(const ofMesh* meshA, const ofMesh* meshB, const vector<int> correspondences, Mat* trans){
    ofVec3f p = mesh0.getCentroid();
    ofVec3f pPrime = mesh1.getCentroid();
    ofVec3f q;
    ofVec3f qPrime;

    int meshSize = mesh0.getVertices().size();
    float b[3][3];
    for(int i = 0; i < meshSize; i++){
        q = mesh0.getVertex(i)-p;
        qPrime = mesh1.getVertex(correspondences[i])-pPrime;
        b[0][0] = b[0][0]+q.x*qPrime.x; b[0][1] = b[0][1]+q.x*qPrime.y; b[0][2] = b[0][2]+q.x*qPrime.z;
        b[1][0] = b[1][0]+q.y*qPrime.x; b[1][1] = b[1][1]+q.y*qPrime.y; b[1][2] = b[1][2]+q.y*qPrime.z;
        b[2][0] = b[2][0]+q.z*qPrime.x; b[2][1] = b[2][1]+q.z*qPrime.y; b[2][2] = b[2][2]+q.z*qPrime.z;
        
    }
 

    Mat H = Mat(3,3, CV_32FC1,b);

    Mat u = Mat(3,3, CV_32FC1);
    Mat s = Mat(3,3, CV_32FC1);
    Mat vt = Mat(3,3, CV_32FC1);


    SVD::compute(H,s,vt,u);

    Mat v = vt.t();
    Mat ut = u.t();



    Mat R = v*ut;

    


    if(determinant(R) <= 0){
        R.at<float>(0,2) = -1*R.at<float>(0,2);
        R.at<float>(1,2) = -1*R.at<float>(1,2);
        R.at<float>(2,2) = -1*R.at<float>(2,2);
    }

    Mat pMat = Mat(3, 1, CV_32FC1); 
    Mat pPrimeMat = Mat(3, 1, CV_32FC1); 
    pMat.at<float>(0,0) = p.x;
    pMat.at<float>(1,0) = p.y;
    pMat.at<float>(2,0) = p.z;

    pPrimeMat.at<float>(0,0) = pPrime.x;
    pPrimeMat.at<float>(1,0) = pPrime.y;
    pPrimeMat.at<float>(2,0) = pPrime.z;

    Mat T = pPrimeMat-R*pMat;

    cout << "Rotation:\n" << R << endl;
    cout << "Translation:\n" << T << endl;

    
    trans->at<float>(0,0) = R.at<float>(0,0); trans->at<float>(0,1) = R.at<float>(0,1); trans->at<float>(0,2) = R.at<float>(0,2); trans->at<float>(0,3) = T.at<float>(0,0);
    trans->at<float>(1,0) = R.at<float>(1,0); trans->at<float>(1,1) = R.at<float>(1,1); trans->at<float>(1,2) = R.at<float>(1,2); trans->at<float>(1,3) = T.at<float>(1,0);
    trans->at<float>(2,0) = R.at<float>(2,0); trans->at<float>(2,1) = R.at<float>(2,1); trans->at<float>(2,2) = R.at<float>(2,2); trans->at<float>(2,3) = T.at<float>(2,0);

    cout << "INTERNAL TRANS:\n" << *trans << endl;
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

Mat Scan3dApp::convertMeshToMat(const ofMesh* mesh){
    int meshSize = mesh->getVertices().size();
    float a[4][meshSize];
    for(int i = 0; i < meshSize; i++){
        a[0][i] = mesh->getVertex(i).x;
        a[1][i] = mesh->getVertex(i).y;
        a[2][i] = mesh->getVertex(i).z;
        a[3][i] = 1.0;
    }
    return Mat(4, meshSize, CV_32FC1,a);
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
    switch(displayState){
        case DEFAULT:
            glPointSize(10);
            ofPushMatrix();
            glEnable(GL_DEPTH_TEST);
            mesh0.drawVertices();
            mesh1.drawVertices();
            glDisable(GL_DEPTH_TEST);
            ofPopMatrix();
            break;

        case TRANSFORMATION:
            glPointSize(10);
            ofPushMatrix();
            glEnable(GL_DEPTH_TEST);
            resultMesh.drawVertices();
            glDisable(GL_DEPTH_TEST);
            ofPopMatrix();
          

            glPointSize(15);
            ofPushMatrix();
            glEnable(GL_DEPTH_TEST);
            transLine.drawVertices();
            glDisable(GL_DEPTH_TEST);
            ofPopMatrix();
            break;
    }
    
    
    
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
            displayState = DEFAULT;
            break;
        case 50: // 2
            displayState = TRANSFORMATION;
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
