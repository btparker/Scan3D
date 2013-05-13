#include "Scan3dApp.h"
using namespace cv;

//--------------------------------------------------------------
void Scan3dApp::setup(){

    bDrawPointCloud = false;

    //Initializing sobel kernels
    sobelHorizontal[0][0] = -1; sobelHorizontal[0][1] = 0; sobelHorizontal[0][2] = 1;
    sobelHorizontal[1][0] = -2; sobelHorizontal[1][1] = 0; sobelHorizontal[1][2] = 2;
    sobelHorizontal[2][0] = -1; sobelHorizontal[2][1] = 0; sobelHorizontal[2][2] = 1;

    sobelVertical[0][0] =  1; sobelVertical[0][1] =  2; sobelVertical[0][2] = 1;
    sobelVertical[1][0] =  0; sobelVertical[1][1] =  0; sobelVertical[1][2] = 0;
    sobelVertical[2][0] = -1; sobelVertical[2][1] = -2; sobelVertical[2][2] = -1;

    laplacianOfGaussian[0][0] = -1; laplacianOfGaussian[0][1] = -1; laplacianOfGaussian[0][2] = -1;
    laplacianOfGaussian[1][0] = -1; laplacianOfGaussian[1][1] = 8; laplacianOfGaussian[1][2] = -1;
    laplacianOfGaussian[2][0] = -1; laplacianOfGaussian[2][1] = -1; laplacianOfGaussian[2][2] = -1;


    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetFrameRate(30);


    loadSettings();

    /*
        Initializing the text for the bottom message bar
    */
    messageBarText = "";
    messageBarSubText = "";
    messageBarHeight = 70;
    messageBarFont.loadFont("HelveticaNeueLTStd-Bd.otf", 20);
    messageBarSubTextFont.loadFont("HelveticaNeueLTStd-Lt.otf", 15);

    //Setting and notifying states
    //programState = SETUP;
    cout << "SETUP STATE (press SPACE to continue)" << endl;
    displayState = COLOR;

    switch(inputType){
        case VIDEO:
            vid.loadMovie(inputVideoFile);
            vid.play();
            vid.update(); //to get height and width to load
            width = vid.getWidth();
            height = vid.getHeight();
            break;
        case IMAGE_SEQUENCE:
            cout << "dir path = " << dir.path() << endl;
            cout << "dir first file = " << dir.getPath(0) << endl;
            bufferOfImage.loadImage(dir.getPath(0));
            bufferOfImage.update();
            width = bufferOfImage.getWidth();
            height = bufferOfImage.getHeight();
            break;
    }

    //bufferOfImage.allocate(width,height,OF_IMAGE_COLOR);
    bufferOfxCvColorImage.allocate(width,height);
    bufferOfxCvGrayscaleImage.allocate(width,height);
    
    colorFrame.allocate(width,height);
    maxColorImg.allocate(width,height);
    grayscaleFrame.allocate(width,height);
    minImg.allocate(width,height);
    minImg.set(255);
    maxImg.allocate(width,height);
    maxImg.set(0);
    temporalImg.allocate(width,height);
    enterFrame.allocate(width,height);
    enterFrame.set(0);
    exitFrame.allocate(width,height);
    exitFrame.set(0);
    diffFrame.allocate(width,height);

    
    shadowThreshImg.allocate(width,height);

    ofSetWindowShape(width,height+messageBarHeight);

    frameBufferSize = 100;
    frames.resize(frameBufferSize,bufferOfxCvGrayscaleImage);
    

    topSectionColor.r = 255;
    topSectionColor.g = 155;
    topSectionColor.b = 0;
    topSectionColor.a = 100;

    bottomSectionColor.r = 0;
    bottomSectionColor.g = 200;
    bottomSectionColor.b = 255;
    bottomSectionColor.a = 100;

    fullSection.x = 0;
    fullSection.y = 0;
    fullSection.width = width;
    fullSection.height = height;

    settingTopSection = false;
    settingBottomSection = false;

    
    frameIndex = 0;

    // Calibration Variables (Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
    camCalFrame = 0;
    boardXCount = 8;
    boardYCount = 6;
    numBoards = camCalDir.numFiles();
    successes = 0;
    boardPatternSize = boardXCount*boardYCount;
    boardSquareSize = 30.0; // in mm
    boardNumInternalCornersCV = cvSize( boardXCount, boardYCount);

    image_points = cvCreateMat(numBoards*boardPatternSize,2,CV_32FC1);
    object_points = cvCreateMat(numBoards*boardPatternSize,3,CV_32FC1);
    point_counts = cvCreateMat(numBoards,1,CV_32SC1);
    intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
    distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
    corners = new CvPoint2D32f[ boardPatternSize ];

    vertPlane = ofxPlane(ofPoint(0,0,0),ofVec3f(0,0,1));
    horizPlane = ofxPlane(ofPoint(0,0,0),ofVec3f(0,1,0));

    cout << "Asserting vertical plane direction is in proper coordinates ... ";
    assertPoint(vertPlane.pt+vertPlane.normal);
    cout << "passed!" << endl;
    cout << "Asserting horizontal plane direction is in proper coordinates ... ";
    assertPoint(horizPlane.pt+horizPlane.normal);
    cout << "passed!" << endl;

    cout << "Asserting input vertical plane points are in proper coordinates ... ";
    assertPoint(verticalPlaneObjectPts[0]);
    assertPoint(verticalPlaneObjectPts[1]);
    assertPoint(verticalPlaneObjectPts[2]);
    assertPoint(verticalPlaneObjectPts[3]);
    cout << "passed!" << endl;
    cout << "Asserting input horizontal plane points are in proper coordinates ... ";
    assertPoint(horizontalPlaneObjectPts[0]);
    assertPoint(horizontalPlaneObjectPts[1]);
    assertPoint(horizontalPlaneObjectPts[2]);
    assertPoint(horizontalPlaneObjectPts[3]);
    cout << "passed!" << endl;

    ofVec3f tN = ofVec3f(0,1,1);
    tN.normalize();
    negYnegZPlane = ofxPlane(ofPoint(0,50,50),tN);



    paused = false;

    points3dSubstate = POINTS3D_PROCESSING;



    ofPoint pts[3];

    pts[0] = ofPoint(0,0,0);
    pts[1] = ofPoint(0,1,0);
    pts[2] = ofPoint(1,0,0);

    ofVec3f expectedNormal = ofVec3f(0,0,1);
    expectedNormal.normalize();
    float expectedD = (expectedNormal.x*pts[0].x+expectedNormal.y*pts[0].y+expectedNormal.z*pts[0].z);

    ofxPlane testPlane = ofxPlane(3,pts);
    cout << endl;
    cout << endl;
    cout << "Test plane params [" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ", " << testPlane.d << "]" << endl;
    cout << "Expected plane params [" << expectedNormal.x << ", " << expectedNormal.y << ", " << expectedNormal.z << ", " << expectedD << "]" << endl;

    pts[0] = ofPoint(-1,2,0);
    pts[1] = ofPoint(3,1,4);
    pts[2] = ofPoint(0,-1,2);

    expectedNormal = ofVec3f(2,-8,5);
    expectedNormal.normalize();
    expectedD = (expectedNormal.x*pts[0].x+expectedNormal.y*pts[0].y+expectedNormal.z*pts[0].z);
    testPlane = ofxPlane(3,pts);
    cout << endl;
    cout << endl;
    cout << "Test plane params [" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ", " << testPlane.d << "]" << endl;
    cout << "Expected plane params [" << expectedNormal.x << ", " << expectedNormal.y << ", " << expectedNormal.z << ", " << expectedD << "]" << endl;
    pts[0] = ofPoint(0,0,5);
    pts[1] = ofPoint(1,0,5);
    pts[2] = ofPoint(0,1,5);

    expectedNormal = ofVec3f(0,0,1);
    expectedNormal.normalize();
    expectedD = (expectedNormal.x*pts[0].x+expectedNormal.y*pts[0].y+expectedNormal.z*pts[0].z);
    testPlane = ofxPlane(3,pts);
    cout << endl;
    cout << endl;
    cout << "Test plane params [" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ", " << testPlane.d << "]" << endl;
    cout << "Expected plane params [" << expectedNormal.x << ", " << expectedNormal.y << ", " << expectedNormal.z << ", " << expectedD << "]" << endl;

    pts[0] = ofPoint(1, -6, 0);
    pts[1] = ofPoint(-4, 2, -2);
    pts[2] = ofPoint(-2, 4, 1);

    expectedNormal = ofVec3f(28,11,-26);
    expectedNormal.normalize();
    expectedD = (expectedNormal.x*pts[0].x+expectedNormal.y*pts[0].y+expectedNormal.z*pts[0].z);
    testPlane = ofxPlane(3,pts);
    cout << endl;
    cout << endl;
    cout << "Test plane params [" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ", " << testPlane.d << "]" << endl;
    cout << "Expected plane params [" << expectedNormal.x << ", " << expectedNormal.y << ", " << expectedNormal.z << ", " << expectedD << "]" << endl;

    pts[0] = ofPoint(-8,-3,7);
    pts[1] = ofPoint(-8, 7, 4);
    pts[2] = ofPoint(-2, -4,-8);

    expectedNormal = ofVec3f(147,18,60);
    expectedNormal.normalize();
    expectedD = (expectedNormal.x*pts[0].x+expectedNormal.y*pts[0].y+expectedNormal.z*pts[0].z);
    testPlane = ofxPlane(3,pts);
    cout << endl;
    cout << endl;
    cout << "Test plane params [" << testPlane.normal.x << ", " << testPlane.normal.y << ", " << testPlane.normal.z << ", " << testPlane.d << "]" << endl;
    cout << "Expected plane params [" << expectedNormal.x << ", " << expectedNormal.y << ", " << expectedNormal.z << ", " << expectedD << "]" << endl;

    cout << endl;
    cout << endl;
    
}

void Scan3dApp::assertPoint(ofPoint pt){
    assert(pt.y >= -FLT_EPSILON); // Bloody negative y coordinates
    assert(pt.z >= -FLT_EPSILON); // At least the z coordinate is normal
}


//--------------------------------------------------------------
/**
    Loads settings from 'settings.xml', located in the data folder of the project
*/
void Scan3dApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "** Loading Settings File **" << endl;
		settings.pushTag("settings");
            settings.pushTag("user");
                settings.pushTag("input");
                    string input = settings.getValue("type","NONE");
                    if(input == "VIDEO"){
                        inputType = VIDEO;
                        inputVideoFile = settings.getValue("src","");
                        cout << "   Using video: " << inputVideoFile << endl;
                    }
                    else if(input == "IMAGE_SEQUENCE"){
                        inputType = IMAGE_SEQUENCE;
                        dir = ofDirectory(settings.getValue("src",""));
                        dir.listDir();
                        dir.sort();

                        cout << "   Using image sequence folder: " << dir.path() << endl;
                    }

    			
                settings.popTag(); //pop input
                settings.pushTag("calibration");
                    settings.pushTag("cam");
                        programState = CAMERA_CALIBRATION;
                        camCalDir = ofDirectory(settings.getValue("frames",""));
                        intrinsicFilename = ofToDataPath("intrinsic.xml");
                        distortionFilename = ofToDataPath("distortion.xml");
                        if(settings.tagExists("intrinsicFile") && settings.tagExists("distortionFile")){
                            intrinsicFilename = settings.getValue("intrinsicFile","intrinsic.xml");
                            distortionFilename = settings.getValue("distortionFile","distortion.xml");
                            camCalSubstate = CAM_CAL_LOADING;
                        }
                        else{
                            camCalDir.listDir();
                            camCalDir.sort();

                            cout << "   Using camera calibration sequence folder: " << camCalDir.path() << endl;
                            camCalSubstate = CAM_CAL_PROCESSING;  
                        }
                    settings.popTag();
                settings.popTag();
                settings.pushTag("misc");
                    zeroCrossingThreshold = settings.getValue("zeroCrossingThreshold", 0);
                settings.popTag();// pop misc
            settings.popTag(); // pop user
			settings.pushTag("scene");

				settings.pushTag("verticalPlane");

                    settings.pushTag("section");
                        if(settings.tagExists("imagePts")){
                            setupSubState = BOTTOM_SECTION;
                        }
                        settings.pushTag("imagePts");
                            topSection.x = settings.getValue("x",0.0);
                            topSection.y = settings.getValue("y",0.0);
                            topSection.width = settings.getValue("width",0.0);
                            topSection.height = settings.getValue("height",0.0);
                        settings.popTag();
                    settings.popTag(); // pop section
					cout << "   Loaded vertical plane points as [BR,TR,TL,BL]:"<< endl;

                    for(int i = 0; i < 4; i++){
                        settings.pushTag("marker"+ofToString(i));
                            if(settings.tagExists("imagePt")){
                                setupSubState = B_TL;
                            }
        					settings.pushTag("imagePt");
        					    verticalPlaneImagePts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0));
        					    cout << "      image:[" << verticalPlaneImagePts[i].x << ", " << verticalPlaneImagePts[i].y << "]";
        					settings.popTag(); // pop pts
                            settings.pushTag("objectPt");
                                verticalPlaneObjectPts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0),settings.getValue("z",0.0));
                                cout << "      object:[" << verticalPlaneObjectPts[i].x << ", " << verticalPlaneObjectPts[i].y << ", " << verticalPlaneObjectPts[i].z << "]" << endl;
                            settings.popTag(); // pop pts
                        settings.popTag(); // pop markers
                    }
				settings.popTag(); // pop verticalPlane

				settings.pushTag("horizontalPlane");
                    settings.pushTag("section");
                        if(settings.tagExists("imagePts")){
                            setupSubState = T_TL;
                        }
                        settings.pushTag("imagePts");
                            bottomSection.x = settings.getValue("x",0.0);
                            bottomSection.y = settings.getValue("y",0.0);
                            bottomSection.width = settings.getValue("width",0.0);
                            bottomSection.height = settings.getValue("height",0.0);
                        settings.popTag();
                    settings.popTag(); // pop section
					cout << "   Loaded horizontal plane points as [BR,TR,TL,BL]:"<< endl;
                    for(int i = 0; i < 4; i++){
    					settings.pushTag("marker"+ofToString(i));
                            if(settings.tagExists("imagePt")){
                                setupSubState = ESTIMATE_CAMERA;
                            }
                            settings.pushTag("imagePt");
        					    horizontalPlaneImagePts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0));
        					    cout << "      image:[" << horizontalPlaneImagePts[i].x << ", " << horizontalPlaneImagePts[i].y << "]";
    					    settings.popTag(); // pop pts
                            settings.pushTag("objectPt");
                                 horizontalPlaneObjectPts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0),settings.getValue("z",0.0));
                                 cout << "      object:[" << horizontalPlaneObjectPts[i].x << ", " << horizontalPlaneObjectPts[i].y << ", " << horizontalPlaneObjectPts[i].z << "]" << endl;
                            settings.popTag(); // pop pts
                        settings.popTag(); // pop markers
                    }   
                settings.popTag(); // pop verticalPlane

			settings.popTag(); // pop scene
            

		settings.popTag(); // pop settings
		cout << "** Done Loading Settings **" << endl;
	}
	else {
		cout << "No settings file to load." << endl;
	}


}

//--------------------------------------------------------------
/**
    Saves settings to 'settings.xml', located in the data folder of the project
*/
void Scan3dApp::saveSettings(){
    ofxXmlSettings settings;
    cout << "** Saving Settings File **" << endl;
    settings.addTag("settings");
    settings.pushTag("settings");
        settings.addTag("user");
        settings.pushTag("user");
            settings.addTag("input");
            settings.pushTag("input");
                
                if(inputType == VIDEO){
                    settings.setValue("type","VIDEO");
                    inputVideoFile = settings.setValue("src",inputVideoFile);
                    cout << "   Set video: " << inputVideoFile << endl;
                }
                else if(inputType == IMAGE_SEQUENCE){
                    settings.setValue("type","IMAGE_SEQUENCE");
                    inputType = IMAGE_SEQUENCE;
                    settings.setValue("src",dir.path());
                    cout << "   Set image sequence folder: " << dir.path() << endl;
                }
            
            settings.popTag(); //pop input
            settings.addTag("calibration");
            settings.pushTag("calibration");
                settings.addTag("cam");
                settings.pushTag("cam");
                    settings.setValue("frames",camCalDir.path());
                    settings.setValue("intrinsicFile",intrinsicFilename);
                    settings.setValue("distortionFile",distortionFilename);
                settings.popTag(); //pop cam
            settings.popTag(); //pop calibration
            settings.addTag("misc");
            settings.pushTag("misc");
                settings.setValue("zeroCrossingThreshold", zeroCrossingThreshold);
            settings.popTag();// pop misc
        settings.popTag(); //pop user
        settings.addTag("scene");
        settings.pushTag("scene");
            settings.addTag("verticalPlane");
            settings.pushTag("verticalPlane");
                settings.addTag("section");
                settings.pushTag("section");
                    settings.addTag("imagePts");
                    settings.pushTag("imagePts");
                        settings.setValue("x",topSection.x);
                        settings.setValue("y",topSection.y);
                        settings.setValue("width",topSection.width);
                        settings.setValue("height",topSection.height);
                    settings.popTag(); // pop imagePts
                settings.popTag(); // pop section
                cout << "   Setting vertical plane points as [BR,TR,TL,BL]:"<< endl;
                for(int i = 0; i < 4; i++){
                    settings.addTag("marker"+ofToString(i));
                    settings.pushTag("marker"+ofToString(i));
                        settings.addTag("imagePt");
                        settings.pushTag("imagePt");
                            settings.setValue("x",verticalPlaneImagePts[i].x); settings.setValue("y",verticalPlaneImagePts[i].y);
                            cout << "      [" << verticalPlaneImagePts[i].x << ", " << verticalPlaneImagePts[i].y << "]" << endl;
                        settings.popTag(); // pop imagePt
                        settings.addTag("objectPt");
                        settings.pushTag("objectPt");
                            settings.setValue("x",verticalPlaneObjectPts[i].x); settings.setValue("y",verticalPlaneObjectPts[i].y); settings.setValue("z",verticalPlaneObjectPts[i].z);
                        settings.popTag(); // pop objectPt
                    settings.popTag(); // pop markers
                }
            settings.popTag(); // pop verticalPlane

            settings.addTag("horizontalPlane");
            settings.pushTag("horizontalPlane");
                settings.addTag("section");
                settings.pushTag("section");
                    settings.addTag("imagePts");
                    settings.pushTag("imagePts");
                        settings.setValue("x",bottomSection.x);
                        settings.setValue("y",bottomSection.y);
                        settings.setValue("width",bottomSection.width);
                        settings.setValue("height",bottomSection.height);
                    settings.popTag(); // pop imagePts
                settings.popTag(); // pop section
                cout << "   Setting horizontal plane points as [BR,TR,TL,BL]:"<< endl;
                for(int i = 0; i < 4; i++){
                    settings.addTag("marker"+ofToString(i));
                    settings.pushTag("marker"+ofToString(i));
                        settings.addTag("imagePt");
                        settings.pushTag("imagePt");
                            settings.setValue("x",horizontalPlaneImagePts[i].x); settings.setValue("y",horizontalPlaneImagePts[i].y);
                            cout << "      [" << horizontalPlaneImagePts[i].x << ", " << horizontalPlaneImagePts[i].y << "]" << endl;
                        settings.popTag(); // pop imagePt
                        settings.addTag("objectPt");
                        settings.pushTag("objectPt");
                            settings.setValue("x",horizontalPlaneObjectPts[i].x); settings.setValue("y",horizontalPlaneObjectPts[i].y); settings.setValue("z",horizontalPlaneObjectPts[i].z);
                        settings.popTag(); // pop objectPt
                    settings.popTag(); // pop marker
                }
            settings.popTag(); // pop horizontalPlane

        settings.popTag(); // pop scene

    settings.popTag(); // pop settings
    settings.saveFile("settings.xml");
    cout << "** Done Saving Settings **" << endl;
}

//--------------------------------------------------------------
void Scan3dApp::clearSettings(){
   topSection.set(0,0,0,0);
   bottomSection.set(0,0,0,0);
   for(int i = 0; i < 4; i++){
        verticalPlaneImagePts[i].x = 0;
        verticalPlaneImagePts[i].y = 0;
        horizontalPlaneImagePts[i].x = 0;
        horizontalPlaneImagePts[i].y = 0;
    }
    programState = SETUP;
    setupSubState = TOP_SECTION;
}

//--------------------------------------------------------------
void Scan3dApp::update(){
    if(!paused){
        switch(programState){
            case CAMERA_CALIBRATION:
                camCalUpdate();
                break;
            case SETUP:
            {
                setupUpdate();
                break;
            }
            case CAPTURE:
            {
                captureUpdate();
                break;
            }
            case PROCESSING:
            {     
                processingUpdate();
                break;
            }
            case POINTS3D:
            {    
                points3dUpdate();
                break;
            }

        }
    }
}
void Scan3dApp::camCalUpdate(){
    ofFile intrinsicFile;
    ofFile distortionFile;
    messageBarText = "CAMERA CALIBRATION";
    switch(camCalSubstate){
        case CAM_CAL_PROCESSING:
            messageBarSubText = "Processing calibration frames";
            if(camCalFrame < numBoards){
                bufferOfImage.loadImage(camCalDir.getPath(camCalFrame));
                colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
                grayscaleFrame = colorFrame;

                //(Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
                IplImage* image = colorFrame.getCvImage();
                IplImage* gray_image = grayscaleFrame.getCvImage();

                 
                //Find chessboard corners:
                int found = cvFindChessboardCorners(
                    image, 
                    boardNumInternalCornersCV, 
                    corners, 
                    &corner_count,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                );

                
               
                //Get Subpixel accuracy on those corners

                cvFindCornerSubPix(
                    gray_image, 
                    corners, 
                    corner_count,
                    cvSize(11,11),
                    cvSize(-1,-1), 
                    cvTermCriteria(
                        CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 
                        30, 
                        0.1 
                    )
                );
                //Draw it
                cvDrawChessboardCorners(
                    image, 
                    boardNumInternalCornersCV, 
                    corners,
                    corner_count, 
                    found
                );
                
               
                // If we got a good board, add it to our data
                if( corner_count == boardPatternSize ) {
                    int step = successes*boardPatternSize;
                    for( int i=step, j=0; j<boardPatternSize; ++i,++j ) {
                        CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
                        CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
                        CV_MAT_ELEM(*object_points,float,i,0) = j/boardXCount;
                        CV_MAT_ELEM(*object_points,float,i,1) = j%boardXCount;
                        CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
                    }
                    CV_MAT_ELEM(*point_counts, int,successes,0) = boardPatternSize;
                    successes++;
                }

                

                colorFrame = image;
                camCalFrame++;
            }
            else{
                //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
                CvMat* object_points2 = cvCreateMat(successes*boardPatternSize,3,CV_32FC1);
                CvMat* image_points2 = cvCreateMat(successes*boardPatternSize,2,CV_32FC1);
                CvMat* point_counts2 = cvCreateMat(successes,1,CV_32SC1);
                //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
               
                for(int i = 0; i<successes*boardPatternSize; ++i) {
                    CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0);
                    CV_MAT_ELEM( *image_points2, float,i,1) = CV_MAT_ELEM( *image_points, float, i, 1);
                    CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0) ;
                    CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1) ;
                    CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2) ;
                }
                for(int i=0; i<successes; ++i){ //These are all the same number
                    CV_MAT_ELEM( *point_counts2, int, i, 0) = CV_MAT_ELEM( *point_counts, int, i, 0);
                }
                cvReleaseMat(&object_points);
                cvReleaseMat(&image_points);
                cvReleaseMat(&point_counts);
                // At this point we have all of the chessboard corners we need.
                // Initialize the intrinsic matrix such that the two focal
                // lengths have a ratio of 1.0
                //
                CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
                CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
                //CV_MAT_ELEM( *intrinsic_matrix, float, 2, 2) = 1.0f;


                
                //CALIBRATE THE CAMERA!
                cvCalibrateCamera2(
                    object_points2, 
                    image_points2,
                    point_counts2, 
                    cvSize(width,height),
                    intrinsic_matrix, 
                    distortion_coeffs,
                    NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
                );

                printf("\n");
                printf("Intrinsic Matrix: ");
                printf("\n");
                for(int r = 0; r < 3; r++){
                    for(int c = 0; c < 3; c++){
                        CvScalar scal = cvGet2D(intrinsic_matrix,r,c); 
                        printf( "%f\t", scal.val[0]); 
                    }
                    printf("\n");
                }
                printf("\n");
                printf("\n");

                focal_length = ofVec2f(CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *intrinsic_matrix, float, 1,1));
                cout << "Focal length not absurd ... ";
                assert(focal_length.x > 0 && focal_length.y > 0);
                cout << " passed! [" << focal_length << ']' << endl;

                principal_point = ofVec2f(CV_MAT_ELEM( *intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *intrinsic_matrix, float, 1,2));

                cout << "Principal point near image center not absurd   pp[" << principal_point << "], center[" << ofPoint(width/2,height/2) << "] ... ";
                assert(abs(principal_point.x - width/2) < width/4 && abs(principal_point.y - height/2) < height/4);
                cout << " passed!"<< endl;
                
                // SAVE THE INTRINSICS AND DISTORTIONS
                cvSave(intrinsicFilename.c_str(),intrinsic_matrix);
                cvSave(distortionFilename.c_str(),distortion_coeffs);

                mapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                mapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                cvInitUndistortMap(
                    intrinsic_matrix,
                    distortion_coeffs,
                    mapx,
                    mapy
                );

                
                
                programState = SETUP;
            }
            break;

        case CAM_CAL_LOADING:
            messageBarSubText = "Loading calibration file";
            
            intrinsicFile.open(intrinsicFilename, ofFile::ReadWrite, false);
            if(!intrinsicFile.exists()){
                camCalSubstate = CAM_CAL_PROCESSING;
                update();
            }

            distortionFile.open(distortionFilename, ofFile::ReadWrite, false);
            if(!distortionFile.exists()){
                camCalSubstate = CAM_CAL_PROCESSING;
                update();
            }
            intrinsic_matrix = (CvMat*)cvLoad(intrinsicFilename.c_str());
            distortion_coeffs = (CvMat*)cvLoad(distortionFilename.c_str());
            
            printf("\n");
            printf("Intrinsic Matrix: ");
            printf("\n");
            for(int r = 0; r < 3; r++){
                for(int c = 0; c < 3; c++){
                    CvScalar scal = cvGet2D(intrinsic_matrix,r,c); 
                    printf( "%f\t", scal.val[0]); 
                }
                printf("\n");
            }
            printf("\n");
            printf("\n");

            focal_length = ofVec2f(CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *intrinsic_matrix, float, 1,1));
            cout << "Focal length not absurd ... ";
            assert(focal_length.x > 0 && focal_length.y > 0);
            cout << " passed! [" << focal_length << ']' << endl;

            principal_point = ofVec2f(CV_MAT_ELEM( *intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *intrinsic_matrix, float, 1,2));
            cout << "Principal point near image center not absurd   pp[" << principal_point << "], center[" << ofPoint(width/2,height/2) << "] ... ";
            assert(abs(principal_point.x - width/2) < width/4 && abs(principal_point.y - height/2) < height/4);
            cout << " passed!"<< endl;

            mapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            mapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            cvInitUndistortMap(
                intrinsic_matrix,
                distortion_coeffs,
                mapx,
                mapy
            );

            programState = SETUP;
            break;
    }
    
    
}

void Scan3dApp::setupUpdate(){
    ofPoint imagePts[8];
    ofPoint objectPts[8];
    ofPoint pixelCoord;
    ofxRay3d ray;
    ofxPlane testPlane;
    ofPoint intersectPt;
    ofxRay3d centerRay;
    ofVec3f centerPt;
    ofNode lookat;
    ofVec3f easyCamPos;

    messageBarText = "SETUP";
    switch(setupSubState){
        case TOP_SECTION:
            messageBarSubText = "Click and drag top rectangle (or press c to clear settings and start over)";
            break;
        case  BOTTOM_SECTION:
            messageBarSubText = "Click and drag bottom rectangle (or press c to clear settings and start over)";
            break;
        case  T_TL:
            messageBarSubText = "Click to select the top left marker on the vertical backplane (or press c to clear settings and start over)";
            break;
        case  T_TR:
            messageBarSubText = "Click to select the top right marker on the vertical backplane (or press c to clear settings and start over)";
            break;
        case  T_BR:
            messageBarSubText = "Click to select the bottom right marker on the vertical backplane (or press c to clear settings and start over)";
            break;
        case  T_BL:
            messageBarSubText = "Click to select the bottom left marker on the vertical backplane (or press c to clear settings and start over)";
            break;
        case  B_TL:
            messageBarSubText = "Click to select the top left marker on the horizontal backplane (or press c to clear settings and start over)";
            break;
        case  B_TR:
            messageBarSubText = "Click to select the top right marker on the horizontal backplane (or press c to clear settings and start over)";
            break;
        case  B_BL:
            messageBarSubText = "Click to select the bottom left marker on the horizontal backplane (or press c to clear settings and start over)";
            break;
        case B_BR:
            messageBarSubText = "Click to select the bottom right marker on the horizontal backplane (or press c to clear settings and start over)";
            break;
        case ESTIMATE_CAMERA:
            messageBarSubText = "Estimating camera pose...";
            for(int i = 0; i < 8; i++){
                if(i < 4){
                    imagePts[i] = verticalPlaneImagePts[i];
                    objectPts[i] = verticalPlaneObjectPts[i];
                }
                else{
                    imagePts[i] = horizontalPlaneImagePts[i-4];
                    objectPts[i] = horizontalPlaneObjectPts[i-4];
                }
                
            }
            computeExtrinsicMatrix(objectPts,imagePts,intrinsic_matrix, distortion_coeffs);
            // cout << "Points and such "<< endl;
            // cout << verticalPlaneObjectPts[0] << endl;
            // cout << verticalPlaneImagePts[0] << endl;

            // pixelCoord = pt3DToPixel(intrinsic_matrix,extrinsic_matrix,verticalPlaneObjectPts[0]);

            // cout << pixelCoord << endl;

            // ray = pixelToRay(intrinsic_matrix,extrinsic_matrix,verticalPlaneImagePts[2]);

            // intersectPt = ray.intersect(vertPlane);

            // cout << intersectPt << endl;

            // testPlane = vertPlane.interpolate(ofxPlane(ofPoint(0,0,0),ofVec3f(0,-1,0)), 0.5);

            // cout << testPlane.normal << endl;


            centerRay = pixelToRay(intrinsic_matrix,extrinsic_matrix, ofPoint(width/2,height/2));
            centerPt = (ofVec3f)centerRay.intersect(horizPlane);
            if(centerPt.z <=0){
                centerPt = (ofVec3f)centerRay.intersect(vertPlane);
            }
            
            lookat.setPosition(centerPt);

            easyCamPos = (ofVec3f)camPos;
            

            easyCam.setPosition(easyCamPos);
            easyCam.setTarget(lookat);
            //easyCam.setScale(1,1,-1);

            cout << "camPos:" << endl;
            cout << camPos << endl;

            cout << "easyCam pos:" << endl;
            cout << easyCam.getPosition() << endl;

            cout << "easyCam distance:" << endl;
            cout << easyCam.getDistance() << endl;

            cout << "easyCam target position:" << endl;
            cout << easyCam.getTarget().getPosition() << endl;
            setupSubState = WAITING;
            break;
        case WAITING:
            
            
            messageBarSubText = "Press spacebar to commence scanning (or press c to clear settings and start over)";                    
            break;
    }

    /*
        Initializing with first frame (either from image sequence or video)
    */
    
    switch(inputType){
        case VIDEO:
            vid.firstFrame();
            vid.update();
            if(vid.isFrameNew()){
                colorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());
                colorFrame.flagImageChanged();
                colorFrame.updateTexture();
                colorFrame.remap(mapx,mapy);

                grayscaleFrame = colorFrame;
            }
            break;
        case IMAGE_SEQUENCE:
            bufferOfImage.loadImage(dir.getPath(frameIndex));
            colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
            colorFrame.flagImageChanged();
            colorFrame.updateTexture();
            colorFrame.remap(mapx,mapy);
            grayscaleFrame = colorFrame;
            break;
    }
}


ofPoint Scan3dApp::pt3DToPixel(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint pt3D){
    CvMat* A = cvCreateMat( 3, 4, CV_32FC1 );
    cvMatMul(intrinsic_matrix,extrinsic_matrix,A); //combining into one matrix
    CvMat* x = cvCreateMat( 4, 1, CV_32FC1 ); // homogenized point
    CV_MAT_ELEM(*x, float, 0,0) = pt3D.x;
    CV_MAT_ELEM(*x, float, 1,0) = pt3D.y;
    CV_MAT_ELEM(*x, float, 2,0) = pt3D.z;
    CV_MAT_ELEM(*x, float, 3,0) = 1.0;

    CvMat* b = cvCreateMat( 3, 1, CV_32FC1 ); // pixel coordinate solving for
    cvMatMul(A,x,b);
    CV_MAT_ELEM(*b,float,0,0) /= CV_MAT_ELEM(*b,float,2,0);
    CV_MAT_ELEM(*b,float,1,0) /= CV_MAT_ELEM(*b,float,2,0);
    CV_MAT_ELEM(*b,float,2,0) /= CV_MAT_ELEM(*b,float,2,0);
    ofPoint pixelRes = ofPoint(CV_MAT_ELEM(*b,float,0,0),CV_MAT_ELEM(*b,float,1,0),CV_MAT_ELEM(*b,float,2,0));

    cvReleaseMat(&A);
    cvReleaseMat(&x);
    cvReleaseMat(&b);

    return pixelRes;
}

void Scan3dApp::convertOfPointsToCvMat(ofPoint *pts, int dimensions, int size, CvMat* output){
    if(pts == NULL){
        return;
    }
    for(int i = 0; i < size; i++){
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

void Scan3dApp::captureUpdate(){
    messageBarText = "SCANNING";
    switch(displayState){
        case COLOR:
            messageBarText += "  (COLOR)";
            break;
        case GRAYSCALE:
            messageBarText += "  (GRAYSCALE)";
            break;
        case MINIMAGE:
            messageBarText += "  (MIN)";
            break;
        case MAXIMAGE:
            messageBarText += "  (MAX)";
            break;
        case SHADOWTHRESHIMAGE:
            messageBarText += "  (SHADOW THRESH)";
            break;
    }

    messageBarSubText = "Press [1, 2, 3, 4, 5] for [COLOR, GRAYSCALE, MIN, MAX, SHADOW THRESH] display";
    
    bool framesDone = false;
    switch(inputType){
        case VIDEO:
            vid.update();
            if(vid.isFrameNew()){
                colorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());
                colorFrame.flagImageChanged();
                colorFrame.updateTexture();
                colorFrame.remap(mapx,mapy);}


            framesDone = vid.getIsMovieDone();
           
            break;
        case IMAGE_SEQUENCE:
            if(frameIndex < dir.numFiles()){
                bufferOfImage.loadImage(dir.getPath(frameIndex));
                colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
                colorFrame.flagImageChanged();
                colorFrame.updateTexture();
                colorFrame.remap(mapx,mapy);}

            else{
                framesDone = true; 
            }
            break;

    }

    if(framesDone){
        colorFrame = maxColorImg;
        messageBarText = "PROCESSING";
        messageBarSubText = "";
        programState = PROCESSING;
        planes.resize(numFrames,ofxPlane());

        cout << "PROCESSING STATE" << endl;
        frameIndex = 0;
    }
    else{
        numFrames = frameIndex+1;
    }

    //Uncomment to save out color frames
    // bufferOfImage.setFromPixels(colorFrame.getPixelsRef());
    // filename = "output/grayscaleFrames/gsframe";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);
        
    grayscaleFrame = colorFrame;

    //resizing vector if needed
    if((unsigned)frameIndex >= frames.capacity()){
        frames.resize(2*frames.size(),bufferOfxCvGrayscaleImage);
    }

    frames[frameIndex] = grayscaleFrame;

    //Uncomment to save out grayscale frames
    // bufferOfxCvColorImage = grayscaleFrame;
    // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
    // filename = "output/grayscaleFrames/gsframe";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);

    //Update min/max images
    unsigned char* minImgPixels = minImg.getPixels();
    unsigned char* maxImgPixels = maxImg.getPixels();
    unsigned char* maxColorImgPixels = maxColorImg.getPixels();
    unsigned char* grayscaleFramePixels = grayscaleFrame.getPixels();
    unsigned char* colorFramePixels = colorFrame.getPixels();
    unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();

    int i = 0;
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            i = y*width+x;
            minImgPixels[i] = min(minImgPixels[i],grayscaleFramePixels[i]);
            maxImgPixels[i] = max(maxImgPixels[i],grayscaleFramePixels[i]);
            shadowThreshImgPixels[i] = ((minImgPixels[i]+maxImgPixels[i])/2.0);
            maxColorImgPixels[3*i] = max(maxColorImgPixels[3*i],colorFramePixels[3*i]);
            maxColorImgPixels[3*i+1] = max(maxColorImgPixels[3*i+1],colorFramePixels[3*i+1]);
            maxColorImgPixels[3*i+2] = max(maxColorImgPixels[3*i+2],colorFramePixels[3*i+2]);
        }  
    }

    //A tad inefficient, but this handles a bug where if you draw these it doesn't set new values
    minImg.setFromPixels(minImgPixels,width,height);
    maxImg.setFromPixels(maxImgPixels,width,height);
    shadowThreshImg.setFromPixels(shadowThreshImgPixels,width,height);
    maxColorImg.setFromPixels(maxColorImgPixels,width,height);


    
    frameIndex++;
    
}

void Scan3dApp::processingUpdate(){
    ofPoint pts[3];
    grayscaleFrame = frames[frameIndex];
    diffFrame.set(0);
    bufferOfxCvGrayscaleImage.set(0);
    bufferOfxCvGrayscaleImage = frames[0];;
    bufferOfxCvGrayscaleImage -= shadowThreshImg;
    bufferOfxCvGrayscaleImage.blurGaussian();
    bufferOfxCvGrayscaleImage.blurGaussian();
    bufferOfxCvGrayscaleImage.blurGaussian();
    bufferOfxCvGrayscaleImage -= 5;
    bufferOfxCvGrayscaleImage.threshold(30,true);

    diffFrame = grayscaleFrame;
    diffFrame -= shadowThreshImg;
    diffFrame -= bufferOfxCvGrayscaleImage;
    //diffFrame.blurGaussian();
    



    /*  
    *   Computing the enter and exit frames, used to create the temporal image
    */
    unsigned char* diffFramePixels = diffFrame.getPixels();
    unsigned char* temporalImgPixels = temporalImg.getPixels();
    unsigned char* enterFramePixels = enterFrame.getPixels();
    unsigned char* exitFramePixels = exitFrame.getPixels();
    
    
    for(int r = 0; r < height; r++){
        for(int c = 0; c < width; c++){
            if(diffFramePixels[c+r*width] == 0){ // pixel in shadow
                if(enterFramePixels[c+r*width] == 0){ // Never been in shadow before
                    float framePixelMapping = ofMap(frameIndex,0,numFrames,0.0,255.0);

                    enterFramePixels[c+r*width] = framePixelMapping; // Setting the enter frame to the current frame

                    ofColor temporalColor;


                    if(framePixelMapping <= zeroCrossingThreshold){
                        temporalColor = ofColor(0,0,0);
                    }
                    else{
                        temporalColor = ofColor::fromHsb(framePixelMapping,255,255);
                    }
                    
                    
                    temporalImgPixels[3*(c+r*width)] =      temporalColor.r;
                    temporalImgPixels[3*(c+r*width)+1] =    temporalColor.g;
                    temporalImgPixels[3*(c+r*width)+2] =    temporalColor.b;
                }
                
            }
            else{ // Pixel is not in shadow
                if(enterFramePixels[c+r*width] == 0){ // Never been in shadow before
                    // do squat
                }
                else{ // Pixel has been in shadow but is no longer
                    if(exitFramePixels[c+r*width]== 0){ // First time exiting shadow
                        exitFramePixels[c+r*width]= ofMap(frameIndex,0,numFrames,0.0,255.0);
                    }
                    else{
                        // do squat
                    }
                }
                
            }
        }
    }
    
    enterFrame.setFromPixels(enterFramePixels,width,height);
    exitFrame.setFromPixels(exitFramePixels,width,height);
    temporalImg.setFromPixels(temporalImgPixels,width,height);

    diffFrame.threshold(0);
    diffFrame = computeGradientImage(diffFrame,RIGHT);

    
    topLine = computeLineFromZeroCrossings(diffFrame,topSection);
    bottomLine = computeLineFromZeroCrossings(diffFrame,bottomSection);  

    if(topLine.isInit() && bottomLine.isInit()){
        planePts.setMode(OF_PRIMITIVE_POINTS);

        testLine = ofxLine2d(ofVec2f(1,0),topLine.intersection(bottomLine));

        ofxLine2d yMinLine = ofxLine2d(ofVec2f(1,0),ofPoint(0,0));
        ofxLine2d yMaxLine = ofxLine2d(ofVec2f(1,0),ofPoint(0,height));
        ofxRay3d ray;
        ofPoint pt;
        ofColor color;

        

        ofPoint topLineTopPoint = yMinLine.intersection(topLine);
        ray = pixelToRay(intrinsic_matrix,extrinsic_matrix,topLineTopPoint);
        pt = ray.intersect(vertPlane);
        pts[0] = pt;
        planePts.addVertex((ofVec3f)pt);
        color.setHsb(ofMap(frameIndex,0,numFrames,0.0,255.0),255,255);
        planePts.addColor(color);

        ofPoint lineCrossPoint = bottomLine.intersection(topLine);
        ray = pixelToRay(intrinsic_matrix,extrinsic_matrix,lineCrossPoint);
        pt = ray.intersect(vertPlane);
        if(pt.y <= FLT_EPSILON || pt.z <= FLT_EPSILON){
            pt = ray.intersect(horizPlane);
        }
        pts[1] = pt;
        planePts.addVertex((ofVec3f)pt);
        planePts.addColor(color);

        ofPoint bottomLineBottomPoint = yMaxLine.intersection(bottomLine);
        ray = pixelToRay(intrinsic_matrix,extrinsic_matrix,bottomLineBottomPoint);
        pt = ray.intersect(horizPlane);
        pts[2] = pt;
        planePts.addVertex((ofVec3f)pt);
        planePts.addColor(color);

        

        
        

        // ofxLine3d top3dLine = projectLineOntoPlane(topLine,vertPlane,intrinsic_matrix,extrinsic_matrix);
        // ofxLine3d bottom3dLine = projectLineOntoPlane(bottomLine,horizPlane,intrinsic_matrix,extrinsic_matrix);

        // ofxPlane plane = ofxPlane(top3dLine, bottom3dLine);
        ofxPlane plane = ofxPlane(3,pts);

        
        planes[frameIndex] = plane;
    }

    //cout << "topLine " << topLine.dir.x << " " << topLine.dir.y << endl;
    //cout << "bottomLine " << bottomLine.dir.x << " " << bottomLine.dir.y << endl;

    

        
        

        // cout << '[' << planeFromLines.normal.x << ',' << planeFromLines.normal.y << ',' << planeFromLines.normal.z << ']' << endl;


    // cout << '[' << planeFromLines.normal.x << ',' << planeFromLines.normal.y << ',' << planeFromLines.normal.z << ']' << endl;


     //Uncomment to save out diff frames
    // bufferOfxCvColorImage = diffFrame;
    // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
    // filename = "output/diffFrames/diffFrame";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);

    //Uncomment to save out zero crossing frames
    // bufferOfxCvColorImage = enterFrame;
    // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
    // filename = "output/enterFrames/enterFrame";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);


    frameIndex++;
    if((unsigned)frameIndex == numFrames){
        //Uncomment to save out temporal image
        bufferOfImage.setFromPixels(temporalImg.getPixelsRef());
        bufferOfImage.saveImage("output/temporalImg.tiff");

        programState = POINTS3D;

        cout << "POINTS3D STATE" << endl;
        messageBarText = "POINTS3D";
        messageBarSubText = "Computing 3d Points...";
        //Uncomment to save out min/max/shadowthresh frames
        bufferOfxCvColorImage = minImg;
        bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
        bufferOfImage.saveImage("output/minImg.tiff");
        bufferOfxCvColorImage = maxImg;
        bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
        bufferOfImage.saveImage("output/maxImg.tiff");
        bufferOfxCvColorImage = shadowThreshImg;
        bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
        bufferOfImage.saveImage("output/shadowThreshImg.tiff");
        
        
    }

    diffFrame = bufferOfxCvGrayscaleImage;
}

void Scan3dApp::points3dUpdate(){
    
    
    points.resize(width*height,ofPoint(5,5,5));
    colors.resize(width*height,ofColor(255,255,255));
    int numPoints = 0;

    unsigned char* temporalImgPixels = temporalImg.getPixels(); 
    unsigned char* colorImgPixels = colorFrame.getPixels(); 
    switch(points3dSubstate){
            case POINTS3D_PROCESSING:
                for(int r = 0; r < height; r++){
                    for(int c = 0; c < width; c++){ 
                        
                        ofColor temporalPixel; 
                        //cout << '[r,c] : [' << r << ',' << c << ']' << endl;         
                        temporalPixel[0] = temporalImgPixels[3*(c+r*width)];
                        temporalPixel[1] = temporalImgPixels[3*(c+r*width)+1];
                        temporalPixel[2] = temporalImgPixels[3*(c+r*width)+2];

                        if(temporalPixel.getBrightness() == 0){
                            continue;
                        }
                        else{
                            float computedFrameIndex = getFrameFromColor(temporalPixel);
                            if(!isPlaneAtFrameIndex(computedFrameIndex)){
                                continue;
                            }
                            ofxPlane plane = getPlaneFromFrameIndex(computedFrameIndex);
                            if(plane.isInit()){
                                ofxRay3d ray = pixelToRay(intrinsic_matrix,extrinsic_matrix, ofPoint(c,r));
                                
                                //ofPoint pt = (((int)computedFrameIndex) % 2) == 0 ? ray.intersect(vertPlane) : ray.intersect(horizPlane);
                                ofPoint pt = ray.intersect(plane);
                                // cout << "Ray origin should be camera pos: " << endl;
                                // cout << "   camPos: ";
                                // cout << camPos << endl;
                                // cout << "   ray.origin: ";
                                // cout << ray.origin << endl;
                                // cout << "Ray dir and tes vec (intersect pt minus ray.origin, normalized) should be the same: " << endl;
                                // cout << "   ray.dir: ";
                                // cout << ray.dir << endl;
                                // ofVec3f testVec = pt-ray.origin;
                                // testVec.normalize();
                                // cout << "   testVec: ";
                                // cout << testVec << endl;
                                if(pt.y <= 0){
                                    pt.y = 0;
                                }
                                if(pt.z <= 0){
                                    pt.z = 0;
                                }

                                if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z){
                                    // not a valid point
                                }
                                else{
                                    ofPoint pixelPt = pt3DToPixel(intrinsic_matrix,extrinsic_matrix,pt);
                                    points[numPoints] = pt;
                                    
                                    ofColor pixelColor = ofColor(   (int)colorImgPixels[3*((int)pixelPt.x+(int)pixelPt.y*width)],
                                                                    (int)colorImgPixels[3*((int)pixelPt.x+(int)pixelPt.y*width)+1],
                                                                    (int)colorImgPixels[3*((int)pixelPt.x+(int)pixelPt.y*width)+2]
                                                                );


                                    colors[numPoints] = pixelColor;
                                    numPoints++;
                                }
                                
                                //cout << pt << endl;
                            }
                            
                            
                        }
                    }
                }

                writePointsToFile(numPoints);
                
                
                messageBarText = "POINTS3D";
                messageBarSubText = "Computing 3d Points...";
                points3dSubstate = POINTS3D_WAITING;
                bDrawPointCloud = true;
                break;
            case POINTS3D_WAITING:
            {
                
                messageBarText = "POINTS3D";
                messageBarSubText = "Done!";
                break;
            }
        }

        
   

    //do nothing
}

void Scan3dApp::writePointsToFile(int numPoints){
    ofFile outputFile("scan3d_output.ply", ofFile::WriteOnly);
    outputFile <<"ply\n";
    outputFile <<"format ascii 1.0\n";
    outputFile <<"element vertex ";
    outputFile << ofToString(numPoints);
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
    
    for(int i = 0; i < numPoints; i++){
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

    messageBarFont.drawString(messageBarText,10,height+messageBarHeight/2-5);
    messageBarSubTextFont.drawString(messageBarSubText,10,height+messageBarHeight-15);

    switch(displayState){
        case COLOR:
            colorFrame.draw(0, 0);
            break;
        case GRAYSCALE:
            grayscaleFrame.draw(0, 0);
            break;
        case MINIMAGE:
            minImg.draw(0, 0);
            break;
        case MAXIMAGE:
            maxImg.draw(0, 0);
            break;
        case SHADOWTHRESHIMAGE:
            temporalImg.draw(0, 0);
            break;
        case DIFF:
            diffFrame.draw(0, 0);
            break;

    }

    switch(programState){
        case SETUP:
            setupDraw();
            break;
        case CAPTURE:
            captureDraw();
            break; 
        case PROCESSING:
            if(bDrawPointCloud){
                easyCam.begin();
                drawPointCloud();
                easyCam.end();
            }
            else{
                processingDraw();
            }
            
            break;
        case POINTS3D:
            if(bDrawPointCloud){
                easyCam.begin();
                drawPointCloud();
                easyCam.end();
            }
            break;
    }
}

void Scan3dApp::camCalDraw(){

}

void Scan3dApp::setupDraw(){
    drawSectionRectangles();
    drawMarkerPoints();
}

void Scan3dApp::captureDraw(){
    drawSectionRectangles();
    drawMarkerPoints();
}

void Scan3dApp::processingDraw(){
    drawSectionRectangles();
    drawMarkerPoints();

    fullSection.x = 0;
    fullSection.y = 0;
    fullSection.width = width;
    fullSection.height = height;

    if(topLine.isLineInRegion(topSection) && bottomLine.isLineInRegion(bottomSection) && isPointInRegion(topLine.intersection(bottomLine),fullSection)){
        ofSetColor(topSectionColor);
        ofNoFill();
        ofSetLineWidth(4);
        topLine.drawLineInRegion(topSection);
        
        ofSetLineWidth(1);
        topLine.drawLineInRegion(fullSection);

        ofSetColor(bottomSectionColor);
        ofNoFill();
        ofSetLineWidth(4);
        bottomLine.drawLineInRegion(bottomSection);

        ofNoFill();
        ofSetLineWidth(1);
        bottomLine.drawLineInRegion(fullSection); 
    }
}



//--------------------------------------------------------------
/**
    Determine if point is in region

    @param pt The point to check
    @param roi The region of interest
    @returns bool
*/
bool Scan3dApp::isPointInRegion(ofPoint pt, ofRectangle roi){
    if(pt.x == -1 || pt.y == -1){
        return false;
    }
    return  pt.x >= roi.x && 
            pt.x <= (roi.x + roi.width) && 
            pt.y >= roi.y && 
            pt.y <= (roi.y + roi.height);
}


//--------------------------------------------------------------
/**
    Draws the section rectangles as a color overlay
*/
void Scan3dApp::drawSectionRectangles(){
    ofSetLineWidth(2);
    ofNoFill();
    ofEnableAlphaBlending();
    ofSetColor(topSectionColor);
    ofRect(topSection);

    ofSetColor(bottomSectionColor);
    ofRect(bottomSection);

    ofDisableAlphaBlending();
}

//--------------------------------------------------------------
/**
    Draws the marker points
*/
void Scan3dApp::drawMarkerPoints(){
    
    ofSetColor(topSectionColor);
    ofSetLineWidth(1);
    for(int i = 0; i < 4; i++){
        ofLine(verticalPlaneImagePts[i].x-15,verticalPlaneImagePts[i].y,verticalPlaneImagePts[i].x+15,verticalPlaneImagePts[i].y);
        ofLine(verticalPlaneImagePts[i].x,verticalPlaneImagePts[i].y-15,verticalPlaneImagePts[i].x,verticalPlaneImagePts[i].y+15);
    }

    ofSetColor(bottomSectionColor);
    ofSetLineWidth(1);
    for(int i = 0; i < 4; i++){
        ofLine(horizontalPlaneImagePts[i].x-15,horizontalPlaneImagePts[i].y,horizontalPlaneImagePts[i].x+15,horizontalPlaneImagePts[i].y);
        ofLine(horizontalPlaneImagePts[i].x,horizontalPlaneImagePts[i].y-15,horizontalPlaneImagePts[i].x,horizontalPlaneImagePts[i].y+15);
    }

    
}

//--------------------------------------------------------------
/**
    Computes the best fit line of the pixel coordinates of the zero crossings in an image

    @param img The (likely zero-crossing) image that has white pixels
    @param roi (optional) A rectangle that defines the region of interest to look over img 
    @returns ofxLine2d a line generated from the zero crossings
*/
ofxLine2d Scan3dApp::computeLineFromZeroCrossings(ofxCvGrayscaleImage img, ofRectangle roi){
    int roi_x0 = roi.x;
    int roi_y0 = roi.y;
    
    int roi_x1 = roi.x+roi.width;
    int roi_y1 = roi.y+roi.height;

    ofxLine2d line;
           
    if(img.countNonZeroInRegion(roi.x,roi.y,roi.width,roi.height) <= 3){
        //ofLogError() << "Not enough non-zero values in image to compute line";
        return line;
    }

    CvPoint * points=(CvPoint*)malloc( (roi_y1-roi_y0) * sizeof(points[0]));

    

    unsigned char* imgPixels = img.getPixels();
    int i;
    // Loop over image (entire or roi)
    for(int r = roi_y0; r < roi_y1; r++){
        for(int c = roi_x0; c < roi_x1; c++){
            i = c+r*img.width;
            if(imgPixels[i] > 0){
                points[r-roi_y0].x = c;
                points[r-roi_y0].y = r;
                break;
            }
        }    
    }

    CvMat point_mat = cvMat( 1, (roi_y1-roi_y0), CV_32SC2, points);
    float result[4];// to store the results

    cvFitLine(&point_mat,CV_DIST_HUBER ,0,0.01,0.01,result);

    
    line.set(result[0],result[1],result[2],result[3]);


    free(points);

    return line;
}


//--------------------------------------------------------------
/**
    

    @param img 
    @param windowSize
    @param x 
    @param y 
    @returns ofPoint
*/
ofPoint Scan3dApp::getNearestCorner(ofxCvGrayscaleImage img, int windowSize, int x, int y){
    Point2f cvPt(x,y);
    vector<Point2f> cvPts;
    cvPts.resize(1,cvPt);
    Size winSize = Size( windowSize, windowSize );
    Size zeroZone = Size( -1, -1 );
    Mat imgMat(img.getCvImage());
    TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
    cornerSubPix(imgMat, cvPts, winSize, zeroZone, criteria);
    ofPoint cornerPt;
    cornerPt.x = cvPts[0].x;
    cornerPt.y = cvPts[0].y;
    return cornerPt;
}

void Scan3dApp::computeExtrinsicMatrix(ofPoint *objectPoints, ofPoint *imagePoints, CvMat* cameraMatrix, CvMat* distCoeffs){
    CvMat* cvObjectPoints = cvCreateMat( 8, 3, CV_32FC1 );
    CvMat* cvImagePoints = cvCreateMat( 8, 2, CV_32FC1 );
    CvMat* tvec = cvCreateMat(3,1, CV_32FC1); 
    CvMat* rvec = cvCreateMat(3,1, CV_32FC1); 
    convertOfPointsToCvMat(objectPoints,3,8, cvObjectPoints);
    convertOfPointsToCvMat(imagePoints,2,8, cvImagePoints);
    cvFindExtrinsicCameraParams2(cvObjectPoints,cvImagePoints,cameraMatrix, distCoeffs,rvec,tvec);
    cvReleaseMat(&cvObjectPoints);
    cvReleaseMat(&cvImagePoints);

    CvMat* rotmat = cvCreateMat( 3,3, CV_32FC1 );


    cvRodrigues2(rvec,rotmat);

    extrinsic_matrix = cvCreateMat( 3, 4, CV_32FC1 );
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            CV_MAT_ELEM(*extrinsic_matrix,float,i,j) = CV_MAT_ELEM(*rotmat,float,i,j);
        }  
    }

    for(int i = 0; i < 3; i++){
        CV_MAT_ELEM(*extrinsic_matrix,float,i,3) = CV_MAT_ELEM(*tvec,float,i,0); 
    }

    CvMat* transRotmat = cvCreateMat( 3,3, CV_32FC1 );
    cvTranspose(rotmat,transRotmat);


    CvMat* camRotVec = cvCreateMat(3,1, CV_32FC1); 
    cvRodrigues2(transRotmat,camRotVec);

    CvMat* negTransRotmat = cvCreateMat( 3,3, CV_32FC1 );
    cvScale(transRotmat,negTransRotmat,-1);

    CvMat* camTransVec = cvCreateMat(3,1, CV_32FC1);  
    cvMatMul(negTransRotmat,tvec,camTransVec);

    camRotMat = cvCreateMat(3,3, CV_32FC1); 
    cvRodrigues2(camRotVec,camRotMat);

    camPos = ofPoint(CV_MAT_ELEM(*camTransVec,float,0,0),CV_MAT_ELEM(*camTransVec,float,1,0),CV_MAT_ELEM(*camTransVec,float,2,0));

    invIntrinsic = cvCreateMat( 3,3, CV_32FC1 );
    cvInv(cameraMatrix,invIntrinsic);


    cvReleaseMat(&tvec);
    cvReleaseMat(&rvec);
    cvReleaseMat(&rotmat);   
}

ofxRay3d Scan3dApp::pixelToRay(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint imagePt){
    CvMat* homoImgPt = cvCreateMat(3,1, CV_32FC1); 

    CV_MAT_ELEM(*homoImgPt,float,0,0) = imagePt.x;
    CV_MAT_ELEM(*homoImgPt,float,1,0) = imagePt.y;
    CV_MAT_ELEM(*homoImgPt,float,2,0) = 1.0;

    CvMat* resPt = cvCreateMat(3,1, CV_32FC1); 

    cvMatMul(invIntrinsic,homoImgPt,resPt);

    CvMat* rotResPt = cvCreateMat(3,1, CV_32FC1); 

    cvMatMul(camRotMat,resPt,rotResPt);

    ofVec3f dir = ofVec3f(CV_MAT_ELEM(*rotResPt,float,0,0),CV_MAT_ELEM(*rotResPt,float,1,0),CV_MAT_ELEM(*rotResPt,float,2,0));
    
    dir.normalize();

    cvReleaseMat(&homoImgPt);
    cvReleaseMat(&resPt);
    cvReleaseMat(&rotResPt);

    return ofxRay3d(camPos,dir);
}

// ofPoint Scan3dApp::rayPlaneIntersection(ofPoint planePt, ofVec3f planeNormal, ofPoint rayOrigin, ofVec3f rayDirection){

//     return ofPoint(0,0,0);
// }

/*

Convert your 2d point into a homogenous point (give it a third coordinate equal to 1) and then multiply by the inverse of your camera intrinsics matrix. For example

cv::Matx31f hom_pt(point_in_image.x, point_in_image.y, 1);
hom_pt = camera_intrinsics_mat.inv()*hom_pt; //put in world coordinates

cv::Point3f origin(0,0,0);
cv::Point3f direction(hom_pt(0),hom_pt(1),hom_pt(2));

//To get a unit vector, direction just needs to be normalized
direction *= 1/cv::norm(direction);
origin and direction now define the ray in world space corresponding to that image point. Note that here the origin is centered on the camera, 
you can use your camera pose to transform to a different origin. Distortion coefficients map from your actual camera to the pinhole camera model 
and should be used at the very beginning to find your actual 2d coordinate. The steps then are:

Undistort 2d coordinate with distortion coefficients
Convert to ray (as shown above)
Move that ray to whatever coordinate system you like.

*/


//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 32: //SPACEBAR
            if(programState ==  SETUP){
                saveSettings();
                programState = CAPTURE;
            }

            break;
        case 49:
            displayState = COLOR;

            break;
        case 50:
            displayState = GRAYSCALE;
            break;
        case 51:
            displayState = MINIMAGE;
            break;
        case 52:
            displayState = MAXIMAGE;
            break;
        case 53:
            displayState = SHADOWTHRESHIMAGE;
            break;
        case 54:
            displayState = DIFF;
            break;
        case 'q':
            std::exit(1);
            break;
        case 'p':
            paused = !paused;
            break;
        case 'o':
            bDrawPointCloud = !bDrawPointCloud;
            break;
        case 'c':
            if(programState == SETUP){
                clearSettings();    
            }
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
    if(programState == SETUP){
        switch(setupSubState){
            case TOP_SECTION:
                topSection.setWidth(x-topSection.x);
                topSection.setHeight(y-topSection.y);
                break;
            case  BOTTOM_SECTION:
                bottomSection.setWidth(x-bottomSection.x);
                bottomSection.setHeight(y-bottomSection.y);

                if(bottomSection.width > topSection.width){
                    bottomSection.width = topSection.width;
                }
                break;
        }
    }
}

//--------------------------------------------------------------
void Scan3dApp::mousePressed(int x, int y, int button){
     if(programState == SETUP){
        switch(setupSubState){
            case TOP_SECTION:
                topSection.setPosition(x,y);
                break;
            case  BOTTOM_SECTION:
                if(x < topSection.x){
                    bottomSection.setPosition(topSection.x,y);
                }
                else if(x > (topSection.x+topSection.width)){
                    bottomSection.setPosition((topSection.x+topSection.width),y);
                }
                else{
                    int tx = x - topSection.x;
                    topSection.x = x;
                    topSection.width -= tx;
                    bottomSection.setPosition(x,y);     
                }
                break;
        }
    }
}

//--------------------------------------------------------------
void Scan3dApp::mouseReleased(int x, int y, int button){
    /*
    if(programState == CAMERA_CALIBRATION){
        switch(camCalSubstate){
            case  TL: 
                camCalFrame++;
                break;
        }
    }
    else */
    if(programState == SETUP){
        switch(setupSubState){
            case TOP_SECTION:
                setupSubState = BOTTOM_SECTION;
                break;
            case  BOTTOM_SECTION:
                topSection.width = bottomSection.width;
                setupSubState = T_BR;
                break;
            case  T_BR:                
                verticalPlaneImagePts[0] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = T_TR;
                break;
            case  T_TR:
                verticalPlaneImagePts[1] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = T_TL;
                break;
            case  T_TL:
                verticalPlaneImagePts[2] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = T_BL;
                break;
            case  T_BL:
                verticalPlaneImagePts[3] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = B_BR;
                break;
            case  B_BR:
                horizontalPlaneImagePts[0] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = B_TR;
                break;
            case  B_TR:
                horizontalPlaneImagePts[1] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = B_TL;
                break;
            case  B_TL:
                horizontalPlaneImagePts[2] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = B_BL;
                break;
            case B_BL:
                horizontalPlaneImagePts[3] = getNearestCorner(grayscaleFrame,10,x,y);
                setupSubState = WAITING;
                break;       
        }
    }
}

//--------------------------------------------------------------
void Scan3dApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void Scan3dApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void Scan3dApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
ofxCvGrayscaleImage Scan3dApp::computeGradientImage(ofxCvGrayscaleImage &input, int direction){
    int sum = 0;
    int sumX = 0;
    int sumY = 0;
    unsigned char* inputPixelData = input.getPixels();
    
    int heightVal = input.getHeight();
    int widthVal = input.getWidth();


    unsigned char* outputPixelData = new unsigned char[widthVal*heightVal];

    for(int yPx = 0; yPx < heightVal; yPx++){
        for(int xPx = 0; xPx < widthVal; xPx++){
            sumX = 0;
            sumY = 0;
            if(yPx == 0){
                sum = 0;
            }
            else if(xPx == 0){
                sum = 0;
            }
            else{
                for(int i = -1; i <= 1; i++){
                    for(int j = -1; j <= 1; j++){
                        switch(direction){
                            case UP:
                                sumY = sumY + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelVertical[j+1][i+1];
                                break;
                            case DOWN:
                                sumY = sumY + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelVertical[1-j][i+1];
                                break;
                            case LEFT:
                                sumX = sumX + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelHorizontal[j+1][1-i];
                                break;
                            case RIGHT:
                                sumX = sumX + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelHorizontal[j+1][i+1];
                                break;
                            case VERTICAL:
                                sumY = sumY + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelVertical[j+1][i+1];
                                break;
                            case HORIZONTAL:
                                sumX = sumX + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelHorizontal[j+1][i+1];
                                break;
                            case BOTH:
                                sumY = sumY + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelVertical[j+1][i+1];
                                sumX = sumX + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*sobelHorizontal[j+1][i+1];
                                break;
                            case LOG:
                                sumY = sumY + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*laplacianOfGaussian[j+1][i+1];
                                sumX = sumX + (int)inputPixelData[(yPx+j)*widthVal+xPx+i]*laplacianOfGaussian[j+1][i+1];
                                break;
                        }
                    }
                }
                switch(direction){
                    case UP:
                        if(sumY < 0){
                            sumY = 0;
                        }
                        sum = sumY;
                        break;
                    case DOWN:
                        if(sumY < 0){
                            sumY = 0;
                        }
                        sum = sumY;
                        break;
                    case LEFT:
                        if(sumX < 0){
                            sumX = 0;
                        }
                        sum = sumX;
                        break;
                    case RIGHT:
                        if(sumX < 0){
                            sumX = 0;
                        }
                        sum = sumX;
                        break;
                    case VERTICAL:
                        sum = abs(sumY);
                        break;
                    case HORIZONTAL:
                        sum = abs(sumX);
                        break;
                    case BOTH:
                        sum = abs(sumX)+abs(sumY);
                        break;
                    case LOG:
                        sum = abs(sumX)+abs(sumY);
                        break;
                }
            }
            if(sum > 255){
                sum = 255;
            }
            if(sum < 0){
                sum = 0;
            }
            outputPixelData[yPx*widthVal+xPx] = (unsigned char)(sum);
        }
    }
    ofxCvGrayscaleImage out;
    out.allocate(widthVal,heightVal);
    out.setFromPixels(outputPixelData,widthVal,heightVal);
    return out;
}

ofxLine3d Scan3dApp::projectLineOntoPlane(ofxLine2d line, ofxPlane plane, const CvMat* intrinsicMat, const CvMat* extrinsicMat){
    ofPoint pixPt0 = line.pt;
    ofPoint pixPt1 = line.pt+line.dir;

    ofxRay3d ray0 = pixelToRay(intrinsicMat,extrinsicMat,pixPt0);
    ofxRay3d ray1 = pixelToRay(intrinsicMat,extrinsicMat,pixPt1);

    ofPoint worldPt0 = ray0.intersect(plane);
    ofPoint worldPt1 = ray1.intersect(plane);

    ofVec3f dir = worldPt1 - worldPt0;

    return ofxLine3d(dir.x,dir.y,dir.z,worldPt0.x,worldPt0.y,worldPt0.z);
}

float Scan3dApp::getFrameFromColor(ofColor color){
    float hue = color.getHue();
    float computedFrameIndex = ofMap(hue,0.0,255.0,0,numFrames);
    return computedFrameIndex;
}


bool Scan3dApp::isPlaneAtFrameIndex(float fi){
    int discreteFrameIndex = (int)fi;
    float interpolateFrameValue = fi - discreteFrameIndex;
    if(discreteFrameIndex > 0){
        ofxPlane plane0 = planes[discreteFrameIndex-1];
        ofxPlane plane1 = planes[discreteFrameIndex];
        return plane0.isInit() && plane1.isInit();

    }
    else{
        return planes[0].isInit();
    }
}

ofxPlane Scan3dApp::getPlaneFromFrameIndex(float fi){
    int discreteFrameIndex = (int)fi;
    float interpolateFrameValue = fi - discreteFrameIndex;
    if(discreteFrameIndex > 0){
        ofxPlane plane0 = planes[discreteFrameIndex];
        ofxPlane plane1 = planes[discreteFrameIndex+1];
        return plane0.interpolate(plane1,interpolateFrameValue);

    }
    else{
        return planes[0];
    }
}

void Scan3dApp::drawPointCloud() {
    ofBackground(0);
    ofMesh mesh;
    int step = 1;
    ofxPlane plane;
    ofVec3f vec;

    if(programState == POINTS3D){
        switch(points3dSubstate){
            case POINTS3D_WAITING:
                mesh.setMode(OF_PRIMITIVE_POINTS);
                
                for(int i =0; i < points.size(); i += step){  
                    mesh.addColor(colors[i]);
                    mesh.addVertex((ofVec3f)points[i]);
                }
                glPointSize(3);
                ofPushMatrix();
                // the projected points are 'upside down' and 'backwards'
                //ofScale(1, -1, -1);
                //ofTranslate(camPos.x,camPos.y,camPos.z); // center the points a bit

                glEnable(GL_DEPTH_TEST);
                mesh.drawVertices();
                glDisable(GL_DEPTH_TEST);
                ofPopMatrix();

                glPointSize(6);
                ofPushMatrix();
                // the projected points are 'upside down' and 'backwards'
                //ofScale(1, -1, -1);
                //ofTranslate(camPos.x,camPos.y,camPos.z); // center the points a bit

                glEnable(GL_DEPTH_TEST);
                planePts.drawVertices();
                glDisable(GL_DEPTH_TEST);
                ofPopMatrix();
                break;
        }
    }
    if(programState == PROCESSING){
        

        glPointSize(5);
        ofPushMatrix();
        // the projected points are 'upside down' and 'backwards'
        //ofScale(1, -1, -1);
        //ofTranslate(camPos.x,camPos.y,camPos.z); // center the points a bit

        glEnable(GL_DEPTH_TEST);
        planePts.drawVertices();
        glDisable(GL_DEPTH_TEST);
        ofPopMatrix();

    }
    
}
