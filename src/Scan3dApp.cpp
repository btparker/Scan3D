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
    ofSetWindowTitle("3D SCAN");
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


    printf("[width,height] = [%d,%d]\n",width,height);

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
    noiseImg.allocate(width,height);
    noiseImg.set(0);
    temporalImg.allocate(width,height);
    rowMappingColorImg.allocate(width,height);
    rowMappingColorImg.set(0);
    colMappingColorImg.allocate(width,height);
    colMappingColorImg.set(0);
    diffFrame.allocate(width,height);
    invDiffFrame.allocate(width,height);

    rowMapping16Img = cvCreateImage(cvSize(width,height),IPL_DEPTH_16U,1);
    colMapping16Img = cvCreateImage(cvSize(width,height),IPL_DEPTH_16U,1);


    codeImg.allocate(projWidth,projHeight);
    

    
    shadowThreshImg.allocate(width,height);

    ofSetWindowShape(width,height+messageBarHeight);

    numFrames = dir.numFiles();
    
    
    frames.resize(numFrames,bufferOfxCvGrayscaleImage);

    numColMapFrames = log2(projWidth);
    colFrames.resize(numColMapFrames,bufferOfxCvGrayscaleImage);
    invColFrames.resize(numColMapFrames,bufferOfxCvGrayscaleImage);

    numRowMapFrames = log2(projHeight);
    rowFrames.resize(numRowMapFrames,bufferOfxCvGrayscaleImage);
    invRowFrames.resize(numRowMapFrames,bufferOfxCvGrayscaleImage);

    codeFrames.resize(numFrames,codeImg);

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
    colIndex = 0;
    rowIndex = 0;

    // Camera Calibration Variables (Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
    camCalFrame = 0;
    camNumBoards = camCalDir.numFiles();
    camSuccesses = 0;
    camBoardPatternSize = camBoardXCount*camBoardYCount;
    camBoardNumInternalCornersCV = cvSize( camBoardXCount, camBoardYCount);

    cam_image_points = cvCreateMat(camNumBoards*camBoardPatternSize,2,CV_32FC1);
    cam_object_points = cvCreateMat(camNumBoards*camBoardPatternSize,3,CV_32FC1);
    cam_point_counts = cvCreateMat(camNumBoards,1,CV_32SC1);
    cam_intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
    cam_distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
    cam_corners = new CvPoint2D32f[ camBoardPatternSize ];

    // Projector Calibration Variables (Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
    projCalFrame = 0;
    projNumBoards = projCalDir.numFiles();
    projSuccesses = 0;
    projBoardPatternSize = projBoardXCount*projBoardYCount;
    projBoardNumInternalCornersCV = cvSize( projBoardXCount, projBoardYCount);

    proj_image_points = cvCreateMat(projNumBoards*projBoardPatternSize,2,CV_32FC1);
    proj_object_points = cvCreateMat(projNumBoards*projBoardPatternSize,3,CV_32FC1);
    proj_point_counts = cvCreateMat(projNumBoards,1,CV_32SC1);
    proj_intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
    proj_distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
    proj_corners = new CvPoint2D32f[ projBoardPatternSize ];


    // Scene planes


    ofVec3f tN = ofVec3f(0,1,1);
    tN.normalize();
    negYnegZPlane = ofxPlane(ofPoint(0,50,50),tN);



    paused = false;

    points3dSubstate = POINTS3D_WAITING;

    bufferOfImage.loadImage(projCalDir.getPath(projCalDir.numFiles()-1));
    colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);

    vertPlane = ofxPlane(ofPoint(0,0,0),ofVec3f(0,0,-1));

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
                    projWidth = settings.getValue("projWidth",1024);
                    projHeight = settings.getValue("projHeight",1024);
                    string projTypeS = settings.getValue("projType","BIN");
                    projType = (projTypeS == "BIN") ? BIN : GRAY;

                settings.popTag(); //pop input
                settings.pushTag("calibration");
                    settings.pushTag("cam");
                        programState = CAMERA_CALIBRATION;
                        camCalDir = ofDirectory(settings.getValue("frames",""));
                        camCalDir.listDir();
                        camCalDir.sort();
                        camIntrinsicFilename = ofToDataPath("cam_intrinsic.xml");
                        camDistortionFilename = ofToDataPath("cam_distortion.xml");
                        camBoardXCount = settings.getValue("camBoardXCount",8);
                        camBoardYCount = settings.getValue("camBoardYCount",6);
                        camBoardSquareSize = settings.getValue("camBoardSquareSize",30.0); //in mm 
                        if(settings.tagExists("camIntrinsicFile") && settings.tagExists("camDistortionFile")){
                            camIntrinsicFilename = settings.getValue("camIntrinsicFile","cam_intrinsic.xml");
                            camDistortionFilename = settings.getValue("camDistortionFile","cam_distortion.xml");
                            camCalSubstate = CAM_CAL_LOADING;
                        }
                        else{
                            

                            cout << "   Using camera calibration sequence folder: " << camCalDir.path() << endl;
                            camCalSubstate = CAM_CAL_PROCESSING;  
                        }
                    settings.popTag();
                    settings.pushTag("proj");
                        projCalDir = ofDirectory(settings.getValue("frames",""));
                        projCalDir.listDir();
                        projCalDir.sort();
                        projIntrinsicFilename = ofToDataPath("proj_intrinsic.xml");
                        projDistortionFilename = ofToDataPath("proj_distortion.xml");
                        projBoardXCount = settings.getValue("projBoardXCount",7);
                        projBoardYCount = settings.getValue("projBoardYCount",7);
                        projBoardSquareSize = settings.getValue("projBoardSquareSize",30.0); //in mm 


                        if(settings.tagExists("projIntrinsicFile") && settings.tagExists("projDistortionFile")){
                            projIntrinsicFilename = settings.getValue("projIntrinsicFile","proj_intrinsic.xml");
                            projDistortionFilename = settings.getValue("projDistortionFile","proj_distortion.xml");
                            projCalSubstate = PROJ_CAL_LOADING;
                        }
                        else{
                            

                            cout << "   Using projector calibration sequence folder: " << projCalDir.path() << endl;
                            projCalSubstate = PROJ_CAL_PROCESSING;  
                        }
                        settings.pushTag("plane");
                            for(int i = 0; i < 4; i++){
                                settings.pushTag("marker"+ofToString(i));
                                    settings.pushTag("imagePt");
                                        projPlaneImagePts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0));
                                        cout << "      image:[" << projPlaneImagePts[i].x << ", " << projPlaneImagePts[i].y << "]";
                                    settings.popTag(); // pop pts
                                    settings.pushTag("objectPt");
                                        projPlaneObjectPts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0),settings.getValue("z",0.0));
                                        cout << "      object:[" << projPlaneObjectPts[i].x << ", " << projPlaneObjectPts[i].y << ", " << projPlaneObjectPts[i].z << "]" << endl;
                                    settings.popTag(); // pop pts
                                settings.popTag(); // pop markers
                            }
                        settings.popTag();
                    settings.popTag();
                settings.popTag();
                settings.pushTag("misc");
                    minThreshold = settings.getValue("minThreshold", 0);
                    maxThreshold = settings.getValue("maxThreshold", 0);
                settings.popTag();// pop misc
            settings.popTag(); // pop user
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
                settings.setValue("projWidth",projWidth);
                settings.setValue("projHeight",projHeight);
                settings.setValue("projType",(projType == BIN) ? "BIN" : "GRAY");
            settings.popTag(); //pop input
            settings.addTag("calibration");
            settings.pushTag("calibration");
                settings.addTag("cam");
                settings.pushTag("cam");
                    settings.setValue("frames",camCalDir.path());
                    settings.setValue("camBoardXCount",camBoardXCount);
                    settings.setValue("camBoardYCount",camBoardYCount);
                    settings.setValue("camBoardSquareSize",camBoardSquareSize);
                    settings.setValue("camIntrinsicFile",camIntrinsicFilename);
                    settings.setValue("camDistortionFile",camDistortionFilename);
                settings.popTag(); //pop cam
                settings.addTag("proj");
                settings.pushTag("proj");
                    settings.setValue("frames",projCalDir.path());
                    settings.setValue("projBoardXCount",projBoardXCount);
                    settings.setValue("projBoardYCount",projBoardYCount);
                    settings.setValue("projBoardSquareSize",projBoardSquareSize);
                    settings.setValue("projIntrinsicFile",projIntrinsicFilename);
                    settings.setValue("projDistortionFile",projDistortionFilename);
                    settings.addTag("plane");
                    settings.pushTag("plane");
                        cout << "   Setting projector plane points as [BR,TR,TL,BL]:"<< endl;
                        for(int i = 0; i < 4; i++){
                            settings.addTag("marker"+ofToString(i));
                            settings.pushTag("marker"+ofToString(i));
                                settings.addTag("imagePt");
                                settings.pushTag("imagePt");
                                    settings.setValue("x",projPlaneImagePts[i].x); settings.setValue("y",projPlaneImagePts[i].y);
                                    cout << "      [" << projPlaneImagePts[i].x << ", " << projPlaneImagePts[i].y << "]" << endl;
                                settings.popTag(); // pop imagePt
                                settings.addTag("objectPt");
                                settings.pushTag("objectPt");
                                    settings.setValue("x",projPlaneObjectPts[i].x); settings.setValue("y",projPlaneObjectPts[i].y); settings.setValue("z",projPlaneObjectPts[i].z);
                                settings.popTag(); // pop objectPt
                            settings.popTag(); // pop markers
                        }
                    settings.popTag(); // pop plane
                settings.popTag(); //pop proj
            settings.popTag(); //pop calibration
            settings.addTag("misc");
            settings.pushTag("misc");
                settings.setValue("minThreshold", minThreshold);
                settings.setValue("maxThreshold", maxThreshold);
            settings.popTag();// pop misc
        settings.popTag(); //pop user
        settings.addTag("scene");
    settings.popTag(); // pop settings
    settings.saveFile("settings.xml");
    cout << "** Done Saving Settings **" << endl;
}

//--------------------------------------------------------------
void Scan3dApp::clearProjectorSettings(){
   for(int i = 0; i < 4; i++){
        projPlaneImagePts[i].x = 0;
        projPlaneImagePts[i].y = 0;
    }
    programState = PROJECTOR_CALIBRATION;
    projCalSubstate = BR;
}

//--------------------------------------------------------------
void Scan3dApp::update(){
    if(!paused){
        
        switch(programState){
            case CAMERA_CALIBRATION:
                camCalUpdate();
                break;
            case PROJECTOR_CALIBRATION:
                projCalUpdate();
                break;
            case CAPTURE:
            {
                captureUpdate();
                break;
            }
            case PROCESSING:
                processingUpdate();
                break;
            case POINTS3D:
            {    
                points3dUpdate();
                break;
            }

        }
    }




}

void Scan3dApp::camCalUpdate(){
    ofFile camIntrinsicFile;
    ofFile camDistortionFile;
    messageBarText = "CAMERA CALIBRATION";
    switch(camCalSubstate){
        case CAM_CAL_PROCESSING:
            messageBarSubText = "Processing camera calibration frames";
            if(camCalFrame < camNumBoards){
                bufferOfImage.loadImage(camCalDir.getPath(camCalFrame));
                colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
                grayscaleFrame = colorFrame;

                //(Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
                IplImage* image = colorFrame.getCvImage();
                IplImage* gray_image = grayscaleFrame.getCvImage();

                 
                //Find chessboard cam_corners:
                int found = cvFindChessboardCorners(
                    image, 
                    camBoardNumInternalCornersCV, 
                    cam_corners, 
                    &cam_corner_count,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                );

                
               
                //Get Subpixel accuracy on those cam_corners

                cvFindCornerSubPix(
                    gray_image, 
                    cam_corners, 
                    cam_corner_count,
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
                    camBoardNumInternalCornersCV, 
                    cam_corners,
                    cam_corner_count, 
                    found
                );
                
               
                // If we got a good board, add it to our data
                if( cam_corner_count == camBoardPatternSize ) {
                    int step = camSuccesses*camBoardPatternSize;
                    for( int i=step, j=0; j<camBoardPatternSize; ++i,++j ) {
                        CV_MAT_ELEM(*cam_image_points, float,i,0) = cam_corners[j].x;
                        CV_MAT_ELEM(*cam_image_points, float,i,1) = cam_corners[j].y;
                        CV_MAT_ELEM(*cam_object_points,float,i,0) = j/camBoardXCount;
                        CV_MAT_ELEM(*cam_object_points,float,i,1) = j%camBoardXCount;
                        CV_MAT_ELEM(*cam_object_points,float,i,2) = 0.0f;
                    }
                    CV_MAT_ELEM(*cam_point_counts, int,camSuccesses,0) = camBoardPatternSize;
                    camSuccesses++;
                }

                

                colorFrame = image;
                camCalFrame++;
            }
            else{
                //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
                CvMat* cam_object_points2 = cvCreateMat(camSuccesses*camBoardPatternSize,3,CV_32FC1);
                CvMat* cam_image_points2 = cvCreateMat(camSuccesses*camBoardPatternSize,2,CV_32FC1);
                CvMat* cam_point_counts2 = cvCreateMat(camSuccesses,1,CV_32SC1);
                //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
               
                for(int i = 0; i<camSuccesses*camBoardPatternSize; ++i) {
                    CV_MAT_ELEM( *cam_image_points2, float, i, 0) = CV_MAT_ELEM( *cam_image_points, float, i, 0);
                    CV_MAT_ELEM( *cam_image_points2, float,i,1) = CV_MAT_ELEM( *cam_image_points, float, i, 1);
                    CV_MAT_ELEM(*cam_object_points2, float, i, 0) = CV_MAT_ELEM( *cam_object_points, float, i, 0) ;
                    CV_MAT_ELEM( *cam_object_points2, float, i, 1) = CV_MAT_ELEM( *cam_object_points, float, i, 1) ;
                    CV_MAT_ELEM( *cam_object_points2, float, i, 2) = CV_MAT_ELEM( *cam_object_points, float, i, 2) ;
                }
                for(int i=0; i<camSuccesses; ++i){ //These are all the same number
                    CV_MAT_ELEM( *cam_point_counts2, int, i, 0) = CV_MAT_ELEM( *cam_point_counts, int, i, 0);
                }
                cvReleaseMat(&cam_object_points);
                cvReleaseMat(&cam_image_points);
                cvReleaseMat(&cam_point_counts);
                // At this point we have all of the chessboard cam_corners we need.
                // Initialize the intrinsic matrix such that the two focal
                // lengths have a ratio of 1.0
                //
                CV_MAT_ELEM( *cam_intrinsic_matrix, float, 0, 0 ) = 1.0f;
                CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1, 1 ) = 1.0f;
                //CV_MAT_ELEM( *cam_intrinsic_matrix, float, 2, 2) = 1.0f;


                
                //CALIBRATE THE CAMERA!
                cvCalibrateCamera2(
                    cam_object_points2, 
                    cam_image_points2,
                    cam_point_counts2, 
                    cvSize(width,height),
                    cam_intrinsic_matrix, 
                    cam_distortion_coeffs,
                    NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
                );

                printf("\n");
                printf("Camera Intrinsic Matrix: \n");
                printf("\n");
                for(int r = 0; r < 3; r++){
                    cout << "[";
                    for(int c = 0; c < 3; c++){
                        CvScalar scal = cvGet2D(cam_intrinsic_matrix,r,c); 
                        printf( "%f,\t", scal.val[0]); 
                    }
                    printf("]\n");
                }
                printf("\n");
                
                cam_focal_length = ofVec2f(CV_MAT_ELEM( *cam_intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1,1));

                cout << "Camera Focal Length: [" << cam_focal_length << "]\n" << endl;

                cam_principal_point = ofVec2f(CV_MAT_ELEM( *cam_intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1,2));

                cout << "Camera Principal Point: [" << cam_principal_point << "]\n" << endl;

                cam_skew_coeff = CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1, 0 );

                cout << "Camera Skew Coefficient (alpha_c): " << cam_skew_coeff  << "\n"<< endl;
                CV_MAT_ELEM( *cam_distortion_coeffs, float, 4,0) = 0.0;
                printf("Camera Distortion Coefficients (kc):");
                printf("\n");
                for(int r = 0; r < 5; r++){
                        CvScalar scal = cvGet2D(cam_distortion_coeffs,r,0); 
                        printf( "[%f]\n", scal.val[0]); 
                }
                printf("\n");
                
                // SAVE THE INTRINSICS AND DISTORTIONS
                cvSave(camIntrinsicFilename.c_str(),cam_intrinsic_matrix);
                cvSave(camDistortionFilename.c_str(),cam_distortion_coeffs);

                cammapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                cammapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                cvInitUndistortMap(
                    cam_intrinsic_matrix,
                    cam_distortion_coeffs,
                    cammapx,
                    cammapy
                );

                
                
                programState = PROJECTOR_CALIBRATION;

                
                cvReleaseMat(&cam_image_points);
            }
            break;

        case CAM_CAL_LOADING:
            messageBarSubText = "Loading camera calibration files";
            
            camIntrinsicFile.open(camIntrinsicFilename, ofFile::ReadWrite, false);
            if(!camIntrinsicFile.exists()){
                camCalSubstate = CAM_CAL_PROCESSING;
                update();
            }

            camDistortionFile.open(camDistortionFilename, ofFile::ReadWrite, false);
            if(!camDistortionFile.exists()){
                camCalSubstate = CAM_CAL_PROCESSING;
                update();
            }
            cam_intrinsic_matrix = (CvMat*)cvLoad(camIntrinsicFilename.c_str());
            cam_distortion_coeffs = (CvMat*)cvLoad(camDistortionFilename.c_str());
            
            printf("\n");
            printf("Camera Intrinsic Matrix: \n");
            printf("\n");
            for(int r = 0; r < 3; r++){
                cout << "[";
                for(int c = 0; c < 3; c++){
                    CvScalar scal = cvGet2D(cam_intrinsic_matrix,r,c); 
                    printf( "%f,\t", scal.val[0]); 
                }
                printf("]\n");
            }
            printf("\n");
            
            cam_focal_length = ofVec2f(CV_MAT_ELEM( *cam_intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1,1));

            cout << "Camera Focal Length: [" << cam_focal_length << "]\n" << endl;

            cam_principal_point = ofVec2f(CV_MAT_ELEM( *cam_intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1,2));

            cout << "Camera Principal Point: [" << cam_principal_point << "]\n\n" << endl;

            cam_skew_coeff = CV_MAT_ELEM( *cam_intrinsic_matrix, float, 1, 0 );

            cout << "Camera Skew Coefficient (alpha_c): " << cam_skew_coeff  << "\n\n"<< endl;
            CV_MAT_ELEM( *cam_distortion_coeffs, float, 4,0) = 0.0;
            printf("Camera Distortion Coefficients (kc):");
            printf("\n");
            for(int r = 0; r < 5; r++){
                    CvScalar scal = cvGet2D(cam_distortion_coeffs,r,0); 
                    printf( "[%f]\n", scal.val[0]); 
            }
            printf("\n");

            cammapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            cammapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            cvInitUndistortMap(
                cam_intrinsic_matrix,
                cam_distortion_coeffs,
                cammapx,
                cammapy
            );

            programState = PROJECTOR_CALIBRATION;
            break;
    }
    
    
}

void Scan3dApp::projCalUpdate(){
    ofFile projIntrinsicFile;
    ofFile projDistortionFile;
    messageBarText = "PROJECTOR CALIBRATION";
    
    switch(projCalSubstate){
        case PROJ_CAL_PROCESSING:
            messageBarSubText = "Processing projector calibration frames";
            if(projCalFrame < projNumBoards){
                bufferOfImage.loadImage(projCalDir.getPath(projCalFrame));
                colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
                grayscaleFrame = colorFrame;

                //(Adapted for use from Learning OpenCV Computer Vision with the OpenCV Library)
                IplImage* image = colorFrame.getCvImage();
                IplImage* gray_image = grayscaleFrame.getCvImage();

                 
                //Find chessboard proj_corners:
                int found = cvFindChessboardCorners(
                    image, 
                    projBoardNumInternalCornersCV, 
                    proj_corners, 
                    &proj_corner_count,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                );

                
               
                //Get Subpixel accuracy on those proj_corners

                cvFindCornerSubPix(
                    gray_image, 
                    proj_corners, 
                    proj_corner_count,
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
                    projBoardNumInternalCornersCV, 
                    proj_corners,
                    proj_corner_count, 
                    found
                );
                
               
                // If we got a good board, add it to our data
                if( proj_corner_count == projBoardPatternSize ) {
                    int step = projSuccesses*projBoardPatternSize;
                    for( int i=step, j=0; j<projBoardPatternSize; ++i,++j ) {
                        CV_MAT_ELEM(*proj_image_points, float,i,0) = proj_corners[j].x;
                        CV_MAT_ELEM(*proj_image_points, float,i,1) = proj_corners[j].y;
                        CV_MAT_ELEM(*proj_object_points,float,i,0) = j/projBoardXCount;
                        CV_MAT_ELEM(*proj_object_points,float,i,1) = j%projBoardXCount;
                        CV_MAT_ELEM(*proj_object_points,float,i,2) = 0.0f;
                    }
                    CV_MAT_ELEM(*proj_point_counts, int,projSuccesses,0) = projBoardPatternSize;
                    projSuccesses++;
                }

                

                colorFrame = image;
                projCalFrame++;
            }
            else{
                //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
                CvMat* proj_object_points2 = cvCreateMat(projSuccesses*projBoardPatternSize,3,CV_32FC1);
                CvMat* proj_image_points2 = cvCreateMat(projSuccesses*projBoardPatternSize,2,CV_32FC1);
                CvMat* proj_point_counts2 = cvCreateMat(projSuccesses,1,CV_32SC1);
                //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
               
                for(int i = 0; i<projSuccesses*projBoardPatternSize; ++i) {
                    CV_MAT_ELEM( *proj_image_points2, float, i, 0) = CV_MAT_ELEM( *proj_image_points, float, i, 0);
                    CV_MAT_ELEM( *proj_image_points2, float,i,1) = CV_MAT_ELEM( *proj_image_points, float, i, 1);
                    CV_MAT_ELEM(*proj_object_points2, float, i, 0) = CV_MAT_ELEM( *proj_object_points, float, i, 0) ;
                    CV_MAT_ELEM( *proj_object_points2, float, i, 1) = CV_MAT_ELEM( *proj_object_points, float, i, 1) ;
                    CV_MAT_ELEM( *proj_object_points2, float, i, 2) = CV_MAT_ELEM( *proj_object_points, float, i, 2) ;
                }
                for(int i=0; i<projSuccesses; ++i){ //These are all the same number
                    CV_MAT_ELEM( *proj_point_counts2, int, i, 0) = CV_MAT_ELEM( *proj_point_counts, int, i, 0);
                }
                cvReleaseMat(&proj_object_points);
                cvReleaseMat(&proj_image_points);
                cvReleaseMat(&proj_point_counts);
                // At this point we have all of the chessboard proj_corners we need.
                // Initialize the intrinsic matrix such that the two focal
                // lengths have a ratio of 1.0
                //
                CV_MAT_ELEM( *proj_intrinsic_matrix, float, 0, 0 ) = 1.0f;
                CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1, 1 ) = 1.0f;
                //CV_MAT_ELEM( *proj_intrinsic_matrix, float, 2, 2) = 1.0f;


                
                //CALIBRATE THE CAMERA!
                cvCalibrateCamera2(
                    proj_object_points2, 
                    proj_image_points2,
                    proj_point_counts2, 
                    cvSize(width,height),
                    proj_intrinsic_matrix, 
                    proj_distortion_coeffs,
                    NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
                );

                printf("\n");
                printf("Projector Intrinsic Matrix: \n");
                printf("\n");
                for(int r = 0; r < 3; r++){
                    cout << "[";
                    for(int c = 0; c < 3; c++){
                        CvScalar scal = cvGet2D(proj_intrinsic_matrix,r,c); 
                        printf( "%f,\t", scal.val[0]); 
                    }
                    printf("]\n");
                }
                printf("\n");
                
                proj_focal_length = ofVec2f(CV_MAT_ELEM( *proj_intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1,1));

                cout << "Projector Focal Length: [" << proj_focal_length << "]\n" << endl;

                proj_principal_point = ofVec2f(CV_MAT_ELEM( *proj_intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1,2));

                cout << "Projector Principal Point: [" << proj_principal_point << "]\n\n" << endl;

                proj_skew_coeff = CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1, 0 );

                cout << "Projector Skew Coefficient (alpha_c): " << proj_skew_coeff  << "\n\n"<< endl;
                CV_MAT_ELEM( *proj_distortion_coeffs, float, 4,0) = 0.0;
                printf("Projector Distortion Coefficients (kc):");
                printf("\n");
                for(int r = 0; r < 5; r++){
                        CvScalar scal = cvGet2D(proj_distortion_coeffs,r,0); 
                        printf( "[%f]\n", scal.val[0]); 
                }
                printf("\n");
                
                // SAVE THE INTRINSICS AND DISTORTIONS
                cvSave(projIntrinsicFilename.c_str(),proj_intrinsic_matrix);
                cvSave(projDistortionFilename.c_str(),proj_distortion_coeffs);

                projmapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                projmapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
                cvInitUndistortMap(
                    proj_intrinsic_matrix,
                    proj_distortion_coeffs,
                    projmapx,
                    projmapy
                );

                
                
                projCalSubstate = PROJ_CAL_WAITING;

                
                cvReleaseMat(&proj_image_points);
            }
            break;

        case PROJ_CAL_LOADING:
            messageBarSubText = "Loading projector calibration file";
            
            projIntrinsicFile.open(projIntrinsicFilename, ofFile::ReadWrite, false);
            if(!projIntrinsicFile.exists()){
                projCalSubstate = PROJ_CAL_PROCESSING;
                update();
            }

            projDistortionFile.open(projDistortionFilename, ofFile::ReadWrite, false);
            if(!projDistortionFile.exists()){
                projCalSubstate = PROJ_CAL_PROCESSING;
                update();
            }
            proj_intrinsic_matrix = (CvMat*)cvLoad(projIntrinsicFilename.c_str());
            proj_distortion_coeffs = (CvMat*)cvLoad(projDistortionFilename.c_str());
            
            printf("\n");
            printf("Projector Intrinsic Matrix: \n");
            printf("\n");
            for(int r = 0; r < 3; r++){
                cout << "[";
                for(int c = 0; c < 3; c++){
                    CvScalar scal = cvGet2D(proj_intrinsic_matrix,r,c); 
                    printf( "%f,\t", scal.val[0]); 
                }
                printf("]\n");
            }
            printf("\n");
            
            proj_focal_length = ofVec2f(CV_MAT_ELEM( *proj_intrinsic_matrix, float, 0, 0 ),CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1,1));

            cout << "Projector Focal Length: [" << proj_focal_length << "]\n" << endl;

            proj_principal_point = ofVec2f(CV_MAT_ELEM( *proj_intrinsic_matrix, float, 0,2 ),CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1,2));

            cout << "Projector Principal Point: [" << proj_principal_point << "]\n" << endl;

            proj_skew_coeff = CV_MAT_ELEM( *proj_intrinsic_matrix, float, 1, 0 );

            cout << "Projector Skew Coefficient (alpha_c): " << proj_skew_coeff  << "\n"<< endl;
            CV_MAT_ELEM( *proj_distortion_coeffs, float, 4,0) = 0.0;
            printf("Projector Distortion Coefficients (kc):");
            printf("\n");
            for(int r = 0; r < 5; r++){
                    CvScalar scal = cvGet2D(proj_distortion_coeffs,r,0); 
                    printf( "[%f]\n", scal.val[0]); 
            }
            printf("\n");

            projmapx = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            projmapy = cvCreateImage( cvGetSize(colorFrame.getCvImage()), IPL_DEPTH_32F, 1 );
            cvInitUndistortMap(
                proj_intrinsic_matrix,
                proj_distortion_coeffs,
                projmapx,
                projmapy
            );

            projCalSubstate = PROJ_CAL_WAITING;
            break;

        case PROJ_CAL_WAITING:
            messageBarSubText = "Press spacebar to use point settings and continue (or press c to clear marker points and start over)";
            break;
        case  TL:
            messageBarSubText = "Click to select the top left marker on the projector plane (or press c to clear settings and start over)";
            break;
        case  TR:
            messageBarSubText = "Click to select the top right marker on the projector plane (or press c to clear settings and start over)";
            break;
        case  BR:
            messageBarSubText = "Click to select the bottom right marker on the projector plane (or press c to clear settings and start over)";
            break;
        case  BL:
            messageBarSubText = "Click to select the bottom left marker on the projector plane (or press c to clear settings and start over)";
            break;
    }
    
}



ofPoint Scan3dApp::pt3DToPixel(const CvMat* intrinsicMat, const CvMat* extrinsicMat, const CvMat* distCoeffs, ofPoint pt3D){
    CvMat* A = cvCreateMat( 3, 4, CV_32FC1 );
    cvMatMul(intrinsicMat,extrinsicMat,A); //combining into one matrix
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

    cout << "Frame Index " << frameIndex << endl;
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
                colorFrame.remap(cammapx,cammapy);
            }


            framesDone = vid.getIsMovieDone();
           
            break;
        case IMAGE_SEQUENCE:
            if(frameIndex < numFrames){
                bufferOfImage.loadImage(dir.getPath(frameIndex));
                colorFrame.setFromPixels(bufferOfImage.getPixels(),width,height);
                colorFrame.flagImageChanged();
                colorFrame.updateTexture();
                colorFrame.remap(cammapx,cammapy);
            }

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
        frameIndex = 0; // skipping the all light and all dark frames
        rowIndex = 0;
        colIndex = 0;
    }
    else{
        grayscaleFrame = colorFrame;
        int power = (frameIndex <= 2*log2(projWidth)+1) ? frameIndex/2 : ((frameIndex/2)-log2(projWidth));
        bool inverse = (frameIndex%2 == 1);
        int type = (frameIndex <= 2*log2(projWidth)+1) ? HORIZONTAL : VERTICAL;

        if(projType == BIN){
            codeImg = computeBinCodeImage(projWidth,projHeight,power,inverse,type);
        }
        else if(projType == GRAY){
            codeImg = computeGrayCodeImage(projWidth,projHeight,power,inverse,type);
        }
        
        codeFrames[frameIndex] = codeImg;

        if(frameIndex == 0){
            maxColorImg = colorFrame;
            maxImg = grayscaleFrame;
        }
        else if(frameIndex == 1){
            minImg = grayscaleFrame;
            unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();
            unsigned char* maxImgPixels = maxImg.getPixels();
            unsigned char* minImgPixels = minImg.getPixels();

            for(int r = 0; r < height; r++){
                for(int c = 0; c < width; c++){
                    shadowThreshImgPixels[c+r*width] = (maxImgPixels[c+r*width]+minImgPixels[c+r*width])/2.0;
                }
            }

            shadowThreshImg.setFromPixels(shadowThreshImgPixels,width,height);

            noiseImg = shadowThreshImg;
            noiseImg -= minImg;
            noiseImg.blurGaussian();
            noiseImg -= minThreshold;
            noiseImg.threshold(0,false);
            
        }
        else{
            grayscaleFrame *= noiseImg;
            if(colIndex < numColMapFrames){
                if(frameIndex%2 == 0){
                    colFrames[colIndex] = grayscaleFrame;
                }
                else if(frameIndex%2 == 1){
                    invColFrames[colIndex] = grayscaleFrame;
                    colIndex++;
                }
            }
            else if(rowIndex < numRowMapFrames){
                if(frameIndex%2 == 0){
                    rowFrames[rowIndex] = grayscaleFrame;
                }
                else if(frameIndex%2 == 1){
                    invRowFrames[rowIndex] = grayscaleFrame;
                    rowIndex++;
                }
            }
        }

        frames[frameIndex] = grayscaleFrame;    
        frameIndex++;  
    }

    //Uncomment to save out color frames
    // bufferOfImage.setFromPixels(colorFrame.getPixelsRef());
    // filename = "output/grayscaleFrames/gsframe";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);


        
    
}

void Scan3dApp::processingUpdate(){

    messageBarText = "PROCESSING";
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
        case CTEMPORAL:
            messageBarText += "  (CTEMPORAL)";
            break;
        case RTEMPORAL:
            messageBarText += "  (RTEMPORAL)";
            break;
    }

    bool hasColFrame = colIndex < numColMapFrames;
    bool hasRowFrame = rowIndex < numRowMapFrames;
    

    //Done with frames
    if(!hasColFrame && !hasRowFrame){
        colIndex = 0;
        rowIndex = 0;
        programState = POINTS3D;
    }
    else{
        unsigned char* colFramePixels;
        unsigned char*  invColFramePixels;

        if(hasColFrame){
            colFramePixels = colFrames[colIndex].getPixels();
            invColFramePixels = invColFrames[colIndex].getPixels();
        }

        unsigned char* rowFramePixels;
        unsigned char*  invRowFramePixels;

        if(hasRowFrame){
            rowFramePixels = rowFrames[rowIndex].getPixels();
            invRowFramePixels = invRowFrames[rowIndex].getPixels();
        }

        const unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();

        unsigned char* rowMappingColorPixels = rowMappingColorImg.getPixels();
        unsigned char* colMappingColorPixels = colMappingColorImg.getPixels();
        unsigned short* rowMappingPixels = (unsigned short*)rowMapping16Img->imageData;
        unsigned short* colMappingPixels = (unsigned short*)colMapping16Img->imageData;

        ofColor colMappingColor;
        ofColor rowMappingColor;

        int maxColValue = pow(2,numColMapFrames);
        int maxRowValue = pow(2,numRowMapFrames);

        printf("Processing [colIndex, rowIndex, projWidth, projHeight, maxColValue, maxRowValue] = [%d, %d, %d, %d, %d, %d]\n",colIndex,rowIndex, projWidth, projHeight, maxColValue, maxRowValue);

        int shadowThreshPixel; 

        int colFramePixel,invColFramePixel; 
        int rowFramePixel,invRowFramePixel; 

        unsigned short rowMappingPixel;
        unsigned short colMappingPixel;

        bool colIlluminated;
        bool rowIlluminated;


        for(int r = 0; r < height; r++){
            for(int c = 0; c < width; c++){
                shadowThreshPixel = shadowThreshImgPixels[c+r*width];
                colIlluminated = false;
                rowIlluminated = false;

                if(hasColFrame){
                    colFramePixel = colFramePixels[c+r*width];
                    invColFramePixel = invColFramePixels[c+r*width];

                    colIlluminated = colFramePixel >= (shadowThreshPixel+maxThreshold) && invColFramePixel <= (shadowThreshPixel-minThreshold);
                }
                if(hasRowFrame){
                    rowFramePixel = rowFramePixels[c+r*width];
                    invRowFramePixel = invRowFramePixels[c+r*width];

                    rowIlluminated = rowFramePixel >= (shadowThreshPixel+maxThreshold) && invRowFramePixel <= (shadowThreshPixel-minThreshold);
                }


                colMappingPixel = colMappingPixels[c+r*width];
                rowMappingPixel = rowMappingPixels[c+r*width];

                if(colIlluminated){
                    colMappingPixel |= 1 << (numColMapFrames - colIndex - 1);                
                }

                if(rowIlluminated){
                    rowMappingPixel |= 1 << (numRowMapFrames - rowIndex - 1);            
                }

                // if(hasColFrame){
                //     colMappingPixel = colMappingPixel << 1;
                // }
                // if(hasRowFrame){
                //     rowMappingPixel = rowMappingPixel << 1;
                // }
                
                


                colMappingPixels[c+r*width] = colMappingPixel;
                rowMappingPixels[c+r*width] = rowMappingPixel;

                if(colMappingPixel == 0){
                    colMappingColor.setHsb(0,0,0);
                }
                else{
                    colMappingColor.setHsb(ofMap(colMappingPixel,0,maxColValue,0,255),255,255);   
                }
                colMappingColorPixels[3*(c+r*width)+0] = colMappingColor.r; 
                colMappingColorPixels[3*(c+r*width)+1] = colMappingColor.g; 
                colMappingColorPixels[3*(c+r*width)+2] = colMappingColor.b;

                if(rowMappingPixel == 0){
                    rowMappingColor.setHsb(0,0,0);
                }
                else{
                    rowMappingColor.setHsb(ofMap(rowMappingPixel,0,maxRowValue,0,255),255,255);   
                }
                rowMappingColorPixels[3*(c+r*width)+0] = rowMappingColor.r; 
                rowMappingColorPixels[3*(c+r*width)+1] = rowMappingColor.g; 
                rowMappingColorPixels[3*(c+r*width)+2] = rowMappingColor.b; 
            }
        }

        colMappingColorImg.setFromPixels(colMappingColorPixels,width,height);
        rowMappingColorImg.setFromPixels(rowMappingColorPixels,width,height);
        colIndex++;
        rowIndex++;
    }

    
}



void Scan3dApp::points3dUpdate(){
    
    
    points.resize(width*height,ofPoint(5,5,5));
    colors.resize(width*height,ofColor(255,255,255));
    int numPoints = 0;

    unsigned char* temporalImgPixels = temporalImg.getPixels(); 
    unsigned char* colorImgPixels = colorFrame.getPixels(); 
    // unsigned char* rowMappingPixels = rowMapping16Img.getPixels(); 

    // float rowMappingPixel = 0.0;
    switch(points3dSubstate){
            case POINTS3D_PROCESSING:
                for(int r = 0; r < height; r++){
                    for(int c = 0; c < width; c++){ 

                        // rowMappingPixel = rowMappingPixels[c+r*width];

                        // printf("FramePixel %f\n",(rowMappingPixel));
                        
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
                                ofxRay3d ray = camPixelToRay(cam_intrinsic_matrix,cam_extrinsic_matrix, ofPoint(c,r));

                                if(ray.dir.dot(plane.normal) < 0.1){ // too orthogonal, prone to high error
                                    continue;
                                }
                                
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
                                    ofPoint pixelPt = pt3DToPixel(cam_intrinsic_matrix,cam_extrinsic_matrix,cam_distortion_coeffs, pt);
                                    points[numPoints] = pt;
                                    
                                    ofColor pixelColor = ofColor(   (int)colorImgPixels[3*((int)pixelPt.x+(int)pixelPt.y*width)],(int)colorImgPixels[3*((int)pixelPt.x+(int)pixelPt.y*width)+1],
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
        case CTEMPORAL:
            colMappingColorImg.draw(0, 0);
            break;
        case RTEMPORAL:
            rowMappingColorImg.draw(0, 0);
            break;
    }

    switch(programState){
        case PROJECTOR_CALIBRATION:
            if(projCalSubstate != PROJ_CAL_PROCESSING && projCalSubstate != PROJ_CAL_LOADING){
                ofSetColor(0,255,0);
                ofSetLineWidth(2);
                for(int i = 0; i < 4; i++){
                    ofLine(projPlaneImagePts[i].x-15,projPlaneImagePts[i].y,projPlaneImagePts[i].x+15,projPlaneImagePts[i].y);
                    ofLine(projPlaneImagePts[i].x,projPlaneImagePts[i].y-15,projPlaneImagePts[i].x,projPlaneImagePts[i].y+15);
                }
            }
            break;
        case CAPTURE:
            captureDraw();
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

void Scan3dApp::captureDraw(){
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

/*

First to understand how you calculate it, it would help you if you read some things about the pinhole camera model and simple perspective projection. For a quick glimpse, check this. I'll try to update with more.

So, let's start by the opposite which describes how a camera works: project a 3d point in the world coordinate system to a 2d point in our image. According to the camera model:

P_screen = I * P_world

or (using homogeneous coordinates)

| x_screen | = I * | x_world |
| y_screen |       | y_world |
|    1     |       | z_world |
                   |    1    |
where

I = | f_x    0    c_x    0 | 
    |  0    f_y   c_y    0 |
    |  0     0     1     0 |
is the 3x4 intrinsics matrix, f being the focal point and c the center of projection.

If you solve the system above, you get:

x_screen = (x_world/z_world)*f_x + c_x
y_screen = (y_world/z_world)*f_y + c_y
But, you want to do the reverse, so your answer is:

x_world = (x_screen - c_x) * z_world / f_x
y_world = (y_screen - c_y) * z_world / f_y




*/

void Scan3dApp::computeCameraExtrinsicMatrix(int num, ofPoint *objectPoints, ofPoint *imagePoints, CvMat* cameraMatrix, CvMat* distCoeffs){
  
    CvMat* cvObjectPoints = cvCreateMat( num, 3, CV_32FC1 );
    CvMat* cvImagePoints = cvCreateMat( num, 2, CV_32FC1 );
    CvMat* tvec = cvCreateMat(3,1, CV_32FC1); 
    CvMat* rvec = cvCreateMat(3,1, CV_32FC1); 
    convertOfPointsToCvMat(objectPoints,3,num, cvObjectPoints);
    convertOfPointsToCvMat(imagePoints,2,num, cvImagePoints);
    cvFindExtrinsicCameraParams2(cvObjectPoints,cvImagePoints,cameraMatrix, distCoeffs,rvec,tvec);
    cvReleaseMat(&cvObjectPoints);
    cvReleaseMat(&cvImagePoints);

    printf("tvec: [%f,%f,%f]\n",
        CV_MAT_ELEM(*tvec,float,0,0),
        CV_MAT_ELEM(*tvec,float,1,0),
        CV_MAT_ELEM(*tvec,float,2,0));

    CvMat* rotmat = cvCreateMat( 3,3, CV_32FC1 );


    cvRodrigues2(rvec,rotmat);

    printf("rotmat: \n[%f,%f,%f]\n[%f,%f,%f]\n[%f,%f,%f]\n\n",
        CV_MAT_ELEM(*rotmat,float,0,0),
        CV_MAT_ELEM(*rotmat,float,0,1),
        CV_MAT_ELEM(*rotmat,float,0,2),
        CV_MAT_ELEM(*rotmat,float,1,0),
        CV_MAT_ELEM(*rotmat,float,1,1),
        CV_MAT_ELEM(*rotmat,float,1,2),
        CV_MAT_ELEM(*rotmat,float,2,0),
        CV_MAT_ELEM(*rotmat,float,2,1),
        CV_MAT_ELEM(*rotmat,float,2,2)
    );

    cam_extrinsic_matrix = cvCreateMat( 3, 4, CV_32FC1 );
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            CV_MAT_ELEM(*cam_extrinsic_matrix,float,i,j) = CV_MAT_ELEM(*rotmat,float,i,j);
        }  
    }

    for(int i = 0; i < 3; i++){
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,i,3) = CV_MAT_ELEM(*tvec,float,i,0); 
    }


    printf("cam_extrinsic_matrix: \n[%f,%f,%f,%f]\n[%f,%f,%f,%f]\n[%f,%f,%f,%f]\n\n",
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,0,0),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,0,1),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,0,2),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,0,3),

        CV_MAT_ELEM(*cam_extrinsic_matrix,float,1,0),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,1,1),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,1,2),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,1,3),

        CV_MAT_ELEM(*cam_extrinsic_matrix,float,2,0),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,2,1),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,2,2),
        CV_MAT_ELEM(*cam_extrinsic_matrix,float,2,3)

    );

    CvMat* rotmatTranspose = cvCreateMat(3,3,CV_32FC1);
    cvTranspose(rotmat,rotmatTranspose);

    CvMat* negRotmatTranspose = cvCreateMat(3,3,CV_32FC1);
    cvScale(rotmatTranspose,negRotmatTranspose,-1);
    cout << "Play your fiddle hard" << endl;
    CvMat* cameraPosition = cvCreateMat(3,1,CV_32FC1);
    cvMatMul(negRotmatTranspose,tvec,cameraPosition);
    cout << "Dem boys got deh blues" << endl;
    camPos = ofPoint(CV_MAT_ELEM(*cameraPosition,float,0,0),CV_MAT_ELEM(*cameraPosition,float,1,0),CV_MAT_ELEM(*cameraPosition,float,2,0));


    printf("camPos: [");
    cout << camPos << "]\n\n" << endl;

    cvReleaseMat(&tvec);
    cvReleaseMat(&rvec);
    cvReleaseMat(&rotmat);   
    cvReleaseMat(&rotmatTranspose);   
    cvReleaseMat(&negRotmatTranspose);   
}

ofxRay3d Scan3dApp::camPixelToRay(const CvMat* intrinsicMat, const CvMat* extrinsicMat, ofPoint imagePt){
    CvMat* homoImgPt = cvCreateMat(3,1, CV_32FC1); 

    CV_MAT_ELEM(*homoImgPt,float,0,0) = imagePt.x;
    CV_MAT_ELEM(*homoImgPt,float,1,0) = imagePt.y;
    CV_MAT_ELEM(*homoImgPt,float,2,0) = 1.0;

    CvMat* resPt = cvCreateMat(3,1, CV_32FC1); 

    cvInv(intrinsicMat,camInvIntrinsic);

    cvMatMul(camInvIntrinsic,homoImgPt,resPt);

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
void Scan3dApp::setCameraAndProjector(){
    computeCameraExtrinsicMatrix(4,projPlaneObjectPts, projPlaneImagePts, cam_intrinsic_matrix, cam_distortion_coeffs);

    ofPoint testPt;

    for(int i = 0; i < 4; i++){
        testPt = pt3DToPixel(cam_intrinsic_matrix, cam_extrinsic_matrix, cam_distortion_coeffs, projPlaneObjectPts[i]);
        cout << "Reprojection [expected][returned] : [" << projPlaneImagePts[i] << "] [" << testPt << "]" << endl;
    }
    printf("camPos:\n");
    cout << camPos << endl;
    ofxRay3d centerRay = camPixelToRay(cam_intrinsic_matrix,cam_extrinsic_matrix, ofPoint(width/2,height/2));

    ofVec3f centerPt = (ofVec3f)centerRay.intersect(vertPlane);

    ofNode lookat;

    lookat.setPosition(centerPt);

    ofVec3f easyCamPos = (ofVec3f)camPos;
    

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

}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 32: //SPACEBAR
            if(programState ==  PROJECTOR_CALIBRATION){
                setCameraAndProjector();
                programState = CAPTURE;
                saveSettings();
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
            displayState = CTEMPORAL;
            break;
        case 54:
            displayState = RTEMPORAL;
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
            if(programState == PROJECTOR_CALIBRATION){
                clearProjectorSettings();    
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
}

//--------------------------------------------------------------
void Scan3dApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void Scan3dApp::mouseReleased(int x, int y, int button){
    if(programState == PROJECTOR_CALIBRATION){
        switch(projCalSubstate){
            case  BR:                
                projPlaneImagePts[0] = getNearestCorner(grayscaleFrame,10,x,y);
                projCalSubstate = TR;
                break;
            case  TR:
                projPlaneImagePts[1] = getNearestCorner(grayscaleFrame,10,x,y);
                projCalSubstate = TL;
                break;
            case  TL:
                projPlaneImagePts[2] = getNearestCorner(grayscaleFrame,10,x,y);
                projCalSubstate = BL;
                break;
            case  BL:
                projPlaneImagePts[3] = getNearestCorner(grayscaleFrame,10,x,y);
                projCalSubstate = PROJ_CAL_WAITING;
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

    ofxRay3d ray0 = camPixelToRay(intrinsicMat,extrinsicMat,pixPt0);
    ofxRay3d ray1 = camPixelToRay(intrinsicMat,extrinsicMat,pixPt1);

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
                // cout << "Camera Position: ["<< camPos << "]" << endl;
                ofxRay3d centerRay = camPixelToRay(cam_intrinsic_matrix,cam_extrinsic_matrix, ofPoint(width/2,height/2));
                ofVec3f centerPt = (ofVec3f)centerRay.intersect(vertPlane);
                // cout << "Camera look: ["<< centerPt << "]" << endl;
                for(int i =0; i < 4; i += step){  
                    ofVec3f vec0, vec1;
                    mesh.addColor(ofColor(0,255,0));
                    vec0 = (ofVec3f)projPlaneObjectPts[i];
                    mesh.addVertex(vec0);

                    mesh.addColor(ofColor(255,0,0));

                    ofxRay3d ray = camPixelToRay(cam_intrinsic_matrix,cam_extrinsic_matrix, projPlaneImagePts[i]);
                    // cout << "ray: ["<< ray.dir << "]" << endl;
                    vec1 = (ofVec3f)ray.intersect(vertPlane);
                    mesh.addVertex(vec1);
                    // cout << "["<< vec0 << "]   [" << vec1 << "]" << endl;
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
}

ofxCvGrayscaleImage Scan3dApp::computeBinCodeImage(int w, int h, int power, bool inverse, int type){
    int stepSize = 0;
    int numSteps = 0;
    ofxCvGrayscaleImage output;
    output.allocate(w,h);
    output.set(0);
    if(power == 0){ //Skipping all the other nonsense
        return output;
    }
    else{
        stepSize = (type == HORIZONTAL) ? (int)(w/pow(2,power)) : (int)(h/pow(2,power));
        numSteps = (int)(w/stepSize);
        for(int step = 1; step < numSteps; step+= 2){

            (type == HORIZONTAL) ? output.setROI(step*stepSize,0,stepSize,h) : output.setROI(0,step*stepSize,w,stepSize);

            output.set(255);


            output.resetROI();
        }
    }

    if(inverse){
        output.invert();
    }

    return output;
}

ofxCvGrayscaleImage Scan3dApp::computeGrayCodeImage(int w, int h, int power, bool inverse, int type){
    int stepSize = 0;
    int stepOffset = 0;
    int numSteps = 0;
    ofxCvGrayscaleImage output;
    output.allocate(w,h);
    output.set(0);
    if(power == 0){ //Skipping all the other nonsense
        return output;
    }
    else{
        stepSize = (type == HORIZONTAL) ? (int)(w/pow(2,power-1)) : (int)(h/pow(2,power-1));
        stepOffset = stepSize/2;
        numSteps = w/stepSize;
        //cout << "[w,h,stepSize,numSteps] : ["<< w << ", " << h << ", " << stepSize << ", " << numSteps << "]" <<endl;
        
        for(int step = 0; step < numSteps; step+= 2){
            //cout << "roi[x,w] : ["<< step*stepSize+stepOffset << ", " << stepSize << "]" << endl;
            (type == HORIZONTAL) ? output.setROI(step*stepSize+stepOffset,0,stepSize,h) : output.setROI(0,step*stepSize+stepOffset,w,stepSize);
            //cout << "ROI now: ["<< output.getROI().x << ", " << output.getROI().y << ", " << output.getROI().width << ", " << output.getROI().height << "]" <<endl;
            output.set(255);


            output.resetROI();
        }
    }

    if(inverse){
        output.invert();
    }

    return output;
}




