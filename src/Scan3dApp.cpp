#include "Scan3dApp.h"
using namespace cv;

//--------------------------------------------------------------
void Scan3dApp::setup(){
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
    zeroCrossingFrame.allocate(width,height);
    diffFrame.allocate(width,height);
    previousDiffFrame.allocate(width,height);
    previousDiffFrame.set(0);

    
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

    columnIndices.resize(height);

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

    vertPlane = ofxPlane(ofPoint(5,5,0),ofVec3f(0,0,-1));
    horizPlane = ofxPlane(ofPoint(5,0,5),ofVec3f(0,1,0));

    paused = false;
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
                    zeroCrossingThreshold = settings.getValue("zeroCrossingThreshold", 20);
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
            case VISUALIZATION:
            {    
                visualizationUpdate();
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
                
                
                //CALIBRATE THE CAMERA!
                cvCalibrateCamera2(
                    object_points2, 
                    image_points2,
                    point_counts2, 
                    cvGetSize( colorFrame.getCvImage() ),
                    intrinsic_matrix, 
                    distortion_coeffs,
                    NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
                );
                
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
        cout << "PROCESSING STATE" << endl;
        frameIndex = 0;
    }

    //Uncomment to save out color frames
    // bufferOfImage.setFromPixels(colorFrame.getPixelsRef());
    // filename = "output/grayscaleFrames/gsframe";
    // filename += ofToString(frameIndex);
    // filename += ".tiff";
    // bufferOfImage.saveImage(filename);
        
    grayscaleFrame = colorFrame;

    //resizing vector if needed
    if(frameIndex >= frames.capacity()){
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
            shadowThreshImgPixels[i] = (int)((minImgPixels[i]+maxImgPixels[i])/2);
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
    unsigned char* temporalImgPixels = temporalImg.getPixels(); 

    grayscaleFrame = frames[frameIndex];
    zeroCrossingFrame.set(0);
    diffFrame.set(0);
    previousDiffFrame.set(0);

    if(frameIndex>0){
        previousDiffFrame = frames[frameIndex-1];
        previousDiffFrame -= shadowThreshImg;
    }

    diffFrame = frames[frameIndex];
    diffFrame -= shadowThreshImg;

    if(frameIndex>0){
        bufferOfxCvGrayscaleImage = diffFrame;
        bufferOfxCvGrayscaleImage.absDiff(previousDiffFrame);
        diffFrame -= bufferOfxCvGrayscaleImage;

        unsigned char* diffFramePixels = diffFrame.getPixels();
        unsigned char* zeroCrossingFramePixels = zeroCrossingFrame.getPixels();
        
        for(int r = 0; r < height; r++){
            for(int c = columnIndices[r]; c < width; c++){
                if(diffFramePixels[c+r*width] > zeroCrossingThreshold){
                    zeroCrossingFramePixels[c+r*width] = 255;
                    for(int j = columnIndices[r]; j < c; j++){

                        float hue = ((float)frameIndex-1)/(float)frames.size()*255.0;
                        float sat = 255;
                        float bri = 255;

                        ofColor c1Color = ofColor::fromHsb(hue,sat,bri);

                        hue = ((float)frameIndex)/(float)frames.size()*255.0;

                        ofColor c2Color = ofColor::fromHsb(hue,sat,bri);

                        ofColor interColor = c1Color;
                        interColor.lerp(c2Color,ofMap(j,columnIndices[r],c,0.0,1.0));
                        
                        temporalImgPixels[3*(j+r*width)] = interColor[0];
                        temporalImgPixels[3*(j+r*width)+1] = interColor[1];
                        temporalImgPixels[3*(j+r*width)+2] = interColor[2];
                    }
                    columnIndices[r] = c;
                    break;
                }
            }
        }
        temporalImg.setFromPixels(temporalImgPixels,width,height);
        zeroCrossingFrame.setFromPixels(zeroCrossingFramePixels,width,height);

        if(frameIndex > 0){
            topLine = computeLineFromZeroCrossings(zeroCrossingFrame,topSection);
            bottomLine = computeLineFromZeroCrossings(zeroCrossingFrame,bottomSection);  

            //cout << "topLine " << topLine.dir.x << " " << topLine.dir.y << endl;
            //cout << "bottomLine " << bottomLine.dir.x << " " << bottomLine.dir.y << endl;
        }
        

         //Uncomment to save out diff frames
        // bufferOfxCvColorImage = diffFrame;
        // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
        // filename = "output/diffFrames/diffFrame";
        // filename += ofToString(frameIndex);
        // filename += ".tiff";
        // bufferOfImage.saveImage(filename);

        //Uncomment to save out zero crossing frames
        // bufferOfxCvColorImage = zeroCrossingFrame;
        // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
        // filename = "output/zeroCrossingFrames/zeroCrossingFrame";
        // filename += ofToString(frameIndex);
        // filename += ".tiff";
        // bufferOfImage.saveImage(filename);
    }

    frameIndex++;
    if(frameIndex == frames.size()){
        //Uncomment to save out temporal image
        bufferOfImage.setFromPixels(temporalImg.getPixelsRef());
        bufferOfImage.saveImage("output/temporalImg.tiff");

        programState = VISUALIZATION;
        cout << "VISUALIZATION STATE" << endl;
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
}

void Scan3dApp::visualizationUpdate(){
    messageBarText = "VISUALIZATION";
    messageBarSubText = "";

    //do nothing
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
            processingDraw();
            
            
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

    ofxLine2d line;
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
