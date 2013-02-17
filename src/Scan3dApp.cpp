#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
    loadSettings();

    messageBarText = "";
    messageBarSubText = "";
    messageBarHeight = 70;
    messageBarFont.loadFont("HelveticaNeueLTStd-Bd.otf", 20);
    messageBarSubTextFont.loadFont("HelveticaNeueLTStd-Lt.otf", 15);
    
    programState = SETUP;

    zeroCrossingThreshold = 20;
    cout << "SETUP STATE (press SPACE to continue)" << endl;
    displayState = COLOR;
    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetFrameRate(30);

    switch(inputType){
        case VIDEO:
            vid.loadMovie(inputVideoFile);
            vid.play();
            vid.update(); //to get height and width to load
            width = vid.getWidth();
            height = vid.getHeight();
            break;
    }
    
    colorFrame.allocate(width,height);
    maxColorImg.allocate(width,height);
    grayscaleFrame.allocate(width,height);
    minImg.allocate(width,height);
    minImg.set(255);
    maxImg.allocate(width,height);
    maxImg.set(0);
    temporalImg.allocate(width,height);

    bufferOfImage.allocate(width,height,OF_IMAGE_COLOR);
    bufferOfxCvColorImage.allocate(width,height);
    bufferOfxCvGrayscaleImage.allocate(width,height);
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
    
    topSection.x = 0;
    topSection.y = 0;
    topSection.width = 0;
    topSection.height = 0;

    bottomSection.x = 0;
    bottomSection.y = 0;
    bottomSection.width = 0;
    bottomSection.height = 0;

    settingTopSection = false;
    settingBottomSection = false;
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

            settings.pushTag("input");
                string input = settings.getValue("type","NONE");
                if(input == "VIDEO"){
                    inputType = VIDEO;
                    inputVideoFile = settings.getValue("src","");
                    cout << "   Using video: " << inputVideoFile << endl;
                }
			
			settings.popTag(); //pop input
			settings.pushTag("scene");

				settings.pushTag("verticalPlane");
					cout << "   Loaded vertical plane points as [TL,TR,BR,BL]:"<< endl;
					settings.pushTag("pts");

					for(int i = 0; i < 4; i++){
					    verticalPlanePts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0));
					    cout << "      [" << verticalPlanePts[i].x << ", " << verticalPlanePts[i].y << "]" << endl;
					}

					settings.popTag(); // pop pts
				settings.popTag(); // pop verticalPlane

				settings.pushTag("horizontalPlane");
					cout << "   Loaded horizontal plane points as [TL,TR,BR,BL]:"<< endl;
					settings.pushTag("pts");

					for(int i = 0; i < 4; i++){
					    horizontalPlanePts[i].set(settings.getValue("x",0.0),settings.getValue("y",0.0));
					    cout << "      [" << horizontalPlanePts[i].x << ", " << horizontalPlanePts[i].y << "]" << endl;
					}
					
					settings.popTag(); // pop pts
 				settings.popTag(); // pop horizontalPlane

			settings.popTag(); // pop scene

		settings.popTag(); // pop settings
		cout << "** Done Loading Settings **" << endl;
	}
	else {
		cout << "No settings file to load." << endl;
	}
}

//--------------------------------------------------------------
void Scan3dApp::update(){
    switch(programState){

        case SETUP:
        {
            messageBarText = "SETUP";
            if(topSection.width == 0 || settingTopSection){
                
                messageBarSubText = "Click and drag top rectangle";
            }
            else if(bottomSection.width == 0 || settingBottomSection){
                messageBarSubText = "Click and drag bottom rectangle";
            }
            else{
                messageBarSubText = "Press spacebar to commence scanning";
            }
            frameIndex = 0;
            switch(inputType){
                case VIDEO:
                    vid.firstFrame();
                    vid.update();
                    if(vid.isFrameNew()){
                        colorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());
                    }
                    break;
            }
            //do nothing for now, waiting for spacebar
            break;
        }
        case CAPTURE:
        {
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
            
            switch(inputType){
                case VIDEO:
                    vid.update();
                    if(vid.isFrameNew()){
                        colorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());
                    }
                    if(vid.getIsMovieDone()){
                        colorFrame = maxColorImg;
                        messageBarText = "PROCESSING";
                        messageBarSubText = "";
                        programState = PROCESSING;
                        cout << "PROCESSING STATE" << endl;
                        frameIndex = 0;
                    }
                    break;
            }

            //Uncomment to save out color frames
            // bufferOfImage.setFromPixels(colorFrame.getPixelsRef());
            // string filename = "output/grayscaleFrames/gsframe";
            // filename += ofToString(frameIndex);
            // filename += ".tiff";
            // bufferOfImage.saveImage(filename);
                
            grayscaleFrame = colorFrame;
            if(frameIndex >= frames.capacity()){
                frames.resize(2*frames.size(),bufferOfxCvGrayscaleImage);
            }
        
            frames[frameIndex] = grayscaleFrame;

            //Uncomment to save out grayscale frames
            // bufferOfxCvColorImage = grayscaleFrame;
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // string filename = "output/grayscaleFrames/gsframe";
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
            break;
        }
        case PROCESSING:
        {     
            unsigned char* temporalImgPixels = temporalImg.getPixels();
            unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();            

            grayscaleFrame = frames[frameIndex];


            //Uncomment to save out min/max/shadowthresh frames
            // bufferOfxCvColorImage = minImg;
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // bufferOfImage.saveImage("output/minImg.tiff");
            // bufferOfxCvColorImage = maxImg;
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // bufferOfImage.saveImage("output/maxImg.tiff");
            // bufferOfxCvColorImage = shadowThreshImg;
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // bufferOfImage.saveImage("output/shadowThreshImg.tiff");

            diffFrames.resize(frames.size(),bufferOfxCvGrayscaleImage);
            zeroCrossingFrames.resize(frames.size(),bufferOfxCvGrayscaleImage);
            
            int columnIndex = 0;

            vector<int> columnIndices;
            columnIndices.resize(height);

            diffFrames[frameIndex] = frames[frameIndex];
            diffFrames[frameIndex] -= shadowThreshImg;
            if(frameIndex>0){
                bufferOfxCvGrayscaleImage = diffFrames[frameIndex];
                bufferOfxCvGrayscaleImage.absDiff(diffFrames[frameIndex-1]);
                diffFrames[frameIndex] -= bufferOfxCvGrayscaleImage;

                unsigned char* diffFramePixels = diffFrames[frameIndex].getPixels();
                unsigned char* zeroCrossingFramePixels = zeroCrossingFrames[frameIndex].getPixels();
                
                for(int r = 0; r < height; r++){
                    for(int c = columnIndices[r]; c < width; c++){
                        if(diffFramePixels[c+r*width] > zeroCrossingThreshold){
                            zeroCrossingFramePixels[c+r*width] = 255;
                            for(int j = columnIndices[r]; j < c; j++){
                                ofColor c1Color = ofColor::blue;
                                c1Color.lerp(ofColor::red,ofMap(frameIndex-1, 0, frames.size(), 0.0, 1.0));

                                ofColor c2Color = ofColor::blue;
                                c2Color.lerp(ofColor::red,ofMap(frameIndex, 0, frames.size(), 0.0, 1.0));

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
            

            //Uncomment to save out diff frames
            // bufferOfxCvColorImage = diffFrames[i];
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // string filename = "output/diffFrames/diffFrame";
            // filename += ofToString(i);
            // filename += ".tiff";
            // bufferOfImage.saveImage(filename);

            //Uncomment to save out zero crossing frames
            // bufferOfxCvColorImage = zeroCrossingFrames[i];
            // bufferOfImage.setFromPixels(bufferOfxCvColorImage.getPixelsRef());
            // string filename = "output/zeroCrossingFrames/zeroCrossingFrame";
            // filename += ofToString(i);
            // filename += ".tiff";
            // bufferOfImage.saveImage(filename);

            }

            frameIndex++;
            if(frameIndex == frames.size()-1){
                //Uncomment to save out temporal image
                bufferOfImage.setFromPixels(temporalImg.getPixelsRef());
                bufferOfImage.saveImage("output/temporalImg.tiff");

                programState = VISUALIZATION;
                cout << "VISUALIZATION STATE" << endl;
            }
       
            break;
        }
        case VISUALIZATION:
        {    
            messageBarText = "VISUALIZATION";
            messageBarSubText = "";

            //do nothing
            break;
        }

    }
}

//--------------------------------------------------------------
void Scan3dApp::draw(){
    
    // draw the image sequence at the new frame count
    //ofSetColor(255);
    
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
            shadowThreshImg.draw(0, 0);
            break;
    }

    switch(programState){
        case SETUP:
            drawSectionRectangles();
            break;
    }
}

//--------------------------------------------------------------
/**
    Draws the section rectangles as a color overlay
*/
void Scan3dApp::drawSectionRectangles(){
    ofEnableAlphaBlending();
    ofSetColor(topSectionColor);
    ofRect(topSection);

    ofSetColor(bottomSectionColor);
    ofRect(bottomSection);

    ofDisableAlphaBlending();
}

//--------------------------------------------------------------
/**
    Computes the best fit line of the pixel coordinates of the zero crossings in an image

    @param img The (likely zero-crossing) image that has white pixels
    @param roi (optional) A rectangle that defines the region of interest to look over img 
    @returns ofxLine a line generated from the zero crossings
*/
ofxLine Scan3dApp::computeLineEquationFromZeroCrossings(ofxCvGrayscaleImage img, ofRectangle roi){
    
    

    int roi_x0 = 0;
    int roi_y0 = 0;

    int roi_x1 = img.width;
    int roi_y1 = img.height;

    if(roi.isEmpty()){
        roi_x0 = roi.x;
        roi_y0 = roi.y;
        
        roi_x1 = roi.x+roi.width;
        roi_y1 = roi.y+roi.height;
           
    }


    CvPoint * points=(CvPoint*)malloc( roi_y1-roi_y0 * sizeof(points[0]));

    

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

    CvMat point_mat = cvMat( 1, roi_y1-roi_y0, CV_32SC2, points);
    float result[4];// to store the results

    cvFitLine(&point_mat,CV_DIST_HUBER ,0,0.01,0.01,result);

    ofxLine line;
    line.set(result[0],result[1],result[2],result[3]);


    free(points);

    return line;
}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 32:
            if(programState ==  SETUP){
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
    if(settingTopSection){
        topSection.setWidth(x-topSection.x);
        topSection.setHeight(y-topSection.y);
    }
    else if(settingBottomSection){

        bottomSection.setWidth(x-bottomSection.x);
        bottomSection.setHeight(y-bottomSection.y);

        if(bottomSection.width > topSection.width){
            bottomSection.width = topSection.width;
        }
    }
}

//--------------------------------------------------------------
void Scan3dApp::mousePressed(int x, int y, int button){
    if(topSection.width == 0){
        topSection.setPosition(x,y);
        settingTopSection = true;
    }
    else if(bottomSection.width == 0){
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
        
        settingTopSection = false;
        settingBottomSection = true;
    }
}

//--------------------------------------------------------------
void Scan3dApp::mouseReleased(int x, int y, int button){
    if(topSection.width > bottomSection.width && bottomSection.width > 0){
        topSection.width = bottomSection.width;
    }
    settingTopSection = false;
    settingBottomSection = false;
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
