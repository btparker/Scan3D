#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
    loadSettings();
    
    programState = SETUP;

    zeroCrossingThreshold = 20;
    cout << "SETUP STATE (press SPACE to continue)" << endl;
    displayState = COLOR;
    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetWindowShape(1280,720);
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

    ofSetWindowShape(width,height);

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
            switch(inputType){
                case VIDEO:
                    vid.update();
                    if(vid.isFrameNew()){
                        colorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());
                    }
                    if(vid.getIsMovieDone()){
                        programState = PROCESSING;
                        cout << "PROCESSING STATE" << endl;
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
            unsigned char* grayscaleFramePixels = grayscaleFrame.getPixels();
            int i = 0;
            for(int y = 0; y < height; y++){
                for(int x = 0; x < width; x++){
                    i = y*width+x;
                    minImgPixels[i] = min(minImgPixels[i],grayscaleFramePixels[i]);
                    maxImgPixels[i] = max(maxImgPixels[i],grayscaleFramePixels[i]);
                }  
            }

            //A tad inefficient, but this handles a bug where if you draw these it doesn't set new values
            minImg.setFromPixels(minImgPixels,width,height);
            maxImg.setFromPixels(maxImgPixels,width,height);
            
            frameIndex++;
            break;
        }
        case PROCESSING:
        {   
            unsigned char* minImgPixels = minImg.getPixels();
            unsigned char* maxImgPixels = maxImg.getPixels();
            unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();
            unsigned char* temporalImgPixels = temporalImg.getPixels();


            int i = 0;
            for(int y = 0; y < height; y++){
                for(int x = 0; x < width; x++){
                    i = y*width+x;
                    shadowThreshImgPixels[i] = (int)((minImgPixels[i]+maxImgPixels[i])/2);
                }  
            }
            shadowThreshImg.setFromPixels(shadowThreshImgPixels,width,height);

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
            for(int i = 0; i < frames.size(); i++){
                diffFrames[i] = frames[i];
                diffFrames[i] -= shadowThreshImg;
                if(i>0){
                    bufferOfxCvGrayscaleImage = diffFrames[i];
                    bufferOfxCvGrayscaleImage.absDiff(diffFrames[i-1]);
                    diffFrames[i] -= bufferOfxCvGrayscaleImage;

                    unsigned char* diffFramePixels = diffFrames[i].getPixels();
                    unsigned char* zeroCrossingFramePixels = zeroCrossingFrames[i].getPixels();
                    
                    for(int r = 0; r < height; r++){
                        for(int c = columnIndices[r]; c < width; c++){
                            if(diffFramePixels[c+r*width] > zeroCrossingThreshold){
                                zeroCrossingFramePixels[c+r*width] = 255;
                                for(int j = columnIndices[r]; j < c; j++){
                                    ofColor c1Color = ofColor::blue;
                                    c1Color.lerp(ofColor::red,ofMap(i-1, 0, frames.size(), 0.0, 1.0));

                                    ofColor c2Color = ofColor::blue;
                                    c2Color.lerp(ofColor::red,ofMap(i, 0, frames.size(), 0.0, 1.0));

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

            //Uncomment to save out temporal image
            bufferOfImage.setFromPixels(temporalImg.getPixelsRef());
            bufferOfImage.saveImage("output/temporalImg.tiff");


            programState = VISUALIZATION;
            cout << "VISUALIZATION STATE" << endl;
            break;
        }
        case VISUALIZATION:
        {    
            //do nothing
            break;
        }

    }
    



}

//--------------------------------------------------------------
void Scan3dApp::draw(){
    
    // draw the image sequence at the new frame count
    //ofSetColor(255);
    

    ofSetColor(255);

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
void Scan3dApp::drawSectionRectangles(){
    ofEnableAlphaBlending();
    ofSetColor(topSectionColor);
    ofRect(topSection);

    ofSetColor(bottomSectionColor);
    ofRect(bottomSection);

    ofDisableAlphaBlending();
}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 32:
            cout << "CAPTURE STATE" << endl;
            programState = CAPTURE;
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
