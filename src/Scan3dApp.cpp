#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
    loadSettings();

    programState = SETUP;
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

    shadowThreshImg.allocate(width,height);

    ofSetWindowShape(width,height);
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
                
            grayscaleFrame = colorFrame;

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
            
            break;
        }
        case PROCESSING:
        {   
            unsigned char* minImgPixels = minImg.getPixels();
            unsigned char* maxImgPixels = maxImg.getPixels();
            unsigned char* shadowThreshImgPixels = shadowThreshImg.getPixels();

            int i = 0;
            for(int y = 0; y < height; y++){
                for(int x = 0; x < width; x++){
                    i = y*width+x;
                    shadowThreshImgPixels[i] = (int)((minImgPixels[i]+maxImgPixels[i])/2);
                }  
            }
            shadowThreshImg.setFromPixels(shadowThreshImgPixels,width,height);

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
void Scan3dApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void Scan3dApp::dragEvent(ofDragInfo dragInfo){ 

}
