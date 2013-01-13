#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
	loadSettings();
    
    displayState = COLOR;
    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetWindowShape(1280,720);
    ofSetFrameRate(30);
    // Read the directory for the images
    // we know that they are named in seq
    ofDirectory dir(imgDir);

    dir.listDir();
	dir.sort();

    int nFiles = dir.numFiles();
    int width;
    int height;
    
    if(nFiles) {
        cout << "** Loading image frames ... ";
        for(int i=0; i<dir.numFiles(); i++) {
            
            // add the image to the vector
            string filePath = dir.getPath(i);
            ofImage frame;
            frame.loadImage(filePath);
            width = frame.getWidth();
            height = frame.getHeight();
            ofxCvColorImage colorImg;
            colorImg.allocate(width,height);
            colorImages.push_back(colorImg);
            colorImages.back().setFromPixels(frame.getPixels(),width,height);
            
            //Create a grayscale copy of each frame
            ofxCvGrayscaleImage gs;
        	gs.allocate(width,height);
            gsImages.push_back(gs);
            gsImages.back() = colorImages.back();
            
        }
        cout << "done!" << endl;
        cout << "** Creating difference frames ... ";
        //Now we create the difference image.
        for(int i=1; i<dir.numFiles(); i++) {
        	ofxCvGrayscaleImage gs;
            gs.allocate(width,height);
            gs = gsImages[i];
            gs -= gsImages[i-1];
            diffImages.push_back(gs);
            
            gs.threshold(30);
            threshImages.push_back(gs);
        }
        cout << "done!" << endl;
        
        cout << "** Creating threshold frames ... ";
        
        
        for(int i=0; i<dir.numFiles()-1; i++) {
            unsigned char * threshPixels = threshImages[i].getPixels();
            ofxCvGrayscaleImage edge;
            edge.allocate(width,height);
            edge.set(255);
            unsigned char * edgePixels = edge.getPixels();
            int firstWhitePixel = 0;
            int lastWhitePixel = 0;
            int midWhitePixel = 0;
            
            bool whiteDetected = false;
            for (int y = 0; y < height; y++) {
                whiteDetected = false;
                
                for (int x = 0; x < width; x++) {
                    int index = y*width + x;
                    //if pixel is not black
                    if (threshPixels[index] != 0) {
                        if(!whiteDetected){
                            whiteDetected = true;
                            firstWhitePixel = x;
                        }
                        else{
                            lastWhitePixel = x;
                        }
                       
                    }
                    if(threshPixels[index] == 0 && whiteDetected){
                        midWhitePixel = int((lastWhitePixel-firstWhitePixel)/2)+firstWhitePixel;
                        index = y*width + midWhitePixel;
                        edgePixels[index] = 0;
                        break;
                    }
                }
                //If we have found the white pixels, get the middle one
                //Draw this to the new bwImages vector (for now).
            }
            edge.setFromPixels(edgePixels,width,height);
            edgeImages.push_back(edge);
        }
        cout << "great success!" << endl;

        
    }
    else {
        cout << "Could not find folder\n" << endl;
    }
    
    frameIndex = 0;

}

//--------------------------------------------------------------
void Scan3dApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "** Loading Settings File **" << endl;
		settings.pushTag("settings");

			imgDir = settings.getValue("imgDir","");
			cout << "   Loaded image directory: " << imgDir << endl;

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
    
}

//--------------------------------------------------------------
void Scan3dApp::draw(){
    
    // We need some images if not, return
    if((int)colorImages.size() <= 0) {
        ofSetColor(255);
        ofDrawBitmapString("No Images...", 150, ofGetHeight()/2);
        return;
    }
    
    frameIndex = (frameIndex+1)%(int)diffImages.size();
    
    // draw the image sequence at the new frame count
    //ofSetColor(255);
    switch(displayState){
        case COLOR:
            colorImages[frameIndex].draw(0, 0);
            break;
        case GRAYSCALE:
            gsImages[frameIndex].draw(0, 0);
            break;
        case DIFF:
            diffImages[frameIndex].draw(0, 0);
            break;
        case THRESH:
            threshImages[frameIndex].draw(0, 0);
            break;
        case EDGE:
            edgeImages[frameIndex].draw(0, 0);
            break;
        default:
            colorImages[frameIndex].draw(0, 0);
    }
    
    
}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){
    switch(key){
        case 49:
            displayState = COLOR;
            break;
        case 50:
            displayState = GRAYSCALE;
            break;
        case 51:
            displayState = DIFF;
            break;
        case 52:
            displayState = THRESH;
            break;
        case 53:
            displayState = EDGE;
            break;
        default:
            displayState = COLOR;
            //nothing
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