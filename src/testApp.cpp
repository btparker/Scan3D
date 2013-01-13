#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	loadSettings();
    
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
    
    if(nFiles) {
        
        for(int i=0; i<dir.numFiles(); i++) {
            
            // add the image to the vector
            string filePath = dir.getPath(i);
            colorImages.push_back(ofxCvColorImage());
            
            ofImage frame;
            frame.loadImage(filePath);
            int width = frame.getWidth();
            int height = frame.getHeight();
            
            colorImages.back().allocate(width,height);
            colorImages.back().setFromPixels(frame.getPixels(),width,height);
            
            //Create a grayscale copy of each frame
            gsImages.push_back(ofxCvGrayscaleImage());
            gsImages.back() = colorImages.back();
            
        }
        
        //Now we create the difference image.
        for(int i=1; i<dir.numFiles(); i++) {
            diffImages.push_back(ofxCvGrayscaleImage());
            diffImages.back().absDiff(gsImages[i-1], gsImages[i]);
        }
        
    }
    else {
        cout << "Could not find folder\n" << endl;   
    }
    
    frameIndex = 0;

}

//--------------------------------------------------------------
void testApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "*** Loading Settings File ***" << endl;
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
		cout << "*** Done Loading Settings ***" << endl;
	}
	else {
		cout << "No settings file to load." << endl;
	}
}

//--------------------------------------------------------------
void testApp::update(){
    
}

//--------------------------------------------------------------
void testApp::draw(){
    
    // We need some images if not, return
    if((int)colorImages.size() <= 0) {
        ofSetColor(255);
        ofDrawBitmapString("No Images...", 150, ofGetHeight()/2);
        return;
    }
    
    frameIndex = (frameIndex+1)%(int)diffImages.size();
    
    // draw the image sequence at the new frame count
    ofSetColor(255);
    diffImages[frameIndex].draw(0, 0);
    
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}