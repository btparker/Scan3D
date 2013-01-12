#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	loadSettings();
    
    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetWindowShape(1280,720);
    
    // Read the directory for the images
    // we know that they are named in seq
    ofDirectory dir;
    
    int nFiles = dir.listDir(imgDir);
    
    if(nFiles) {
        
        for(int i=0; i<dir.numFiles(); i++) {
            
            // add the image to the vector
            string filePath = dir.getPath(i);
            colorImages.push_back(ofImage());
            colorImages.back().loadImage(filePath);
            
            //Create a grayscale copy of each frame
            gsImages.push_back(ofImage());
            gsImages.back().clone(colorImages.back());
            gsImages.back().setImageType(OF_IMAGE_GRAYSCALE);
            
        }
        
        //Now we create the difference image.
        //for(i=1; i<dir.numFiles(); i++) {
            //diffImages.push_back(ofImage());
            
        //}
        
    }
    else printf("Could not find folder\n");
    
    frameIndex = 0;

}

//--------------------------------------------------------------
void testApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "*** Loading settings file ***" << endl;
		settings.pushTag("settings");

			imgDir = settings.getValue("imgDir","");

			settings.pushTag("scene");

				settings.pushTag("verticalPlane");
					for(int i = 0; i < 4; i++){
					    
					}
					settings.pushTag("pts");

					settings.popTag(); // pop pts
				settings.popTag(); // pop verticalPlane

				settings.pushTag("horizontalPlane");
					settings.pushTag("pts");

					settings.popTag(); // pop pts
 				settings.popTag(); // pop horizontalPlane

			settings.popTag(); // pop scene

		settings.popTag(); // pop settings
	}
	else{
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
    
    frameIndex = (frameIndex+1)%(int)colorImages.size();
    
    // draw the image sequence at the new frame count
    ofSetColor(255);
    gsImages[frameIndex].draw(0, 0);
    
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