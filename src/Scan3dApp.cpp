#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
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
            diffImages.push_back(gs);
            diffImages.back().absDiff(gsImages[i-1], gsImages[i]);
        }
        cout << "done!" << endl;
        
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
    ofSetColor(255);
    diffImages[frameIndex].draw(0, 0);
    
}

//--------------------------------------------------------------
void Scan3dApp::keyPressed(int key){

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