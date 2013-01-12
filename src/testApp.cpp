#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	loadSettings();
}

//--------------------------------------------------------------
void testApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "*** Loading settings file ***" << endl;
		settings.pushTag("settings");

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
		cout << "*****************************" << endl;
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