#include "Scan3dApp.h"

//--------------------------------------------------------------
void Scan3dApp::setup(){
    //Initializing sobel kernels
    sobelHorizontal[0][0] = -1; sobelHorizontal[0][1] = 0; sobelHorizontal[0][2] = 1;
    sobelHorizontal[1][0] = -2; sobelHorizontal[1][1] = 0; sobelHorizontal[1][2] = 2;
    sobelHorizontal[2][0] = -1; sobelHorizontal[2][1] = 0; sobelHorizontal[2][2] = 1;

    sobelVertical[0][0] =  1; sobelVertical[0][1] =  2; sobelVertical[0][2] = 1;
    sobelVertical[1][0] =  0; sobelVertical[1][1] =  0; sobelVertical[1][2] = 0;
    sobelVertical[2][0] = -1; sobelVertical[2][1] = -2; sobelVertical[2][2] = -1;



	loadSettings();

    displayState = COLOR;
    ofBackground(0);
    ofSetWindowTitle("3D SCAN ALL THE THINGS");
    ofSetWindowShape(1280,720);
    ofSetFrameRate(30);

    vid.loadMovie("/Users/tylerparker/Dropbox/Projects/Active/3D Scanning/oF/apps/myApps/Scan3D/bin/data/monkey_scan_2.mov");//inputVideoFile);
    vid.play();
    vid.update(); //to get height and width to load
    currentColorFrame.allocate(vid.getWidth(),vid.getHeight());
    previousColorFrame.allocate(vid.getWidth(),vid.getHeight());

    currentGrayscaleFrame.allocate(vid.getWidth(),vid.getHeight());
    previousGrayscaleFrame.allocate(vid.getWidth(),vid.getHeight());
    
    diffFrame.allocate(vid.getWidth(),vid.getHeight());
    
    
    //threshImage = computeThresholdImage(diffImage);

}

/**
void computeThresholdImage(){
    unsigned char * threshPixels;
    threshPixels = new unsigned char[img.width * img.height *3 ];
     = threshImages[i].getPixels();
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
    edge.clear();
    thre
}
**/

//--------------------------------------------------------------
void Scan3dApp::loadSettings(){
	/* Load settings file */
	if(settings.loadFile("settings.xml")){
		cout << "** Loading Settings File **" << endl;
		settings.pushTag("settings");

			inputVideoFile = settings.getValue("inputVideoFile","");
			cout << "   Loaded image directory: " << inputVideoFile << endl;

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
    vid.update();

    if(vid.isFrameNew()){

        previousColorFrame = currentColorFrame;

        currentColorFrame.setFromPixels(vid.getPixels(),vid.getWidth(),vid.getHeight());

        
        previousGrayscaleFrame = previousColorFrame;
        
        
        currentGrayscaleFrame = currentColorFrame;

    
        diffFrame = currentGrayscaleFrame;
        diffFrame -= previousGrayscaleFrame;  

    }

}

//--------------------------------------------------------------
void Scan3dApp::draw(){
    
    // draw the image sequence at the new frame count
    //ofSetColor(255);
    switch(displayState){
        case COLOR:
            currentColorFrame.draw(0, 0);
            break;
        case GRAYSCALE:
            currentGrayscaleFrame.draw(0, 0);
            break;
        case DIFF:
            diffFrame.draw(0, 0);
            break;
        /**
        case THRESH:
            threshFrame.draw(0, 0);
            break;
        case EDGE:
            edgeFrame.draw(0, 0);
            break;
        case CORNER:
            cornerMap.draw(0, 0);
            break;
        **/
        default:
            currentColorFrame.draw(0, 0);
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
        /**
        case 52:
            displayState = THRESH;
            break;
        case 53:
            displayState = EDGE;
            break;
        case 54:
            displayState = CORNER;
            break;
        **/
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



//--------------------------------------------------------------
ofxCvGrayscaleImage Scan3dApp::computeGradientImage(ofxCvGrayscaleImage &input, int direction){
    int sum, sumX,sumY;
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
    delete outputPixelData;
    return out;
}
