/*
 * Copyright (C) 2020 Csaba Jakabos
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <list>

// Function to calculate 
// the linear extrapolation 
double extrapolate(cv::Point Point1,cv::Point Point2, double y) { 
    double x; 
    x = Point1.x 
        + (y - Point1.y) 
        / (Point2.y - Point1.y) 
        * (Point2.x - Point1.x); 
  
    return x; 
}

// Function to calculate 
// the radian conversion
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Function to simplify mask
// with dilate and erode
void simplifyMask(cv::Mat &mask) {
    int iterations = 4;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(mask, mask, kernel, cv::Point(-1,-1), iterations);
    cv::erode(mask, mask, kernel, cv::Point(-1,-1), iterations);
}

// Function to get another car on screen
void getKiwiCar(cv::Mat &image,cv::Mat &imagePrint, 
	int cutPosX, int cutPosY, 
	int &kiwiDistanceX, int &kiwiDistanceY) {

    // Crop, blur and hsv
    cv::Mat blurredImage;
    cv::Mat hsvImage;
    cv::GaussianBlur(imagePrint, blurredImage, cv::Size(11, 11), 0);
    cv::cvtColor(blurredImage, hsvImage, cv :: COLOR_BGR2HSV);
	
    //Red mask, lower and upper, as they span around 180
    cv::Mat maskRed;
    cv::Mat maskRed1;
    cv::Mat maskRed2;
    cv::inRange(hsvImage, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), maskRed1);
    cv::inRange(hsvImage, cv::Scalar(150, 100, 50), cv::Scalar(180, 255, 255), maskRed2);
	
    //Add masks together
    cv::bitwise_or(maskRed1, maskRed2, maskRed);

    // Black mask
    cv::Mat maskBlack;
    cv::inRange(hsvImage, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), maskBlack);
	
    // Combined red and black mask
    cv::Mat mask;
    cv::bitwise_or(maskRed, maskBlack, mask);
	
    // Threshold, erode and dilate
    simplifyMask(mask);
    cv::Mat thresh_gray;
    cv::morphologyEx(mask, thresh_gray, cv::MORPH_CLOSE, 
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));

    // Find biggest contour
    std::vector<std::vector<cv::Point>> kiwiContours;
    cv::findContours(thresh_gray, kiwiContours, 
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    //Size filter, if any, check biggest size
    if (kiwiContours.size() != 0) {

    	double maxArea = 0;
    	int maxAreaContourId = -1;
    	for (int j = 0; j < (int)kiwiContours.size(); j++) {
            double newArea = cv::contourArea(kiwiContours.at(j));
            if (newArea > maxArea) {
                maxArea = newArea;
                maxAreaContourId = j;
            } // End if
    	}

        // get bounding box and check size	
		cv::Rect boundKiwi = cv::boundingRect(kiwiContours[maxAreaContourId]);

		if (boundKiwi.height > 30 && boundKiwi.width > 60 && 
		        (float)boundKiwi.width/boundKiwi.height > 1.5f) {
		        kiwiDistanceX = cutPosX + boundKiwi.x - int(imagePrint.size().width/2);
		        kiwiDistanceY = int(imagePrint.size().height) - cutPosY - boundKiwi.y;
		
		        // if large enough draw rectangle
		        cv::rectangle(imagePrint, cvPoint(boundKiwi.x,boundKiwi.y), 
		                      cvPoint(boundKiwi.x+boundKiwi.width, 
		                              boundKiwi.y+boundKiwi.height), 
		                      cv::Scalar(0,255,0), -1);
		}
    }

    cv::imshow("imagePrint", imagePrint);
}

//Function for checking intersection scenario
void checkIntersection(cv::Mat &image, int &intersection, cv::Point &intersectionPoint)
{
    cv::Mat canvas_red = image.clone();
    
    
    // define mask area with box points
    cv::Mat canvas_red_mask = cv::Mat::zeros(image.size(), image.type()); 

    //1st box and draw
    std::vector<cv::Point> box_pnt;	
    std::vector<std::vector<cv::Point> > contours;   
    box_pnt.push_back(cv::Point(int(image.size().width/2.1) - 20,25));
    box_pnt.push_back(cv::Point(int(image.size().width/2.1) - 400,25));
    box_pnt.push_back(cv::Point(0, image.size().height));
    box_pnt.push_back(cv::Point(360, image.size().height));
    contours.push_back(box_pnt);
    
    cv::drawContours(canvas_red_mask, contours, 0,cv::Scalar(255,255,255), -1);
    
    //clear box   
    box_pnt.clear();	
    contours.clear();
    
    //2nd box and draw
    box_pnt.push_back(cv::Point(int(image.size().width - image.size().width/2.1) + 20,25));
    box_pnt.push_back(cv::Point(int(image.size().width - image.size().width/2.1) + 400,25));
    box_pnt.push_back(cv::Point(image.size().width - 0, image.size().height));
    box_pnt.push_back(cv::Point(image.size().width - 360, image.size().height));
    contours.push_back(box_pnt);
    
    cv::drawContours(canvas_red_mask, contours, 0,cv::Scalar(255,255,255), -1);
    
    //keep only shaped area and check for red cones
    cv::bitwise_and(canvas_red, canvas_red_mask, canvas_red);
    cv::cvtColor(canvas_red, canvas_red, cv :: COLOR_BGR2HSV);
    cv::inRange(canvas_red, cv::Scalar(150, 100, 20), cv::Scalar(180, 255, 255), canvas_red);
    morphologyEx(canvas_red,canvas_red, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));
    std::vector<std::vector<cv::Point>> contours2;

    //Get contours of red areas and count if they exist on both sides
    cv::findContours(canvas_red, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
    int counterLHS = 0;
    int counterRHS = 0;
    if (contours2.size() > 3) {
        std::vector<std::vector<cv::Point>> contours_poly(contours2.size());
        std::vector<cv::Rect> boundRect(contours2.size());
        std::vector<cv::Point> boundRectPoints;
        for(unsigned int i = 0; i < contours2.size(); i++) {
    
            cv::approxPolyDP(contours2[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(contours_poly[i]);
            boundRectPoints.push_back(boundRect[i].br());                   
            boundRectPoints.push_back(boundRect[i].tl()); 
    		
    		if (boundRect[i].br().x < 580 && boundRect[i].br().x > 150) {
    		  counterLHS = counterLHS + 1;
    		}
    		
    		if (boundRect[i].br().x > 700 && boundRect[i].br().x < 900) {
    		  counterRHS = counterRHS + 1;
    		}
    
    	}
    	
    	cv::Rect boundRect2;
    	boundRect2 = boundingRect(boundRectPoints);
    	cv::rectangle(canvas_red, boundRect2.tl(), boundRect2.br(), cv::Scalar(180, 255, 255), 2);
    
    
    	cv::Point center_of_rect = (boundRect2.br() + boundRect2.tl()) * 0.5;
    	intersectionPoint = center_of_rect;    

    	// if there is red objects both sides it is intersection and send
    	// rectangular coordinates
    	if (counterLHS > 0 && counterRHS > 0) {
    		intersection = 1;
    	}
                   
    
    } //endif
}

//Get track lines
void getTrackLines(cv::Mat &cones, cv::Mat &croppedImage_sizefilter, 
                   cv::Point &near, cv::Point &far, 
                   std::vector<cv::Point> &pointList, int side) {				

    double refPoint = 150.0;
    double refPoint2 = 190.0;
    int counter{0};
    int counter2{0};
				
    /// Find contours https://stackoverflow.com/questions/55252296/why-cvfindcontours-return-so-many-contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cones, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    
    
    // Added bounding as a temp variable instead
    cv::Rect boundRectCones;
    
    
    /// Get the moments and plot points as in openCV official documentation
    /// https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
    /// And bounging boxes from:
    /// https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    ///  Get the moments:
    std::vector<cv::Moments> muCones(contours.size());
    ///  Get the mass centers:
    std::vector<cv::Point2f> mcCones(contours.size());
    std::vector<cv::Point2f> mcCones_sizefilter(contours.size()); 
	
	
    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros(cones.size(), CV_8UC3);
    
    
    cv::Scalar colorBoxCones = cv::Scalar(0, 0, 0);
    cv::Scalar colorBoxCones_center = cv::Scalar(0, 0, 0);
    
    
    colorBoxCones = cv::Scalar(94,206,165,255);//Scalar(94,206,165,255)
    colorBoxCones_center = cv::Scalar(0,0,255,255);//Scalar(94,206,165,255)
      
    
    cv::Point2f mcCones_temp;
    std::vector<cv::Point2f> mcCones_temp_vector;
    
    //Draw boxes at center points of contours based on their size
    for(unsigned int i = 0; i < contours.size(); i++) { 
    	muCones[i] = moments(contours[i], false); 
        mcCones[i] = cv::Point2d(muCones[i].m10/muCones[i].m00 , muCones[i].m01/muCones[i].m00);
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        
        boundRectCones = cv::boundingRect(contours_poly[i]);
        //cv::rectangle(croppedImage, cvPoint(boundRectCones.x,boundRectCones.y), cvPoint(boundRectCones.x+boundRectCones.width, boundRectCones.y+boundRectCones.height), colorBoxCones, 2);
        //cv::circle(croppedImage, mcCones[i], 4, colorBoxCones_center, -1, 8, 0);
        
        // Checking bounding box size, if it is too large or small or ratio is strange remove it
        // take only those mcCones mass centers which are relevant
        if (boundRectCones.height > 20 && boundRectCones.height < 100 && 
    	    boundRectCones.width > 6 && boundRectCones.width < 70 && 
    		boundRectCones.width / boundRectCones.height < 0.6  && 
    		boundRectCones.height / boundRectCones.width < 3) {
    		
            cv::rectangle(croppedImage_sizefilter, cvPoint(boundRectCones.x,boundRectCones.y), 
    					  cvPoint(boundRectCones.x+boundRectCones.width, 
    					          boundRectCones.y+boundRectCones.height), 
    					  colorBoxCones, 2);
    					  
            mcCones_sizefilter.push_back(mcCones[i]);
        	
            // take only those mcCones mass centers which are relevant
            mcCones_temp = mcCones[i];
            mcCones_temp_vector.push_back(mcCones_temp);
            //std::cout << "The points are" << mcCones_temp << std::endl;
            cv::circle(croppedImage_sizefilter, mcCones_temp, 4, colorBoxCones_center, -1, 8, 0);
    
        }
    
    }
    
    int lineWidth = 6;
    
    //Draw lines at relevant points
    
    if (side == 0) {
    	pointList.push_back(cv::Point(0,370));
    }

    if (side == 1) {
    	pointList.push_back(cv::Point(1280,370));
    }
    
    for(unsigned int i = 0; i<mcCones_temp_vector.size(); i++) {
        //pt.push_back(mcCones.at(i));
        pointList.push_back(mcCones_temp_vector.at(i));
    }

    if (side == 0) {
		cv::polylines(croppedImage_sizefilter, pointList, false, cv::Scalar(94,206,165,255), lineWidth);
    	if (pointList.size() < 2) {
    		pointList.push_back(cv::Point(320,0));
    	}
    }

    if (side == 1) {
		cv::polylines(croppedImage_sizefilter, pointList, false, cv::Scalar(94,206,165,255), lineWidth);
    	if (pointList.size() < 2) {
    		pointList.push_back(cv::Point(960,0));
    	}
    }
    
    
    near = cv::Point(640.0,(unsigned int)refPoint2);
    far = cv::Point(640.0,(unsigned int)refPoint);
    
    if (pointList.size() >= 2) {
        for(unsigned int i = 1; i < pointList.size(); i++) { 
            if (pointList[i-1].y > refPoint && pointList[i].y < refPoint) {
    		
            	//std::cout << "itt a pont: " << refPoint << std::endl;
            	counter = counter + 1;
    			// Finding the extrapolation 
    			//std::cout << "Value of x at y = 30 : "
         		//<< extrapolate(pointList[i], pointList[i - 1], refPoint) << std::endl;
         		far.x = (unsigned int)extrapolate(pointList[i], pointList[i - 1], refPoint);
    		
    		}
    		
    		
            if (pointList[i-1].y > refPoint2 && pointList[i].y < refPoint2) {
    		
            	//std::cout << "itt a pont: " << refPoint2 << std::endl;
            	counter = counter2 + 1;
    			// Finding the extrapolation 
    			//std::cout << "Value of x at y = 30 : "
         		//<< extrapolate(pointList[i], pointList[i - 1], refPoint2) << std::endl;
         		near.x = (unsigned int)extrapolate(pointList[i], pointList[i - 1], refPoint2);
    		
    	    }
        }
    	
        if (counter == 0) {
    		// Finding the extrapolation 
    		//std::cout << "Value of x at y = 30 : "
         	//<< extrapolate(pointList[pointList.size() - 2], pointList[pointList.size() - 1], refPoint) << std::endl;
    
    		far.x = (unsigned int)extrapolate(pointList[pointList.size() - 2], pointList[pointList.size() - 1], refPoint);
        }
        if (counter == 0) {
    		// Finding the extrapolation 
    		//std::cout << "Value of x at y = 30 : "
         	//<< extrapolate(pointList[pointList.size() - 2], pointList[pointList.size() - 1], refPoint2) << std::endl;
    
    		near.x = (unsigned int)extrapolate(pointList[pointList.size() - 2], pointList[pointList.size() - 1], refPoint2);
        }
    }
    
}


// MAIN
int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height"))) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex distancesMutex;
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env) {
                auto senderStamp = env.senderStamp();
                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

                // Store distance readings.
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // TODO: Do something with the frame.

                // Invert colors
                //cv::bitwise_not(img, img);

                // Draw a red rectangle
                //cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));	
                
                // Cut off some part of screen
                //X,Y, Width, Height - X,Y start of cut and Widht Height size of cut from there
                cv::Rect myROI(0, 350, 1280, 370);
                cv::Mat croppedImage = img(myROI);
                cv::rectangle(croppedImage, cv::Point(320,370), cv::Point(960,200), cv::Scalar(255, 255, 255), -1);
				
                // Check if this cloning is reducing performance, used it to not plot on original image, croppedImage is still connected to that
                cv::Mat croppedImage_sizefilter = croppedImage.clone();         

                //Gaussian blur to remove high frequency noise
                cv::Mat blurred;
                cv::GaussianBlur(croppedImage_sizefilter, blurred, cv::Size(11, 11), 0);

                //Color filter as in lecture
                cv::Mat hsv;
                cv::cvtColor(blurred, hsv, cv :: COLOR_BGR2HSV);
                cv::Scalar hsvLow(110, 50, 50); // Note : H [0,180], S [0,255], V [0, 255]
                cv::Scalar hsvHi(130, 255, 255);
                cv::Scalar hsvLow_yellow(15, 0, 0);
                cv::Scalar hsvHi_yellow(36, 255, 255);
                cv::Mat blueCones;
                cv::Mat yellowCones;
                cv::inRange(hsv, hsvLow, hsvHi, blueCones);
                cv::inRange(hsv, hsvLow_yellow, hsvHi_yellow, yellowCones);
                
                // check threshold here: https://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html
                cv::threshold(blueCones, blueCones, 0, 255, cv::THRESH_BINARY);
                cv::threshold(yellowCones, yellowCones, 0, 255, cv::THRESH_BINARY);

                simplifyMask(yellowCones);
                simplifyMask(blueCones);


                std::vector<cv::Point> pointListBlue;
                std::vector<cv::Point> pointListYellow;
                
                cv::Point nearBlue = cv::Point(640.0, 0.0);
                cv::Point nearYellow = cv::Point(640.0, 0.0);
                cv::Point farBlue = cv::Point(640.0, 0.0);
                cv::Point farYellow = cv::Point(640.0, 0.0);
                
                getTrackLines(yellowCones, croppedImage_sizefilter, nearYellow, farYellow, pointListYellow, 0);
                getTrackLines(blueCones, croppedImage_sizefilter, nearBlue, farBlue, pointListBlue, 1);
                
                
                
                // Reverse the vector 
                std::reverse(pointListYellow.begin(), pointListYellow.end()); 
                
                std::vector<cv::Point> AB;
                AB.reserve(pointListBlue.size() + pointListYellow.size()); // preallocate memory
                AB.insert(AB.end(), pointListBlue.begin(), pointListBlue.end());
                AB.insert(AB.end(), pointListYellow.begin(), pointListYellow.end());
                
                //std::cout << "AB size: " << AB.size() << std::endl;
                std::vector<std::vector<cv::Point> > fillContAll;
                fillContAll.push_back(AB);
                
                //cv::Mat mask = cv::Mat::zeros(croppedImage_sizefilter.size(), croppedImage_sizefilter.type()); 
                //cv::Mat mask(croppedImage_sizefilter.size(),croppedImage_sizefilter.type(), cv::Scalar(255));
                cv::Mat mask = cv::Mat::zeros(croppedImage_sizefilter.size(), CV_8UC1); 
                
                
                cv::Scalar mask_value = cv::Scalar(255);
                if (AB.size() > 2) {
                cv::fillPoly(mask,
                		fillContAll,
                		mask_value);
                
                }
                
                //Now you can copy your source image to destination image with masking
                //https://stackoverflow.com/questions/31554187/opencv-binary-image-mask-for-image-analysis-in-c
                // create dst with background color of your choice
                
                cv::Mat maskedImage = cv::Mat(croppedImage_sizefilter.size(), CV_8UC3);
                
                // Do one of the two following lines:
                maskedImage.setTo(cv::Scalar(255, 0, 0));  // Set all pixels to (180, 180, 180)
                croppedImage_sizefilter.copyTo(maskedImage, mask);  // Copy pixels within contour to
                
                
                cv::circle(maskedImage, nearBlue,10, cv::Scalar(255, 255, 255),CV_FILLED, 8,0);
                cv::circle(maskedImage, farBlue,10, cv::Scalar(255, 255, 255),CV_FILLED, 8,0);
                cv::circle(maskedImage, nearYellow,10, cv::Scalar(255, 255, 255),CV_FILLED, 8,0);
                cv::circle(maskedImage, farYellow,10, cv::Scalar(255, 255, 255),CV_FILLED, 8,0);
                
                
                cv::Point farPoint = cv::Point((unsigned int)((farBlue.x + farYellow.x)/2), farBlue.y);
                cv::Point nearPoint = cv::Point((unsigned int)((nearBlue.x + nearYellow.x)/2), nearBlue.y);
                cv::circle(maskedImage, farPoint,5, cv::Scalar(255,0,0),CV_FILLED, 8,0);
                cv::circle(maskedImage, nearPoint,5, cv::Scalar(255,0,0),CV_FILLED, 8,0);
                
                
                int intersection = 0;
                cv::Point intersectionPoint;
                checkIntersection(img, intersection, intersectionPoint); 
                
                if (intersection == 1) {
                
                   farPoint = intersectionPoint;
                   nearPoint = intersectionPoint;
                
                
                }
                
                // 
                int kiwiDistanceX = 6666;
                int kiwiDistanceY = 6666;
                int cutPosX = 0;
                int cutPosY = 0;
                
                if (intersection == 0) {
                	getKiwiCar(croppedImage, maskedImage, cutPosX, cutPosY, kiwiDistanceX, kiwiDistanceY); 
                }
                
                if (intersection == 1) {
                	getKiwiCar(img, img, cutPosX, cutPosY, kiwiDistanceX, kiwiDistanceY); 
                }
                
                std::cout << "kiwiDistanceX = " << kiwiDistanceX << std::endl;
                std::cout << "kiwiDistanceY = " << kiwiDistanceY << std::endl;
                std::cout << "intersection = " << intersection << std::endl;
                
                // Calculate near and far angle
                double fovy = 48.8;
                double fovx = (maskedImage.size().width / (maskedImage.size().height + 400.0)) * fovy;
                double farAngle = deg2rad((float)(((float)farPoint.x - (float)maskedImage.size().width / 2.0f) /(float)maskedImage.size().width) * (float)fovx);
                double nearAngle = deg2rad((float)(((float)nearPoint.x - (float)maskedImage.size().width / 2.0f) / (float)maskedImage.size().width) * (float)fovx);
                
                
                if (nearAngle > 1.5 || nearAngle < -1.5) {
                	nearAngle = 0;
                }
                if (farAngle > 1.5 || farAngle < -1.5) {
                	farAngle = 0;
                }   
                
                
                ////// 3 Picture printing Section
                
                /// Show picture as in lecture
                if (VERBOSE) {
                    //cv::imshow(sharedMemory->name().c_str(), img);
                    //Added imshow here instead
                    //cv::imshow("Processed", blueCones);
                    //cv::imshow("Blue cones", blueCones);
                    //cv::imshow("Yellows cones", yellowCones);
                    //cv::imshow("Blue cones2", blueCones2);
                    //cv::imshow("Yellows cones2", yellowCones2);
                    //cv::imshow("Blue cones3", blueCones3);
                    //cv::imshow("Yellows cones3", yellowCones3);
                    //cv::imshow("canny yellow", cannyYellow);
                    //cv::imshow("Dilate", dilate);
                    //cv::imshow("Erode", erode);
                    //cv::imshow("Drawing", drawing);
                    //cv::imshow("croppedImage_sizefilter", croppedImage_sizefilter);
                    //cv::imshow("mask", mask);
                    //cv::imshow("croppedImage", croppedImage);
                    //cv::imshow("maskedImage", maskedImage);
                    //cv::imshow("intersectionCheck", intersectionCheck);
                    //cv::imshow("Drawing2", drawing2);
                    //imshow("Contours", contours);
                    //imshow("Blurred", blurred);
                    //cv::imshow("Canny edges", canny);
                    //cv::imshow("Hough", hough);
                    //cv::imshow("Contours", contours);
                    //cv::imshow("Cropped image", croppedImage); 
                    //cv::imshow("Cropped image", croppedImage); 
                    cv::waitKey(1);
                }



                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                //{
                //    std::lock_guard<std::mutex> lck(distancesMutex);
                //    std::cout << "front = " << front << ", "
                //              << "left = " << left << ", "
                //              << "right = " << right << "." << std::endl;
                //}

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.
                //opendlv::proxy::AngleReading ar;
                //ar.angle(123.45f);
                //od4.send(ar);

                // Python 1134 Distance reading in X and Y
                opendlv::logic::perception::ObjectDistance od;
                od.distance((float)kiwiDistanceY);
                od4.send(od,cluon::time::now(), 0);
                od.distance((float)kiwiDistanceX);
                od4.send(od,cluon::time::now(), 1);

                // Python 1130, sender stamp 1 is intersection, 0 is not
                opendlv::logic::perception::Object ob;
                ob.objectId(intersection);
                od4.send(ob,cluon::time::now(), 0);

                // Python 1171, aim direction, near 0, far 1 stample
                opendlv::logic::action::AimDirection ad;
                ad.azimuthAngle((float)nearAngle);
                od4.send(ad,cluon::time::now(), 0);
                ad.azimuthAngle((float)farAngle);
                od4.send(ad,cluon::time::now(), 1);

                ////////////////////////////////////////////////////////////////
                // Steering and acceleration/decelration.
                //
                // Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
                // Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
                //opendlv::proxy::GroundSteeringRequest gsr;
                //gsr.groundSteering(0);
                //od4.send(gsr);

                // Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
                // Be careful!
                //opendlv::proxy::PedalPositionRequest ppr;
                //ppr.position(0);
                //od4.send(ppr);
            }
        }
        retCode = 0;
    }
    return retCode;
}

