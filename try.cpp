/*******************************************************************************
*   Copyright 2013-2014 EPFL                                                   *
*   Copyright 2013-2014 Quentin Bonnard                                        *
*                                                                              *
*   This file is part of chilitags.                                            *
*                                                                              *
*   Chilitags is free software: you can redistribute it and/or modify          *
*   it under the terms of the Lesser GNU General Public License as             *
*   published by the Free Software Foundation, either version 3 of the         *
*   License, or (at your option) any later version.                            *
*                                                                              *
*   Chilitags is distributed in the hope that it will be useful,               *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*   GNU Lesser General Public License for more details.                        *
*                                                                              *
*   You should have received a copy of the GNU Lesser General Public License   *
*   along with Chilitags.  If not, see <http://www.gnu.org/licenses/>.         *
*******************************************************************************/
#include <math.h> 
#include <vector>
#include <typeinfo>
#include <iostream>

#include <chilitags.hpp>

#include <opencv2/core/core.hpp> // for cv::Mat
#include <opencv2/highgui/highgui.hpp> // for camera capture

using namespace std;
using namespace cv;

double distance_between_tags = 50;
struct point 
{
    double x,y;
};

double norm (point p); // get the norm of a vector

point landmarks[3];
point finalPose;

point trilateration(point point1, point point2, point point3, double r1, double r2, double r3) ;

int main(int argc, char* argv[])
{
    cout
        << "Usage: "<< argv[0]
        << " [-c <tag configuration (YAML)>] [-i <camera calibration (YAML)>] [-d <int>] \n";

    const char* intrinsicsFilename = 0;
    const char* configFilename = 0;

    for( int i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i], "-c") == 0 )
            configFilename = argv[++i];
        else if( strcmp(argv[i], "-i") == 0 )
            intrinsicsFilename = argv[++i];
        else if( strcmp(argv[i], "-d") == 0 )
            distance_between_tags = atoi(argv[++i]);
    }

    /*****************************/
    /*    Init camera capture    */
    /*****************************/
    int cameraIndex = 0;
    cv::VideoCapture capture(cameraIndex);
    if (!capture.isOpened())
    {
        cerr << "Unable to initialise video capture.\n";
        return 1;
    }

    /******************************/
    /* Setting up pose estimation */
    /******************************/
#ifdef OPENCV3
    float inputWidth  = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    float inputHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
#else
    float inputWidth  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    float inputHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
#endif

    chilitags::Chilitags3D chilitags3D(Size(inputWidth, inputHeight));

    if (configFilename) chilitags3D.readTagConfiguration(configFilename);

    if (intrinsicsFilename) {
        Size calibratedImageSize = chilitags3D.readCalibration(intrinsicsFilename);
#ifdef OPENCV3
        capture.set(cv::CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
#else
        capture.set(CV_CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
#endif
    }

    /*****************************/
    /*             Go!           */
    /*****************************/
    cout << "May the force be with us.\n\n";
    cout << "Looking for tags...\n";
    cv::Mat inputImage;

    //TopLeft, TopRight, BottomRight, BottomLeft
    float distances [] = { -1,-1,-1, -1 };
    int counts [] = { 0, 0, 0, 0};

    int triangulation_ok = 1;
    int firstTime []= {0, 0, 0, 0};

    for (; 'q' != (char) cv::waitKey(10); ) {
        capture.read(inputImage);
        
        for (auto& kv : chilitags3D.estimate(inputImage)) {
            
            if (kv.first == "Tag1(TopLeft)"){
                if( firstTime[0] == 0 ){
                    cout << "Detected "<<kv.first <<"\n";
                }
                firstTime[0]++;

                counts[0]++;
                counts[1] = counts[2] = counts[3] = 0;
                if(counts[0] <= 10){
                    distances[0] = Mat(kv.second).at<float>(2,3);
                }  
            }
            else if (kv.first == "Tag2(TopRight)"){
                if( firstTime[1] == 0 ){
                    cout << "Detected "<<kv.first <<"\n";
                }
                firstTime[1]++;

                counts[1]++;
                counts[0] = counts[2] = counts[3] = 0;
                if(counts[1] <= 10){
                    distances[1] = Mat(kv.second).at<float>(2,3);
                } 
            }
            else if (kv.first == "Tag3(BottomRight)"){
                if( firstTime[3] == 0 ){
                    cout << "Detected "<<kv.first <<"\n";
                }
                firstTime[3]++;

                counts[2]++;
                counts[0] = counts[1] = counts[3] = 0;
                if(counts[2] <= 10){
                    distances[2] = Mat(kv.second).at<float>(2,3);
                } 
            }
            else if (kv.first == "Tag4(BottomLeft)"){
                if( firstTime[3] == 0 ){
                    cout << "Detected "<<kv.first <<"\n";
                }
                firstTime[3]++;

                counts[3]++;
                counts[0] = counts[1] = counts[2] = 0;
                if(counts[3] <= 10){
                    distances[3] = Mat(kv.second).at<float>(2,3);
                }   
            }
        }
        triangulation_ok = 0;
        for(int iter = 0; iter<4; iter++){
            if(distances[iter] != -1){
                triangulation_ok++;
            }
        }

        if(triangulation_ok>=3){
            break;
        }
    }

    cout<< "\n";
    float final_array[] = {0,0,0};
    int j = 0; 
    for (int iter = 0; iter<4; iter++){
        if(distances[iter] != -1){
            final_array[j] = distances[iter];
            j++;
            switch(iter){
                case 0:
                    cout << "Distance from TopLeft is - " << distances[iter] << "\n";
                    break;
                case 1:
                    cout << "Distance from TopRight is - " << distances[iter] << "\n";
                    break;
                case 2:
                    cout << "Distance from BottomRight is - " << distances[iter] << "\n";
                    break;
                case 3:
                    cout << "Distance from BottomLeft is - " << distances[iter] << "\n";    
                    break;
            }
        }
    }

    
    int landmark_iter = 0; 
    if(triangulation_ok>=3){
        cout << "\n" << "Finding location...";
        
        for(int y=0; y<4; y++){
            if(distances[y] == -1){
                continue;
            }
            if(landmark_iter < 3){
                if(y == 0){
                    point temp = {0, distance_between_tags};
                    landmarks[landmark_iter] =  temp;
                }
                else if(y == 1){
                    point temp = {distance_between_tags, distance_between_tags};
                    landmarks[landmark_iter] =  temp;
                }
                else if(y == 2){            
                    point temp = {distance_between_tags, 0};
                    landmarks[landmark_iter] =  temp;
                }
                else if(y == 3){
                    point temp = {0, 0};
                    landmarks[landmark_iter] =  temp;
                }
            }
            landmark_iter++;
        }
        
        finalPose = trilateration(landmarks[0],landmarks[1],landmarks[2],final_array[0],final_array[1],final_array[2]);
        cout<<"\n"<<"Im at "<<finalPose.x<<" , "<<finalPose.y<<endl;
    }
    else{
        cout << "\n" <<"Sorry.. I was not able to find out where I am. Sad.";
    }

    capture.release();

    return 0;
}

//trilateration

double norm (point p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}

point trilateration(point point1, point point2, point point3, double r1, double r2, double r3) {
    point resultPose;
    //unit vector in a direction from point1 to point 2
    double p2p1Distance = pow(pow(point2.x-point1.x,2) + pow(point2.y-   point1.y,2),0.5);
    point ex = {(point2.x-point1.x)/p2p1Distance, (point2.y-point1.y)/p2p1Distance};
    point aux = {point3.x-point1.x,point3.y-point1.y};
    //signed magnitude of the x component
    double i = ex.x * aux.x + ex.y * aux.y;
    //the unit vector in the y direction. 
    point aux2 = { point3.x-point1.x-i*ex.x, point3.y-point1.y-i*ex.y};
    point ey = { aux2.x / norm (aux2), aux2.y / norm (aux2) };
    //the signed magnitude of the y component
    double j = ey.x * aux.x + ey.y * aux.y;
    //coordinates
    double x = (pow(r1,2) - pow(r2,2) + pow(p2p1Distance,2))/ (2 * p2p1Distance);
    double y = (pow(r1,2) - pow(r3,2) + pow(i,2) + pow(j,2))/(2*j) - i*x/j;
    //result coordinates
    double finalX = point1.x+ x*ex.x + y*ey.x;
    double finalY = point1.y+ x*ex.y + y*ey.y;
    resultPose.x = finalX;
    resultPose.y = finalY;

    // We dont want to be overly accurate
    if(finalPose.x < 0.00001 && finalPose.x > -0.00001){
        finalPose.x = 0;    
    }
    if(finalPose.y < 0.00001 && finalPose.y > -0.00001){
        finalPose.y = 0;
    }
    return resultPose;
}