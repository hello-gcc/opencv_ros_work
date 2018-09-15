/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/opencv.hpp>  
#include <iostream>  
#include <stdio.h>

using namespace cv;
using namespace std;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}




static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

int stereo_match(IplImage * left_image, IplImage * right_image, CvMat *Q_input)
{
    #if 0
    std::string img1_filename = "";
    std::string img2_filename = "";
    std::string intrinsic_filename = "";
    std::string extrinsic_filename = "";
    std::string disparity_filename = "";
    std::string point_cloud_filename = "";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    cv::CommandLineParser parser(argc, argv,
        "{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{scale|1|}{i||}{e||}{o||}{p||}");
    if(parser.has("help"))
    {
        print_help();
        return 0;
    }
    img1_filename = parser.get<std::string>(0);
    img2_filename = parser.get<std::string>(1);
    if (parser.has("algorithm"))
    {
        std::string _alg = parser.get<std::string>("algorithm");
        alg = _alg == "bm" ? STEREO_BM :
            _alg == "sgbm" ? STEREO_SGBM :
            _alg == "hh" ? STEREO_HH :
            _alg == "var" ? STEREO_VAR :
            _alg == "sgbm3way" ? STEREO_3WAY : -1;
    }
    numberOfDisparities = parser.get<int>("max-disparity");
    SADWindowSize = parser.get<int>("blocksize");
    scale = parser.get<float>("scale");
    no_display = parser.has("no-display");
    if( parser.has("i") )
        intrinsic_filename = parser.get<std::string>("i");
    if( parser.has("e") )
        extrinsic_filename = parser.get<std::string>("e");
    if( parser.has("o") )
        disparity_filename = parser.get<std::string>("o");
    if( parser.has("p") )
        point_cloud_filename = parser.get<std::string>("p");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help();
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        print_help();
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
    if( img1_filename.empty() || img2_filename.empty() )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    int color_mode = alg == STEREO_BM ? 0 : -1;
    Mat img1 = imread(img1_filename, color_mode);
    Mat img2 = imread(img2_filename, color_mode);

    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;

    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
    }
    #endif
    Mat img1 = cvarrToMat(left_image);
    Mat img2 = cvarrToMat(right_image);
    Mat Q = cvarrToMat(Q_input);
    Size img_size = img1.size();
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;

    int SADWindowSize = 11;
    int numberOfDisparities = 32;
    Rect roi1, roi2;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);

    string disparity_filename = "";
    string point_cloud_filename = "";

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( 1 )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        //printf("press any key to continue...");
        fflush(stdout);
        //waitKey();
        //printf("\n");
    }

    if(!disparity_filename.empty())
        imwrite(disparity_filename, disp8);

    if(!point_cloud_filename.empty())
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename.c_str(), xyz);
        printf("\n");
    }

    return 0;
}

int main(int argc, char** argv)
{  
    //cout << "测试两个摄像头同时读取数据" << endl;  

    //读取内部参数
    CvMat *Intrinsics_Camera_Left = (CvMat *)cvLoad("./calibrate/Intrinsics_Camera_Left.xml");
    CvMat *Intrinsics_Camera_Right = (CvMat *)cvLoad("./calibrate/Intrinsics_Camera_Right.xml");
    CvMat *Distortion_Camera_Left = (CvMat *)cvLoad("./calibrate/Distortion_Camera_Left.xml");
    CvMat *Distortion_Camera_Right = (CvMat *)cvLoad("./calibrate/Distortion_Camera_Right.xml");
    CvMat *Translation_matlab = (CvMat *)cvLoad("./calibrate/Translation.xml");
    CvMat *RotRodrigues_matlab = (CvMat *)cvLoad("./calibrate/RotRodrigues.xml");
    CvMat *R_opencv = cvCreateMat(3, 3, CV_64F);
    cvRodrigues2(RotRodrigues_matlab, R_opencv);

    //创建映射阵
    IplImage *Left_Mapx = cvCreateImage(cvSize(640,480), IPL_DEPTH_32F, 1);
    IplImage *Left_Mapy = cvCreateImage(cvSize(640,480), IPL_DEPTH_32F, 1);
    IplImage *Right_Mapx = cvCreateImage(cvSize(640,480), IPL_DEPTH_32F, 1);
    IplImage *Right_Mapy = cvCreateImage(cvSize(640,480), IPL_DEPTH_32F, 1);
    CvMat *Rl = cvCreateMat(3, 3, CV_64F);
    CvMat *Rr = cvCreateMat(3, 3, CV_64F);
    CvMat *Pl = cvCreateMat(3, 4, CV_64F);
    CvMat *Pr = cvCreateMat(3, 4, CV_64F);
    CvMat *Q =  cvCreateMat(4, 4, CV_64F);
    //cvStereoRectify(Intrinsics_Camera_Left, Intrinsics_Camera_Right, \
                    Distortion_Camera_Left, Distortion_Camera_Right, \
                    cvSize(640, 480), R_opencv, Translation_matlab,  \
                    Rl, Rr, Pl, Pr);
    cvStereoRectify(Intrinsics_Camera_Left, Intrinsics_Camera_Right, \
        Distortion_Camera_Left, Distortion_Camera_Right, \
        cvSize(640, 480), R_opencv, Translation_matlab,  \
        Rl, Rr, Pl, Pr, Q, 1024, 0);//增加图像缩放，去除死区
    cvInitUndistortRectifyMap(Intrinsics_Camera_Left, Distortion_Camera_Left, Rl, Pl, \
                              Left_Mapx, Left_Mapy);
    cvInitUndistortRectifyMap(Intrinsics_Camera_Right, Distortion_Camera_Right, Rr, Pr, \
                              Right_Mapx, Right_Mapy);

    CvCapture* cap_left;  
    CvCapture* cap_right;  

    IplImage *img0 , *img1;  
    IplImage *img_left, *img_right;  
    IplImage *img_left_Change, *img_right_Change;  
    cvNamedWindow("camera_left");  
    cvNamedWindow("camera_right");  

    cap_left = cvCreateCameraCapture(3);  
    assert(cap_left != NULL);  
    cap_right = cvCreateCameraCapture(2);  
    assert(cap_right != NULL);  

    while(1)  
    {  
        cvGrabFrame(cap_left);
        cvGrabFrame(cap_right);

        img0 = cvRetrieveFrame(cap_left);
        img1 = cvRetrieveFrame(cap_right);

        if( !img0 || !img1)
        {  
            cout << "camera0 error" << endl;  
            break;  
        }  

        img_left = cvCloneImage(img0);
        img_right = cvCloneImage(img1);
        img_left_Change = cvCloneImage(img0);
        img_right_Change = cvCloneImage(img1);
        cvRemap(img_left, img_left_Change, Left_Mapx, Left_Mapy);
        cvRemap(img_right, img_right_Change, Right_Mapx, Right_Mapy);

        cvLine(img_left_Change, cvPoint(0,48), cvPoint(640-1, 48), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*2), cvPoint(640-1, 48*2), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*3), cvPoint(640-1, 48*3), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*4), cvPoint(640-1, 48*4), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*5), cvPoint(640-1, 48*5), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*6), cvPoint(640-1, 48*6), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*7), cvPoint(640-1, 48*7), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*8), cvPoint(640-1, 48*8), cvScalar(255, 0, 0));
        cvLine(img_left_Change, cvPoint(0,48*9), cvPoint(640-1, 48*9), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48), cvPoint(640-1, 48), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*2), cvPoint(640-1, 48*2), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*3), cvPoint(640-1, 48*3), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*4), cvPoint(640-1, 48*4), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*5), cvPoint(640-1, 48*5), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*6), cvPoint(640-1, 48*6), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*7), cvPoint(640-1, 48*7), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*8), cvPoint(640-1, 48*8), cvScalar(255, 0, 0));
        cvLine(img_right_Change, cvPoint(0,48*9), cvPoint(640-1, 48*9), cvScalar(255, 0, 0));

        cvShowImage("camera_left", img_left);  
        cvShowImage("camera_right", img_right);  
        cvShowImage("camera_left_Change", img_left_Change);  
        cvShowImage("camera_right_Change", img_right_Change);  

        // stereo match
        // change image data from IplImage* to  Mat*
        // stereo_match(img_left_Change, img_right_Change, Q);
        // cout << "CvMat Q:" << Q << endl;

        char c = cvWaitKey(33);  
        if(c == 27)   
            break;  
        else if(c == 's')
        {
            stereo_match(img_left_Change, img_right_Change, Q);
        }

        cvReleaseImage(&img_left);
        cvReleaseImage(&img_right);
        cvReleaseImage(&img_left_Change);
        cvReleaseImage(&img_right_Change);

    }  

    cvReleaseCapture(&cap_left);  
    cvReleaseCapture(&cap_right);  
    cvDestroyWindow("camera_left");  
    cvDestroyWindow("camera_right");  
 
    return 0;  

}
