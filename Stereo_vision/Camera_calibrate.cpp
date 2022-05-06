#include"Camera_calibrate.h"

using namespace std;
using namespace cv;

void Calibrate_Camera(int width, int height)
{
	vector<vector<Point3f>> objectpoint;

	vector<vector<Point2f>> imgpointsL;
	vector<vector<Point2f>> imgpointsR;

	vector <Point3f> chess_board;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			chess_board.push_back(Point3f(i, j, 0));
		}
	}
	vector<Point2f> corners_L_cam;
	vector<Point2f> corners_R_cam;

	Mat cam_matrix_1;
	Mat gray_L;
	Mat cam_matrix_2;
	Mat gray_R;
	int n = 0;

	//VideoCapture cap_R(1);
	while(1)
	{
		VideoCapture cap_L(2);
		VideoCapture cap_R(1);
		cap_L >> cam_matrix_1;
		cvtColor(cam_matrix_1, gray_L, COLOR_BGR2GRAY);
		cap_R >> cam_matrix_2;
		cvtColor(cam_matrix_2, gray_R, COLOR_BGR2GRAY);
		bool SL, SR;
		SL = findChessboardCorners(gray_L, cv::Size(height, width), corners_L_cam);
		SR = findChessboardCorners(gray_R, cv::Size(height, width), corners_R_cam);
		if (SL && SR)
		{
			TermCriteria criteria (TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001);

			cornerSubPix(gray_L, corners_L_cam, Size(11, 11), Size(-1, -1), criteria);
			cornerSubPix(gray_R, corners_R_cam, Size(11, 11), Size(-1, -1), criteria);
			
			drawChessboardCorners(gray_L, Size(height, width), corners_L_cam, SL);
			drawChessboardCorners(gray_R, Size(height, width), corners_R_cam, SR);

			objectpoint.push_back(chess_board);

			imgpointsL.push_back(corners_L_cam);
			imgpointsR.push_back(corners_R_cam);
			n++;

		}

		imshow("Framesc", gray_L);
		imshow("Frames2", gray_R);
		if (n == 7) { break; }

		waitKey(0);
	}
	Mat mtxL, distL, R_L, T_L;
	Mat mtxR, distR, R_R, T_R;
	Mat Rot, Trns, Emat, Fmat;
	Mat new_mtxL, new_mtxR;

	calibrateCamera(objectpoint, imgpointsL, gray_L.size(), mtxL, distL, R_L, T_L);

	new_mtxL = getOptimalNewCameraMatrix(mtxL, distL, gray_L.size(), 1, gray_L.size(), 0);
	
	calibrateCamera(objectpoint, imgpointsR, gray_R.size(), mtxR, distR, R_R, T_R);

	new_mtxR = getOptimalNewCameraMatrix(mtxR, distR, gray_R.size(), 1, gray_R.size(), 0);

	TermCriteria crit = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 1e-6);

	stereoCalibrate(objectpoint,imgpointsL,imgpointsR,new_mtxL,distL,new_mtxR,distR,gray_R.size(),Rot,Trns,Emat,Fmat,CALIB_FIX_INTRINSIC,crit);

	Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;

	stereoRectify(new_mtxL, distL, new_mtxR, distR, gray_R.size(), Rot, Trns, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

	Mat LStereo1;
	Mat LStereo2;

	Mat RStereo1;
	Mat RStereo2;

	initUndistortRectifyMap(new_mtxL,distL,rect_l,proj_mat_l,gray_R.size(), CV_16SC2 ,LStereo1, LStereo2);
	initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r, gray_R.size(), CV_16SC2, RStereo1, RStereo2);

	FileStorage matx = FileStorage("STEREOMATRIX.xml", FileStorage::WRITE);
	matx.write("Left_SMat_x", LStereo1);
	matx.write("Left_SMat_y", LStereo2);
	matx.write("Right_SMat_x", RStereo1);
	matx.write("Right_SMat_y", RStereo2);
}


void _DeptMap()
{
	int SWS = 5;
	int PFS = 5;
	int preFilterType = 1;
	//int preFilterSize = 1;
	int preFiltCap = 31;
	int minDisp = 0;
	int numOfDisp = 8;
	int TxtrThrshld = 10;
	int unicRatio = 15;
	int SpcklRng = 0;
	int SpklWinSze = 0;
	FileStorage cv_file2 = FileStorage("C:/Users/ggury/source/repos/Stereo_vision/Stereo_vision/STEREOMATRIX.xml", FileStorage::READ);
	//matx.open("STEREOMATRIX.xml", FileStorage::READ);
	Mat left_s_x, left_s_y;
	Mat right_s_x, right_s_y;
	cv_file2["Left_SMat_x"] >> left_s_x;
	cv_file2["Left_SMat_y"] >> left_s_y;
	cv_file2["Right_SMat_x"] >> right_s_x;
	cv_file2["Right_SMat_y"] >> right_s_y;
	cv_file2.release();
	Mat cam_matrix_1;
	Mat cam_matrix_2;
	Mat gray_L;
	Mat gray_R;
	Ptr<StereoBM> bm = StereoBM::create();
	namedWindow("Image");
	cv::createTrackbar("SWS", "Image", &SWS, 50);
	cv::createTrackbar("PFS", "Image", &PFS, 255);
	cv::createTrackbar("PreFiltCap", "Image", &preFiltCap, 62);
	cv::createTrackbar("PreFiltType", "Image", &preFilterType, 1);
	cv::createTrackbar("MinDISP", "Image", &minDisp, 25);
	cv::createTrackbar("NumOfDisp", "Image", &numOfDisp, 18);
	cv::createTrackbar("TxtrThrshld", "Image", &TxtrThrshld, 100);
	cv::createTrackbar("UnicRatio", "Image", &unicRatio, 100);
	cv::createTrackbar("SpcklRng", "Image", &SpcklRng, 100);
	cv::createTrackbar("SpklWinSze", "Image", &SpklWinSze, 25);
	Mat disp, disp8, colored;



	while (1)
	{
		SWS = getTrackbarPos("SWS", "Image");
		PFS = getTrackbarPos("PFS", "Image");
		preFiltCap = getTrackbarPos("PreFiltCap", "Image");
		minDisp = getTrackbarPos("MinDISP", "Image");
		numOfDisp = getTrackbarPos("NumOfDisp", "Image");
		TxtrThrshld = getTrackbarPos("TxtrThrshld", "Image");
		unicRatio = getTrackbarPos("UnicRatio", "Image");
		SpcklRng = getTrackbarPos("SpcklRng", "Image");
		SpklWinSze = getTrackbarPos("SpklWinSze", "Image");
		preFilterType = getTrackbarPos("PreFiltType", "Image");

		Mat left_calibrated, right_calibrated;
		VideoCapture cap_L(1);
		VideoCapture cap_R(2);
		cap_L >> cam_matrix_1;
		cvtColor(cam_matrix_1, gray_L, COLOR_BGR2GRAY);
		cap_R >> cam_matrix_2;
		cvtColor(cam_matrix_2, gray_R, COLOR_BGR2GRAY);
		cv::remap(gray_L, left_calibrated, left_s_x, left_s_y, cv::INTER_LANCZOS4, BORDER_CONSTANT, 0);
		cv::remap(gray_R, right_calibrated, right_s_x, right_s_y, cv::INTER_LANCZOS4, BORDER_CONSTANT, 0);


		bm->setPreFilterCap(preFiltCap);
		bm->setPreFilterSize(PFS);
		bm->setPreFilterType(preFilterType);
		bm->setBlockSize(SWS);
		bm->setMinDisparity(minDisp);
		bm->setNumDisparities(numOfDisp);
		bm->setTextureThreshold(TxtrThrshld);
		bm->setUniquenessRatio(unicRatio);
		bm->setSpeckleWindowSize(SpklWinSze);
		bm->setSpeckleRange(SpcklRng);
		bm->setDisp12MaxDiff(-1);

		bm->compute(left_calibrated, right_calibrated, disp);
		disp.convertTo(disp8, CV_32F, 1.0);
		disp8 = (disp8 / 16.0f - (float)minDisp) / ((float)numOfDisp);
		//applyColorMap(disp8, colored, cv::COLORMAP_JET);
		imshow("colored", disp8);
		waitKey(0);
	}
}