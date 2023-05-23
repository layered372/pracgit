#include <iostream>
#include <io.h>
#include "CamPoseEst.h"

#include <time.h> 

#define PI 3.14159265

int main()
{
	printf("main test\n");

	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/1_down3_before_aline.jpg";
	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/2_down1.3_before_aline.jpg";	// f_name
	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/3_down0_before_aline.jpg";	// f_name
	char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/4_up3_before_aline.jpg";

	//Mat i_img = Mat(RGB5M_HEIGHT, RGB5M_WIDTH, CV_8U);
	//memcpy(i_img.data, i_yuv, sizeof(unsigned char) * RGB5M_WXH);

	Mat i_img_rgb = imread(fname, IMREAD_COLOR);

	Mat i_img;
	cvtColor(i_img_rgb, i_img, COLOR_RGB2GRAY);

	namedWindow("i_img", WINDOW_NORMAL);
	imshow("i_img", i_img);

	int idx_h = 4;			// center line
	int idx_pts = 12;		// points on center line
	
	CamPoseEst campos;
	vector<Point2f> imgpts;

	campos.initParam();
	int detectpattern = campos.detectPatternPts(i_img, imgpts);
	vector<Point2f> linpts(idx_pts);
	vector<Point2f> linpts_rnd(idx_pts);
	vector<Point2f> linpts_grad(11);
	double sum_x = 0.0, sum_y = 0.0;
	double mean_x = 0.0, mean_y = 0.0;
	Point2f base(-520, 0);

	cout << "[imgpts] : " << endl << imgpts << endl << endl;

	copy(imgpts.begin()+idx_pts*idx_h, imgpts.begin()+idx_pts*idx_h+idx_pts, linpts.begin());
	cout << "[linpts] : " << endl << linpts << endl;
	cout << "linpts[0]" << linpts[0] << endl;
	cout << "linpts[11]" << linpts[11] << endl;

	for (int i = 0; i < idx_pts; i++) {
		linpts_rnd[i].x = round(linpts[i].x);
		linpts_rnd[i].y = round(linpts[i].y);
	}

	for (int i = 0; i < idx_pts-1; i++) {
		linpts_grad[i].x = linpts_rnd[i+1].x - linpts_rnd[i].x;
		linpts_grad[i].y = linpts_rnd[i+1].y - linpts_rnd[i].y;

		sum_x += linpts_grad[i].x;
		sum_y += linpts_grad[i].y;
	}

	mean_x = sum_x / (idx_pts-1);
	mean_y = sum_y / (idx_pts-1);

	cout << "[linpts_rnd]: " << endl << linpts_rnd << endl << endl;
	cout << "[linpts_grad]: " << endl << linpts_grad << endl << endl;
	cout << "[mean_x]: " << mean_x << endl;
	cout << "[mean_y]: " << mean_y << endl;
	
	float angle = float(atan2(mean_y, mean_x) * (180/PI)); // rad to deg
	cout << "[degree] : " << angle << endl << endl;


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ref) image rotation

	/*float centerX = (linpts_rnd[5].x + linpts_rnd[6].x) / 2;
	float centerY = (linpts_rnd[5].y + linpts_rnd[6].y) / 2;*/

	float centerX = linpts_rnd[0].x;
	float centerY = i_img.rows; 


	Mat M = getRotationMatrix2D(Point2f(centerX, centerY),
			angle,		// rotation degree (-: clockwise, +: anticlockwise)
			1.0);		// scale

	cout << "[M]: " << M << endl;
	
	Mat img_rotated;
	warpAffine(i_img, img_rotated, M, i_img.size());
	namedWindow("img_rotated", WINDOW_NORMAL);
	imshow("img_rotated", img_rotated);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	float rad = float(angle * (PI/180));		// deg to rad
	
	float rot_x = cos(rad)*base.x + -sin(rad)*base.y;
	float rot_y = sin(rad)*base.x +  cos(rad)*base.y;
	cout << "[base]: " << base << endl;
	printf("[rot_x]: %lf, [rot_y]: %lf\n", rot_x, rot_y);

	float delta_x = rot_x - base.x;
	float delta_y = rot_y - base.y;
	printf("[delta_x]: %lf, [delta_y]: %lf\n", delta_x, delta_y);

	waitKey(0);

	return 0;
}

int main_org()
{

	// parameters -------------------------------------------------------------
	double  loc[3], deg[3];
	int		selMode = 1; //0: DDA Cal. 1: Camera Pose Estimation
	clock_t start, end; 
	double  estRuntime;
	
	// info file ---------------------------------------------------------------
	//char rdir[] = "../../99_IMG/NEWMP/test3/";	// reference image directory
	//char tdir[] = "../../99_IMG/NEWMP/test3/";	// target image directory
	
	//char rdir[] = "../../99_IMG/BRZ_MP2/";	// reference image directory
	//char tdir[] = "../../99_IMG/BRZ_MP2/";	// target image directory

	//char rdir[] = "../../99_IMG/circle_sym/";	// reference image directory
	//char tdir[] = "../../99_IMG/circle_sym/";	// target image directory

	//char rdir[] = "../../99_IMG/IMG220108/circ_sym_cal/";	// reference image directory
	//char tdir[] = "../../99_IMG/IMG220108/circ_sym_cal/";	// target image directory

	//char rdir[] = "../../99_IMG/test_mkimg/";	// reference image directory
	//char tdir[] = "../../99_IMG/test_mkimg/";	// target image directory

	//char rdir[] = "../../99_IMG/IMG220108/circ_sym_cal/";	// reference image directory
	//char tdir[] = "../../99_IMG/IMG220108/circ_sym_cal/";	// target image directory
	//char rdir[] = "../../99_IMG/IMG_Circle/IMG220413_DDACAL_ochang/";	// reference image directory
	//char tdir[] = "../../99_IMG/IMG_Circle/IMG220413_DDACAL_ochang/";	// target image directory


	char rdir[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/";	// reference image directory
	char tdir[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/";	// target image directory


	char fname_dda[] = "dda_caldata.bin";
	char fname_cam[] = "cam_caldata.bin";
	

	// load reference info. ----------------------------------------------------
	CamPoseEst campos;
	campos.initParam();
	if (selMode)
	{
		bool succ = campos.readData(rdir, fname_dda);
		if (!succ)
		{
			printf("cannot read data(dda cal)!!\n");
			return false;
		}
	}
	else
	{
		
		bool succ = campos.genAndWriteData(rdir, fname_cam, fname_dda);
		if (!succ)
		{
			printf("Cannot generate and Write Cal. Data!!\n");
			return false;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// estimation of delta x&y  according to chuck change

	char ifname[100];
	String wfile0 = tdir + String("test_rst.txt");
	FILE* fp_write0 = fopen(wfile0.c_str(), "w");


	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/1_down3_before_aline.jpg";
	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/1_down3_before_aline.jpg";
	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/2_down1.3_before_aline.jpg";	// f_name
	//char fname[] = "../../99_IMG/IMG_Circle_OChang/IMG220415_RotPattern/3_down0_before_aline.jpg";	// f_name

	sprintf(ifname, "%s1_down3_before_aline.jpg", tdir);
	Mat inimg = imread(ifname);
	Mat gray;
	cvtColor(inimg, gray, COLOR_RGB2GRAY);
	
	namedWindow("[gray]", WINDOW_NORMAL);
	imshow("[gray]", gray);

	int idx_h = 4;			// center line
	int idx_pts = 12;		// points on center line

	vector<Point2f> imgpts;

	campos.initParam();
	int detectpattern = campos.detectPatternPts(gray, imgpts);
	vector<Point2f> linpts(idx_pts);
	vector<Point2f> linpts_rnd(idx_pts);
	vector<Point2f> linpts_grad(11);
	double sum_x = 0.0, sum_y = 0.0;
	double mean_x = 0.0, mean_y = 0.0;
	Point2f base(-520, 0);

	cout << "[imgpts] : " << endl << imgpts << endl << endl;

	copy(imgpts.begin() + idx_pts * idx_h, imgpts.begin() + idx_pts * idx_h + idx_pts, linpts.begin());
	cout << "[linpts] : " << endl << linpts << endl;
	cout << "linpts[0]" << linpts[0] << endl;
	cout << "linpts[11]" << linpts[11] << endl;

	for (int i = 0; i < idx_pts; i++) {
		linpts_rnd[i].x = round(linpts[i].x);
		linpts_rnd[i].y = round(linpts[i].y);
	}

	for (int i = 0; i < idx_pts - 1; i++) {
		linpts_grad[i].x = linpts_rnd[i + 1].x - linpts_rnd[i].x;
		linpts_grad[i].y = linpts_rnd[i + 1].y - linpts_rnd[i].y;

		sum_x += linpts_grad[i].x;
		sum_y += linpts_grad[i].y;
	}

	mean_x = sum_x / (idx_pts - 1);
	mean_y = sum_y / (idx_pts - 1);

	cout << "[linpts_rnd]: " << endl << linpts_rnd << endl << endl;
	cout << "[linpts_grad]: " << endl << linpts_grad << endl << endl;
	cout << "[mean_x]: " << mean_x << endl;
	cout << "[mean_y]: " << mean_y << endl;

	float angle = float(atan2(mean_y, mean_x) * (180 / PI)); // rad to deg
	cout << "[angle] : " << angle << endl << endl;

	float rad = float(angle * (PI / 180));		// deg to rad

	float rot_x = cos(rad) * base.x + -sin(rad) * base.y;
	float rot_y = sin(rad) * base.x + cos(rad) * base.y;
	cout << "[base]: " << base << endl;
	printf("[rot_x]: %lf, [rot_y]: %lf\n", rot_x, rot_y);

	float delta_x = rot_x - base.x;
	float delta_y = rot_y - base.y;
	printf("[delta_x]: %lf, [delta_y]: %lf\n", delta_x, delta_y);


	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ref) image rotation

	float centerX = (linpts_rnd[5].x + linpts_rnd[6].x) / 2;
	float centerY = (linpts_rnd[5].y + linpts_rnd[6].y) / 2;

	Mat M = getRotationMatrix2D(Point2f(centerX, centerY),
		angle,		// rotation degree (-: clockwise, +: anticlockwise)
		1.0);		// scale

	cout << "[M]: " << M << endl << endl;

	Mat img_rotated;
	warpAffine(gray, img_rotated, M, gray.size());
	namedWindow("img_rotated", WINDOW_NORMAL);
	imshow("img_rotated", img_rotated);


	cout << "================================================================================== " << endl;
	cout << "camera pose estimation " << endl;

	// run camera pose estimation ---------------------------------------------
	start = clock(); // start time
	int state_est = campos.estimateCamPose(img_rotated.data, loc, deg);
	if (state_est <= 0)
	{
		printf("Cannot estimate camera pose.. \n");
		return false;
	}
	end = clock();	 // end time

	// display results -------------------------------------------------------------
	printf("loc:\n %.15f\n %.15f\n %.15f\n\n", loc[0], loc[1], loc[2]);
	printf("deg:\n %.15f\n %.15f\n %.15f\n\n", deg[0], deg[1], deg[2]);
	estRuntime = (double)(end - start);
	cout << "result : " << ((estRuntime) / CLOCKS_PER_SEC) << " seconds" << endl << endl;


	fprintf(fp_write0, "time - %lf :     %lf	%lf		%lf		%lf		%lf		%lf\n",
		estRuntime,
		loc[0], loc[1], loc[2],
		deg[0], deg[1], deg[2]);

	

	/* test estimateCamPose
	//-----------------------------------------------------
	
	char ifname[100];
	unsigned char *in_yuv = (unsigned char*)malloc(sizeof(char)* RGB5M_WXH*1.5);
	String wfile0 = tdir + String("test_rst.txt");
	FILE *fp_write0 = fopen(wfile0.c_str(), "w");
	//for (int fnum_tar = 0; fnum_tar < 11; fnum_tar++)
	for (int fnum_tar = 7; fnum_tar < 11; fnum_tar++)
	{
		////sprintf(ifname, "%simg_yuv_%04d.raw", tdir, fnum_tar);
		//sprintf(ifname, "%simg_yuv5m_%04d.raw", tdir, fnum_tar);
		//
		//FILE *in_fp_yuv = fopen(ifname, "rb");
		//if (in_fp_yuv == NULL)
		//{
		//	printf("file not found.. \n");
		//	return false;
		//}
		//fread(in_yuv, sizeof(char), RGB5M_WXH*1.5, in_fp_yuv);
		//fclose(in_fp_yuv);


		//sprintf(ifname, "%simg_rgbm_%04d.jpg", tdir, fnum_tar);
		sprintf(ifname, "%simg_rgb_%04d.jpg", tdir, fnum_tar);
		Mat inimg = imread(ifname);
		Mat gray;
		cvtColor(inimg, gray, COLOR_RGB2GRAY);
		memcpy(in_yuv, gray.data, RGB5M_WXH);



		// run camera pose estimation ---------------------------------------------
		start = clock(); // start time
		int state_est = campos.estimateCamPose(in_yuv, loc, deg);
		if (state_est <= 0)
		{
			printf("Cannot estimate camera pose.. \n");
			return false;
		}
		end = clock();	 // end time

		// display results -------------------------------------------------------------
		printf("loc:\n %.15f\n %.15f\n %.15f\n\n", loc[0], loc[1], loc[2]);
		printf("deg:\n %.15f\n %.15f\n %.15f\n\n", deg[0], deg[1], deg[2]);
		estRuntime = (double)(end - start);
		cout << "result : " << ((estRuntime) / CLOCKS_PER_SEC) << " seconds" << endl << endl;

		

		fprintf(fp_write0, "%d time %lf :     %lf	%lf		%lf		%lf		%lf		%lf\n",
			fnum_tar, estRuntime,
			loc[0], loc[1], loc[2],
			deg[0], deg[1], deg[2]);

	}
	*/

	campos.deinitParam();


	// release --------------------------------------------------------------
	//free(in_yuv);


	return 0;
}

int estimateAngle(Mat inimg)
{   
    double loc[3], deg[3];
    int selMode = 1;
    clock_t start, end;
    double estRuntime;

    char rdir[] = "";
    char tdir[] = "";

    char fname_dda[] = "";
    char fname_cam[] = "";

    CamPoseEst campos;
    campos.initPram();

    if (selMode)
    {
        bool succ = campos.readData(rdir, fname_dda);

        if (!succ)
        {
            printf("cannot read data(dda cal)!!\n");
            return false;
        }
    }
    else
    {
        bool succ = campos.genAndWriteData(rdir, fname)cam, fname_dda);

        if (!succ)
        {
            printf("Cannot generate and write al. data..!!\n);
            return false;
        }
    }

    char ifname[100];

    Strin wfile0 = tdir + String("test_rst.txt");
    FILE* fp_write0 = fope(wfile0.c_str(), "w");

    sprintf(ifname, 





}
































