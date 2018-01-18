#pragma unmanaged
#include <opencv2\opencv.hpp>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

enum InputSrc
{
	CameraStream,
	Images
};

class Process
{
	public:
		static void main();
		static void SetupCameraStream();
		static void SetupImagesProcess();

		static void RotateImage(Mat&, Mat&, int);

		static void DrawRotatedRect(Mat&, RotatedRect, Scalar, int = 1);
		static float PointDistance(Point, Point);
		static Point2f& GetTopLeftVertice(RotatedRect);
		static void DynamicMeanValueThreshold(Mat&, Mat&);
		static void ProcessImageFront(Mat&, Mat&);
		static void ProcessImageBack(Mat&, Mat&);
		static bool InPercentRange(float, float, float);
		static Point2f MiddlePoint(Point2f, Point2f);
		static bool IsInTolerance(float, float, float);
	private:
		static const char QUIT_CHAR = 'q';
		static const int IMG_COLS = 320;
		static const int IMG_ROWS = 240;
		static const int FPS = 20;

		static const int MIN_CONTOUR_AREA = 100;

		static const int RELATIVE_CENTER_X = 107;

		static const string RESOLUTION;
		static const string CAMERA_URL_FRONT;
		static const string CAMERA_URL_BACK;
		static const string INPUT_FRONT_WINDOW_NAME;
		static const string INPUT_BACK_WINDOW_NAME;
		static const string OUTPUT_FRONT_WINDOW_NAME;
		static const string OUTPUT_BACK_WINDOW_NAME;

		static const int NUM_OF_IMAGES_TO_PROCESS;
		static const string *IMAGES_TO_PROCESS;
		
		static const string CAMERA_CONNECTION_ERROR_MSG;
		static const string NO_FRAME_ERROR_MSG;

		static const Scalar COLOR_RED;
		static const Scalar COLOR_BLUE;
		static const Scalar COLOR_YELLOW;
		
		static const InputSrc INPUT_SRC = CameraStream;
};