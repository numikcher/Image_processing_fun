#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;

enum InputSrc
{
	CameraStream,
	Images,
	UsbCameraPreview
};

class Process
{
public:
	static void main();

	
private:
	static void SetupUsbCameraPreviewStream();
	static void SetupCameraStream();
	static void SetupImagesProcess();
	static void ProcessImage(Mat& src, Mat& dst);
	static void DrawRotatedRect(Mat& p_img, RotatedRect p_rect, Scalar p_color, int p_thickness);
	static void DynamicMeanValueThreshold(Mat& src, Mat& dst);
	static bool InPercentRange(float p_val, float p_threshold, float p_range_percent);
	static float PointDistance(Point p1, Point p2);

	static const char QUIT_CHAR = 'q';
	static const int IMG_COLS = 320;
	static const int IMG_ROWS = 240;
	static const int FPS = 20;

	static const int MIN_CONTOUR_AREA = 200;

	static const int RELATIVE_CENTER_X = 107;

	static const string RESOLUTION;
	static const string CAMERA_URL;
	static const string INPUT_WINDOW_NAME;
	static const string OUTPUT_WINDOW_NAME;

	static const int NUM_OF_IMAGES_TO_PROCESS;
	static const string *IMAGES_TO_PROCESS;

	static const string CAMERA_CONNECTION_ERROR_MSG;
	static const string NO_FRAME_ERROR_MSG;

	static const Scalar COLOR_RED;
	static const Scalar COLOR_BLUE;
	static const Scalar COLOR_YELLOW;

	static const int USB_CAM_INDEX = 0;

	static const InputSrc INPUT_SRC = CameraStream;
};