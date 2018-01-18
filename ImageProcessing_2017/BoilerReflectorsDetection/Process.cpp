#include "Process.h"

const string Process::RESOLUTION = to_string(IMG_COLS) + "x" + to_string(IMG_ROWS);
//const string Process::CAMERA_URL = "http://root:root@192.168.0.90/axis-cgi/mjpg/video.cgi?resolution=" + RESOLUTION + "&fps=" + to_string(FPS);
const string Process::CAMERA_URL = "http://172.22.11.2:1182/stream.mjpg";//stream.mjpg
const string Process::INPUT_WINDOW_NAME = "input";
const string Process::OUTPUT_WINDOW_NAME = "output";

const int Process::NUM_OF_IMAGES_TO_PROCESS = 4;
const string *Process::IMAGES_TO_PROCESS = new string[Process::NUM_OF_IMAGES_TO_PROCESS]
{
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg1_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg2_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg3_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg4_Cropped.png"
};

const string Process::CAMERA_CONNECTION_ERROR_MSG = "ERROR [1]: Unable to connect to camera. Check connection or URL.";
const string Process::NO_FRAME_ERROR_MSG = "ERROR [2]: No frame received. Check camera connection.";

const Scalar Process::COLOR_RED(0, 0, 255);
const Scalar Process::COLOR_BLUE(255, 0, 0);
const Scalar Process::COLOR_YELLOW(0, 255, 255);



void Process::main()
{
	switch (INPUT_SRC)
	{
	case CameraStream:
		SetupCameraStream();
		break;

	case Images:
		SetupImagesProcess();
		break;

	case UsbCameraPreview:
		SetupUsbCameraPreviewStream();
		break;
	}
}

void Process::SetupUsbCameraPreviewStream()
{
	VideoCapture vc;
	Mat frame;

	if (!vc.open(USB_CAM_INDEX))
	{
		cout << CAMERA_CONNECTION_ERROR_MSG << endl;
		return;
	}

	while (true)
	{
		if (!vc.read(frame))
		{
			cout << NO_FRAME_ERROR_MSG << endl;
		}

		imshow(INPUT_WINDOW_NAME, frame);

		if (waitKey(1) == QUIT_CHAR)
		{
			break;
		}
	}
}

void Process::SetupCameraStream()
{
	VideoCapture v_c;
	Mat frame;
	
	if (!v_c.open(CAMERA_URL))
	{
		cout << CAMERA_CONNECTION_ERROR_MSG << endl;
		return;
	}

	while (true)
	{
		if (!v_c.read(frame))
		{
			cout << NO_FRAME_ERROR_MSG << endl;
		}

		if (frame.cols > 0 && frame.rows > 0)
		{
			Point2f src_center(frame.cols / 2.0F, frame.rows / 2.0F);
			Mat rot_tmp = getRotationMatrix2D(src_center, 180, 1.0);
			Mat rot_frame;
			warpAffine(frame, rot_frame, rot_tmp, frame.size());

			Mat dst;

			ProcessImage(rot_frame, dst);

			imshow(INPUT_WINDOW_NAME, rot_frame);
			imshow(OUTPUT_WINDOW_NAME, dst);

			if (waitKey(1) == QUIT_CHAR)
			{
				break;
			}
		}
	}
}

void Process::SetupImagesProcess()
{
	Mat input, output;
	for (int i = 0; i < NUM_OF_IMAGES_TO_PROCESS; i++)
	{
		input = imread(IMAGES_TO_PROCESS[i]);
		ProcessImage(input, output);
		imshow(OUTPUT_WINDOW_NAME + " " + to_string(i + 1), output);
	}

	waitKey();
}

void Process::ProcessImage(Mat& src, Mat& dst)
{
	Mat binary_img;
	DynamicMeanValueThreshold(src, binary_img);

	vector<vector<Point>> contours;
	findContours(binary_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	cvtColor(binary_img, dst, CV_GRAY2BGR);

	vector<Point> contour;
	RotatedRect min_rect;
	vector<vector<Point>> selected_contours;
	vector<RotatedRect> min_rects;
	float area_ratio;
	int count = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		contour = contours[i];

		if (contourArea(contour) > MIN_CONTOUR_AREA)
		{
			count++;

			selected_contours.push_back(contour);
		}
	}

	RotatedRect result;
	bool has_found = false;
	if (count >= 2)
	{
		RotatedRect combined_min_rect;
		for (int i = 0; i < count; i++)
		{
			for (int j = i + 1; j < count; j++)
			{
				vector<Point> points;
				Point2f* tmp_pts1 = new Point2f[4];
				Point2f* tmp_pts2 = new Point2f[4];
				minAreaRect(selected_contours[0]).points(tmp_pts1);
				minAreaRect(selected_contours[1]).points(tmp_pts2);
				for (int i = 0; i < 4; i++)
				{
					points.push_back(tmp_pts1[i]);
					points.push_back(tmp_pts2[i]);
				}

				combined_min_rect = minAreaRect(points);

				float area_ratio = combined_min_rect.size.area() / (contourArea(selected_contours[0]) + contourArea(selected_contours[1]));

				if (InPercentRange(area_ratio, 2.2, 0.1))
				{
					result = combined_min_rect;
					has_found = true;
					break;
				}
			}

			if (has_found)
			{
				break;
			}
		}
	}

	int shape_pixel_width = result.size.height;
	float shape_to_img_width_ratio = (float)shape_pixel_width / (1.15*IMG_COLS);

	float shape_real_size = 0.24;//meters
	
	float distance = (shape_real_size) / shape_to_img_width_ratio;

	//printf("%f\n", distance);

	printf("Center: (%f,%f)\n", result.center.x, result.center.y);

	DrawRotatedRect(dst, result, COLOR_RED, 1);
}

void Process::DrawRotatedRect(Mat& p_img, RotatedRect p_rect, Scalar p_color, int p_thickness)
{
	Point2f *vertices = new Point2f[4];
	p_rect.points(vertices);

	for (int i = 0; i < 4; i++)
	{
		line(p_img, vertices[i], vertices[(i + 1) % 4], p_color, p_thickness);
	}
}

void Process::DynamicMeanValueThreshold(Mat& src, Mat& dst)
{
	Mat bgr_channels[3];
	split(src, bgr_channels);
	Mat green_channel_img = bgr_channels[1];
	Mat red_channel_img = bgr_channels[2];

	Mat img = green_channel_img - red_channel_img;

	int total_val = 0;
	int max = 0;
	int pixel_count = IMG_COLS * IMG_ROWS;
	uchar pixel_value;
	for (int y = 0; y < IMG_ROWS; y++)
	{
		for (int x = 0; x < IMG_COLS; x++)
		{
			pixel_value = img.at<uchar>(y, x);
			total_val += pixel_value;
			if (pixel_value > max)
			{
				max = pixel_value;
			}
		}
	}

	float avg_value = (float)total_val / pixel_count;

	int total_deviation = 0;
	for (int y = 0; y < IMG_ROWS; y++)
	{
		for (int x = 0; x < IMG_COLS; x++)
		{
			pixel_value = img.at<uchar>(y, x);
			total_deviation += pow((pixel_value - avg_value), 2);
		}
	}

	float st_d = (float)total_deviation / pixel_count;
	float population_st_d = sqrt(st_d);

	float threshold_value = avg_value + 1.75 * population_st_d;

	Mat val_threshold_img;
	threshold(img, dst, threshold_value, 255, ThresholdTypes::THRESH_BINARY);
}

bool Process::InPercentRange(float p_val, float p_threshold, float p_range_percent)
{
	return (p_val >= p_threshold * (1 - p_range_percent) && p_val <= p_threshold * (1 + p_range_percent));
}

float Process::PointDistance(Point p1, Point p2)
{
	Point diff = p1 - p2;
	float dis = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
	return dis;
}