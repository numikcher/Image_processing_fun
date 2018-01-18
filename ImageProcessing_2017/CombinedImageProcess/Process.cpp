#include "SmartDashboardUser.h"

#pragma unmanaged
#include "Process.h"

const string Process::RESOLUTION = to_string(IMG_COLS) + "x" + to_string(IMG_ROWS);
//const string Process::CAMERA_URL = "http://10.26.30.48/axis-cgi/mjpg/video.cgi?resolution=" + RESOLUTION + "&fps=" + to_string(FPS);
const string Process::CAMERA_URL_FRONT = "http://roborio-2630-frc.local:1182/stream.mjpg";//stream.mjpg
const string Process::CAMERA_URL_BACK = "http://roborio-2630-frc.local:1181/stream.mjpg";//stream.mjpg
const string Process::INPUT_FRONT_WINDOW_NAME = "input front";
const string Process::INPUT_BACK_WINDOW_NAME = "input back";
const string Process::OUTPUT_FRONT_WINDOW_NAME = "output front";
const string Process::OUTPUT_BACK_WINDOW_NAME = "output back";
const int OFFSET_CAMERA_FROM_BOILER = 6;
const int Process::NUM_OF_IMAGES_TO_PROCESS = 4;
const string *Process::IMAGES_TO_PROCESS = new string[Process::NUM_OF_IMAGES_TO_PROCESS]
{
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg1_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg2_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg3_Cropped.png",
	"C:/Users/Software 3.1/Documents/FRC 2017/Image Processing/ReflectorsImgs/Cropped/ReflectorsImg4_Cropped.png"
};

const string Process::CAMERA_CONNECTION_ERROR_MSG = "ERROR [1]: Unable to connect to camera. Check connection or URL.";
const string Process::NO_FRAME_ERROR_MSG = "ERROR [2]: No frame received. Check camera connection. Try to connet again.";

const Scalar Process::COLOR_RED(0, 0, 255);
const Scalar Process::COLOR_BLUE(255, 0, 0);
const Scalar Process::COLOR_YELLOW(0, 255, 255);



void Process::main()
{
	SetupCameraStream();
}

void Process::SetupCameraStream()
{
	VideoCapture vc_front;
	Mat frame_front;

	VideoCapture vc_back;
	Mat frame_back;

	if (!vc_front.open(CAMERA_URL_FRONT))
	{
		cout << CAMERA_CONNECTION_ERROR_MSG << endl;		
		return;
	}

	if (!vc_back.open(CAMERA_URL_BACK))
	{
		cout << CAMERA_CONNECTION_ERROR_MSG << endl;
		return;
	}

	while (true)
	{
		/*if (!vc_front.read(frame_front))
		{
			cout << NO_FRAME_ERROR_MSG << endl;
			vc_front.open(CAMERA_URL_FRONT);
		}*/

		if (!vc_back.read(frame_back))
		{
			cout << NO_FRAME_ERROR_MSG << endl;
			vc_back.open(CAMERA_URL_BACK);
		}

		if (frame_back.cols > 0 && frame_back.rows > 0)
		{
			Mat rot_frame_back;
			//RotateImage(frame_back, rot_frame_back, 0);

			Mat dst_back;
			
			ProcessImageBack(frame_back, dst_back);

			imshow(INPUT_BACK_WINDOW_NAME, frame_back);
			imshow(OUTPUT_BACK_WINDOW_NAME, dst_back);

			moveWindow(INPUT_BACK_WINDOW_NAME, 0, 450);
			moveWindow(OUTPUT_BACK_WINDOW_NAME, 320, 450);


			if (waitKey(1) == QUIT_CHAR)
			{
				break;
			}
		}

		if (frame_front.cols > 0 && frame_front.rows > 0)
		{
			Mat rot_frame_front;
			RotateImage(frame_front, rot_frame_front, 0);
			
			Mat dst_front;

			ProcessImageFront(frame_front, dst_front);

			imshow(INPUT_FRONT_WINDOW_NAME, frame_front);
			imshow(OUTPUT_FRONT_WINDOW_NAME, dst_front);

			moveWindow(INPUT_FRONT_WINDOW_NAME, 0, 205);
			moveWindow(OUTPUT_FRONT_WINDOW_NAME, 320, 205);

			if (waitKey(1) == QUIT_CHAR)
			{
				break;
			}
		}
	}
}

void Process::RotateImage(Mat& src, Mat& dst, int angle)
{
	Point2f src_center(src.cols / 2.0F, src.rows / 2.0F);
	Mat rot_tmp = getRotationMatrix2D(src_center, angle, 1.0);
	warpAffine(src, dst, rot_tmp, src.size());
}

void Process::ProcessImageFront(Mat& src, Mat& dst)
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

				if (IsInTolerance(area_ratio, 2.2,0.6))
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

	//printf("Center: (%f,%f)\n", result.center.x, result.center.y);

	DrawRotatedRect(dst, result, COLOR_RED, 1);

	Point2f center_of_rect = result.center;
	float offset_from_middle = center_of_rect.x  - (dst.cols/2.0) + OFFSET_CAMERA_FROM_BOILER;
	SmartDashboardUser::Set("boiler_offset_from_center", std::to_string(offset_from_middle));

	SmartDashboardUser::Set("boiler_width", std::to_string(result.size.width));

	line(dst, Point(dst.cols / 2 + OFFSET_CAMERA_FROM_BOILER, 0), Point(dst.cols / 2 + OFFSET_CAMERA_FROM_BOILER, dst.rows), COLOR_YELLOW);
}

void Process::ProcessImageBack(Mat& src, Mat& dst)
{
	if (src.rows <= 0 || src.cols <= 0)
	{
		dst = src;
		return;
	}

	Mat binary_img;
	DynamicMeanValueThreshold(src, binary_img);

	std::vector<std::vector<cv::Point>> contours;
	findContours(binary_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	cvtColor(binary_img, dst, CV_GRAY2BGR);

	std::vector<Point> contour;
	RotatedRect min_rect;
	vector<vector<Point>> rect_contours;
	vector<RotatedRect> min_rects;
	for (int i = 0; i < contours.size(); i++)
	{
		contour = contours[i];

		if (contourArea(contour) > 150) // check if contour above min size
		{
			RotatedRect min_rect = minAreaRect(contours[i]);
			float contour_area = contourArea(contour);
			float min_rect_area = min_rect.size.area();

			float contour_to_min_rect_area_ratio = min_rect_area / contour_area;

			if (InPercentRange(contour_to_min_rect_area_ratio, 1, 0.7)) // check how much of a rectangle every contour is
			{
				float rect_size_ratio = min_rect.size.width / min_rect.size.height;

				if (true)//InPercentRange(rect_size_ratio, 0.4, 0.4) || InPercentRange(rect_size_ratio, 2.5, 0.4)) // check rectangle size ratio
				{
					rect_contours.push_back(contour);
					min_rects.push_back(min_rect);
				}
				else
				{
					cout << "Failed because of proportions" << endl;
				}
			}
			else
			{
				cout << "Failed because of rectangleness" << endl;
			}
		}
	}

	drawContours(dst, rect_contours, -1, COLOR_BLUE, 2);

	vector<Point> cont1, cont2;
	RotatedRect rect1, rect2;
	Point p1, p2;
	bool has_found = false;
	float width1, width2, height1, height2;
	for (int i = 0; i < rect_contours.size(); i++)
	{
		for (int j = i + 1; j < rect_contours.size(); j++)
		{
			if (i != j)
			{
				cont1 = rect_contours[i];
				cont2 = rect_contours[j];
				rect1 = min_rects[i];
				rect2 = min_rects[j];

				width1 = min(rect1.size.width, rect1.size.height);
				width2 = min(rect2.size.width, rect2.size.height);
				height1 = max(rect1.size.width, rect1.size.height);
				height2 = max(rect2.size.width, rect2.size.height);

				if (InPercentRange(height1 / height2, 1, 0.5) &&
					InPercentRange(width1 / width2, 1, 0.5)) // check if close in height
				{
					if (true)//InPercentRange(rect1.angle, rect2.angle, 0.2)) // check if close in angle
					{
						p1 = rect1.center;
						p2 = rect2.center;

						float distance = PointDistance(p1, p2);
						

						float relative_distance = (width1 + width2) / 2.0 * /*5.5*/4.125;
						
						if (InPercentRange(distance, relative_distance, 0.5)) // check distance between rectangles
						{
							cout << "all is vvell" << endl;

							vector<vector<Point>> contours_to_print;
							contours_to_print.push_back(cont1);
							contours_to_print.push_back(cont2);
							
							DrawRotatedRect(dst, rect1, COLOR_RED, 2);
							DrawRotatedRect(dst, rect2, COLOR_RED, 2);

							vector<Point> points;
							Point2f* vertices_1 = new Point2f[4];
							Point2f* vertices_2 = new Point2f[4];

							rect1.points(vertices_1);
							rect2.points(vertices_2);

							for (int i = 0; i < 4; i++)
							{
								points.push_back(vertices_1[i]);
								points.push_back(vertices_2[i]);
							}

							RotatedRect final_rect = minAreaRect(points);
							float final_rect_width = max(final_rect.size.width, final_rect.size.height);
							DrawRotatedRect(dst, final_rect, COLOR_YELLOW);


							Point2f middle_point = MiddlePoint(rect1.center, rect2.center);
							float offset = middle_point.x - (dst.cols / 2.0) - 33.5;
							SmartDashboardUser::Set("offset_from_middle", to_string(offset));
							SmartDashboardUser::Set("reflectors_width", to_string(final_rect_width));
							SmartDashboardUser::Set("is_within_image_processing", "true");
							has_found = true;
							break;
						}
						else
						{
							cout << "Failed because of distance between rectangles" << endl;
						}
					}
					else
					{
						cout << "Failed because of angle difference" << endl;
					}
				}
				else
				{
					cout << "Failed because of different sizes" << endl;
				}
			}
			else
			{
				continue;
			}
		}
		if (has_found)
		{
			break;
		}
		else
		{
			SmartDashboardUser::Set("is_within_image_processing", "false");
		}
	}
	line(dst, Point(dst.cols / 2 + 33.5, 0), Point(dst.cols / 2 + 33.5, dst.rows), COLOR_YELLOW, 2);
	cout << "----------------" << endl;
}

Point2f& Process::GetTopLeftVertice(RotatedRect rect)
{
	Point2f* vertices = new Point2f[4];
	rect.points(vertices);

	Point2f* most_left_1 = nullptr; 
	Point2f* most_left_2 = nullptr;

	for (int i = 0; i < 4; i++)
	{
		if (most_left_1 == nullptr || vertices[i].x < most_left_1->x)
		{
			if (most_left_1 != nullptr)
			{
				most_left_2 = new Point2f(*most_left_1);
			}
			
			most_left_1 = new Point2f(vertices[i]);
		}
		else if (most_left_2 == nullptr || vertices[i].x < most_left_2->x)
		{
			most_left_2 = new Point2f(vertices[i]);
		}
	}

	delete vertices;

	if (most_left_1->y > most_left_2->y)
	{
		return *most_left_2;
	}
	return *most_left_1;
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
	Mat blue_channel_img = bgr_channels[0];
	Mat green_channel_img = bgr_channels[1];
	Mat red_channel_img = bgr_channels[2];

	Mat img = green_channel_img - red_channel_img - blue_channel_img;
	
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
	threshold(img, val_threshold_img, threshold_value, 255, ThresholdTypes::THRESH_BINARY);

	/*Mat val_threshold;
	threshold(green_channel_img, val_threshold, threshold_value, 255, ThresholdTypes::THRESH_BINARY);

	Mat sat_threshold_min;
	threshold(green_channel_img, sat_threshold_min, 150, 255, ThresholdTypes::THRESH_BINARY);
	
	Mat sat_threshold_max;
	threshold(green_channel_img, sat_threshold_max, 200, 255, ThresholdTypes::THRESH_BINARY_INV);

	dst = val_threshold & sat_threshold_min & sat_threshold_max;
	*/
	//Mat blue_threshold_inv;
	//threshold(blue_channel_img, blue_threshold_inv, 100, 255, ThresholdTypes::THRESH_BINARY_INV);
	//dst = blue_threshold_inv;
	//dst = dst & blue_threshold_inv;

	/*Mat hsv;
	cvtColor(src, hsv, CV_BGR2HSV);
	
	Mat hsv_channels[3];
	split(hsv, hsv_channels);

	Mat hue_channel = hsv_channels[0];

	//dst = hue_channel;
	Mat min_hue_threshold, max_hue_threshold_inv;
	threshold(hue_channel, min_hue_threshold, -1, 255, ThresholdTypes::THRESH_BINARY);
	threshold(hue_channel, max_hue_threshold_inv, 720, 255, ThresholdTypes::THRESH_BINARY_INV);
	*/
	//dst = /*val_threshold_img &*/ (min_hue_threshold /*& max_hue_threshold_inv*/);
	dst = val_threshold_img;
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

Point2f Process::MiddlePoint(Point2f p_p1, Point2f p_p2)
{
	Point2f p;
	p.x = ((p_p1.x + p_p2.x) / 2.0);
	p.y = ((p_p1.y + p_p2.y) / 2.0);
	return p;
}

bool Process::IsInTolerance(float p_num, float p_threshold, float p_tolerance)
{
	return (p_num > p_threshold - p_tolerance && p_num < p_threshold + p_tolerance);
}