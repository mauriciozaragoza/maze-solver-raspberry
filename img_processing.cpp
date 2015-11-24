#include "img_processing.h"

void ImageProc::resize_to_max(Mat &img, int max_dimension)
{
	double ratio_x = img.cols / (double)max_dimension,
		   ratio_y = img.rows / (double)max_dimension;

	Size new_size;

	if (ratio_x > ratio_y) 
	{
		new_size.width = max_dimension;
		new_size.height = img.rows / ratio_x;
	}
	else
	{
		new_size.height = max_dimension;
		new_size.width = img.cols / ratio_y;
	}

	resize(img, img, new_size);
}

void ImageProc::preprocess_image(Mat &img, Mat &dest)
{
	const int IMAGE_THRESH_SEGMENTS = 6;
	// const int MAX_THRESH = 192;
	const Mat kernel = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);

	int blur_x = img.cols / 100,
		blur_y = img.rows / 100;

	// Must be odd-numbered
	blur_x += (1 - blur_x % 2);
	blur_y += (1 - blur_y % 2);

	// Equalize histogram
	// equalizeHist(img, dest);

	// Blur to remove noisy bits
	GaussianBlur(
		dest, 
		dest, 
		Size(blur_x, blur_y),
		0);

	// threshold(
	// 	dest, 
	// 	dest, 
	// 	128, 
	// 	255, 
	// 	CV_THRESH_BINARY);

	// Apply threshold on segments of the image
	for (int i = 0; i < IMAGE_THRESH_SEGMENTS; i++) 
	{
		for (int j = 0; j < IMAGE_THRESH_SEGMENTS; j++)
		{
			Mat segment = dest(Rect(
				j * dest.cols / IMAGE_THRESH_SEGMENTS,
				i * dest.rows / IMAGE_THRESH_SEGMENTS,
				((j + 1) * dest.cols / IMAGE_THRESH_SEGMENTS) - (j * dest.cols / IMAGE_THRESH_SEGMENTS), // dest.cols / IMAGE_THRESH_SEGMENTS,
				((i + 1) * dest.rows / IMAGE_THRESH_SEGMENTS) - (i * dest.rows / IMAGE_THRESH_SEGMENTS) // dest.rows / IMAGE_THRESH_SEGMENTS
			));

			int thresh = threshold(
				segment, 
				segment, 
				0, 
				255, 
				CV_THRESH_BINARY | CV_THRESH_OTSU);

			// Just make it white if there are no walls in it
			// if (thresh > MAX_THRESH)
			// {
			// 	segment = Scalar(255);
			// }

			// adaptiveThreshold(
		 //        segment, 
		 //        segment, 
		 //        255, 
		 //        ADAPTIVE_THRESH_MEAN_C,
		 //        THRESH_BINARY,
		 //        5,
		 //        3);
		}
	}

	// bitwise_not(dest, dest);
	// dilate(dest, dest, kernel);
	// bitwise_not(dest, dest);
}

void ImageProc::get_point(Mat img, Point &point, int color, int tolerance, int min_saturation, int min_lightness)
{
	Mat HSV;

	cvtColor(img, HSV, CV_BGR2HSV);

	// cout << "goes from: " << Scalar((color + 180 - tolerance) % 180, 150, 150) << " to " << Scalar((color + tolerance) % 180, 255, 255) << endl;

	int value1 = (color + 180 - tolerance) % 180,
		value2 = (color + tolerance) % 180;

	// inRange(HSV, Scalar(min(value1, value2), 150, 150), Scalar(max(value1, value2), 255, 255), HSV);
	inRange(HSV, Scalar(color, min_saturation, min_lightness), Scalar(color + tolerance, 255, 255), HSV);

	for (int y = 0; y < HSV.size().height; y++)
	{
		uchar *row = HSV.ptr(y);
		for (int x = 0; x < HSV.size().width; x++)
		{
			if (row[x] == 255)
			{
				point.x = x;
				point.y = y;

				return;
			}
		}
	}
}

bool ImageProc::get_triangle(Mat image, Vec2f &angle, Point &position)
{
	const Mat kernel = Mat::ones(5, 5, CV_8UC1);
    vector< vector<Point> > contours;
    vector<Point> triangle;

    cvtColor(image, image, COLOR_BGR2GRAY);
    preprocess_image(image, image);
    // inRange(image, Scalar(170, 50, 50), Scalar(10, 255, 255), image);
    // imshow("DA TRIANGLE", image);
    // return false;

    Canny(image, image, 0, 50, 5);
	dilate(image, image, kernel);
    findContours(
        image, 
        contours, 
        // hierarchy, 
        CV_RETR_TREE, 
        CV_CHAIN_APPROX_SIMPLE);

    // Find the biggest triangle
    double largest_area = 0;
    int largest_index = -1;

    // cout << "found: " << contours.size() << " polygons" << endl;

    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> approximated_triangle;
        double current_area;

        approxPolyDP(
        	contours[i], 
        	approximated_triangle, 
        	arcLength(Mat(contours[i]), true) * 0.02,
        	true);

    	// cout << contours[i].size() << " omg " << approximated_triangle.size() << endl;

    	// for (int j = 0; j < approximated_triangle.size(); j++)
    	// {
    	// 	cout << approximated_triangle[j] << " ";
    	// }

    	// cout << endl;

        // not a rectangle or convex
        if (approximated_triangle.size() != 3 ||
            !isContourConvex(approximated_triangle)) 
        {
            continue;
        }

        current_area = contourArea(approximated_triangle);

        if (current_area > largest_area)
        {
            largest_area = current_area;
            largest_index = i;
        }
    }

    if (largest_index == -1)
    {
        // imshow("NO", image);
    	return false;
    }

    approxPolyDP(contours[largest_index], triangle, 10, true);

    double side01, side02, side12;
    Point front, back;

    side01 = (triangle[0].x - triangle[1].x) * (triangle[0].x - triangle[1].x) + (triangle[0].y - triangle[1].y) * (triangle[0].y - triangle[1].y);
    side02 = (triangle[0].x - triangle[2].x) * (triangle[0].x - triangle[2].x) + (triangle[0].y - triangle[2].y) * (triangle[0].y - triangle[2].y);
    side12 = (triangle[1].x - triangle[2].x) * (triangle[1].x - triangle[2].x) + (triangle[1].y - triangle[2].y) * (triangle[1].y - triangle[2].y);

    if (side01 < side02 && side01 < side12)
    {
    	back.x = (triangle[0].x + triangle[1].x) / 2.0;
    	back.y = (triangle[0].y + triangle[1].y) / 2.0;
    	front.x = triangle[2].x;
    	front.y = triangle[2].y;
    }
    else if (side02 < side01 && side02 < side12)
    {
    	back.x = (triangle[0].x + triangle[2].x) / 2.0;
    	back.y = (triangle[0].y + triangle[2].y) / 2.0;
    	front.x = triangle[1].x;
    	front.y = triangle[1].y;
    }
	else
	{
    	back.x = (triangle[1].x + triangle[2].x) / 2.0;
    	back.y = (triangle[1].y + triangle[2].y) / 2.0;
    	front.x = triangle[0].x;
    	front.y = triangle[0].y;
    }

    angle[0] = front.x - back.x;
    angle[1] = front.y - back.y;

    circle(image, front, 10, Scalar(255));
    circle(image, back, 5, Scalar(255));

    angle = angle / norm(angle);
    position = front;

    // imshow("canny", image);

    return true;
}