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

void ImageProc::get_point(Mat img, Point2f &point, int color, int tolerance)
{
	Mat HSV;

	cvtColor(img, HSV, CV_BGR2HSV);

	// cout << "goes from: " << Scalar((color + 180 - tolerance) % 180, 150, 150) << " to " << Scalar((color + tolerance) % 180, 255, 255) << endl;

	int value1 = (color + 180 - tolerance) % 180,
		value2 = (color + tolerance) % 180;

	inRange(HSV, Scalar(min(value1, value2), 150, 150), Scalar(max(value1, value2), 255, 255), HSV);

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

Mat ImageProc::undistorted_grid(Mat processed_grid, Mat original_grid)
{
	vector<Vec2f> lines;
	HoughLines(processed_grid, lines, 1, CV_PI / 180, 200);

	// Now detect the lines on extremes
	Vec2f topEdge = Vec2f(1000, 1000);
	double topYIntercept = 100000, topXIntercept = 0;
	Vec2f bottomEdge = Vec2f(-1000, -1000);
	double bottomYIntercept = 0, bottomXIntercept = 0;
	Vec2f leftEdge = Vec2f(1000, 1000);
	double leftXIntercept = 100000, leftYIntercept = 0;
	Vec2f rightEdge = Vec2f(-1000, -1000);
	double rightXIntercept = 0, rightYIntercept = 0;

	for (int i = 0; i<lines.size(); i++)
	{
		Vec2f current = lines[i];
		float p = current[0];
		float theta = current[1];

		if (p == 0 && theta == -100)
			continue;

		double xIntercept, yIntercept;
		xIntercept = p / cos(theta);
		yIntercept = p / (cos(theta) * sin(theta));

		if (theta > CV_PI * 80 / 180 && theta < CV_PI * 100 / 180)
		{
			if (p<topEdge[0])
				topEdge = current;
			if (p>bottomEdge[0])
				bottomEdge = current;
		}
		else if (theta < CV_PI * 10 / 180 || theta > CV_PI * 170 / 180)
		{
			if (xIntercept>rightXIntercept)
			{
				rightEdge = current;
				rightXIntercept = xIntercept;
			}
			else if (xIntercept <= leftXIntercept)
			{
				leftEdge = current;
				leftXIntercept = xIntercept;
			}
		}
	}

	Point left1, left2, right1, right2, bottom1, bottom2, top1, top2;
	int height = processed_grid.size().height;
	int width = processed_grid.size().width;

	if (leftEdge[1] != 0)
	{
		left1.x = 0;
		left1.y = leftEdge[0] / sin(leftEdge[1]);
		left2.x = width;
		left2.y = -left2.x / tan(leftEdge[1]) + left1.y;
	}
	else
	{
		left1.y = 0;
		left1.x = leftEdge[0] / cos(leftEdge[1]);
		left2.y = height;
		left2.x = left1.x - height*tan(leftEdge[1]);
	}

	if (rightEdge[1] != 0)
	{
		right1.x = 0;
		right1.y = rightEdge[0] / sin(rightEdge[1]);
		right2.x = width;
		right2.y = -right2.x / tan(rightEdge[1]) + right1.y;
	}
	else
	{
		right1.y = 0;
		right1.x = rightEdge[0] / cos(rightEdge[1]);
		right2.y = height;
		right2.x = right1.x - height*tan(rightEdge[1]);
	}

	bottom1.x = 0;
	bottom1.y = bottomEdge[0] / sin(bottomEdge[1]);

	bottom2.x = width;
	bottom2.y = -bottom2.x / tan(bottomEdge[1]) + bottom1.y;

	top1.x = 0;
	top1.y = topEdge[0] / sin(topEdge[1]);
	top2.x = width;
	top2.y = -top2.x / tan(topEdge[1]) + top1.y;

	// Next, we find the intersection of  these four lines
	double leftA = left2.y - left1.y;
	double leftB = left1.x - left2.x;
	double leftC = leftA*left1.x + leftB*left1.y;

	double rightA = right2.y - right1.y;
	double rightB = right1.x - right2.x;
	double rightC = rightA*right1.x + rightB*right1.y;

	double topA = top2.y - top1.y;
	double topB = top1.x - top2.x;
	double topC = topA*top1.x + topB*top1.y;

	double bottomA = bottom2.y - bottom1.y;
	double bottomB = bottom1.x - bottom2.x;
	double bottomC = bottomA*bottom1.x + bottomB*bottom1.y;

	// Intersection of left and top
	double detTopLeft = leftA*topB - leftB*topA;
	CvPoint ptTopLeft = cvPoint((topB*leftC - leftB*topC) / detTopLeft, (leftA*topC - topA*leftC) / detTopLeft);

	// Intersection of top and right
	double detTopRight = rightA*topB - rightB*topA;
	CvPoint ptTopRight = cvPoint((topB*rightC - rightB*topC) / detTopRight, (rightA*topC - topA*rightC) / detTopRight);

	// Intersection of right and bottom
	double detBottomRight = rightA*bottomB - rightB*bottomA;
	CvPoint ptBottomRight = cvPoint((bottomB*rightC - rightB*bottomC) / detBottomRight, (rightA*bottomC - bottomA*rightC) / detBottomRight);

	// Intersection of bottom and left
	double detBottomLeft = leftA*bottomB - leftB*bottomA;
	CvPoint ptBottomLeft = cvPoint((bottomB*leftC - leftB*bottomC) / detBottomLeft, (leftA*bottomC - bottomA*leftC) / detBottomLeft);

	int maxLength = (ptBottomLeft.x - ptBottomRight.x)*(ptBottomLeft.x - ptBottomRight.x) + (ptBottomLeft.y - ptBottomRight.y)*(ptBottomLeft.y - ptBottomRight.y);
	int temp = (ptTopRight.x - ptBottomRight.x)*(ptTopRight.x - ptBottomRight.x) + (ptTopRight.y - ptBottomRight.y)*(ptTopRight.y - ptBottomRight.y);

	if (temp>maxLength)
		maxLength = temp;

	temp = (ptTopRight.x - ptTopLeft.x)*(ptTopRight.x - ptTopLeft.x) + (ptTopRight.y - ptTopLeft.y)*(ptTopRight.y - ptTopLeft.y);

	if (temp>maxLength)
		maxLength = temp;

	temp = (ptBottomLeft.x - ptTopLeft.x)*(ptBottomLeft.x - ptTopLeft.x) + (ptBottomLeft.y - ptTopLeft.y)*(ptBottomLeft.y - ptTopLeft.y);

	if (temp>maxLength)
		maxLength = temp;

	maxLength = sqrt((double)maxLength);

	Point2f src[4], dst[4];
	src[0] = ptTopLeft;
	dst[0] = Point2f(0, 0);
	src[1] = ptTopRight;
	dst[1] = Point2f(maxLength - 1, 0);
	src[2] = ptBottomRight;
	dst[2] = Point2f(maxLength - 1, maxLength - 1);
	src[3] = ptBottomLeft;
	dst[3] = Point2f(0, maxLength - 1);

	Mat undistorted = Mat(Size(maxLength, maxLength), CV_8UC1);

	warpPerspective(original_grid, undistorted, getPerspectiveTransform(src, dst), Size(maxLength, maxLength));
	return undistorted;
}