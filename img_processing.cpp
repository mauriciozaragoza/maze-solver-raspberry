#include "img_processing.h"

void ImageProc::processing_grid(Mat original_grid, Mat result_grid)
{
	GaussianBlur(original_grid, result_grid, Size(11, 11), 0);
	adaptiveThreshold(result_grid, result_grid, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 3);
	bitwise_not(result_grid, result_grid);
}

void ImageProc::get_point(Mat grid, Point &point, int color)
{
	Mat HSV;
	Mat hsv_img;

	cvtColor(grid, HSV, CV_BGR2HSV);
	inRange(HSV, Scalar(color, 150, 150), Scalar(color + 10, 255, 255), hsv_img);

	for (int y = 0; y < hsv_img.size().height; y++)
	{
		uchar *row = hsv_img.ptr(y);
		for (int x = 0; x < hsv_img.size().width; x++)
		{
			if (row[x] == 255)
			{
				point.x = x;
				point.y = y;
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