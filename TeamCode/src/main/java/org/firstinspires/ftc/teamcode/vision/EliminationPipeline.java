package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class EliminationPipeline extends OpenCvPipeline {
	private Mat alternateColorSpace = new Mat();

	public static Scalar lower = new Scalar(123.4,145.0,0);
	public static Scalar higher = new Scalar(255,255,82.3);
	public static boolean tuning_mode = false;

	public static double high1 = 120;
	public static double high2 = 240;
	public static double high3 = 100;
	public static double low1 = 60;
	public static double low2 = 200;
	public static double low3 = 70;


	public boolean hasStarted = false;


	public static double minimum_contour_size = 40000;

	public boolean right_is_visible = false;

	private Mat maskedInputMat = new Mat();
	private Mat binaryMat      = new Mat();

	ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

	TSEContourPipeline.position TSEPosition = TSEContourPipeline.position.MIDDLE;

	@Override
	public Mat processFrame(Mat input) {

		lower = new Scalar(low1,low2,low3);
		higher = new Scalar(high1, high2, high3);

		int columns = input.cols();

		int right = columns / 2;
		double second_right = columns / 2.5;


		Imgproc.cvtColor(input, alternateColorSpace, Imgproc.COLOR_RGB2HSV);
		Imgproc.blur(input,input,new Size(50,50));
		Core.inRange(alternateColorSpace,lower,higher,binaryMat);
		maskedInputMat.release();
		/*
		 * Now, with our binary Mat, we perform a "bitwise and"
		 * to our input image, meaning that we will perform a mask
		 * which will include the pixels from our input Mat which
		 * are "255" in our binary Mat (meaning that they're inside
		 * the range) and will discard any other pixel outside the
		 * range (RGB 0, 0, 0. All discarded pixels will be black)
		 */
		Core.bitwise_and(input, input, maskedInputMat, binaryMat);
		contours.clear();
		Imgproc.findContours(binaryMat,contours,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		double largestContourSize = 0;
		int largestContourIndex = 0;
		for (int i = 0; i < contours.size(); ++i) {

			double area = Imgproc.contourArea(contours.get(i));
			if (area > largestContourSize) {
				largestContourSize = area;
				largestContourIndex = i;
			}
		}
		try {
			MatOfPoint largestContour = contours.get(largestContourIndex);
			Rect rectangle = Imgproc.boundingRect(largestContour);
			Imgproc.drawContours(input,contours,largestContourIndex,new Scalar(255,0,0));
			Imgproc.rectangle(input,rectangle,new Scalar(0,255,255));
			int boundingBoxPosition = rectangle.x;

			if (right_is_visible) {

				if (boundingBoxPosition < right) {
					TSEPosition = TSEContourPipeline.position.MIDDLE;
				} else {
					TSEPosition = TSEContourPipeline.position.RIGHT;
				}
				if (largestContourSize < minimum_contour_size) {
					TSEPosition = TSEContourPipeline.position.LEFT;
				}

			} else {
				if (boundingBoxPosition < second_right) {
					TSEPosition = TSEContourPipeline.position.LEFT;
				} else {
					TSEPosition = TSEContourPipeline.position.MIDDLE;
				}

				if (largestContourSize < minimum_contour_size) {
					TSEPosition = TSEContourPipeline.position.RIGHT;
				}

			}

		} catch (IndexOutOfBoundsException e) {

			// if we get here and the right is theoretically visible then it must be the left.
			if (right_is_visible) {
				TSEPosition = TSEContourPipeline.position.LEFT;
			} else {
				TSEPosition = TSEContourPipeline.position.RIGHT;
			}

			System.out.println("out of bounds exception was caught, try extending the detection tolerances or it isnt visible lol");
		}
		Imgproc.putText(input,"" + TSEPosition + " area is: " + largestContourSize, new Point(columns / 2.0, binaryMat.width() / 2.0),2,1,new Scalar(255,0,0));
		hasStarted = true;

		if (tuning_mode) {
			return binaryMat;
		}

		return input;
	}

	public boolean isHasStarted() {
		return hasStarted;
	}

	public enum position {
		LEFT,
		MIDDLE,
		RIGHT
	}

	public TSEContourPipeline.position getTSEPosition() {
		return TSEPosition;
	}


	public void setRight_is_visible(boolean right_is_visible) {
		this.right_is_visible = right_is_visible;
	}
}
