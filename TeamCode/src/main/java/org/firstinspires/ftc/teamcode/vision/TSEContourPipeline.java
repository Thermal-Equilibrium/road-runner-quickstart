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
public class TSEContourPipeline extends OpenCvPipeline {
	private Mat alternateColorSpace = new Mat();

	public static Scalar lower = new Scalar(0,0,0);
	public static Scalar higher = new Scalar(0,0,0);

	public static boolean tuning_mode = false;
	public static double high1 = 120;
	public static double high2 = 250;
	public static double high3 = 150;
	public static double low1 = 80;
	public static double low2 = 200;
	public static double low3 = 80;


	public boolean hasStarted = false;

	public boolean right_is_visible = true;

	private Mat maskedInputMat = new Mat();
	private Mat binaryMat      = new Mat();

	ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

	position TSEPosition = position.MIDDLE;

	@Override
	public Mat processFrame(Mat input) {

		if (!hasStarted) {
			hasStarted = true;
		}

		lower = new Scalar(low1,low2,low3);
		higher = new Scalar(high1, high2, high3);

		int columns = input.rows();

		double left = columns / 3.0;
		double right = columns / 1.5;


		Imgproc.cvtColor(input, alternateColorSpace, Imgproc.COLOR_RGB2YCrCb);
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
		Imgproc.blur(input,input,new Size(50,50));
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
			Imgproc.drawContours(binaryMat,contours,largestContourIndex,new Scalar(255,0,0));
			Imgproc.rectangle(binaryMat,rectangle,new Scalar(0,255,255));
			int boundingBoxPosition = rectangle.y;
			if (boundingBoxPosition < left) {
				TSEPosition = position.RIGHT;
			} else if (boundingBoxPosition > left && boundingBoxPosition < right) {
				TSEPosition = position.MIDDLE;
			} else {
				TSEPosition = position.LEFT;
			}
			Imgproc.putText(input,"" + TSEPosition,new Point(rectangle.x - 30,rectangle.y - 20),2,3,new Scalar(255,0,0));


			System.out.println("TSE Position is " + boundingBoxPosition + " left bounding pos is " + left + " right bounding pos is " + right);
		} catch (IndexOutOfBoundsException e) {
			System.out.println("out of bounds exception was caught, try extending the detection tolerances");
		}


		if (tuning_mode) return binaryMat;

		return input;
	}

	public enum position {
		LEFT,
		MIDDLE,
		RIGHT
	}

	public position getTSEPosition() {
		return TSEPosition;
	}

	public void setRight_is_visible(boolean right_is_visible) {
		this.right_is_visible = right_is_visible;
	}
}
