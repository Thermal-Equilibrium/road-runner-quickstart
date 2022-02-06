package org.firstinspires.ftc.teamcode.vision;



import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class GoalDetectionPipeline extends OpenCvPipeline {
	private Mat alternateColorSpace = new Mat();
	public Scalar lower = new Scalar(0,0,0);
	public Scalar higher = new Scalar(255,255,255);
	public double aspectRatio = 100;
	private Mat maskedInputMat = new Mat();
	private Mat binaryMat      = new Mat();

	ArrayList<MatOfPoint> contours = new ArrayList<>();

	@Override
	public Mat processFrame(Mat input) {


		Imgproc.cvtColor(input, alternateColorSpace, Imgproc.COLOR_RGB2HSV);

		Core.inRange(alternateColorSpace,lower,higher,binaryMat);
		maskedInputMat.release();

		//Imgproc.medianBlur(binaryMat,binaryMat,10);
		Core.bitwise_and(input, input, maskedInputMat, binaryMat);

		contours.clear();
		Imgproc.findContours(binaryMat,contours,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); ++i) {
			MatOfPoint contour = contours.get(i);
				if (contour.size().height > 50 && (contour.size().height / contour.size().width) < aspectRatio) { // && contour.size().area() < 100
					Imgproc.drawContours(input, contours, i, new Scalar(255, i * 10, 0));
				}

		}


		return binaryMat;
	}



}
