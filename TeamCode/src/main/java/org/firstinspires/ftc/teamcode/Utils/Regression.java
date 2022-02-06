package org.firstinspires.ftc.teamcode.Utils;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.Arrays;
import java.util.stream.IntStream;

public class Regression {

	/**
	 *
	 * https://rosettacode.org/wiki/Polynomial_regression#Java
	 * @param x x inputs, ie time steps
	 * @param y y inputs, ie values at the time steps
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	public static Quadratic quadraticRegression(double[] x, double[] y) {
		int n = x.length;
		int[] r = IntStream.range(0, n).toArray();
		double xm = Arrays.stream(x).average().orElse(Double.NaN);
		double ym = Arrays.stream(y).average().orElse(Double.NaN);
		double x2m = Arrays.stream(r).map(a -> a * a).average().orElse(Double.NaN);
		double x3m = Arrays.stream(r).map(a -> a * a * a).average().orElse(Double.NaN);
		double x4m = Arrays.stream(r).map(a -> a * a * a * a).average().orElse(Double.NaN);
		double xym = 0.0;
		for (int i = 0; i < x.length && i < y.length; ++i) {
			xym += x[i] * y[i];
		}
		xym /= Math.min(x.length, y.length);
		double x2ym = 0.0;
		for (int i = 0; i < x.length && i < y.length; ++i) {
			x2ym += x[i] * x[i] * y[i];
		}
		x2ym /= Math.min(x.length, y.length);

		double sxx = x2m - xm * xm;
		double sxy = xym - xm * ym;
		double sxx2 = x3m - xm * x2m;
		double sx2x2 = x4m - x2m * x2m;
		double sx2y = x2ym - x2m * ym;

		double b = (sxy * sx2x2 - sx2y * sxx2) / (sxx * sx2x2 - sxx2 * sxx2);
		double c = (sx2y * sxx - sxy * sxx2) / (sxx * sx2x2 - sxx2 * sxx2);
		double a = ym - b * xm - c * x2m;

		return new Quadratic(c,b,a);

	}


}
