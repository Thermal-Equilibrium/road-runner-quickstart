package org.firstinspires.ftc.teamcode.Utils.Matrix;

public class Matrix {

	protected int rows;
	protected int cols;
	protected double [][] arr;

	public Matrix(int rows, int cols) {
		this.rows = rows;
		this.cols = cols;
		arr = new double[rows][cols];
	}



}
