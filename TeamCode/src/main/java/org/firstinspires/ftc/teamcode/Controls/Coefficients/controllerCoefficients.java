package org.firstinspires.ftc.teamcode.Controls.Coefficients;

public class controllerCoefficients {
	public static final double compBotVelocity = 70;
	public static final double compBotAcceleration = 35;
	// controller coefficients for competition 10wd
	public static final PIDFCoefficients compBotTurn = new PIDFCoefficients(0.9, 0, 0,0,0,0.1);
	public static final PVParams translationCoefficients = new PVParams(0.15,.015,0.07,1/compBotVelocity,0,0,.8,1.1);
	// controller coefficients for off season 6wd
	public static final PIDFCoefficients protoBotTurn = new PIDFCoefficients(0.59, 0.16, 0.09, 0, 0, 0.1280);

	// controller coefficients for linear slide subsystem
	public static final PIDFCoefficients slideCoefficients = new PIDFCoefficients(0.01, 0.015, 0,0,0,0.1);




	public static final double protoBotVelocity = 100;
	public static final double protoBotAcceleration = 100;
	public static final double protoBotJerk = 250;



}
