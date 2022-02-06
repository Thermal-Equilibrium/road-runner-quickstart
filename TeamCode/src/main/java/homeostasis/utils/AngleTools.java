package homeostasis.utils;

public class AngleTools {

    /**
     * two times PI
     */
    public static final double TAU = 2 * Math.PI;

    /**
     * ensure angles are within 180 to -180 (pi, -pi)
     *
     * @param radians the radian angle we are assessing
     * @return the adjusted angle in radians
     */
    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= TAU;
        }
        while (radians < -Math.PI) {
            radians += TAU;
        }
        return radians;
    }

}

