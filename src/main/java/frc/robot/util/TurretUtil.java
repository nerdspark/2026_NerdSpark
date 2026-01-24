package frc.robot.util;

import java.math.BigInteger;

import frc.robot.Constants.TurretConstants;

public final class TurretUtil {
    /**
    * Calculates the true mathematical (floored) modulo for doubles.
    * The result has the same sign as the divisor (y).
    */
    public static double floorMod(double x, double y) {
        // Formula: x - y * floor(x / y)
        return x - y * Math.floor(x / y);
    }

    public static double modInverse(double a, double m) {
        // 1. Convert doubles to BigIntegers
        BigInteger bigA = BigInteger.valueOf((long) a);
        BigInteger bigM = BigInteger.valueOf((long) m);
        
        // 2. Use the built-in modInverse (uses Extended Euclidean Algorithm)
        // This will throw an ArithmeticException if gcd(a, m) != 1
        BigInteger result = bigA.modInverse(bigM);
        
        return result.doubleValue();
    }

    // Normalizes to [-pi, pi]
    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static double degreesToMotorRotations(double turretDegrees) {
        return (turretDegrees / 360.0) * TurretConstants.spinRatio;
    }

    public static double normalize180(double degrees) {
        degrees %= 360.0;        // wrap within -360..360
        if (degrees > 180.0) {
            degrees -= 360.0;    // move into -180..180
        } else if (degrees < -180.0) {
            degrees += 360.0;    // move into -180..180
        }
        return degrees;
    }
}