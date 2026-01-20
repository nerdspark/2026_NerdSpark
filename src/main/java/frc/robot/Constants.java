package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class Constants {
    public static final class TurretConstants {
        public static final double spinRatio = 10;
        public static final double spinCancoder1Ratio = 200/19.0; // Turret gear teeth / encoder A gear teeth
        public static final double spinCancoder2Ratio = 200/17.0; // Turret gear teeth / encoder B gear teeth
        public static final double shooterRatio = 2; // TODO
        public static final double shooterWheelRadius = 0.0508; // in meters TODO

        public static final double hoodStow = 0.0;
        public static final double turretOffset = 0;
        
        public static InterpolatingTreeMap<Double, double[]> shooterMap =  new InterpolatingTreeMap<>(
            // inverseInterpolator for Double keys
            InverseInterpolator.forDouble(),

            // interpolator for Double[] values
            (lower, upper, t) -> {
                double[] result = new double[lower.length];
                for (int i = 0; i < lower.length; i++) {
                    result[i] = lower[i] + t * (upper[i] - lower[i]);
                }
                return result;
            }
        );

        // Add numbers to hash map here, Distance, [Hood Pose, Wheel Speed (motor RPS)]
        static {
            // shooterMap.put(, new double[] {});
        }
    }

    public static final class field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo

        public static final Translation2d leftPass = new Translation2d();
        public static final Translation2d rightPass = new Translation2d();
    }
}