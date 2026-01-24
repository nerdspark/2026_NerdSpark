package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class Constants {
    public static final class TurretConstants {
        public static final int spinMotorId = 25;
        public static final double spinKp = 3;
        public static final double spinKi = 0.0;
        public static final double spinKd = 0.08;
        public static final double spinStatorCurrentLimit = 30.0;
        
        public static final int hoodMotorId = 31;
        public static final double hoodKp = 0.0;
        public static final double hoodKi = 0.0;
        public static final double hoodKd = 0.0;
        public static final double hoodStatorCurrentLimit = 40.0;

        public static final int shootMotorId = 32;
        public static final double shootKp = 0.0;
        public static final double shootKi = 0.0;
        public static final double shootKd = 0.0;
        public static final double shootStatorCurrentLimit = 60.0;

        public static final double spinRatio = 210/21.0;
        public static final double spinTeeth = 210;
        public static final double spinCancoder1Teeth = 15;
        public static final double spinCancoder2Teeth = 14; 
        public static final double shooterRatio = 2; // TODO
        public static final double shooterWheelRadius = 0.0508; // in meters TODO

        public static final double hoodStow = 0.0;
        public static final double turretMinDegrees = 0;
        public static final double turretMaxDegrees = 360;
        public static final double turretAimErrorScale = 0.25;
        public static final double chassisRotationBufferDegrees = 85.0;
        public static final double blueHubMaxX = 4.5;
        public static final double redHubMinX = 12.0;
        public static final double turretOffset = 0; // In Radians
        
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
            shooterMap.put(1.0, new double[] {0, 0});
        }
    }

    public final class turretTelemetryConstants {
        public static final String withinLimitKey = "Turret Within Limit?";
        public static final String angleDegKey = "Turret/AngleDeg";
        public static final String spinSetpointRotKey = "Turret/SpinSetpointRot";
        public static final String spinClosedLoopOutputKey = "Turret/SpinClosedLoopOutput";
        public static final String spinMotorVoltsKey = "Turret/SpinMotorVolts";
    }

    public final class turretTuningConstants {
        public static final String enableKey = "TurretTune/Enable";
        public static final String zeroKey = "TurretTune/Zero";
        public static final String activeKey = "TurretTune/Active";
        public static final String kPKey = "TurretTune/kP";
        public static final String kIKey = "TurretTune/kI";
        public static final String kDKey = "TurretTune/kD";
        public static final String setpointDegKey = "TurretTune/SetpointDeg";
        public static final boolean defaultEnable = false;
        public static final boolean defaultZero = false;
        public static final double defaultKP = 0.0;
        public static final double defaultKI = 0.0;
        public static final double defaultKD = 0.0;
        public static final double defaultSetpointDeg = 0.0;
    }

    public final class turretSimConstants {
        public static final double loopPeriodSeconds = 0.02;
        public static final double turretJ = 0.004;
        public static final int motorCount = 1;
    }

    public static final class field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo

        public static final Translation2d leftPass = new Translation2d();
        public static final Translation2d rightPass = new Translation2d();
    }
}
