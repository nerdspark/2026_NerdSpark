package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class Constants {
    public static final class TurretConstants {
        public static final int spinMotorId = 30;
        public static final int hoodMotorId = 31;
        public static final int shootMotorId = 32;
        public static final double spinKp = 0.0; // 0.02 What currently works in the sim might need to change in real life
        public static final double spinKi = 0.0;
        public static final double spinKd = 0.0; // 0.01 What currently works in the sim might need to change in real life
        public static final double spinStatorCurrentLimit = 40.0;

        public static final double hoodKp = 0.0; // Keep for trim if needed
        public static final double hoodKi = 0.0;
        public static final double hoodKd = 0.0;
        public static final double hoodKsVolts = 0.0;
        public static final double hoodKgVolts = 0.0;
        public static final double hoodKvVolts = 0.0;
        public static final double hoodKaVolts = 0.0;
        public static final double hoodFeedforwardOffsetRad = 0.0;
        public static final double hoodStatorCurrentLimit = 40.0;

        public static final double shootKp = 0.0;
        public static final double shootKi = 0.0;
        public static final double shootKd = 0.0;
        public static final double shootStatorCurrentLimit = 60.0;
        public static final double spinRatio = 20;
        public static final double hoodRatio = 30;
        public static final double spinCancoder1Ratio = 200/19.0; // Turret gear teeth / encoder A gear teeth
        public static final double spinCancoder2Ratio = 200/17.0; // Turret gear teeth / encoder B gear teeth
        public static final double shooterRatio = 2;
        public static final double shooterWheelRadius = 0.0508; // in meters
        public static final double shooterMaxMotorRps = 100.0;
        public static final double shotAngleStepDeg = 0.5;
        public static final double shooterMuzzleHeightMeters = 1.0;
        public static final double targetHeightMeters = 2.64;
        public static final double ballMassKg = 0.2268;
        public static final double ballDiameterMeters = 0.1501;

        public static final double hoodStow = 0.0;
        public static final double turretMinDegrees = -90.0;
        public static final double turretMaxDegrees = 90.0;
        public static final double hoodMinDegrees = -90.0;
        public static final double hoodMaxDegrees = 90.0;
        public static final double turretAimErrorScale = 0.25;
        public static final double chassisRotationBufferDegrees = 85.0;
        public static final double blueHubMaxX = 4.5;
        public static final double redHubMinX = 12.0;
        public static final double turretOffset = 0.0; // In radians
        
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

    public static final class turretTelemetryConstants {
        public static final String withinLimitKey = "Turret Within Limit?";
        public static final String angleDegKey = "Turret/AngleDeg";
        public static final String spinSetpointRotKey = "Turret/SpinSetpointRot";
        public static final String spinClosedLoopOutputKey = "Turret/SpinClosedLoopOutput";
        public static final String spinMotorVoltsKey = "Turret/SpinMotorVolts";
        public static final String hoodAngleDegKey = "Hood/AngleDeg";
        public static final String hoodSetpointRotKey = "Hood/SetpointRot";
        public static final String hoodClosedLoopOutputKey = "Hood/ClosedLoopOutput";
        public static final String hoodMotorVoltsKey = "Hood/MotorVolts";
        public static final String shooterSetpointRpsKey = "Shooter/SetpointRps";
        public static final String shooterMotorRpsKey = "Shooter/MotorRps";
        public static final String shooterWheelRpsKey = "Shooter/WheelRps";
        public static final String shooterMotorVoltsKey = "Shooter/MotorVolts";
    }

    public static final class turretTuningConstants {
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

    public static final class hoodTuningConstants {
        public static final String enableKey = "HoodTune/Enable";
        public static final String zeroKey = "HoodTune/Zero";
        public static final String activeKey = "HoodTune/Active";
        public static final String kPKey = "HoodTune/kP";
        public static final String kIKey = "HoodTune/kI";
        public static final String kDKey = "HoodTune/kD";
        public static final String setpointDegKey = "HoodTune/SetpointDeg";
        public static final boolean defaultEnable = false;
        public static final boolean defaultZero = false;
        public static final double defaultKP = 0.0;
        public static final double defaultKI = 0.0;
        public static final double defaultKD = 0.0;
        public static final double defaultSetpointDeg = 0.0;
    }

    public static final class turretSimConstants {
        public static final double loopPeriodSeconds = 0.02;
        public static final double turretJ = 0.004;
        public static final double hoodJ = 0.002;
        public static final double shooterJ = 0.001;
        public static final int motorCount = 1;
    }

    public static final class field {
        public static final Translation2d blueHub = new Translation2d(4.615, 4.040); // May need to redo
        public static final Translation2d redHub = new Translation2d(11.915, 4.040); // May need to redo

        public static final Translation2d leftPass = new Translation2d();
        public static final Translation2d rightPass = new Translation2d();
    }
}
