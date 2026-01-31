package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class Vision {

        public static boolean DOGLOG_ENABLED = false;

        public static final boolean USE_VISION = true;

        public static final String kCameraNameFrontRight = "FrontRightCamera";
        public static final Transform3d kRobotToCamFrontRight =
                new Transform3d(new Translation3d(Units.inchesToMeters(11.5), -Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(0))); //TODO: determine XYZ

        public static final String kCameraNameFrontLeft = "FrontLeftCamera";
        public static final Transform3d kRobotToCamFrontLeft =
                new Transform3d(new Translation3d(Units.inchesToMeters(11.5), Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(55))); //TODO: determine XYZ

        public static final String kCameraNameBackRight = "BackRightCamera";
        public static final Transform3d kRobotToCamBackRight =
                new Transform3d(new Translation3d(-Units.inchesToMeters(11.5), -Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(180))); //TODO: determine XYZ

        public static final String kCameraNameBackLeft = "BackLeftCamera";
        public static final Transform3d kRobotToCamBackLeft =
                new Transform3d(new Translation3d(-Units.inchesToMeters(11.5), Units.inchesToMeters((9)), Units.inchesToMeters(12.5)), 
                new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(125))); //TODO: determine XYZ
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.013; // Changed from 0.05 3/8/25
        public static final double TRANSLATION_TOLERANCE_Y = 0.013; // Changed from 0.05 3/8/25
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.3); // /deg

        //Below same as pathplanner constants
        public static final double MAX_VELOCITY = 3; 
        public static final double MAX_ACCELERATION = 5; 
        public static final double MAX_VELOCITY_ROTATION = 540; 
        public static final double MAX_ACCELARATION_ROTATION = 720;
        
        public static final double VELOCITY_TOLERANCE_X = 4;
        public static final double VELOCITY_TOLERANCE_Y = 4;
        public static final double VELOCITY_TOLERANCE_OMEGA = 5;

        public static final double kPXController = 15; //2.5
        public static final double kIXController = 0.0 ; //0.01d
        public static final double kDXController = 0.1d;
    
        public static final double kPThetaController = 7; //2
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0; //0.0041

        public static final double kPoseAmbiguityThreshold = 0.2;
        public static final double kSingleTagDistanceThreshold = 2.0;

        
    }

    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final String pigeonCanBus = "canivore1";

    public enum AutoDrivePoses {
        LEFT(new Pose2d(FieldConstants.Hub.innerCenterPoint2D.plus(new Translation2d(Units.inchesToMeters(-96), Units.inchesToMeters(-72))), Rotation2d.fromDegrees(0)), 2.4397e6),
        CENTER(new Pose2d(FieldConstants.Hub.innerCenterPoint2D.plus(new Translation2d(Units.inchesToMeters(-96), 0)), Rotation2d.fromDegrees(0)), 2.4397e6),
        RIGHT(new Pose2d(FieldConstants.Hub.innerCenterPoint2D.plus(new Translation2d(Units.inchesToMeters(-96), Units.inchesToMeters(72))), Rotation2d.fromDegrees(0)), 2.4397e6),
        // CLIMB_LEFT(new Pose2d(FieldConstants.Tower.leftUpright.getX() + Units.inchesToMeters(13), FieldConstants.Tower.leftUpright.getY(), Rotation2d.fromDegrees(180)), 2.4397e6), //RELATIVE TO DRIVER STATION
        // CLIMB_RIGHT(new Pose2d(FieldConstants.Tower.rightUpright.getX() + Units.inchesToMeters(13), FieldConstants.Tower.rightUpright.getY(), Rotation2d.fromDegrees(180)), 2.4397e6);   //RELATIVE TO DRIVER STATION
        CLIMB_LEFT(new Pose2d(FieldConstants.Tower.leftUpright.plus(new Translation2d(Units.inchesToMeters(13), 0)), Rotation2d.fromDegrees(180)), 2.4397e6),  //RELATIVE TO DRIVER STATION
        CLIMB_RIGHT(new Pose2d(FieldConstants.Tower.rightUpright.plus(new Translation2d(Units.inchesToMeters(13), 0)), Rotation2d.fromDegrees(180)), 2.4397e6);   //RELATIVE TO DRIVER STATION

        private final Pose2d pose;   // in kilograms
        private final double hoodAngle; // in meters
        AutoDrivePoses(Pose2d pose, double hoodAngle) {
        this.pose = pose;
        this.hoodAngle = hoodAngle;
         }

         public Pose2d getPose() {
            return this.pose;
         }

    }
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
        public static final int shotTrajectoryPoints = 25;

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

    public static final class turretTargetConstants {
        public static final String enableKey = "TurretTarget/Enable";
        public static final String targetXKey = "TurretTarget/X";
        public static final String targetYKey = "TurretTarget/Y";
        public static final boolean defaultEnable = false;
        public static final double defaultTargetX = field.blueHub.getX();
        public static final double defaultTargetY = field.blueHub.getY();
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
