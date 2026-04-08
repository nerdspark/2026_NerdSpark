package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ShooterParams;

public final class Constants {
    public static final double gyroP = 8.3;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.22;

    public static final String CANbus = "canivore1";

    public static class Vision {
        public static boolean DOGLOG_ENABLED = false;
        public static final boolean USE_VISION = true;

        public static final String kCameraNameFrontRight = "FrontRightCamera";
        public static final Transform3d kRobotToCamFrontRight = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.93), -Units.inchesToMeters(10.59), Units.inchesToMeters(7.74)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(-50))
        );

        public static final String kCameraNameFrontLeft = "FrontLeftCamera";
        public static final Transform3d kRobotToCamFrontLeft = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.93), Units.inchesToMeters(10.59), Units.inchesToMeters(7.74)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(50))
        );

        public static final String kCameraNameBackRight = "BackRightCamera";
        public static final Transform3d kRobotToCamBackRight = new Transform3d(
            new Translation3d(Units.inchesToMeters(8.12), -Units.inchesToMeters(11.06), Units.inchesToMeters(7.74)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(-110))
        );

        public static final String kCameraNameBackLeft = "BackLeftCamera";
        public static final Transform3d kRobotToCamBackLeft = new Transform3d(
            new Translation3d(Units.inchesToMeters(8.12), Units.inchesToMeters(11.06), Units.inchesToMeters(7.74)),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-25), Math.toRadians(110))
        );

        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double kXYStdDev = 0.4;
        public static final double kThetaStdDev = 1;

        public static final double TRANSLATION_TOLERANCE_X = 0.013;
        public static final double TRANSLATION_TOLERANCE_Y = 0.013;
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.3);

        public static final double MAX_VELOCITY = 3;
        public static final double MAX_ACCELERATION = 5;
        public static final double MAX_VELOCITY_ROTATION = 540;
        public static final double MAX_ACCELARATION_ROTATION = 720;

        public static final double VELOCITY_TOLERANCE_X = 4;
        public static final double VELOCITY_TOLERANCE_Y = 4;
        public static final double VELOCITY_TOLERANCE_OMEGA = 5;

        public static final double kPXController = 15;
        public static final double kIXController = 0.0;
        public static final double kDXController = 0.1;
        public static final double kPThetaController = 7;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

        public static final double kPoseAmbiguityThreshold = 0.2;
        public static final double kSingleTagDistanceThreshold = 2.0;

        public static final double visionCorrectedRecentlyThreshold = 2; 

        public enum VisionStatus { 
            BEST, OK, BAD
        }
    }

    public static final class TurretConstants {
        public static final double spinRatio = 210 / 21.0;
        public static final double spinTeeth = 210;
        public static final double spinCancoder1Teeth = 15;
        public static final double spinCancoder2Teeth = 14;

        public static final double hoodRatio = 50;
        public static final double shooterWheelRadius = Units.inchesToMeters(4);
        // Ball exit speed as a fraction of wheel surface speed.
        // Set to 1.0 for pure physics model; slippage is handled by SlippageCorrectionMap.
        public static final double shooterLaunchEfficiency = 1.0;
        public static final double shooterMaxMotorRps = 5700.0 / 60.0;
        public static final double shootGearRatio = 2.0;
        public static final double shotAngleStepDeg = 0.5;
        public static final double shooterMuzzleHeightMeters = Units.inchesToMeters(17);
        public static final double targetHeightMeters = Units.inchesToMeters(56.5); //72
        public static final double hoodZeroDegrees = 22.5;
        public static final double hoodMinDegrees = 22.5;
        public static final double hoodMaxDegrees = 90.0;
        public static final double ikEntryAngleTargetDeg = 40.0;
        public static final double ikEntryAngleToleranceDeg = 2.5;
        public static final double lowHoodPreferredDegrees = hoodMinDegrees;
        public static final double passTargetRadiusMeters = Units.inchesToMeters(5.91) / 2.0;
        public static final int passTargetCirclePoints = 24;

        // Offsets for the secondary point to get a better arc
        public static final double secondPointHeight = Units.inchesToMeters(25); //21.5
        public static final double secondPointDistance = Units.inchesToMeters(19);

        public static final Translation2d robotToTurret = new Translation2d(Units.inchesToMeters(6.05), -Units.inchesToMeters(0.279));

        public static final double hoodRetractTime = 0.28;

        public static InterpolatingTreeMap<Double, ShooterParams> map = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            (start, end, t) -> start.interpolate(end, t)
        );

        static {
            // Passing
            map.put(Double.MAX_VALUE, new ShooterParams(4.8, 40));

            map.put(3.422, new ShooterParams(0.0, 32));
            map.put(5.084, new ShooterParams(1.25, 38));
            map.put(4.34, new ShooterParams(1.0, 35));
            map.put(3.678, new ShooterParams(0.0, 35));
            map.put(2.323, new ShooterParams(0.0, 28));
        }
    }

    public enum ShootMode {
        COAST,
        DUTY_CYCLE_BANG_BANG,
        TORQUE_CURRENT_BANG_BANG
    }

    public static final class SOTM {
        public final double turretAngle;
        public final double launchAngle;
        public final double launchMps;

        /**
         * SOTM compensated values
         * @param turretAngle turret needed angle before chassis angle comp in radians
         * @param launchAngle launch angle needed in radians
         * @param launchMps launch speed needed in mps
         */
        public SOTM(double turretAngle, double launchAngle, double launchMps) {
            this.turretAngle = turretAngle;
            this.launchAngle = launchAngle;
            this.launchMps = launchMps;
        }
    }

    public static final class IkSolution {
        public final double hoodDegrees;
        public final double motorRps;

        public IkSolution(double hoodDegrees, double motorRps) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
        }
    }

    public static final class TurretConfig {
        public static final int spinMotorId = 25;
        public static final double spinKp = 7.0;
        public static final double spinKi = 0.0;
        public static final double spinKd = 0.25;
        public static final double spinKs = 0.4;
        public static final double spinKv = 0.0;
        public static final double spinKa = 0.0;
        public static final double spinStatorCurrentLimit = 50.0; //120
        public static final double spinSupplyCurrent = 30.0;
        public static final double spinVelocity = 25;
        public static final double spinAccel = 100;

        public static final int spinCancoder1Id = 26;
        public static final double spinCancoder1Offset = -0.938232421875;
        public static final int spinCancoder2Id = 27;
        public static final double spinCancoder2Offset = -0.918212890625;

        public static final int hoodMotor1Id = 28;
        public static final int hoodMotor2Id = 29;
        public static final double hoodKp1 = 9.0;
        public static final double hoodKp2 = 9.0;
        public static final double hoodKi1 = 0.0;
        public static final double hoodKi2 = 0.0; 
        public static final double hoodKd1 = 0.2;
        public static final double hoodKd2 = 0.2;
        public static final double hoodKs1 = 1.0;
        public static final double hoodKs2 = 1.0;
        public static final double hoodKv1 = 0.0;
        public static final double hoodKv2 = 0.0;
        public static final double hoodKa1 = 0.0;
        public static final double hoodKa2 = 0.0;
        public static final double hoodStatorCurrentLimit = 40.0;
        public static final double hoodSupplyCurrentLimit = 20.0;
        public static final double hoodVelocity = 175;
        public static final double hoodAccel = hoodVelocity / 0.25;

        public static final int shootMotor1Id = 30;
        public static final int shootMotor2Id = 31;
        public static final double kp = 10.0;
        public static final double ki = 40.0;
        public static final double bangbangKp = 999999;
        public static final double peakDutyCycle = 1;
        public static final double peakTorque = 30;
        public static final double peakReverseTorque = -10;
        public static final double shootStatorCurrentLimit = 140.0;
        public static final double shootSupplyCurrentLimit = 60.0;
    }

    public static final class IndexConfig {
        public static final int passThroughId = 32;
        public static final int indexId = 33;
        public static final double statorCurretLimit = 50;
        public static final double supplyCurretLimit = 20;
    }

    public static final class turretTargetConstants {
        public static final String enableKey = "TurretTarget/Enable";
        public static final String targetXKey = "TurretTarget/X";
        public static final String targetYKey = "TurretTarget/Y";
        public static final boolean defaultEnable = false;
        public static final double defaultTargetX = FieldConstants.Hub.topCenterPoint.getX();
        public static final double defaultTargetY = FieldConstants.Hub.topCenterPoint.getY();
    }

    public static final class PassTargetConstants {
        public static final String enableKey = "PassTarget/Enable";
        public static final String targetXKey = "PassTarget/X";
        public static final String targetYKey = "PassTarget/Y";
        public static final String fieldClickKey = "Field/PassTargetClick";
        public static final boolean defaultEnable = false;
        public static final double defaultTargetX = FieldConstants.Hub.topCenterPoint.getX();
        public static final double defaultTargetY = FieldConstants.Hub.topCenterPoint.getY();
    }

    public static final class ShooterOffsetConstants {
        public static final String enableKey = "ShooterOffset/Enable";
        public static final String distancesKey = "ShooterOffset/Distances";
        public static final String hoodOffsetDegKey = "ShooterOffset/HoodOffsetDeg";
        public static final String motorRpsOffsetKey = "ShooterOffset/MotorRpsOffset";
        public static final boolean defaultEnable = false;
    }

    public static final class MapTuneConstants {
        public static final String enableKey = "MapTune/Enable";
        public static final String spinKey = "MapTune/SpinTarget";
        public static final String hoodKey = "MapTune/HoodTarget";
        public static final String shooterKey = "MapTune/ShooterTarget";
        public static final boolean defaultEnable = false;
    }

    public static final class AutoAimConstants {
        public static final String useIKSolverKey = "TurretTarget/UseIKSolver";
        public static final boolean defaultUseIKSolver = true;
    }

    public static final class SlippageCorrectionConstants {
        public static final double defaultDistanceOffset = Units.feetToMeters(-1.0);

        public static final String enableKey = "Slippage/Enable";
        public static final boolean defaultEnable = true;

        // Hood angle (degrees) held fixed during characterization shots.
        public static final String charHoodDegKey = "Slippage/CharHoodDeg";
        public static final double defaultCharHoodDeg = 35.0; 

        // Height (meters) of the surface where characterization balls land.
        // 0.0 = floor shots. Set to targetHeightMeters if shooting at the hub.
        public static final String charTargetHeightMetersKey = "Slippage/CharTargetHeightMeters";
        public static final double defaultCharTargetHeightMeters = 0.0;

        // Characterization data — HA=27 deg, floor shots.
        // X: commanded RPS   Y: observed horizontal distance (meters)
        public static final double[] defaultCommandedRpsPoints    = {25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0};
        public static final double[] defaultObservedDistanceMeters = {0.9906, 1.3208, 1,6256, 1,905, 2.3368, 2,8956, 3.429, 4.0132, 4.6228, 5.6388, 6.0452, 7.1628, 7.239, 8.7376};
        // rps  25, 30, 35, 40, 45, 50,  55,  60,  65,  70,  75,  80,  85,  90, 
        // inch 39, 52, 64, 75, 92, 114, 135, 158, 182, 222, 238, 282, 285, 344,
        

        // SmartDashboard keys for the parallel arrays.
        public static final String commandedRpsPointsKey      = "Slippage/CommandedRpsPoints";
        public static final String observedDistancePointsKey  = "Slippage/ObservedDistanceMeters";

        // Additive offset applied to every efficiency value after interpolation.
        // +offset = curve shifts up → less correction → lower commanded RPS
        // -offset = curve shifts down → more correction → higher commanded RPS
        public static final String efficiencyOffsetKey = "Slippage/EfficiencyOffset";
        public static final double defaultEfficiencyOffset = 0.0;

        // Multiplicative scalar applied after the additive offset.
        // >1.0 = magnifies efficiency → less correction → lower commanded RPS
        // <1.0 = shrinks efficiency → more correction → higher commanded RPS
        public static final String efficiencyScaleKey = "Slippage/EfficiencyScale";
        public static final double defaultEfficiencyScale = 1.0;

        public static final double passFudge = 1.4;
    }

    public static final class IntakeConstants {
        public static final int deployIntakeMotorId = 34;
        public static final int roller1id = 35;
        public static final int roller2id = 36;
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kG = 0.0;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.0;
        public static final double rollerStatorCurrentLimit = 30.0;
        public static final double rollerSupplyCurrentLimit = 25.0;
        public static final double deployStatorCurrentLimit = 60.0;
        public static final double deploySupplyCurrentLimit = 30.0;
        public static final double motionMagicCruiseVelocityFast = 25.0;
        public static final double motionMagicCruiseVelocitySlow = 15.0;
        public static final double motionMagicAcceleration = 50;
        public static final double motionMagicJerk = 0;
        public static final double homePos = 3.0;
        public static final double deployPos = 16.0;
        public static final double shakePos = 5;
        public static final double lowerShakePos = 10;
        public static final double upperShakePos = 6;
    }
    
    public static class LED {
        public static final int id = 40;
        public static final int solidID = 0;
        public static final int blinkID = 1;
        public static final int pulseID = 2;
        public static final int rainbowID = 3;
        public static final int framerate = 16; //fps
        public static final int totalLEDs = 100;
        public static final int testJoystickID = 0;
        public static final double hubDistanceLimitFeet = 8.0;

        public static final int visionBest = 1;
        public static final int visionOk = 2;
        public static final int closeToBub = 3;
        // public static final int aiming = 4;
        public static final int visionBad = 5;
        // public static final int climbReady = 6;
        // public static final int intakeDeployed = 7;
        // public static final int safe = 8;
        // public static final int idle = 9;
        public static final int startup = 10;
        public static final int reset = 11;
    }

}
