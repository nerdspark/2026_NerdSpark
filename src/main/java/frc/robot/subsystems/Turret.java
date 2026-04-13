package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.TurretUtil.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.ShooterParams;
import frc.robot.Constants.TurretConfig;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.ShootMode;
import frc.robot.Constants.SlippageCorrectionConstants;
import frc.robot.Constants.IkSolution;
import frc.robot.Constants.MapTuneConstants;
import frc.robot.Constants.SOTM;
import frc.robot.util.SlippageCorrectionMap;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor1, hoodMotor2, shootMotor1, shootMotor2;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityDutyCycle shootDutyBang = new VelocityDutyCycle(0).withEnableFOC(false);
    private VelocityTorqueCurrentFOC shootTorqueBang = new VelocityTorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC hoodPose = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<Boolean> manualOverride;
    private PassTargetSelectorSubsystem passTargetSelector;

    private double turretAngle = 0;
    private boolean brake = false;
    public boolean shoot = false;
    private boolean pass = false;
    private boolean onOppSide = false;

    private Debouncer torqueCurrentDebouncer = new Debouncer(0.02, DebounceType.kFalling);
    private ShootMode mode = ShootMode.COAST;
    private double velocity = 0;

    private final Field2d m_field = new Field2d();
    private final SlippageCorrectionMap slippageMap = new SlippageCorrectionMap();

    private final boolean isBlue;
    private Pose2d turretPose = new Pose2d();

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<Boolean> manualOverrider, PassTargetSelectorSubsystem passTargetSelector) {
        pose = robotPose;
        speed = speeds;
        manualOverride = manualOverrider;
        this.passTargetSelector = passTargetSelector;

        isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

        canivore = new CANBus(Constants.CANbus);
        spinMotor = new TalonFX(TurretConfig.spinMotorId, canivore);
        hoodMotor1 = new TalonFX(TurretConfig.hoodMotor1Id, canivore);
        hoodMotor2 = new TalonFX(TurretConfig.hoodMotor2Id, canivore);
        shootMotor1 = new TalonFX(TurretConfig.shootMotor1Id, canivore);
        shootMotor2 = new TalonFX(TurretConfig.shootMotor2Id, canivore);
        spinCancoder1 = new CANcoder(TurretConfig.spinCancoder1Id, canivore);
        spinCancoder2 = new CANcoder(TurretConfig.spinCancoder2Id, canivore);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.spinKp)
                .withKI(TurretConfig.spinKi)
                .withKD(TurretConfig.spinKd)
                .withKS(TurretConfig.spinKs)
                .withKV(TurretConfig.spinKv)
                .withKA(TurretConfig.spinKa)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(TurretConfig.spinSupplyCurrent)
                .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
                .withMotionMagicAcceleration(TurretConfig.spinAccel)
            )
            .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.1))
        ;
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.hoodKp1)
                .withKI(TurretConfig.hoodKi1)
                .withKD(TurretConfig.hoodKd1)
                .withKS(TurretConfig.hoodKs1)
                .withKV(TurretConfig.hoodKv1)
                .withKA(TurretConfig.hoodKa1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.hoodSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
                .withMotionMagicAcceleration(TurretConfig.hoodAccel)
            )
        ;
        TalonFXConfiguration hoodConfig2 = hoodConfig1.clone()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            )
        ;
        TalonFXConfiguration shootConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withPeakForwardDutyCycle(TurretConfig.peakDutyCycle)
                .withPeakReverseDutyCycle(0)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.kp)
                .withKI(TurretConfig.ki)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withSlot1(new Slot1Configs()
                .withKP(TurretConfig.bangbangKp)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(TurretConfig.peakTorque)
                .withPeakReverseTorqueCurrent(TurretConfig.peakReverseTorque)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(TurretConfig.shootSupplyCurrentLimit))
                .withSupplyCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig2 = shootConfig1.clone()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            )
        ;
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(TurretConfig.spinCancoder1Offset)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            )
        ;
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(TurretConfig.spinCancoder2Offset)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            )
        ;

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor1.getConfigurator().apply(hoodConfig1);
        hoodMotor2.getConfigurator().apply(hoodConfig2);
        shootMotor1.getConfigurator().apply(shootConfig1);
        shootMotor2.getConfigurator().apply(shootConfig2);

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);

        hoodMotor2.setControl(new Follower(TurretConfig.hoodMotor1Id, MotorAlignmentValue.Opposed));
        shootMotor2.setControl(new Follower(TurretConfig.shootMotor1Id, MotorAlignmentValue.Opposed));

        hoodMotor1.setPosition(0);
        hoodMotor2.setPosition(0);

        double theta1 = spinCancoder1.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        double theta2 = spinCancoder2.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        
        double turretAngle = floorMod(
            (TWO_PI/TurretConstants.spinTeeth) * (
                floorMod(
                    -(TurretConstants.spinCancoder2Teeth/TWO_PI) * theta2, TurretConstants.spinCancoder2Teeth
                ) + TurretConstants.spinCancoder2Teeth * floorMod(
                    modInverse(
                        TurretConstants.spinCancoder2Teeth, TurretConstants.spinCancoder1Teeth
                    ) * (
                        floorMod(-(TurretConstants.spinCancoder1Teeth/TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth) 
                        - floorMod(-(TurretConstants.spinCancoder2Teeth/TWO_PI) * theta2, TurretConstants.spinCancoder2Teeth)
                    ), 
                    TurretConstants.spinCancoder1Teeth
                )
            ), 
            TWO_PI
        );
        // Normalize to -pi to pi
        double motorPositon = (normalizeRadians(turretAngle) * TurretConstants.spinRatio) / TWO_PI;
        spinMotor.setPosition(motorPositon);

        initMapTuneDashboard();
    }

    /**
     * Calculates the dynamic X threshold line for a signal state (shoot or pass),
     * taking into account velocity and direction.
     *
     * @param startingLine      Original X coordinate of the line
     * @param robotX            Current robot X position
     * @param robotVeloX        Current robot X velocity
     * @param safetyMargin      Extra distance to start retracting early
     * @param approachDirection The sign of the robot velo going toward the line
     * @return X value of dynamic trigger line
     */
    private double calcTriggerLine(double startingLine, double robotX, double robotVeloX, double safetyMargin, boolean goingLeft) {
        if (goingLeft) {
            // Only extend the line if moving toward it
            if (robotVeloX < 0.0) {
                // Moving away or stopped, use original line
                return startingLine - safetyMargin;
            }

            // Distance traveled while retracting
            double preTriggerDistance = (-robotVeloX * TurretConstants.hoodRetractTime) - safetyMargin;

            // Move the line backward along approach direction
            return startingLine + preTriggerDistance;
        } else {
            // Only extend the line if moving toward it
            if (-robotVeloX < 0.0) {
                // Moving away or stopped, use original line
                return startingLine + safetyMargin;
            }

            // Distance traveled while retracting
            double preTriggerDistance = (robotVeloX * TurretConstants.hoodRetractTime) - safetyMargin;

            // Move the line backward along approach direction
            return startingLine - preTriggerDistance;
        }
    }

    private ShooterParams aimOnFly(double distance) {
       return TurretConstants.map.get(distance);
    }

    /**
     * When we are under trench stow hood, idle shooter, and continue turret control
     * 
     * @param normalizedError the normalized error of the turret to spin to
     */
    private void turretIdle() {
        hoodPose.Position = -0.001;
        velocity = applyShooterControl(0);
        brake = true;
    }

    /**
    * Aims the turret only
    *
    * @param neededAngle the field-centric target angle minus the chassis heading in radians
    */
    private void aimTurret(double neededAngle) {
        double motorPose = spinMotor.getPosition().getValueAsDouble();
        turretAngle = (motorPose * TWO_PI) / TurretConstants.spinRatio;

        neededAngle = normalizeRadians(neededAngle - Math.toRadians(118));
        //neededAngle = Math.round(neededAngle * 100.0) / 100.0;
        double target = neededAngle;
        if (target > Math.toRadians(165) || target < -Math.toRadians(178)) {
            double err1 = turretAngle - neededAngle;
            double err2 = 0;
            if (turretAngle > 0) {
                err2 = turretAngle - (neededAngle + TWO_PI);
            } else {
                err2 = turretAngle - (neededAngle - TWO_PI);
            }
            if (Math.abs(err2) < Math.abs(err1)) {
                if (turretAngle > 0) {
                    target = neededAngle + TWO_PI;
                } else {
                    target = neededAngle - TWO_PI;
                }
            }
        }
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(target));

        double motorRots = (target * TurretConstants.spinRatio) / TWO_PI;

        brake = Math.abs(target - turretAngle) < Math.toRadians(0.15);

        spinPose.Position = motorRots;
    }

    public boolean turretOnTarget() {
        double maxX = isBlue ? FieldConstants.Tower.maxX : FieldConstants.Tower.oppMaxX; 
        double x = turretPose.getX();
        double y = turretPose.getY();
        boolean inX = isBlue ? x <= maxX : x >= maxX;
        boolean inY = isBlue ? y >= FieldConstants.Tower.minY && y <= FieldConstants.Tower.maxY 
                             : y >= FieldConstants.Tower.oppMinY && y <= FieldConstants.Tower.oppMaxY;
        boolean climb = inX && inY;
        return Math.abs(spinPose.Position - spinMotor.getPosition().getValueAsDouble()) < (pass ? 0.2778 : 0.1389) 
            && Math.abs(hoodPose.Position - hoodMotor1.getPosition().getValueAsDouble()) < (pass ? 1.3889 : 0.6944)
            && (shoot || pass)
            && !climb;
    }

    private double applyShooterControl(double motorRps) {
        if (motorRps < 5) {
            mode = ShootMode.COAST;
            return 0;
        }

        boolean inTolerance = Math.abs(shootMotor1.getVelocity().getValueAsDouble() - motorRps) <= 3;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        mode = torqueCurrentControl ? ShootMode.TORQUE_CURRENT_BANG_BANG : ShootMode.DUTY_CYCLE_BANG_BANG;
        // SmartDashboard.putString("Shoot Mode", mode.toString());

        return motorRps;
    }

    private static final class DirectIkSelection {
        private final double hoodDegrees;
        private final double exitMps;

        private DirectIkSelection(double hoodDegrees, double exitMps) {
            this.hoodDegrees = hoodDegrees;
            this.exitMps = exitMps;
        }
    }

    private IkSolution solveIK(double distanceMeters, Translation2d translation) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        double deltaHeight;
        if (shoot) {
            deltaHeight = FieldConstants.Hub.innerHeight - TurretConstants.shooterMuzzleHeightMeters;
        } else {
            deltaHeight = -TurretConstants.shooterMuzzleHeightMeters;
            //distanceMeters -= SlippageCorrectionConstants.passFudge;
        }

        DirectIkSelection selection = pass
            ? solveTwoPointIKDirect(distanceMeters, deltaHeight, translation)
            : solveMinSpeedEntryAngleIKDirect(distanceMeters, deltaHeight);
        if (selection == null) {
            return null;
        }

        double hoodDeg = Math.max(
            TurretConstants.hoodMinDegrees,
            Math.min(
                TurretConstants.hoodMaxDegrees,
                selection.hoodDegrees
            )
        );
        double exit = Math.max(0.0, selection.exitMps);
        return new IkSolution(hoodDeg, exit);
    }

    private DirectIkSelection solveMinSpeedEntryAngleIKDirect(
        double distanceMeters,
        double deltaHeightMeters
    ) {
        double entryRad = Math.toRadians(TurretConstants.ikEntryAngleTargetDeg);
        double tanTheta = (2 * deltaHeightMeters / distanceMeters) + Math.tan(entryRad);
        double thetaRad = Math.atan(tanTheta); // launch angle from horizontal
        double hoodDeg = 90.0 - Math.toDegrees(thetaRad); // hood = 90 − launch angle

        if (hoodDeg < TurretConstants.hoodMinDegrees || hoodDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }

        double speedMps = solveIKSpeed(distanceMeters, thetaRad, deltaHeightMeters);
        if (!Double.isFinite(speedMps) || speedMps <= 0.0) return null;
        return new DirectIkSelection(hoodDeg, speedMps);
    }

    // private DirectIkSelection solveMinimumSpeedIKDirect(
    //     double distanceMeters,
    //     double deltaHeightMeters
    // ) {
    //     double alphaRad = Math.atan2(deltaHeightMeters, distanceMeters);
    //     double launchRad = 0.5 * (alphaRad + (Math.PI / 2.0));
    //     double hoodDeg = 90 - Math.toDegrees(launchRad);
    //     if (hoodDeg < TurretConstants.hoodMinDegrees || hoodDeg > TurretConstants.hoodMaxDegrees) {
    //         return null;
    //     }
    //     double speedMps = solveIKSpeed(distanceMeters, launchRad, deltaHeightMeters);
    //     if (!Double.isFinite(speedMps) || speedMps <= 0.0) {
    //         return null;
    //     }
    //     return new DirectIkSelection(hoodDeg, speedMps);
    // }

    private DirectIkSelection solveTwoPointIKDirect(
        double d1Meters, double deltaH1Meters,
        Translation2d translation
    ) {
        double d2Meters = translation.getDistance(isBlue ? FieldConstants.Net.center : FieldConstants.Net.oppCenter);
        double deltaH2Meters = FieldConstants.Net.height - TurretConstants.shooterMuzzleHeightMeters;

        double denomK = d1Meters * d2Meters * (d2Meters - d1Meters);
        if (Math.abs(denomK) < 1e-9) return null;

        double K = (deltaH1Meters * d2Meters - deltaH2Meters * d1Meters) / denomK;
        if (!Double.isFinite(K) || K <= 0.0) return null;

        double tanTheta = (deltaH1Meters + K * d1Meters * d1Meters) / d1Meters;
        double thetaRad = Math.atan(tanTheta);              // launch angle from horizontal
        double hoodDeg  = 90.0 - Math.toDegrees(thetaRad); // hood convention: 90 − launchAngle

        if (hoodDeg < TurretConstants.hoodMinDegrees || hoodDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }

        double cosTheta = Math.cos(thetaRad);
        double speedMps = Math.sqrt(9.80665 / (2.0 * K * cosTheta * cosTheta));
        if (!Double.isFinite(speedMps) || speedMps <= 0.0) return null;
        return new DirectIkSelection(hoodDeg, speedMps);
    }

    private double solveIKSpeed(double distanceMeters, double thetaRad, double deltaHeightMeters) {
        double cos = Math.cos(thetaRad);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double tan = Math.tan(thetaRad);
        double denominator = 2.0 * cos * cos * (distanceMeters * tan - deltaHeightMeters);
        if (denominator <= 0.0) {
            return Double.NaN;
        }
        double vSquared = (9.80665 * distanceMeters * distanceMeters) / denominator;
        if (vSquared <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(vSquared);
    }
    
   /**
    * Applies SOTM compensation
    * @param launchMps launch speed in mps
    * @param launchAngle launch angle in radians
    * @param turretRad turret needed angle before chassis angle comp in radians
    * @param speed current field oriented chassis speeds
    * @return
    */
    private SOTM applySOTMComp(double launchMps, double launchAngle, double turretRad, ChassisSpeeds speed) {
        if (SlippageCorrectionConstants.useSOTM) {
            double newLaunchMps = Math.sqrt(Math.pow(launchMps, 2) + Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2) 
                - 2 * launchMps * Math.cos(launchAngle) * (Math.cos(turretRad) * speed.vxMetersPerSecond + Math.sin(turretRad) * speed.vyMetersPerSecond));
            double newTurretAngle = Math.atan2(launchMps * Math.cos(launchAngle) * Math.sin(turretRad) - speed.vyMetersPerSecond, 
                launchMps * Math.cos(launchAngle) * Math.cos(turretRad) - speed.vxMetersPerSecond);
            double newLaunchAngle = launchAngle;
            if (newLaunchMps > 1e-6) {
                newLaunchAngle = Math.asin((launchMps * Math.sin(launchAngle)) / newLaunchMps);
            }

            return new SOTM(newTurretAngle, newLaunchAngle, newLaunchMps);
        } else {
            return new SOTM(turretRad, launchAngle, launchMps);
        }
    }

    private void initMapTuneDashboard() {
        SmartDashboard.setDefaultBoolean(MapTuneConstants.enableKey, MapTuneConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(MapTuneConstants.spinKey, 0);
        SmartDashboard.setDefaultNumber(MapTuneConstants.hoodKey, 0);
        SmartDashboard.setDefaultNumber(MapTuneConstants.shooterKey, 0);
    }

    private void applyLiveMap() {
        spinPose.Position = SmartDashboard.getNumber(MapTuneConstants.spinKey, 0);
        hoodPose.Position = SmartDashboard.getNumber(MapTuneConstants.hoodKey, 0);
        velocity = SmartDashboard.getNumber(MapTuneConstants.shooterKey, 0);
        applyShooterControl(velocity);
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = speed.get();
        Pose2d currPose = pose.get();
        m_field.setRobotPose(currPose);
        Translation2d rotationOffset = TurretConstants.robotToTurret.rotateBy(currPose.getRotation());
        turretPose = new Pose2d(currPose.getTranslation().plus(rotationOffset), currPose.getRotation());

        SmartDashboard.putBoolean("Is Blue", isBlue);
        double shootLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.blueShootLine : FieldConstants.LinesVertical.redShootLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(40), 
            isBlue ? true : false
        );
        double passLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.bluePassLine : FieldConstants.LinesVertical.redPassLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(40), 
            isBlue ? false : true
        );
        double oppPassLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.redShootLine : FieldConstants.LinesVertical.blueShootLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(40), 
            isBlue ? false : true
        );
        double oppNeurtalLine = calcTriggerLine(
            isBlue ? FieldConstants.LinesVertical.redPassLine : FieldConstants.LinesVertical.bluePassLine, 
            turretPose.getX(), 
            speeds.vxMetersPerSecond, 
            Units.inchesToMeters(40), 
            isBlue ? true : false
        );

        shoot = isBlue ? turretPose.getX() < shootLine : turretPose.getX() > shootLine;
        SmartDashboard.putBoolean("Shoot", shoot);
        pass = isBlue ? turretPose.getX() > passLine && turretPose.getX() < oppNeurtalLine || turretPose.getX() > oppPassLine
                       : turretPose.getX() < passLine && turretPose.getX() > oppNeurtalLine || turretPose.getX() < oppPassLine;
        SmartDashboard.putBoolean("Pass", pass);
        onOppSide = isBlue ? turretPose.getX() > oppPassLine : turretPose.getX() < oppPassLine;
        SmartDashboard.putBoolean("Turret/IsOnOppSide", onOppSide);

        if (shoot || pass) {
            Translation2d goalPose;
            Translation2d passPose;
            if (isBlue) {
                goalPose = FieldConstants.Hub.topCenterPoint.toTranslation2d();
                passPose = closerPoint(turretPose, FieldConstants.BluePass.left, FieldConstants.BluePass.right)
                        ? FieldConstants.BluePass.left : FieldConstants.BluePass.right;
            } else {
                goalPose = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
                passPose = closerPoint(turretPose, FieldConstants.RedPass.left, FieldConstants.RedPass.right)
                        ? FieldConstants.RedPass.left : FieldConstants.RedPass.right;
            }

            if (passTargetSelector != null && passTargetSelector.isEnabled()) {
                passPose = passTargetSelector.getTarget();
            }
            
            Translation2d targetPose = shoot ? goalPose : passPose;
            m_field.getObject("Target Pose").setPose(targetPose.getMeasureX(), targetPose.getMeasureY(), new Rotation2d());

            double xError = targetPose.getX() - turretPose.getX();
            double yError = targetPose.getY() - turretPose.getY();
            double errorRad = Math.atan2(yError, xError);
            double distance = turretPose.getTranslation().getDistance(targetPose);
            SmartDashboard.putNumber("Turret/DistanceToTarget", distance);

            boolean useIK = SmartDashboard.getBoolean(
                AutoAimConstants.useIKSolverKey,
                AutoAimConstants.defaultUseIKSolver
            );
                        
            SOTM sotm = null;
            if (useIK) {
                IkSolution ikSolution = solveIK(distance, turretPose.getTranslation());
                if (ikSolution != null) {
                    double launchAngleRad = Math.toRadians(90.0 - ikSolution.hoodDegrees);
                    sotm = applySOTMComp(ikSolution.exitMps, launchAngleRad, errorRad, speeds);
                    SmartDashboard.putBoolean("Turret/IK/HasSolution", true);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", ikSolution.hoodDegrees);
                    SmartDashboard.putNumber("Turret/IK/RequiredExitMps", ikSolution.exitMps);
                } else {
                    SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", 0);
                    SmartDashboard.putNumber("Turret/IK/RequiredExitMps", 0);
                }
            } else {
                SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
                ShooterParams params = aimOnFly(shoot ? distance : Double.MAX_VALUE);
                sotm = applySOTMComp(
                    motorRpsToLaunchSpeedMps(params.shooterSpeed), 
                    Math.toRadians(90 - hoodRotationsToDegrees(params.hoodPose)), 
                    errorRad, 
                    speeds
                );
            }

            if (sotm != null) {
                aimTurret(sotm.turretAngle - turretPose.getRotation().getRadians());
                hoodPose.Position = hoodDegreesToRotations(90 - Math.toDegrees(sotm.launchAngle));
                if (useIK) {
                    velocity = applyShooterControl(slippageMap.correctedMotorRps(launchMpsToMotorRps(sotm.launchMps)));
                    SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", velocity);
                } else {
                    velocity = applyShooterControl(launchMpsToMotorRps(sotm.launchMps));
                }
                double hoodAngleCheck = 90 - Math.toDegrees(sotm.launchAngle);
                if (onOppSide) {
                    velocity = Math.min(velocity, TurretConstants.shooterMaxMotorRps);
                    if (hoodAngleCheck < TurretConstants.hoodMinDegrees || hoodAngleCheck > TurretConstants.hoodMaxDegrees) {
                        velocity = 0;
                        hoodPose.Position = 0;
                    }
                } else if (velocity > TurretConstants.shooterMaxMotorRps || hoodAngleCheck < TurretConstants.hoodMinDegrees
                    || hoodAngleCheck > TurretConstants.hoodMaxDegrees) {
                    velocity = 0;
                    hoodPose.Position = 0;
                }
            } else {
                turretIdle();
            }
        } else {
            turretIdle();
            SmartDashboard.putNumber("Turret/DistanceToTarget", 0.0);
            SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
            SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", Double.NaN);
            SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", Double.NaN);
            // SmartDashboard.putNumber("Turret/IK/RequiredCompMotorRps", Double.NaN);
            // SmartDashboard.putNumber("Turret/IK/PredictedEntryDeg", Double.NaN);
            // SmartDashboard.putBoolean("Turret/IK/UsingEntryBand", false);
        }

        if (manualOverride.get()) {
            hoodPose.Position = 0;
            velocity = 0;
            mode = ShootMode.COAST;
            brake = true;
        }

        boolean mapTuneEnabled = SmartDashboard.getBoolean(
            MapTuneConstants.enableKey,
            MapTuneConstants.defaultEnable
        );
        if (mapTuneEnabled) {
            applyLiveMap();
        }

        // if (brake) {
        //     spinMotor.setControl(new StaticBrake());
        // } else {
        //     spinMotor.setControl(spinPose);
        // }

        // SmartDashboard.putNumber("Turret/SpinAmps", spinMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Turret/SpinSupply", spinMotor.getSupplyCurrent().getValueAsDouble());
        hoodMotor1.setControl(hoodPose);
        // SmartDashboard.putNumber("Turret/HoodCurrentDeg", hoodRotationsToDegrees(hoodMotor1.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Turret/ShooterCurrentRps", shootMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/ShooterAmps", shootMotor1.getStatorCurrent().getValueAsDouble());
        switch (mode) {
            case DUTY_CYCLE_BANG_BANG -> shootMotor1.setControl(shootDutyBang.withVelocity(velocity).withSlot(1));
            case TORQUE_CURRENT_BANG_BANG -> shootMotor1.setControl(shootTorqueBang.withVelocity(velocity).withSlot(0));
            case COAST -> shootMotor1.set(0);
        }

        SmartDashboard.putData("Turret Field", m_field);
        SmartDashboard.putBoolean("Can Index", turretOnTarget());
        SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretAngle));
    }
}
