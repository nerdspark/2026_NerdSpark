package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.TurretUtil.closerPoint;
import static frc.robot.util.TurretUtil.floorMod;
import static frc.robot.util.TurretUtil.hoodDegreesToRotations;
import static frc.robot.util.TurretUtil.hoodRotationsToDegrees;
import static frc.robot.util.TurretUtil.launchMpsToMotorRps;
import static frc.robot.util.TurretUtil.modInverse;
import static frc.robot.util.TurretUtil.motorRpsToLaunchSpeedMps;
import static frc.robot.util.TurretUtil.normalizeRadians;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
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
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.IkSolution;
import frc.robot.Constants.MapTuneConstants;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.SOTM;
import frc.robot.Constants.ShootMode;
import frc.robot.Constants.SlippageCorrectionConstants;
import frc.robot.Constants.TurretConfig;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;
import frc.robot.util.ShooterOffsetMap;
import frc.robot.util.ShooterParams;
import frc.robot.util.SlippageCorrectionMap;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;

    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor1, hoodMotor2, shootMotor1, shootMotor2;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityDutyCycle shootDutyBang = new VelocityDutyCycle(0);
    private VelocityTorqueCurrentFOC shootTorqueBang = new VelocityTorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC hoodPose = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<Boolean> manualOverride;

    private double motorPositon;
    private double turretAngle = 0;
    private boolean brake = false;

    private Debouncer torqueCurrentDebouncer = new Debouncer(0.02, DebounceType.kFalling);
    private ShootMode mode = ShootMode.COAST;
    private double velocity = 0;

    private final Field2d m_field = new Field2d();
    private final ShooterOffsetMap offsetMap = new ShooterOffsetMap();
    private final SlippageCorrectionMap slippageMap = new SlippageCorrectionMap();

    private final boolean isBlue;
    private Pose2d turretPose = new Pose2d();

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<Boolean> manualOverrider) {
        pose = robotPose;
        speed = speeds;
        manualOverride = manualOverrider;

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
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKP(TurretConfig.spinKp)
                        .withKI(TurretConfig.spinKi)
                        .withKD(TurretConfig.spinKd)
                        .withKS(TurretConfig.spinKs)
                        .withKV(TurretConfig.spinKv)
                        .withKA(TurretConfig.spinKa)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(TurretConfig.spinStatorCurrentLimit))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(TurretConfig.spinSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
                        .withMotionMagicAcceleration(TurretConfig.spinAccel))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(0.1));
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKP(TurretConfig.hoodKp1)
                        .withKI(TurretConfig.hoodKi1)
                        .withKD(TurretConfig.hoodKd1)
                        .withKS(TurretConfig.hoodKs1)
                        .withKV(TurretConfig.hoodKv1)
                        .withKA(TurretConfig.hoodKa1)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(TurretConfig.hoodSupplyCurrentLimit))
                        .withSupplyCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
                        .withMotionMagicAcceleration(TurretConfig.hoodAccel));
        TalonFXConfiguration hoodConfig2 = hoodConfig1.clone()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive));
        TalonFXConfiguration shootConfig1 = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withPeakForwardDutyCycle(TurretConfig.peakDutyCycle)
                        .withPeakReverseDutyCycle(0))
                .withSlot0(new Slot0Configs()
                        .withKP(TurretConfig.kp)
                        .withKI(TurretConfig.ki)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSlot1(new Slot1Configs()
                        .withKP(TurretConfig.bangbangKp)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(TurretConfig.peakTorque)
                        .withPeakReverseTorqueCurrent(TurretConfig.peakReverseTorque))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(TurretConfig.shootSupplyCurrentLimit))
                        .withSupplyCurrentLimitEnable(true));
        TalonFXConfiguration shootConfig2 = shootConfig1.clone()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive));
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(1)
                        .withMagnetOffset(TurretConfig.spinCancoder1Offset)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(1)
                        .withMagnetOffset(TurretConfig.spinCancoder2Offset)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

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
                (TWO_PI / TurretConstants.spinTeeth) * (floorMod(
                        -(TurretConstants.spinCancoder1Teeth / TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth)
                        + TurretConstants.spinCancoder1Teeth * floorMod(
                                modInverse(
                                        TurretConstants.spinCancoder1Teeth, TurretConstants.spinCancoder2Teeth)
                                        * (floorMod(-(TurretConstants.spinCancoder2Teeth / TWO_PI) * theta2,
                                                TurretConstants.spinCancoder2Teeth)
                                                - floorMod(-(TurretConstants.spinCancoder1Teeth / TWO_PI) * theta1,
                                                        TurretConstants.spinCancoder1Teeth)),
                                TurretConstants.spinCancoder2Teeth)),
                TWO_PI);
        // Normalize to -pi to pi
        motorPositon = (normalizeRadians(turretAngle) * TurretConstants.spinRatio) / TWO_PI;
        spinMotor.setPosition(motorPositon, 2.5);

        // initMapTuneDashboard();

        // SmartDashboard.setDefaultNumber(
        // SlippageCorrectionConstants.sotmVelocityScaleKey,
        // SlippageCorrectionConstants.defaultSotmVelocityScale
        // );

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
    private double calcTriggerLine(double startingLine, double robotX, double robotVeloX, double safetyMargin,
            boolean goingLeft) {
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
    private void turretIdle(double normalizedError) {
        hoodPose.Position = -0.002;
        velocity = applyShooterControl(30);
        aimTurret(normalizedError);
    }

    /**
     * Aims the turret only
     *
     * @param neededAngle the field-centric target angle minus the chassis heading
     *                    in radians
     */
    private void aimTurret(double neededAngle) {
        motorPositon = spinMotor.getPosition().getValueAsDouble();
        turretAngle = (motorPositon * TWO_PI) / TurretConstants.spinRatio;
        SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretAngle));

        neededAngle = normalizeRadians(neededAngle - Math.toRadians(145));
        neededAngle = Math.round(neededAngle * 100.0) / 100.0;
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(neededAngle));

        double motorRots = (neededAngle * TurretConstants.spinRatio) / TWO_PI;

        brake = Math.abs(motorRots - spinMotor.getPosition().getValueAsDouble()) < 0.004167; // 0.006944;

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
        boolean pass = SmartDashboard.getBoolean("Pass", false);
        return Math.abs(spinPose.Position - spinMotor.getPosition().getValueAsDouble()) < (pass ? 0.2778 : 0.1389)
                && Math.abs(hoodPose.Position - hoodMotor1.getPosition().getValueAsDouble()) < (pass ? 1.3889 : 0.6944)
                && (SmartDashboard.getBoolean("Shoot", false) || pass)
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

    public boolean shooterAtSpeed() {
        velocity = SmartDashboard.getNumber(MapTuneConstants.shooterKey, 0);
        boolean inTolerance = Math.abs(shootMotor1.getVelocity().getValueAsDouble() - velocity) <= 3;
        return inTolerance;
    }

    private double getConfiguredMuzzleHeightMeters() {
        return SmartDashboard.getNumber(
                AutoAimConstants.modelMuzzleHeightMetersKey,
                TurretConstants.shooterMuzzleHeightMeters);
    }

    private double getConfiguredTargetHeightMeters() {
        return SmartDashboard.getNumber(
                AutoAimConstants.modelTargetHeightMetersKey,
                TurretConstants.targetHeightMeters);
    }

    private static final class DirectIkSelection {
        private final double hoodDegrees;
        private final double motorRps;
        private final boolean usedPrimaryObjective;

        private DirectIkSelection(double hoodDegrees, double motorRps, boolean usedPrimaryObjective) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
            this.usedPrimaryObjective = usedPrimaryObjective;
        }
    }

    private static final class PassSolution {
        private final double exitSpeedMps;
        private final double launchAngleRad;
        private final double predictedBearing;

        private PassSolution(double exitSpeedMps, double launchAngleRad, double predictedBearing) {
            this.exitSpeedMps = exitSpeedMps;
            this.launchAngleRad = launchAngleRad;
            this.predictedBearing = predictedBearing;
        }
    }

    private IkSolution solveIK(double distanceMeters, boolean shooting) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        boolean useEntryAngleIK = SmartDashboard.getBoolean(
                AutoAimConstants.useEntryAngleIKKey,
                AutoAimConstants.defaultUseEntryAngleIK);
        double deltaHeight;
        if (shooting) {
            deltaHeight = getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters();
        } else {
            deltaHeight = -getConfiguredMuzzleHeightMeters();
            distanceMeters -= SlippageCorrectionConstants.passFudge;
        }

        DirectIkSelection selection = useEntryAngleIK
                ? solveEntryAngleIKDirect(distanceMeters, deltaHeight)
                : solveMinimumSpeedIKDirect(distanceMeters, deltaHeight);
        if (selection == null) {
            return null;
        }

        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = Math.max(
                TurretConstants.hoodMinDegrees,
                Math.min(
                        TurretConstants.hoodMaxDegrees,
                        selection.hoodDegrees + offsets.hoodOffsetDeg));
        double theoreticalMotorRps = Math.max(
                0.0,
                useEntryAngleIK
                        ? Math.min(TurretConstants.shooterMaxMotorRps, selection.motorRps + offsets.motorRpsOffset)
                        : selection.motorRps + offsets.motorRpsOffset);
        double slippageFactor = slippageMap.efficiencyAt(theoreticalMotorRps);
        double motorRps = slippageMap.correctedMotorRps(theoreticalMotorRps);
        // After slippage correction the commanded RPS can exceed the motor's physical
        // limit even if the theoretical RPS was within bounds — reject the solution.
        if (motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        // SmartDashboard.putBoolean("Turret/IK/UseEntryAngleMode", useEntryAngleIK);
        // SmartDashboard.putBoolean("Turret/IK/UsingEntryBand",
        // selection.usedPrimaryObjective);
        // SmartDashboard.putString(
        // "Turret/IK/SolverMode",
        // useEntryAngleIK ? "EntryAngle" : "MinimumSpeed"
        // );
        // SmartDashboard.putNumber("Turret/Slippage/TheoreticalMotorRps",
        // theoreticalMotorRps);
        // SmartDashboard.putNumber("Turret/Slippage/CorrectedMotorRps", motorRps);
        SmartDashboard.putNumber("Turret/Slippage/EfficiencyFactor", slippageFactor);
        return new IkSolution(hoodDeg, motorRps);
    }

    private DirectIkSelection solveEntryAngleIKDirect(
            double distanceMeters,
            double deltaHeightMeters) {
        double targetEntryRad = Math.toRadians(TurretConstants.ikEntryAngleTargetDeg);
        double desiredThetaRad = Math.atan(Math.tan(targetEntryRad) + (2.0 * deltaHeightMeters / distanceMeters));
        double desiredThetaDeg = Math.toDegrees(desiredThetaRad);
        if (!Double.isFinite(desiredThetaDeg)) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        if (desiredThetaDeg < TurretConstants.hoodMinDegrees || desiredThetaDeg > TurretConstants.hoodMaxDegrees) {
            return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
        }
        double speedMps = solveIKSpeed(distanceMeters, Math.toRadians(desiredThetaDeg), deltaHeightMeters);
        double motorRps = launchMpsToMotorRps(speedMps);
        if (Double.isFinite(motorRps) && motorRps > 0.0 && motorRps <= TurretConstants.shooterMaxMotorRps) {
            return new DirectIkSelection(desiredThetaDeg, motorRps, true);
        }

        return solveMinimumSpeedIKDirect(distanceMeters, deltaHeightMeters);
    }

    private DirectIkSelection solveMinimumSpeedIKDirect(
            double distanceMeters,
            double deltaHeightMeters) {
        double alphaRad = Math.atan2(deltaHeightMeters, distanceMeters);
        double thetaDeg = 90 - Math.toDegrees(0.5 * (alphaRad + (Math.PI / 2.0)));
        double thetaSpeed = Math.toDegrees(0.5 * (alphaRad + (Math.PI / 2.0)));
        if (thetaDeg < TurretConstants.hoodMinDegrees || thetaDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }
        double speedMps = solveIKSpeed(distanceMeters, Math.toRadians(thetaSpeed), deltaHeightMeters);
        double motorRps = launchMpsToMotorRps(speedMps);
        if (!Double.isFinite(motorRps) || motorRps <= 0.0 || motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        return new DirectIkSelection(thetaDeg, motorRps, false);
    }

    private DirectIkSelection solveTwoPointIKDirect(
            double d1Meters, double deltaH1Meters,
            double d2Meters, double deltaH2Meters) {
        double denomK = d1Meters * d2Meters * (d2Meters - d1Meters);
        if (Math.abs(denomK) < 1e-9)
            return null;

        double K = (deltaH1Meters * d2Meters - deltaH2Meters * d1Meters) / denomK;
        if (!Double.isFinite(K) || K <= 0.0)
            return null;

        double tanTheta = (deltaH1Meters + K * d1Meters * d1Meters) / d1Meters;
        double thetaRad = Math.atan(tanTheta); // launch angle from horizontal
        double hoodDeg = 90.0 - Math.toDegrees(thetaRad); // hood convention: 90 − launchAngle

        if (hoodDeg < TurretConstants.hoodMinDegrees || hoodDeg > TurretConstants.hoodMaxDegrees) {
            return null;
        }

        double cosTheta = Math.cos(thetaRad);
        double speedMps = Math.sqrt(9.80665 / (2.0 * K * cosTheta * cosTheta));
        if (!Double.isFinite(speedMps) || speedMps <= 0.0)
            return null;

        double motorRps = launchMpsToMotorRps(speedMps);
        if (!Double.isFinite(motorRps) || motorRps <= 0.0 || motorRps > TurretConstants.shooterMaxMotorRps) {
            return null;
        }
        return new DirectIkSelection(hoodDeg, motorRps, false);
    }

    public IkSolution solveWithRequiredAngle(double distanceMeters) {
        if (distanceMeters <= 0.0)
            return null;
        double hoodDeg = TurretConstants.fixedAngleHoodDegDefault;
        hoodDeg = Math.max(TurretConstants.hoodMinDegrees, Math.min(TurretConstants.hoodMaxDegrees, hoodDeg));
        double launchAngleRad = Math.toRadians(90.0 - hoodDeg);
        double deltaHeight = getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters();
        double exitSpeedMps = solveIKSpeed(distanceMeters, launchAngleRad, deltaHeight);
        if (!Double.isFinite(exitSpeedMps) || exitSpeedMps <= 0.0)
            return null;
        double correctedRps = slippageMap.correctedMotorRps(launchMpsToMotorRps(exitSpeedMps));
        if (!Double.isFinite(correctedRps) || correctedRps <= 0.0 || correctedRps > TurretConstants.shooterMaxMotorRps)
            return null;
        return new IkSolution(hoodDeg, correctedRps);
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

    private double computeEntryAngleDeg(double distanceMeters, double launchSpeedMps, double launchAngleRad) {
        double horizontalSpeed = launchSpeedMps * Math.cos(launchAngleRad);
        if (horizontalSpeed <= 1e-6) {
            return Double.NaN;
        }
        double time = distanceMeters / horizontalSpeed;
        if (!Double.isFinite(time) || time <= 0.0) {
            return Double.NaN;
        }
        double verticalVelocityAtTarget = launchSpeedMps * Math.sin(launchAngleRad) - 9.80665 * time;
        return Math.toDegrees(Math.atan2(-verticalVelocityAtTarget, horizontalSpeed));
    }

    /**
     * Applies SOTM compensation
     * 
     * @param launchMps   launch speed in mps
     * @param launchAngle launch angle in radians
     * @param turretRad   turret needed angle before chassis angle comp in radians
     * @param speed       current field oriented chassis speeds
     * @return
     */
    private SOTM applySOTMComp(double launchMps, double launchAngle, double turretRad, ChassisSpeeds speed) {
        double newLaunchMps = Math.sqrt(
                Math.pow(launchMps, 2) + Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2)
                        - 2 * launchMps * Math.cos(launchAngle) * (Math.cos(turretRad) * speed.vxMetersPerSecond
                                + Math.sin(turretRad) * speed.vyMetersPerSecond));
        double newTurretAngle = Math.atan2(
                launchMps * Math.cos(launchAngle) * Math.sin(turretRad) - speed.vyMetersPerSecond,
                launchMps * Math.cos(launchAngle) * Math.cos(turretRad) - speed.vxMetersPerSecond);
        double newLaunchAngle = launchAngle;
        if (newLaunchMps > 1e-6) {
            newLaunchAngle = Math.asin((launchMps * Math.sin(launchAngle)) / newLaunchMps);
        }

        return new SOTM(newTurretAngle, newLaunchAngle, newLaunchMps);
    }

    private IkSolution solvePassIK(double distanceMeters) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        double deltaHeight = 0 - TurretConstants.shooterMuzzleHeightMeters;

        DirectIkSelection passSelection = solveMinimumSpeedIKDirect(distanceMeters, deltaHeight);
        if (passSelection == null) {
            SmartDashboard.putBoolean("Turret/Pass/HasSolution", false);
            SmartDashboard.putNumber("Turret/Pass/HoodDeg", Double.NaN);
            SmartDashboard.putNumber("Turret/Pass/MotorRps", Double.NaN);
            return null;
        }

        // offset map correction
        ShooterOffsetMap.Offsets offsets = offsetMap.sample(distanceMeters);
        double hoodDeg = Math.max(
                TurretConstants.hoodMinDegrees,
                Math.min(
                        TurretConstants.hoodMaxDegrees,
                        passSelection.hoodDegrees + offsets.hoodOffsetDeg));

        double motorRps = Math.max(0.0, passSelection.motorRps + offsets.motorRpsOffset);
        if (motorRps > TurretConstants.shooterMaxMotorRps) {
            SmartDashboard.putBoolean("Turret/Pass/HasSolution", false);
            return null;
        }

        SmartDashboard.putBoolean("Turret/Pass/HasSolution", true);
        SmartDashboard.putNumber("Turret/Pass/Distance", distanceMeters);
        SmartDashboard.putNumber("Turret/Pass/HoodDeg", hoodDeg);
        SmartDashboard.putNumber("Turret/Pass/MotorRps", motorRps);

        return new IkSolution(hoodDeg, motorRps);
    }

    private PassSolution computePassSolution(Translation2d passPose, ChassisSpeeds speeds) {
        // Raw distance from current turret position
        double xError = passPose.getX() - turretPose.getX();
        double yError = passPose.getY() - turretPose.getY();
        double dist = Math.sqrt(xError * xError + yError * yError);

        // Solve IK
        IkSolution ikSolution = solvePassIK(dist);
        if (ikSolution == null)
            return null;

        double launchAngleRad = Math.toRadians(90.0 - ikSolution.hoodDegrees);
        double exitSpeedMps = solveIKSpeed(
                dist,
                launchAngleRad,
                -TurretConstants.shooterMuzzleHeightMeters);
        if (!Double.isFinite(exitSpeedMps) || exitSpeedMps <= 0.0)
            return null;

        // Flight-time prediction
        double horizontalSpeed = exitSpeedMps * Math.cos(launchAngleRad);
        double flightTime = dist / horizontalSpeed;
        Translation2d predictedTranslation = turretPose.getTranslation().plus(
                new Translation2d(
                        speeds.vxMetersPerSecond * flightTime,
                        speeds.vyMetersPerSecond * flightTime));
        double predXError = passPose.getX() - predictedTranslation.getX();
        double predYError = passPose.getY() - predictedTranslation.getY();
        double predictedBearing = Math.atan2(predYError, predXError);

        // Logging
        SmartDashboard.putNumber("Turret/Pass/FlightTimeSec", flightTime);
        SmartDashboard.putNumber("Turret/Pass/ExitSpeedMps", exitSpeedMps);
        SmartDashboard.putNumber("Turret/Pass/PredictedBearing", Math.toDegrees(predictedBearing));
        SmartDashboard.putNumber("Turret/Pass/Distance", dist);

        return new PassSolution(exitSpeedMps, launchAngleRad, predictedBearing);
    }

    // private void initMapTuneDashboard() {
    // SmartDashboard.setDefaultBoolean(MapTuneConstants.enableKey,
    // MapTuneConstants.defaultEnable);
    // SmartDashboard.setDefaultNumber(MapTuneConstants.spinKey, 0);
    // SmartDashboard.setDefaultNumber(MapTuneConstants.hoodKey, 0);
    // SmartDashboard.setDefaultNumber(MapTuneConstants.shooterKey, 0);
    // }

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
        // ChassisSpeeds speeds = speed.get();
        // Pose2d currPose = pose.get();
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.5, 0.0);
        Pose2d currPose = new Pose2d(6,4,new Rotation2d());

        m_field.setRobotPose(currPose);
        Translation2d rotationOffset = TurretConstants.robotToTurret.rotateBy(currPose.getRotation());
        turretPose = new Pose2d(currPose.getTranslation().plus(rotationOffset), currPose.getRotation());

        SmartDashboard.putBoolean("Is Blue", isBlue);
        double shootLine = calcTriggerLine(
                isBlue ? FieldConstants.LinesVertical.blueShootLine : FieldConstants.LinesVertical.redShootLine,
                turretPose.getX(),
                speeds.vxMetersPerSecond,
                Units.inchesToMeters(40),
                isBlue ? true : false);
        double passLine = calcTriggerLine(
                isBlue ? FieldConstants.LinesVertical.bluePassLine : FieldConstants.LinesVertical.redPassLine,
                turretPose.getX(),
                speeds.vxMetersPerSecond,
                Units.inchesToMeters(40),
                isBlue ? false : true);
        double oppPassLine = calcTriggerLine(
                isBlue ? FieldConstants.LinesVertical.redShootLine : FieldConstants.LinesVertical.blueShootLine,
                turretPose.getX(),
                speeds.vxMetersPerSecond,
                Units.inchesToMeters(40),
                isBlue ? false : true);
        double oppNeutralLine = calcTriggerLine(
                isBlue ? FieldConstants.LinesVertical.redPassLine : FieldConstants.LinesVertical.bluePassLine,
                turretPose.getX(),
                speeds.vxMetersPerSecond,
                Units.inchesToMeters(40),
                isBlue ? true : false);

        boolean shoot = isBlue ? turretPose.getX() < shootLine : turretPose.getX() > shootLine;
        SmartDashboard.putBoolean("Shoot", shoot);
        boolean pass = isBlue
                ? turretPose.getX() > passLine && turretPose.getX() < oppNeutralLine || turretPose.getX() > oppPassLine
                : turretPose.getX() < passLine && turretPose.getX() > oppNeutralLine || turretPose.getX() < oppPassLine;
        SmartDashboard.putBoolean("Pass", pass);

        boolean passTargetPickerEnabled = SmartDashboard.getBoolean(
                PassTargetConstants.enableKey, false);

        SOTM sotm = null;

        if (shoot) {
            Translation2d goalPose = isBlue
                    ? FieldConstants.Hub.topCenterPoint.toTranslation2d()
                    : FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

            m_field.getObject("Target Pose").setPose(
                    goalPose.getX(), goalPose.getY(), new Rotation2d());

            double predictionSeconds = SlippageCorrectionConstants.defaultSotmPredictionSeconds;
            Translation2d predictedTranslation = turretPose.getTranslation().plus(
                    new Translation2d(
                            speeds.vxMetersPerSecond * predictionSeconds,
                            speeds.vyMetersPerSecond * predictionSeconds));
            m_field.getObject("Prediction pose").setPose(
                    predictedTranslation.getX(), predictedTranslation.getY(), turretPose.getRotation());

            double xError = goalPose.getX() - predictedTranslation.getX();
            double yError = goalPose.getY() - predictedTranslation.getY();
            double errorDegrees = Math.atan2(yError, xError);
            double distance = predictedTranslation.getDistance(goalPose);
            distance -= Units.feetToMeters(0.8);
            SmartDashboard.putNumber("Turret/DistanceToTarget", distance);

            boolean useIK = SmartDashboard.getBoolean(
                    AutoAimConstants.useIKSolverKey,
                    AutoAimConstants.defaultUseIKSolver);

            if (useIK) {
                IkSolution ikSolution = solveIK(distance, true);
                if (ikSolution != null) {
                    double launchAngleRad = Math.toRadians(90.0 - ikSolution.hoodDegrees);
                    double exitSpeedMps = solveIKSpeed(
                            distance,
                            launchAngleRad,
                            getConfiguredTargetHeightMeters() - getConfiguredMuzzleHeightMeters());
                    sotm = applySOTMComp(exitSpeedMps, launchAngleRad, errorDegrees, speeds);

                    SmartDashboard.putBoolean("Turret/IK/HasSolution", true);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", ikSolution.hoodDegrees);
                    SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", ikSolution.motorRps);
                } else {
                    turretIdle(normalizeRadians(errorDegrees - turretPose.getRotation().getRadians()));
                    SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
                    SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", Double.NaN);
                    SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", Double.NaN);
                }
            } else {
                ShooterParams params = aimOnFly(distance);
                sotm = applySOTMComp(
                        motorRpsToLaunchSpeedMps(params.shooterSpeed),
                        Math.toRadians(90 - hoodRotationsToDegrees(params.hoodPose)),
                        errorDegrees,
                        speeds);
            }

        } else if (pass && passTargetPickerEnabled) {
            // Read dashboard click coordinate
            double clickX = SmartDashboard.getNumber(
                PassTargetConstants.targetXKey,
                PassTargetConstants.defaultTargetX
            );
            double clickY = SmartDashboard.getNumber(
                PassTargetConstants.targetYKey,
                PassTargetConstants.defaultTargetY
            );
            
            
            Translation2d passPose = new Translation2d(clickX, clickY);

            m_field.getObject("Pass Target").setPose(
                    passPose.getX(), passPose.getY(), new Rotation2d());
            SmartDashboard.putNumber("Turret/DistanceToTarget",
                    turretPose.getTranslation().getDistance(passPose));

            PassSolution passSolution = computePassSolution(passPose, speeds);
            if (passSolution != null) {
                sotm = applySOTMComp(
                        passSolution.exitSpeedMps,
                        passSolution.launchAngleRad,
                        passSolution.predictedBearing,
                        speeds);
            } else {
                double xError = passPose.getX() - turretPose.getX();
                double yError = passPose.getY() - turretPose.getY();
                turretIdle(normalizeRadians(
                        Math.atan2(yError, xError) - turretPose.getRotation().getRadians()));
            }

        } else {
            turretIdle(normalizeRadians(turretPose.getRotation().getRadians()));
            SmartDashboard.putNumber("Turret/DistanceToTarget", 0.0);
            SmartDashboard.putBoolean("Turret/IK/HasSolution", false);
            SmartDashboard.putNumber("Turret/IK/RequiredHoodDeg", Double.NaN);
            SmartDashboard.putNumber("Turret/IK/RequiredMotorRps", Double.NaN);
        }

        // Apply SOTM result to motors — shared by both shoot and pass
        if (sotm != null) {
            aimTurret(normalizeRadians(sotm.turretAngle - turretPose.getRotation().getRadians()));
            hoodPose.Position = hoodDegreesToRotations(90 - Math.toDegrees(sotm.launchAngle));
            if (SmartDashboard.getBoolean(AutoAimConstants.useIKSolverKey, AutoAimConstants.defaultUseIKSolver)
                    || pass) {
                velocity = applyShooterControl(
                        slippageMap.correctedMotorRps(launchMpsToMotorRps(sotm.launchMps)));
            } else {
                velocity = applyShooterControl(launchMpsToMotorRps(sotm.launchMps));
            }
        }

        if (manualOverride.get()) {
            hoodPose.Position = 0;
            velocity = 0;
            mode = ShootMode.COAST;
            brake = true;
        }

        boolean mapTuneEnabled = SmartDashboard.getBoolean(
                MapTuneConstants.enableKey,
                MapTuneConstants.defaultEnable);
        if (mapTuneEnabled) {
            applyLiveMap();
        }

        if (brake) {
            spinMotor.setControl(new StaticBrake());
        } else {
            spinMotor.setControl(spinPose);
        }

        hoodMotor1.setControl(hoodPose);
        SmartDashboard.putNumber("Turret/ShooterCurrentRps", shootMotor1.getVelocity().getValueAsDouble());
        switch (mode) {
            case DUTY_CYCLE_BANG_BANG -> shootMotor1.setControl(shootDutyBang.withVelocity(velocity).withSlot(1));
            case TORQUE_CURRENT_BANG_BANG -> shootMotor1.setControl(shootTorqueBang.withVelocity(velocity).withSlot(0));
            case COAST -> shootMotor1.set(0);
        }

        SmartDashboard.putData("Turret Field", m_field);
        SmartDashboard.putBoolean("Can Index", turretOnTarget());
    }
}
