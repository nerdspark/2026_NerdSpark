package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.TurretConfig;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretTelemetryConstants;
import frc.robot.Constants.turretSimConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.Telemetry;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double GRAVITY = 9.80665;

    private static final class ShotSolution {
        private final double hoodDegrees;
        private final double motorRps;

        private ShotSolution(double hoodDegrees, double motorRps) {
            this.hoodDegrees = hoodDegrees;
            this.motorRps = motorRps;
        }
    }

    private final TalonFX spinMotor;
    private final TalonFX hoodMotor1;
    private final TalonFX hoodMotor2;
    private final TalonFX shootMotor1;
    private final TalonFX shootMotor2;
    private final CANcoder spinCancoder1;
    private final CANcoder spinCancoder2;

    private final VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private final MotionMagicVoltage hoodPose = new MotionMagicVoltage(0);
    private final MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> speed;
    private final Supplier<DriverStation.Alliance> alliance;
    private final Supplier<Boolean> aimTurret;
    private final Telemetry telemetry;

    private boolean manualSpinOverride = false;
    private boolean manualHoodOverride = false;
    private double manualSpinSetpointDeg = 0.0;
    private double manualHoodSetpointDeg = 0.0;
    private double lastSpinKp = Double.NaN;
    private double lastSpinKi = Double.NaN;
    private double lastSpinKd = Double.NaN;
    private double lastHoodKp = Double.NaN;
    private double lastHoodKi = Double.NaN;
    private double lastHoodKd = Double.NaN;

    private DCMotorSim spinSim;
    private TalonFXSimState spinSimState;
    private DCMotorSim hoodSim;
    private TalonFXSimState hoodSimState;
    private DCMotorSim shooterSim;
    private TalonFXSimState shooterSimState;
    private StatusSignal<Angle> spinPositionSignal;
    private StatusSignal<Double> spinClosedLoopOutputSignal;
    private StatusSignal<Voltage> spinMotorVoltageSignal;
    private StatusSignal<Angle> hoodPositionSignal;
    private StatusSignal<Double> hoodClosedLoopOutputSignal;
    private StatusSignal<Voltage> hoodMotorVoltageSignal;
    private StatusSignal<AngularVelocity> shootVelocitySignal;
    private StatusSignal<Voltage> shootMotorVoltageSignal;

    private double turretAngle;

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<DriverStation.Alliance> driverAlliance,
            Supplier<Boolean> aimTurret, Telemetry telemetry) {
        pose = robotPose;
        speed = speeds;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;
        this.telemetry = telemetry;

        spinMotor = new TalonFX(TurretConfig.spinMotorId, Constants.CANbus);
        hoodMotor1 = new TalonFX(TurretConfig.hoodMotor1Id, Constants.CANbus);
        hoodMotor2 = new TalonFX(TurretConfig.hoodMotor2Id, Constants.CANbus);
        shootMotor1 = new TalonFX(TurretConfig.shootMotor1Id, Constants.CANbus);
        shootMotor2 = new TalonFX(TurretConfig.shootMotor2Id, Constants.CANbus);

        spinCancoder1 = new CANcoder(TurretConfig.spinCancoder1Id, Constants.CANbus);
        spinCancoder2 = new CANcoder(TurretConfig.spinCancoder2Id, Constants.CANbus);

        SmartDashboard.setDefaultBoolean(turretTargetConstants.enableKey, turretTargetConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(turretTargetConstants.targetXKey, turretTargetConstants.defaultTargetX);
        SmartDashboard.setDefaultNumber(turretTargetConstants.targetYKey, turretTargetConstants.defaultTargetY);

        Slot0Configs spinSlot = new Slot0Configs()
            .withKP(TurretConfig.spinKp)
            .withKI(TurretConfig.spinKi)
            .withKD(TurretConfig.spinKd)
            .withKS(TurretConfig.spinKs)
            .withKV(TurretConfig.spinKv)
            .withKA(TurretConfig.spinKa);
        MotionMagicConfigs spinMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
            .withMotionMagicAcceleration(TurretConfig.spinAccel);
        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withSlot0(spinSlot)
            .withMotionMagic(spinMagic)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );

        Slot0Configs hoodSlot1 = new Slot0Configs()
            .withKP(TurretConfig.hoodKp1)
            .withKI(TurretConfig.hoodKi1)
            .withKD(TurretConfig.hoodKd1)
            .withKS(TurretConfig.hoodKs1)
            .withKV(TurretConfig.hoodKv1)
            .withKA(TurretConfig.hoodKa1);
        Slot0Configs hoodSlot2 = new Slot0Configs()
            .withKP(TurretConfig.hoodKp2)
            .withKI(TurretConfig.hoodKi2)
            .withKD(TurretConfig.hoodKd2)
            .withKS(TurretConfig.hoodKs2)
            .withKV(TurretConfig.hoodKv2)
            .withKA(TurretConfig.hoodKa2);
        MotionMagicConfigs hoodMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(TurretConfig.hoodVelocity)
            .withMotionMagicAcceleration(TurretConfig.hoodAccel);
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
            .withSlot0(hoodSlot1)
            .withMotionMagic(hoodMagic)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );
        TalonFXConfiguration hoodConfig2 = new TalonFXConfiguration()
            .withSlot0(hoodSlot2)
            .withMotionMagic(hoodMagic)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );

        Slot0Configs shootSlot = new Slot0Configs()
            .withKP(TurretConfig.bangbangKp);
        MotorOutputConfigs shootOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withPeakForwardDutyCycle(TurretConfig.peakDutyCycle)
            .withPeakReverseDutyCycle(-TurretConfig.peakDutyCycle);
        TalonFXConfiguration shootConfig = new TalonFXConfiguration()
            .withMotorOutput(shootOutput)
            .withSlot0(shootSlot)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );

        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConfig.spinCancoder1Offset)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            );
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConfig.spinCancoder2Offset)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            );

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor1.getConfigurator().apply(hoodConfig1);
        hoodMotor2.getConfigurator().apply(hoodConfig2);
        shootMotor1.getConfigurator().apply(shootConfig);
        shootMotor2.getConfigurator().apply(shootConfig);
        hoodMotor2.setControl(new Follower(hoodMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
        shootMotor2.setControl(new Follower(shootMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        spinPositionSignal = spinMotor.getPosition();
        spinClosedLoopOutputSignal = spinMotor.getClosedLoopOutput();
        spinMotorVoltageSignal = spinMotor.getMotorVoltage();
        hoodPositionSignal = hoodMotor1.getPosition();
        hoodClosedLoopOutputSignal = hoodMotor1.getClosedLoopOutput();
        hoodMotorVoltageSignal = hoodMotor1.getMotorVoltage();
        shootVelocitySignal = shootMotor1.getVelocity();
        shootMotorVoltageSignal = shootMotor1.getMotorVoltage();

        if (RobotBase.isSimulation()) {
            DCMotor spinModel = DCMotor.getKrakenX60Foc(turretSimConstants.spinMotorCount);
            double gearing = 1.0 / TurretConstants.spinRatio;
            spinSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(spinModel, turretSimConstants.turretJ, gearing),
                spinModel
            );
            spinSimState = spinMotor.getSimState();

            DCMotor hoodModel = DCMotor.getKrakenX60Foc(turretSimConstants.hoodMotorCount);
            double hoodGearing = 1.0 / TurretConstants.hoodRatio;
            hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(hoodModel, turretSimConstants.hoodJ, hoodGearing),
                hoodModel
            );
            hoodSimState = hoodMotor1.getSimState();

            DCMotor shooterModel = DCMotor.getKrakenX60Foc(turretSimConstants.shooterMotorCount);
            double shooterGearing = 1.0 / TurretConstants.shooterRatio;
            shooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(shooterModel, turretSimConstants.shooterJ, shooterGearing),
                shooterModel
            );
            shooterSimState = shootMotor1.getSimState();
        }

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);
    }

    /**
     * Aims the hood of the turret and sets the wheel speed.
     *
     * @param distance the distance to the center of the hub
     */
    private ShotSolution aimHood(double distance) {
        ShotSolution solution = solveShotForDistance(distance);
        if (solution == null) {
            hoodZero();
            return null;
        }

        hoodPose.Position = degreesToHoodMotorRotations(solution.hoodDegrees);

        double robotHeading = pose.get().getRotation().getRadians();
        double shooterFOA = robotHeading + TurretConstants.turretOffset + turretAngle;
        ChassisSpeeds robotFOS = ChassisSpeeds.fromRobotRelativeSpeeds(speed.get(), pose.get().getRotation());
        double robotSpeed = Math.hypot(robotFOS.vxMetersPerSecond, robotFOS.vyMetersPerSecond);
        double robotVelAngle = Math.atan2(robotFOS.vyMetersPerSecond, robotFOS.vxMetersPerSecond);
        double vParallel = robotSpeed * Math.cos(robotVelAngle - shooterFOA);
        double deltaMotorRPS = (vParallel / (2.0 * Math.PI * TurretConstants.shooterWheelRadius)) * TurretConstants.shooterRatio;

        shootVelocity.Velocity = solution.motorRps - deltaMotorRPS;
        return solution;
    }

    // When we are out of shooting range stop wheels and send hood to stow.
    private void hoodZero() {
        hoodPose.Position = TurretConstants.hoodStow;
        shootVelocity.Velocity = 0;
    }

    /**
     * Aims the turret only.
     *
     * @param neededAngle the field-centric target angle minus the chassis heading in radians
     */
    private void aimTurret(double neededAngle) {
        // Normalize encoders to [0, 1)
        double aN = ((spinCancoder1.getAbsolutePosition().getValueAsDouble() % 1.0) + 1.0) % 1.0;
        double bN = ((spinCancoder2.getAbsolutePosition().getValueAsDouble() % 1.0) + 1.0) % 1.0;

        double bestT = 0.0;
        double bestError = Double.MAX_VALUE;

        // Search over physically possible turret turns
        for (int k = -4; k <= 4; k++) {
            double T = (aN + k) / TurretConstants.spinCancoder1Ratio;

            double bPred = (TurretConstants.spinCancoder2Ratio * T) % 1.0;
            if (bPred < 0) {
                bPred += 1.0;
            }

            double error = Math.abs(bPred - bN);
            error = Math.min(error, 1.0 - error);

            if (error < bestError) {
                bestError = error;
                bestT = T;
            }
        }

        // Convert turret rotations to radians
        double theta = bestT * 2.0 * Math.PI;

        // Wrap to [-pi, pi]
        turretAngle = Math.atan2(Math.sin(theta), Math.cos(theta));

        neededAngle -= TurretConstants.turretOffset;
        neededAngle = normalizeRadians(neededAngle);

        // Compute angular error
        double error = neededAngle - turretAngle;

        // Find the shortest path
        double shortError = error;
        if (shortError > Math.PI) {
            shortError -= TWO_PI;
        } else if (shortError < -Math.PI) {
            shortError += TWO_PI;
        }

        // Does the short path cross the wrap?
        boolean shortPathCrossesWrap = Math.abs(turretAngle) > Math.PI / 2.0
            && Math.abs(neededAngle) > Math.PI / 2.0
            && Math.signum(turretAngle) != Math.signum(neededAngle);

        // Select legal error
        double chosenError = shortPathCrossesWrap ? error : shortError;

        // Command motor
        spinPose.Position = (chosenError / TWO_PI) * TurretConstants.spinRatio;
    }

    public void setManualSpinSetpointDegrees(double degrees) {
        manualSpinSetpointDeg = degrees;
        manualSpinOverride = true;
    }

    public void clearManualSpinOverride() {
        manualSpinOverride = false;
    }

    public void setManualHoodSetpointDegrees(double degrees) {
        manualHoodSetpointDeg = degrees;
        manualHoodOverride = true;
    }

    public void clearManualHoodOverride() {
        manualHoodOverride = false;
    }

    public void setSpinPidGains(double kP, double kI, double kD) {
        if (kP != lastSpinKp || kI != lastSpinKi || kD != lastSpinKd) {
            Slot0Configs slot0 = new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(TurretConfig.spinKs)
                .withKV(TurretConfig.spinKv)
                .withKA(TurretConfig.spinKa);
            spinMotor.getConfigurator().apply(slot0);
            lastSpinKp = kP;
            lastSpinKi = kI;
            lastSpinKd = kD;
        }
    }

    public void setHoodPidGains(double kP, double kI, double kD) {
        if (kP != lastHoodKp || kI != lastHoodKi || kD != lastHoodKd) {
            Slot0Configs slot0 = new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(TurretConfig.hoodKs1)
                .withKV(TurretConfig.hoodKv1)
                .withKA(TurretConfig.hoodKa1);
            hoodMotor1.getConfigurator().apply(slot0);
            hoodMotor2.getConfigurator().apply(slot0);
            lastHoodKp = kP;
            lastHoodKi = kI;
            lastHoodKd = kD;
        }
    }

    public void zeroSpinSimPosition() {
        if (spinSimState == null || spinSim == null) {
            return;
        }
        spinSim.setState(0.0, 0.0);
        spinSimState.setRawRotorPosition(0.0);
        spinSimState.setRotorVelocity(0.0);
    }

    public void zeroHoodSimPosition() {
        if (hoodSimState == null || hoodSim == null) {
            return;
        }
        hoodSim.setState(0.0, 0.0);
        hoodSimState.setRawRotorPosition(0.0);
        hoodSimState.setRotorVelocity(0.0);
    }

    @Override
    public void periodic() {
        if (manualSpinOverride || manualHoodOverride) {
            if (manualSpinOverride) {
                double clamped = Math.max(TurretConstants.turretMinDegrees,
                    Math.min(TurretConstants.turretMaxDegrees, manualSpinSetpointDeg));
                spinPose.Position = degreesToSpinMotorRotations(clamped);
            }
            if (manualHoodOverride) {
                double clamped = Math.max(TurretConstants.hoodMinDegrees,
                    Math.min(TurretConstants.hoodMaxDegrees, manualHoodSetpointDeg));
                hoodPose.Position = degreesToHoodMotorRotations(clamped);
                shootVelocity.Velocity = 0;
            } else {
                hoodZero();
            }
            spinMotor.setControl(spinPose);
            hoodMotor1.setControl(hoodPose);
            shootMotor1.setControl(shootVelocity);
            publishTelemetry();
            clearShotVisualization();
            return;
        }

        if (aimTurret.get()) {
            Pose2d currPose = pose.get();
            Translation2d turretTranslation = getTurretTranslation(currPose);
            boolean useLiveTarget = SmartDashboard.getBoolean(
                turretTargetConstants.enableKey,
                turretTargetConstants.defaultEnable
            );
            if (useLiveTarget) {
                double targetX = SmartDashboard.getNumber(
                    turretTargetConstants.targetXKey,
                    turretTargetConstants.defaultTargetX
                );
                double targetY = SmartDashboard.getNumber(
                    turretTargetConstants.targetYKey,
                    turretTargetConstants.defaultTargetY
                );
                if (telemetry != null) {
                    telemetry.setPassTarget(
                        new Translation2d(targetX, targetY),
                        TurretConstants.passTargetRadiusMeters,
                        TurretConstants.passTargetCirclePoints
                    );
                }
                double xError = targetX - turretTranslation.getX();
                double yError = targetY - turretTranslation.getY();
                double targetRadians = Math.atan2(yError, xError);

                aimTurret(normalizeRadians(targetRadians - currPose.getRotation().getRadians()));
                ShotSolution solution = aimHood(Math.hypot(yError, xError));
                updateShotVisualization(currPose, targetX, targetY, solution);
            } else if (alliance.get() == DriverStation.Alliance.Blue) {
                if (telemetry != null) {
                    telemetry.clearPassTarget();
                }
                if (currPose.getX() <= TurretConstants.blueHubMaxX) {
                    double xError = Field.blueHub.getX() - turretTranslation.getX();
                    double yError = Field.blueHub.getY() - turretTranslation.getY();
                    double hubRadians = Math.atan2(yError, xError);

                    aimTurret(normalizeRadians(hubRadians - currPose.getRotation().getRadians()));
                    ShotSolution solution = aimHood(Math.hypot(yError, xError));
                    updateShotVisualization(currPose, Field.blueHub.getX(), Field.blueHub.getY(), solution);
                } else {
                    // Add passing here if needed.
                    hoodZero();
                    clearShotVisualization();
                }
            } else {
                if (telemetry != null) {
                    telemetry.clearPassTarget();
                }
                if (currPose.getX() >= TurretConstants.redHubMinX) {
                    double xError = Field.redHub.getX() - turretTranslation.getX();
                    double yError = Field.redHub.getY() - turretTranslation.getY();
                    double hubRadians = Math.atan2(yError, xError);

                    aimTurret(normalizeRadians(hubRadians - currPose.getRotation().getRadians()));
                    ShotSolution solution = aimHood(Math.hypot(yError, xError));
                    updateShotVisualization(currPose, Field.redHub.getX(), Field.redHub.getY(), solution);
                } else {
                    // Adding passing here if needed.
                    hoodZero();
                    clearShotVisualization();
                }
            }
        } else {
            hoodZero();
            if (telemetry != null) {
                telemetry.clearPassTarget();
            }
            clearShotVisualization();
        }

        spinMotor.setControl(spinPose);
        hoodMotor1.setControl(hoodPose);
        shootMotor1.setControl(shootVelocity);
        publishTelemetry();
    }

    private Translation2d getTurretTranslation(Pose2d robotPose) {
        return robotPose.getTranslation().plus(
            TurretConstants.robotToTurret.rotateBy(robotPose.getRotation())
        );
    }

    private void publishTelemetry() {
        BaseStatusSignal.refreshAll(
            spinPositionSignal,
            spinClosedLoopOutputSignal,
            spinMotorVoltageSignal,
            hoodPositionSignal,
            hoodClosedLoopOutputSignal,
            hoodMotorVoltageSignal,
            shootVelocitySignal,
            shootMotorVoltageSignal
        );
        double motorRotations = spinPositionSignal.getValueAsDouble();
        double turretDegrees = normalize180((motorRotations / TurretConstants.spinRatio) * 360.0);
        double hoodRotations = hoodPositionSignal.getValueAsDouble();
        double hoodDegrees = normalize180((hoodRotations / TurretConstants.hoodRatio) * 360.0);
        double shooterMotorRps = shootVelocitySignal.getValueAsDouble();
        double shooterWheelRps = shooterMotorRps / TurretConstants.shooterRatio;

        SmartDashboard.putNumber(TurretTelemetryConstants.angleDegKey, turretDegrees);
        SmartDashboard.putNumber(TurretTelemetryConstants.spinSetpointRotKey, spinPose.Position);
        SmartDashboard.putNumber(TurretTelemetryConstants.spinClosedLoopOutputKey, spinClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.spinMotorVoltsKey, spinMotorVoltageSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.hoodAngleDegKey, hoodDegrees);
        SmartDashboard.putNumber(TurretTelemetryConstants.hoodSetpointRotKey, hoodPose.Position);
        SmartDashboard.putNumber(TurretTelemetryConstants.hoodClosedLoopOutputKey, hoodClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.hoodMotorVoltsKey, hoodMotorVoltageSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.shooterSetpointRpsKey, shootVelocity.Velocity / TurretConstants.shooterRatio);
        SmartDashboard.putNumber(TurretTelemetryConstants.shooterMotorRpsKey, shooterMotorRps);
        SmartDashboard.putNumber(TurretTelemetryConstants.shooterWheelRpsKey, shooterWheelRps);
        SmartDashboard.putNumber(TurretTelemetryConstants.shooterMotorVoltsKey, shootMotorVoltageSignal.getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if (spinSim != null && spinSimState != null) {
            spinSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            spinSim.setInputVoltage(spinSimState.getMotorVoltage());
            spinSim.update(turretSimConstants.loopPeriodSeconds);

            double turretRotations = spinSim.getAngularPositionRotations();
            double turretRps = spinSim.getAngularVelocityRPM() / 60.0;
            spinSimState.setRawRotorPosition(turretRotations * TurretConstants.spinRatio);
            spinSimState.setRotorVelocity(turretRps * TurretConstants.spinRatio);
        }

        if (hoodSim != null && hoodSimState != null) {
            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
            hoodSim.update(turretSimConstants.loopPeriodSeconds);

            double hoodRotations = hoodSim.getAngularPositionRotations();
            double hoodRps = hoodSim.getAngularVelocityRPM() / 60.0;
            hoodSimState.setRawRotorPosition(hoodRotations * TurretConstants.hoodRatio);
            hoodSimState.setRotorVelocity(hoodRps * TurretConstants.hoodRatio);
        }

        if (shooterSim != null && shooterSimState != null) {
            shooterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            shooterSim.setInputVoltage(shooterSimState.getMotorVoltage());
            shooterSim.update(turretSimConstants.loopPeriodSeconds);

            double shooterRotations = shooterSim.getAngularPositionRotations();
            double shooterRps = shooterSim.getAngularVelocityRPM() / 60.0;
            shooterSimState.setRawRotorPosition(shooterRotations * TurretConstants.shooterRatio);
            shooterSimState.setRotorVelocity(shooterRps * TurretConstants.shooterRatio);
        }
    }

    private static double degreesToSpinMotorRotations(double degrees) {
        return (degrees / 360.0) * TurretConstants.spinRatio;
    }

    private static double degreesToHoodMotorRotations(double degrees) {
        return (degrees / 360.0) * TurretConstants.hoodRatio;
    }

    private ShotSolution solveShotForDistance(double distanceMeters) {
        if (distanceMeters <= 0.0) {
            return null;
        }
        double deltaHeight = TurretConstants.targetHeightMeters - TurretConstants.shooterMuzzleHeightMeters;
        double bestAngleDeg = Double.NaN;
        double bestMotorRps = Double.POSITIVE_INFINITY;
        double minAngle = TurretConstants.hoodMinDegrees;
        double maxAngle = TurretConstants.hoodMaxDegrees;

        for (double angleDeg = minAngle; angleDeg <= maxAngle; angleDeg += TurretConstants.shotAngleStepDeg) {
            double angleRad = Math.toRadians(angleDeg);
            double speedMps = solveBallisticSpeed(distanceMeters, angleRad, deltaHeight);
            if (!Double.isFinite(speedMps)) {
                continue;
            }
            double wheelRps = speedMps / (TWO_PI * TurretConstants.shooterWheelRadius);
            double motorRps = wheelRps * TurretConstants.shooterRatio;
            if (motorRps <= TurretConstants.shooterMaxMotorRps && motorRps < bestMotorRps) {
                bestMotorRps = motorRps;
                bestAngleDeg = angleDeg;
            }
        }

        if (!Double.isFinite(bestAngleDeg)) {
            return null;
        }
        return new ShotSolution(bestAngleDeg, bestMotorRps);
    }

    private static double solveBallisticSpeed(double distanceMeters, double angleRad, double deltaHeightMeters) {
        double cos = Math.cos(angleRad);
        if (Math.abs(cos) < 1e-6) {
            return Double.NaN;
        }
        double tan = Math.tan(angleRad);
        double denom = 2.0 * cos * cos * (distanceMeters * tan - deltaHeightMeters);
        if (denom <= 0.0) {
            return Double.NaN;
        }
        double numerator = GRAVITY * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denom);
    }

    private void updateShotVisualization(Pose2d robotPose, double targetX, double targetY, ShotSolution solution) {
        if (telemetry == null) {
            return;
        }
        if (solution == null) {
            clearShotVisualization();
            return;
        }

        Translation2d turretTranslation = getTurretTranslation(robotPose);
        double dx = targetX - turretTranslation.getX();
        double dy = targetY - turretTranslation.getY();
        double distance = Math.hypot(dx, dy);
        if (distance <= 1e-6) {
            clearShotVisualization();
            return;
        }

        double turretDirection = robotPose.getRotation().getRadians() + TurretConstants.turretOffset + turretAngle;
        Rotation2d heading = new Rotation2d(turretDirection);

        double hoodRad = Math.toRadians(solution.hoodDegrees);
        double wheelRps = solution.motorRps / TurretConstants.shooterRatio;
        double muzzleSpeed = wheelRps * TWO_PI * TurretConstants.shooterWheelRadius;
        double horizontalSpeed = muzzleSpeed * Math.cos(hoodRad);
        if (horizontalSpeed <= 1e-6) {
            clearShotVisualization();
            return;
        }

        double flightTime = distance / horizontalSpeed;
        int points = Math.max(2, TurretConstants.shotTrajectoryPoints);
        Pose2d[] trajectory = new Pose2d[points];
        for (int i = 0; i < points; i++) {
            double t = flightTime * i / (points - 1);
            double horiz = horizontalSpeed * t;
            double x = turretTranslation.getX() + Math.cos(turretDirection) * horiz;
            double y = turretTranslation.getY() + Math.sin(turretDirection) * horiz;
            trajectory[i] = new Pose2d(x, y, heading);
        }

        Pose2d targetPose = new Pose2d(targetX, targetY, new Rotation2d());
        Pose2d landingPose = trajectory[trajectory.length - 1];

        telemetry.setShotTrajectory(trajectory);
        telemetry.setShotTarget(new Pose2d[] { targetPose });
        telemetry.setShotLanding(new Pose2d[] { landingPose });
    }

    private void clearShotVisualization() {
        if (telemetry == null) {
            return;
        }
        telemetry.setShotTrajectory(new Pose2d[] {});
        telemetry.setShotTarget(new Pose2d[] {});
        telemetry.setShotLanding(new Pose2d[] {});
    }

    private static double normalize180(double degrees) {
        degrees %= 360.0; // wrap within -360..360
        if (degrees > 180.0) {
            degrees -= 360.0; // move into -180..180
        } else if (degrees < -180.0) {
            degrees += 360.0; // move into -180..180
        }
        return degrees;
    }

    // Normalizes to [-pi, pi]
    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
