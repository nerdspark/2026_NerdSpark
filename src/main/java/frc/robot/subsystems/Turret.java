package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.field;
import frc.robot.Constants.turretSimConstants;
import frc.robot.Constants.turretTelemetryConstants;

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
    private final TalonFX hoodMotor;
    private final TalonFX shootMotor;
    private final CANcoder spinCancoder1;
    private final CANcoder spinCancoder2;

    private final VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private final PositionVoltage hoodPose = new PositionVoltage(0);
    private final PositionVoltage spinPose = new PositionVoltage(0);

    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> speed;
    private final Supplier<DriverStation.Alliance> alliance;
    private final Supplier<Boolean> aimTurret;
    private final ArmFeedforward hoodFeedforward;

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
            Supplier<Boolean> aimTurret) {
        pose = robotPose;
        speed = speeds;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;

        spinMotor = new TalonFX(TurretConstants.spinMotorId);
        hoodMotor = new TalonFX(TurretConstants.hoodMotorId);
        shootMotor = new TalonFX(TurretConstants.shootMotorId);

        spinCancoder1 = new CANcoder(0);
        spinCancoder2 = new CANcoder(0);
        hoodFeedforward = new ArmFeedforward(
            TurretConstants.hoodKsVolts,
            TurretConstants.hoodKgVolts,
            TurretConstants.hoodKvVolts,
            TurretConstants.hoodKaVolts
        );

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(TurretConstants.spinKp)
                .withKI(TurretConstants.spinKi)
                .withKD(TurretConstants.spinKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConstants.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(TurretConstants.hoodKp)
                .withKI(TurretConstants.hoodKi)
                .withKD(TurretConstants.hoodKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConstants.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );
        TalonFXConfiguration shootConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                .withKP(TurretConstants.shootKp)
                .withKI(TurretConstants.shootKi)
                .withKD(TurretConstants.shootKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConstants.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            );
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            );
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            );

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        shootMotor.getConfigurator().apply(shootConfig);
        spinPositionSignal = spinMotor.getPosition();
        spinClosedLoopOutputSignal = spinMotor.getClosedLoopOutput();
        spinMotorVoltageSignal = spinMotor.getMotorVoltage();
        hoodPositionSignal = hoodMotor.getPosition();
        hoodClosedLoopOutputSignal = hoodMotor.getClosedLoopOutput();
        hoodMotorVoltageSignal = hoodMotor.getMotorVoltage();
        shootVelocitySignal = shootMotor.getVelocity();
        shootMotorVoltageSignal = shootMotor.getMotorVoltage();

        if (RobotBase.isSimulation()) {
            DCMotor motorModel = DCMotor.getKrakenX60Foc(turretSimConstants.motorCount);
            double gearing = 1.0 / TurretConstants.spinRatio;
            spinSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, turretSimConstants.turretJ, gearing),
                motorModel
            );
            spinSimState = spinMotor.getSimState();

            double hoodGearing = 1.0 / TurretConstants.hoodRatio;
            hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, turretSimConstants.hoodJ, hoodGearing),
                motorModel
            );
            hoodSimState = hoodMotor.getSimState();

            double shooterGearing = 1.0 / TurretConstants.shooterRatio;
            shooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, turretSimConstants.shooterJ, shooterGearing),
                motorModel
            );
            shooterSimState = shootMotor.getSimState();
        }

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);
    }

    /**
     * Aims the hood of the turret and sets the wheel speed.
     *
     * @param distance the distance to the center of the hub
     */
    private void aimHood(double distance) {
        ShotSolution solution = solveShotForDistance(distance);
        if (solution == null) {
            hoodZero();
            return;
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
                .withKD(kD);
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
                .withKD(kD);
            hoodMotor.getConfigurator().apply(slot0);
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
            hoodPose.FeedForward = hoodFeedforward.calculate(getHoodSetpointRadians(), 0.0);
            spinMotor.setControl(spinPose);
            hoodMotor.setControl(hoodPose);
            shootMotor.setControl(shootVelocity);
            publishTelemetry();
            return;
        }

        if (aimTurret.get()) {
            Pose2d currPose = pose.get();

            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (currPose.getX() <= TurretConstants.blueHubMaxX) {
                    double xError = field.blueHub.getX() - currPose.getX();
                    double yError = field.blueHub.getY() - currPose.getY();
                    double hubRadians = Math.atan2(yError, xError);

                    aimTurret(normalizeRadians(hubRadians - currPose.getRotation().getRadians()));
                    aimHood(Math.hypot(yError, xError));
                } else {
                    // Add passing here if needed.
                    hoodZero();
                }
            } else {
                if (currPose.getX() >= TurretConstants.redHubMinX) {
                    double xError = field.redHub.getX() - currPose.getX();
                    double yError = field.redHub.getY() - currPose.getY();
                    double hubRadians = Math.atan2(yError, xError);

                    aimTurret(normalizeRadians(hubRadians - currPose.getRotation().getRadians()));
                    aimHood(Math.hypot(yError, xError));
                } else {
                    // Adding passing here if needed.
                    hoodZero();
                }
            }
        } else {
            hoodZero();
        }

        hoodPose.FeedForward = hoodFeedforward.calculate(getHoodSetpointRadians(), 0.0);
        spinMotor.setControl(spinPose);
        hoodMotor.setControl(hoodPose);
        shootMotor.setControl(shootVelocity);
        publishTelemetry();
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

        SmartDashboard.putNumber(turretTelemetryConstants.angleDegKey, turretDegrees);
        SmartDashboard.putNumber(turretTelemetryConstants.spinSetpointRotKey, spinPose.Position);
        SmartDashboard.putNumber(turretTelemetryConstants.spinClosedLoopOutputKey, spinClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(turretTelemetryConstants.spinMotorVoltsKey, spinMotorVoltageSignal.getValueAsDouble());
        SmartDashboard.putNumber(turretTelemetryConstants.hoodAngleDegKey, hoodDegrees);
        SmartDashboard.putNumber(turretTelemetryConstants.hoodSetpointRotKey, hoodPose.Position);
        SmartDashboard.putNumber(turretTelemetryConstants.hoodClosedLoopOutputKey, hoodClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(turretTelemetryConstants.hoodMotorVoltsKey, hoodMotorVoltageSignal.getValueAsDouble());
        SmartDashboard.putNumber(turretTelemetryConstants.shooterSetpointRpsKey, shootVelocity.Velocity / TurretConstants.shooterRatio);
        SmartDashboard.putNumber(turretTelemetryConstants.shooterMotorRpsKey, shooterMotorRps);
        SmartDashboard.putNumber(turretTelemetryConstants.shooterWheelRpsKey, shooterWheelRps);
        SmartDashboard.putNumber(turretTelemetryConstants.shooterMotorVoltsKey, shootMotorVoltageSignal.getValueAsDouble());
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

    private double getHoodSetpointRadians() {
        double hoodDegrees = (hoodPose.Position / TurretConstants.hoodRatio) * 360.0;
        return Math.toRadians(hoodDegrees) + TurretConstants.hoodFeedforwardOffsetRad;
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
