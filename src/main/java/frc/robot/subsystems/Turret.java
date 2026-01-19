package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.field;
import frc.robot.Constants.turretConstants;
import frc.robot.Constants.turretSimConstants;
import frc.robot.Constants.turretTelemetryConstants;

public class Turret extends SubsystemBase {
    private TalonFX spinMotor, hoodMotor, shootMotor;

    private VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private PositionVoltage hoodPose = new PositionVoltage(0);
    private PositionVoltage spinPose = new PositionVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<DriverStation.Alliance> alliance;
    private Supplier<Boolean> aimTurret;
    private boolean manualOverride = false;
    private double manualSetpointDeg = 0.0;
    private double lastSpinKp = Double.NaN;
    private double lastSpinKi = Double.NaN;
    private double lastSpinKd = Double.NaN;
    private DCMotorSim spinSim;
    private TalonFXSimState spinSimState;
    private StatusSignal<Angle> spinPositionSignal;
    private StatusSignal<Double> spinClosedLoopOutputSignal;
    private StatusSignal<Voltage> spinMotorVoltageSignal;

    public Turret(Supplier<Pose2d> robotPose, Supplier<DriverStation.Alliance> driverAlliance, Supplier<Boolean> aimTurret) {
        pose = robotPose;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;

        spinMotor = new TalonFX(turretConstants.spinMotorId);
        hoodMotor = new TalonFX(turretConstants.hoodMotorId);
        shootMotor = new TalonFX(turretConstants.shootMotorId);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(turretConstants.spinKp)
                .withKI(turretConstants.spinKi)
                .withKD(turretConstants.spinKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(turretConstants.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(turretConstants.hoodKp)
                .withKI(turretConstants.hoodKi)
                .withKD(turretConstants.hoodKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(turretConstants.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                .withKP(turretConstants.shootKp)
                .withKI(turretConstants.shootKi)
                .withKD(turretConstants.shootKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(turretConstants.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        shootMotor.getConfigurator().apply(shootConfig);
        spinPositionSignal = spinMotor.getPosition();
        spinClosedLoopOutputSignal = spinMotor.getClosedLoopOutput();
        spinMotorVoltageSignal = spinMotor.getMotorVoltage();

        if (RobotBase.isSimulation()) {
            DCMotor motorModel = DCMotor.getKrakenX60Foc(turretSimConstants.motorCount);
            double gearing = 1.0 / turretConstants.spinOverrallRatio;
            spinSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, turretSimConstants.turretJ, gearing),
                motorModel
            );
            spinSimState = spinMotor.getSimState();
        }
    }
    
    // Moves the indexer for ball transfer and checks wheel speed
    public void shoot() {
        
    }

    /** 
     * Aims the hood of the turret and sets the wheel speed
     * 
     * @param distance the distance to the center of the hub
    */
    private void aimHood(double distance) {
        double[] map = turretConstants.turretMap.get(distance);

        hoodPose.Position = map[0];
        shootVelocity.Velocity = map[1];
    }

    // When we are out of shooting range stop wheels and send hood to stow
    private void hoodZero() {
        hoodPose.Position = turretConstants.hoodStow;
        shootVelocity.Velocity = 0;
    }

    /**
    * Aims the turret only.
    * Turret is only +90 to -90.
    *
    * @param neededAngle the field-centric target angle minus the chassis heading in degrees
    */
    private void aimTurret(double neededAngle) {
        double motorRotations = spinMotor.getPosition().getValueAsDouble();
        double turretDegrees = (motorRotations / turretConstants.spinOverrallRatio) * 360;
        normalize180(turretDegrees);

        neededAngle -= turretConstants.turretOffset;

        boolean turretOK;
        if (neededAngle > turretConstants.turretMaxDegrees) {
            neededAngle = turretConstants.turretMaxDegrees;
            turretOK = false;
        } else if (neededAngle < turretConstants.turretMinDegrees) {
            neededAngle = turretConstants.turretMinDegrees;
            turretOK = false;
        } else {
            turretOK = true;
        }
        SmartDashboard.putBoolean(turretTelemetryConstants.withinLimitKey, turretOK);

        double error = neededAngle - turretDegrees;
        spinPose.Position = error * turretConstants.turretAimErrorScale;
    }

    /**
     * Aims the chassis and the turret
     * 
     * @param neededAngle the field-centric target angle
     * @param chassisAngle the current heading of the chassis
     * @return the target chassis rotation in degrees
     */
    public double aimChassis(double neededAngle, double chassisAngle) {
        double turretRel = neededAngle - chassisAngle;
        turretRel -= turretConstants.turretOffset;
        
        // aimTurret(turretRel);

        normalize180(turretRel);

        double chassisRotation = 0.0;

        // If target is outside turret limits, compute chassis rotation
        if (turretRel > turretConstants.turretMaxDegrees) {
            chassisRotation = turretRel - turretConstants.chassisRotationBufferDegrees;
        } else if (turretRel < turretConstants.turretMinDegrees) {
            chassisRotation = turretRel + turretConstants.chassisRotationBufferDegrees;
        }

        return chassisRotation + chassisAngle;
    }

    public void setManualSetpointDegrees(double degrees) {
        manualSetpointDeg = degrees;
        manualOverride = true;
    }

    public void clearManualOverride() {
        manualOverride = false;
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

    public void zeroSimPosition() {
        if (spinSimState == null || spinSim == null) {
            return;
        }
        spinSim.setState(0.0, 0.0);
        spinSimState.setRawRotorPosition(0.0);
        spinSimState.setRotorVelocity(0.0);
    }

    @Override
    public void periodic() {
        if (manualOverride) {
            double clamped = Math.max(turretConstants.turretMinDegrees,
                Math.min(turretConstants.turretMaxDegrees, manualSetpointDeg));
            spinPose.Position = degreesToMotorRotations(clamped);
            hoodZero();
            spinMotor.setControl(spinPose);
            hoodMotor.setControl(hoodPose);
            shootMotor.setControl(shootVelocity);
            publishTelemetry();
            return;
        }

        if (aimTurret.get()) {
            Pose2d currPose = pose.get();

            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (currPose.getX() <= turretConstants.blueHubMaxX) {
                    double xError = Math.abs(field.blueHub.getX() - currPose.getX());
                    double yError = Math.abs(field.blueHub.getY() - currPose.getY());
                    double hubDegrees = normalize180(Math.toDegrees(Math.atan(yError/xError)));
                    
                    aimTurret(hubDegrees - normalize180(currPose.getRotation().getDegrees()));
                    aimHood(Math.pow(yError, 2) + Math.pow(xError, 2));
                } else {
                    hoodZero();
                }
            } else {
                if (currPose.getX() >= turretConstants.redHubMinX) {
                    double xError = Math.abs(field.redHub.getX() - currPose.getX());
                    double yError = Math.abs(field.redHub.getY() - currPose.getY());
                    double hubDegrees = normalize180(Math.toDegrees(Math.atan(yError/xError)));
                    
                    aimTurret(hubDegrees - normalize180(currPose.getRotation().getDegrees()));
                    aimHood(Math.pow(yError, 2) + Math.pow(xError, 2));
                } else {
                    hoodZero();
                }
            }

            spinMotor.setControl(spinPose);
            hoodMotor.setControl(hoodPose);
            shootMotor.setControl(shootVelocity);
        } else {
            hoodZero();
        }

        publishTelemetry();
    }

    private void publishTelemetry() {
        BaseStatusSignal.refreshAll(
            spinPositionSignal,
            spinClosedLoopOutputSignal,
            spinMotorVoltageSignal
        );
        double motorRotations = spinPositionSignal.getValueAsDouble();
        double turretDegrees = normalize180((motorRotations / turretConstants.spinOverrallRatio) * 360.0);

        SmartDashboard.putNumber(turretTelemetryConstants.angleDegKey, turretDegrees);
        SmartDashboard.putNumber(turretTelemetryConstants.spinSetpointRotKey, spinPose.Position);
        SmartDashboard.putNumber(turretTelemetryConstants.spinClosedLoopOutputKey, spinClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(turretTelemetryConstants.spinMotorVoltsKey, spinMotorVoltageSignal.getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if (spinSim == null || spinSimState == null) {
            return;
        }

        spinSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        spinSim.setInputVoltage(spinSimState.getMotorVoltage());
        spinSim.update(turretSimConstants.loopPeriodSeconds);

        double turretRotations = spinSim.getAngularPositionRotations();
        double turretRps = spinSim.getAngularVelocityRPM() / 60.0;
        double rotorRotations = turretRotations * turretConstants.spinOverrallRatio;
        double rotorRps = turretRps * turretConstants.spinOverrallRatio;

        spinSimState.setRawRotorPosition(rotorRotations);
        spinSimState.setRotorVelocity(rotorRps);
    }

    private static double degreesToMotorRotations(double turretDegrees) {
        return (turretDegrees / 360.0) * turretConstants.spinOverrallRatio;
    }

    private static double normalize180(double degrees) {
        degrees %= 360.0;        // wrap within -360..360
        if (degrees > 180.0) {
            degrees -= 360.0;    // move into -180..180
        } else if (degrees < -180.0) {
            degrees += 360.0;    // move into -180..180
        }
        return degrees;
    }
}
