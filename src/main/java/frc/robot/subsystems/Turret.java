package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.TurretUtil.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.field;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.turretSimConstants;
import frc.robot.Constants.turretTelemetryConstants;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor, shootMotor;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private PositionVoltage hoodPose = new PositionVoltage(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<DriverStation.Alliance> alliance;
    private Supplier<Boolean> aimTurret;
    private boolean manualOverride = true;
    private double manualSetpointDeg = 0.0;
    private double lastSpinKp = Double.NaN;
    private double lastSpinKi = Double.NaN;
    private double lastSpinKd = Double.NaN;
    private DCMotorSim spinSim;
    private TalonFXSimState spinSimState;
    private StatusSignal<Angle> spinPositionSignal;
    private StatusSignal<Double> spinClosedLoopOutputSignal;
    private StatusSignal<Voltage> spinMotorVoltageSignal;

    private boolean shortPathCrossesWrap;
    private boolean pathLatched = false;
    private double turretAngle;
    private double degrees = 0;
    Supplier<Boolean> joy;

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<DriverStation.Alliance> driverAlliance, Supplier<Boolean> aimTurret, Supplier<Boolean> joystick) {
        pose = robotPose;
        speed = speeds;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;
        joy = joystick;

        canivore = new CANBus("canivore1");
        
        spinMotor = new TalonFX(TurretConstants.spinMotorId, canivore);
        hoodMotor = new TalonFX(TurretConstants.hoodMotorId, canivore);
        shootMotor = new TalonFX(TurretConstants.shootMotorId, canivore);

        spinCancoder1 = new CANcoder(26, canivore);
        spinCancoder2 = new CANcoder(27, canivore);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConstants.spinKp)
                .withKI(TurretConstants.spinKi)
                .withKD(TurretConstants.spinKd)
                .withKS(0)
                .withKV(0)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConstants.spinStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(40)
                .withMotionMagicAcceleration(9000)
            )
        ;
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                .withKP(TurretConstants.hoodKp)
                .withKI(TurretConstants.hoodKi)
                .withKD(TurretConstants.hoodKd)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConstants.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;
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
            )
        ;
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(-0.241455078125)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            )
        ;
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withMagnetOffset(-0.10009765625)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            )
        ;

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        shootMotor.getConfigurator().apply(shootConfig);
        spinPositionSignal = spinMotor.getPosition();
        spinClosedLoopOutputSignal = spinMotor.getClosedLoopOutput();
        spinMotorVoltageSignal = spinMotor.getMotorVoltage();

        if (RobotBase.isSimulation()) {
            DCMotor motorModel = DCMotor.getKrakenX44Foc(turretSimConstants.motorCount);
            double gearing = 1.0 / TurretConstants.spinRatio;
            spinSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, turretSimConstants.turretJ, gearing),
                motorModel
            );
            spinSimState = spinMotor.getSimState();
        }

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);

        spinMotor.setPosition(0);
    }

    /** 
     * Aims the hood of the turret and sets the wheel speed
     * If hood is not tight then look into chassis velocity based correction for hood
     * 
     * @param distance the distance to the center of the hub
    */
    private void aimHood(double distance) {
        double[] map = TurretConstants.shooterMap.get(distance);

        hoodPose.Position = map[0];

        double robotHeading = pose.get().getRotation().getRadians();
        double shooterFOA = robotHeading + TurretConstants.turretOffset + turretAngle;
        ChassisSpeeds robotFOS = ChassisSpeeds.fromRobotRelativeSpeeds(speed.get(), pose.get().getRotation());
        double robotSpeed = Math.hypot(robotFOS.vxMetersPerSecond, robotFOS.vyMetersPerSecond);
        double robotVelAngle = Math.atan2(robotFOS.vyMetersPerSecond, robotFOS.vxMetersPerSecond);
        double vParallel = robotSpeed * Math.cos(robotVelAngle - shooterFOA);
        double deltaMotorRPS = (vParallel / (2.0 * Math.PI * TurretConstants.shooterWheelRadius)) * TurretConstants.shooterRatio;

        shootVelocity.Velocity = map[1] - deltaMotorRPS;
    }

    // When we are out of shooting range stop wheels and send hood to stow
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
        double theta1 = spinCancoder1.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        double theta2 = spinCancoder2.getAbsolutePosition().getValueAsDouble() * TWO_PI;
        
        turretAngle = floorMod(
            (TWO_PI/TurretConstants.spinTeeth) * (
                floorMod(
                    -(TurretConstants.spinCancoder1Teeth/TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth
                ) + TurretConstants.spinCancoder1Teeth * floorMod(
                    modInverse(
                        TurretConstants.spinCancoder1Teeth, TurretConstants.spinCancoder2Teeth
                    ) * (
                        floorMod(-(TurretConstants.spinCancoder2Teeth/TWO_PI) * theta2, TurretConstants.spinCancoder2Teeth) 
                        - floorMod(-(TurretConstants.spinCancoder1Teeth/TWO_PI) * theta1, TurretConstants.spinCancoder1Teeth)
                    ), 
                    TurretConstants.spinCancoder2Teeth
                )
            ), 
            TWO_PI
        );

        // Normalize to -pi to pi
        turretAngle = normalizeRadians(turretAngle);
        SmartDashboard.putNumber("Turret/AngleActualRad", turretAngle);
        SmartDashboard.putNumber("Turret/AngleActualDeg", Math.toDegrees(turretAngle));

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

        if (!pathLatched) {
            // Does the short path cross the wrap?
            shortPathCrossesWrap =
                Math.abs(turretAngle) > Math.PI / 2.0 &&
                Math.abs(neededAngle) > Math.PI / 2.0 &&
                Math.signum(turretAngle) != Math.signum(neededAngle);

            pathLatched = true;
        }

        // Select legal error
        double chosenError = shortPathCrossesWrap ? error : shortError;
        SmartDashboard.putNumber("Turret/ErrorRad", chosenError);
        SmartDashboard.putNumber("Turret/ErrorDeg", Math.toDegrees(chosenError));

        if (pathLatched && Math.abs(chosenError) < Math.toRadians(90)) {
            pathLatched = false;
        }

        // Command motor
        double motorDelta = (chosenError / TWO_PI) * TurretConstants.spinRatio;
        spinPose.Position = motorDelta + spinMotor.getPosition().getValueAsDouble();
    }

    public void setManualSetpointDegrees(double degrees) {
        manualSetpointDeg = normalize180(degrees);
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
        if (joy.get()) {
            degrees += 5;
            setManualSetpointDegrees(degrees);
        }

        if (manualOverride) {
            //double clamped = Math.max(TurretConstants.turretMinDegrees,
                // Math.min(TurretConstants.turretMaxDegrees, manualSetpointDeg));
            // spinPose.Position = degreesToMotorRotations(clamped);
            //hoodZero();
            SmartDashboard.putNumber("Turret/SetpointDeg", manualSetpointDeg);
            SmartDashboard.putNumber("Turret/SetpointRad", Math.toRadians(manualSetpointDeg));
            aimTurret(Math.toRadians(manualSetpointDeg));
            spinMotor.setControl(spinPose);
            // hoodMotor.setControl(hoodPose);
            // shootMotor.setControl(shootVelocity);
            publishTelemetry();
            return;
        }

        if (aimTurret.get()) {
            Pose2d currPose = pose.get();

            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (currPose.getX() <= TurretConstants.blueHubMaxX) {
                    double xError = field.blueHub.getX() - currPose.getX();
                    double yError = field.blueHub.getY() - currPose.getY();
                    double hubDegrees = Math.atan2(yError, xError);
                    
                    aimTurret(normalizeRadians(hubDegrees - currPose.getRotation().getRadians()));
                    aimHood(Math.hypot(yError, xError));
                } else {
                    // Add passing here if needed
                    
                    hoodZero();
                }
            } else {
                if (currPose.getX() >= TurretConstants.redHubMinX) {
                    double xError = field.redHub.getX() - currPose.getX();
                    double yError = field.redHub.getY() - currPose.getY();
                    double hubDegrees = Math.atan2(yError, xError);
                    
                    aimTurret(normalizeRadians(hubDegrees - currPose.getRotation().getRadians()));
                    aimHood(Math.hypot(yError, xError));
                } else {
                    // Adding passing here if needed
                    
                    hoodZero();
                }
            }
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
        double turretDegrees = normalize180((motorRotations / TurretConstants.spinRatio) * 360.0);

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
        double rotorRotations = turretRotations * TurretConstants.spinRatio;
        double rotorRps = turretRps * TurretConstants.spinRatio;

        spinSimState.setRawRotorPosition(rotorRotations);
        spinSimState.setRotorVelocity(rotorRps);
    }
}