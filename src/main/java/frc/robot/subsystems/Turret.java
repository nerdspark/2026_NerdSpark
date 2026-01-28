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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConfig;
import frc.robot.Constants.Field;
import frc.robot.Constants.TurretTelemetryConstants;

public class Turret extends SubsystemBase {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    private CANBus canivore;
    private TalonFX spinMotor, hoodMotor1, hoodMotor2, shootMotor1, shootMotor2;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private PositionVoltage hoodPose = new PositionVoltage(0);
    private MotionMagicVoltage spinPose = new MotionMagicVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<DriverStation.Alliance> alliance;
    private Supplier<Boolean> aimTurret;
    private Supplier<Boolean> driveAndAim;
    
    private StatusSignal<Angle> spinPositionSignal;
    private StatusSignal<Double> spinClosedLoopOutputSignal;
    private StatusSignal<Voltage> spinMotorVoltageSignal;

    private boolean shortPathCrossesWrap;
    private boolean pathLatched = false;
    private double turretAngle;

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<DriverStation.Alliance> driverAlliance, 
        Supplier<Boolean> aimTurret, Supplier<Boolean> driveAndAim) {
        pose = robotPose;
        speed = speeds;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;
        this.driveAndAim = driveAndAim;

        canivore = new CANBus(TurretConfig.CANbus);
        
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
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConfig.spinVelocity)
                .withMotionMagicAcceleration(TurretConfig.spinAccel)
            )
        ;
        TalonFXConfiguration hoodConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
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
            )
        ;
        TalonFXConfiguration hoodConfig2 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.hoodKp2)
                .withKI(TurretConfig.hoodKi2)
                .withKD(TurretConfig.hoodKd2)
                .withKS(TurretConfig.hoodKs2)
                .withKV(TurretConfig.hoodKv2)
                .withKA(TurretConfig.hoodKa2)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.hoodStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig1 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.shootKp1)
                .withKI(TurretConfig.shootKi1)
                .withKD(TurretConfig.shootKd1)
                .withKS(TurretConfig.shootKs1)
                .withKV(TurretConfig.shootKv1)
                .withKA(TurretConfig.shootKa1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig2 = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSlot0(new Slot0Configs()
                .withKP(TurretConfig.shootKp2)
                .withKI(TurretConfig.shootKi2)
                .withKD(TurretConfig.shootKd2)
                .withKS(TurretConfig.shootKs2)
                .withKV(TurretConfig.shootKv2)
                .withKA(TurretConfig.shootKa2)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(TurretConfig.shootStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true)
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

        spinPositionSignal = spinMotor.getPosition();
        spinClosedLoopOutputSignal = spinMotor.getClosedLoopOutput();
        spinMotorVoltageSignal = spinMotor.getMotorVoltage();

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);

        hoodMotor2.setControl(new Follower(TurretConfig.hoodMotor1Id, MotorAlignmentValue.Opposed));
        shootMotor2.setControl(new Follower(TurretConfig.shootMotor1Id, MotorAlignmentValue.Opposed));
    }
    
    /** 
     * Aims the hood of the turret and spins wheels based on shooter map and chassis speeds
     * If hood is not tight then look into chassis velocity based correction for hood
     * 
     * @param distance the distance to the center of the hub
    */
    private void aimOnFly(double distance) {
        double[] map = TurretConstants.map.get(distance);

        hoodPose.Position = map[0];

        double robotHeading = pose.get().getRotation().getRadians();
        double shooterFOA = robotHeading + turretAngle;
        ChassisSpeeds robotFOS = ChassisSpeeds.fromRobotRelativeSpeeds(speed.get(), pose.get().getRotation());
        double robotSpeed = Math.hypot(robotFOS.vxMetersPerSecond, robotFOS.vyMetersPerSecond);
        double robotVelAngle = Math.atan2(robotFOS.vyMetersPerSecond, robotFOS.vxMetersPerSecond);
        double vParallel = robotSpeed * Math.cos(robotVelAngle - shooterFOA);
        double deltaMotorRPS = vParallel / (2.0 * Math.PI * TurretConstants.shooterWheelRadius);

        shootVelocity.Velocity = map[1] - deltaMotorRPS;
    }

    /** 
     * Aims the hood of the turret and spins wheels based on drive to pose position
     * 
     * @param index the index of the drive to pose position
    */
    public void aimAndDrive(int index) {
        hoodPose.Position = TurretConstants.hoodMap[index];
        shootVelocity.Velocity = TurretConstants.wheelMap[index];
    }

    /**
     * When we are out of shooting range stop wheels and send hood to stow
     */
    private void hoodWheelsZero() {
        hoodPose.Position = TurretConstants.hoodStowPose;
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
        SmartDashboard.putNumber(TurretTelemetryConstants.angleRadKey, turretAngle);
        SmartDashboard.putNumber(TurretTelemetryConstants.angleDegKey, Math.toDegrees(turretAngle));

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
        SmartDashboard.putNumber(TurretTelemetryConstants.errorRadKey, chosenError);
        SmartDashboard.putNumber(TurretTelemetryConstants.errorDegKey, Math.toDegrees(chosenError));

        if (pathLatched && Math.abs(chosenError) < Math.toRadians(90)) {
            pathLatched = false;
        }

        // Command motor
        double motorDelta = (chosenError / TWO_PI) * TurretConstants.spinRatio;
        spinPose.Position = motorDelta + spinMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (aimTurret.get() && !driveAndAim.get()) {
            Pose2d currPose = pose.get();

            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (currPose.getX() <= Field.blueHubMaxX) {
                    double xError = Field.blueHub.getX() - currPose.getX();
                    double yError = Field.blueHub.getY() - currPose.getY();
                    double hubDegrees = Math.atan2(yError, xError);
                    
                    aimTurret(normalizeRadians(hubDegrees - currPose.getRotation().getRadians()));
                    aimOnFly(Math.hypot(yError, xError));
                } else {
                    // Add passing here if needed
                    
                    hoodWheelsZero();
                }
            } else {
                if (currPose.getX() >= Field.redHubMinX) {
                    double xError = Field.redHub.getX() - currPose.getX();
                    double yError = Field.redHub.getY() - currPose.getY();
                    double hubDegrees = Math.atan2(yError, xError);
                    
                    aimTurret(normalizeRadians(hubDegrees - currPose.getRotation().getRadians()));
                    aimOnFly(Math.hypot(yError, xError));
                } else {
                    // Adding passing here if needed
                    
                    hoodWheelsZero();
                }
            }
        } else if (!driveAndAim.get()) {
            hoodWheelsZero();
        }

        spinMotor.setControl(spinPose);
        hoodMotor1.setControl(hoodPose);
        shootMotor1.setControl(shootVelocity);

        publishTelemetry();
    }

    private void publishTelemetry() {
        BaseStatusSignal.refreshAll(
            spinPositionSignal,
            spinClosedLoopOutputSignal,
            spinMotorVoltageSignal
        );

        SmartDashboard.putNumber(TurretTelemetryConstants.spinAngleDegKey, spinPositionSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.spinClosedLoopOutputKey, spinClosedLoopOutputSignal.getValueAsDouble());
        SmartDashboard.putNumber(TurretTelemetryConstants.spinMotorVoltsKey, spinMotorVoltageSignal.getValueAsDouble());
    }
}