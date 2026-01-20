package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.field;
import frc.robot.Constants.TurretConstants;

public class Turret implements Subsystem {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    private TalonFX spinMotor, hoodMotor, shootMotor;
    private CANcoder spinCancoder1, spinCancoder2;

    private VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private PositionVoltage hoodPose = new PositionVoltage(0);
    private PositionVoltage spinPose = new PositionVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<ChassisSpeeds> speed;
    private Supplier<DriverStation.Alliance> alliance;
    private Supplier<Boolean> aimTurret;

    private double turretAngle;

    public Turret(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, Supplier<DriverStation.Alliance> driverAlliance, Supplier<Boolean> aimTurret) {
        pose = robotPose;
        speed = speeds;
        alliance = driverAlliance;
        this.aimTurret = aimTurret;

        spinMotor = new TalonFX(0);
        hoodMotor = new TalonFX(0);
        shootMotor = new TalonFX(0);

        spinCancoder1 = new CANcoder(0);
        spinCancoder2 = new CANcoder(0);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        TalonFXConfiguration shootConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
            )
        ;
        CANcoderConfiguration spinCancoder1Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            )
        ;
        CANcoderConfiguration spinCancoder2Config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            )
        ;

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        shootMotor.getConfigurator().apply(shootConfig);

        spinCancoder1.getConfigurator().apply(spinCancoder1Config);
        spinCancoder2.getConfigurator().apply(spinCancoder2Config);
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
        // Normalize encoders to [0, 1)
        double aN = ((spinCancoder1.getAbsolutePosition().getValueAsDouble() % 1.0) + 1.0) % 1.0;
        double bN = ((spinCancoder2.getAbsolutePosition().getValueAsDouble() % 1.0) + 1.0) % 1.0;

        double bestT = 0.0;
        double bestError = Double.MAX_VALUE;

        // Search over physically possible turret turns
        for (int k = -4; k <= 4; k++) {
            double T = (aN + k) / TurretConstants.spinCancoder1Ratio;

            double bPred = (TurretConstants.spinCancoder2Ratio * T) % 1.0;
            if (bPred < 0) bPred += 1.0;

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
        boolean shortPathCrossesWrap =
            Math.abs(turretAngle) > Math.PI / 2.0 &&
            Math.abs(neededAngle) > Math.PI / 2.0 &&
            Math.signum(turretAngle) != Math.signum(neededAngle);

        // Select legal error
        double chosenError = shortPathCrossesWrap ? error : shortError;

        // Command motor
        spinPose.Position = (chosenError / TWO_PI) * TurretConstants.spinRatio;
    }

    @Override
    public void periodic() {
        if (aimTurret.get()) {
            Pose2d currPose = pose.get();

            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (currPose.getX() <= 4.5) {
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
                if (currPose.getX() >= 12) {
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
        
        spinMotor.setControl(spinPose);
        hoodMotor.setControl(hoodPose);
        shootMotor.setControl(shootVelocity);
    }

    // Normalizes to [-pi, pi]
    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}