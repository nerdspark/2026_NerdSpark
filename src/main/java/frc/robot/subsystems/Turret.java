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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.field;
import frc.robot.Constants.turretConstants;

public class Turret implements Subsystem {
    private TalonFX spinMotor, hoodMotor, shootMotor;

    private VelocityVoltage shootVelocity = new VelocityVoltage(0);
    private PositionVoltage hoodPose = new PositionVoltage(0);
    private PositionVoltage spinPose = new PositionVoltage(0);

    private Supplier<Pose2d> pose;
    private Supplier<DriverStation.Alliance> alliance;

    public Turret(Supplier<Pose2d> robotPose, Supplier<DriverStation.Alliance> driverAlliance) {
        pose = robotPose;
        alliance = driverAlliance;

        spinMotor = new TalonFX(0);
        hoodMotor = new TalonFX(0);
        shootMotor = new TalonFX(0);

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

        spinMotor.getConfigurator().apply(spinConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        shootMotor.getConfigurator().apply(shootConfig);
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
    * @param neededAngle the field-relative target angle minus the chassis heading in degrees
    */
    private void aimTurret(double neededAngle) {
        double motorRotations = spinMotor.getPosition().getValueAsDouble();
        double turretDegrees = (motorRotations / turretConstants.spinOverrallRatio) * 360;
        normalize180(turretDegrees);

        boolean turretOK;
        if (neededAngle > 90) {
            neededAngle = 90;
            turretOK = false;
        } else if (neededAngle < -90) {
            neededAngle = -90;
            turretOK = false;
        } else {
            turretOK = true;
        }
        SmartDashboard.putBoolean("Turret Within Limit?", turretOK);

        double error = neededAngle - turretDegrees;
        spinPose.Position = error / 4.0;
    }

    /**
     * Aims the chassis and the turret
     * 
     * @param neededAngle the field-relative target angle
     * @param chassisAngle the current heading of the chassis
     * @return the target chassis rotation in degrees
     */
    public double aimChassis(double neededAngle, double chassisAngle) {
        double turretRel = neededAngle - chassisAngle;
        
        // aimTurret(turretRel);

        normalize180(turretRel);

        double chassisRotation = 0.0;

        // If target is outside turret limits, compute chassis rotation
        if (turretRel > 90.0) {
            chassisRotation = turretRel - 85.0;
        } else if (turretRel < -90.0) {
            chassisRotation = turretRel + 85.0;
        }

        return chassisRotation + chassisAngle;
    }

    @Override
    public void periodic() {
        Pose2d currPose = pose.get();

        if (alliance.get() == DriverStation.Alliance.Blue) {
            if (currPose.getX() <= 4.5) {
                double xError = Math.abs(field.blueHub.getX() - currPose.getX());
                double yError = Math.abs(field.blueHub.getY() - currPose.getY());
                double hubDegrees = normalize180(Math.toDegrees(Math.atan(yError/xError)));
                
                aimTurret(hubDegrees - normalize180(currPose.getRotation().getDegrees()));
                aimHood(Math.pow(yError, 2) + Math.pow(xError, 2));
            } else {
                hoodZero();
            }
        } else {
            if (currPose.getX() >= 12) {
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