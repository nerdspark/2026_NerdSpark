package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FuelSim;

public class SimFuelSubsystem extends SubsystemBase {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;

    public SimFuelSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        if (RobotBase.isSimulation()) {
            configureFuelSim();
        }
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.spawnStartingFuel();
        instance.registerRobot(
            Units.inchesToMeters(25),
            Units.inchesToMeters(29),
            Units.inchesToMeters(25),
            poseSupplier,
            speedsSupplier
        );
        instance.registerIntake(
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            -Units.inchesToMeters(26),
            Units.inchesToMeters(26),
            () -> true
        );

        instance.start();
        SmartDashboard.putData(
            Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true)
        );
    }

    public Translation3d launchVel(double vel, double angle) {
        Translation3d robotTranslation = new Translation3d(
            poseSupplier.get().getX(),
            poseSupplier.get().getY(),
            Units.inchesToMeters(30)
        );
        Pose3d robot = new Pose3d(
            robotTranslation,
            new Rotation3d(poseSupplier.get().getRotation())
        );
        ChassisSpeeds fieldSpeeds = speedsSupplier.get();

        double horizontalVel = Math.cos(Units.degreesToRadians(angle)) * vel;
        double verticalVel = Math.sin(Units.degreesToRadians(angle)) * vel;
        double xVel = horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel = horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }
}
