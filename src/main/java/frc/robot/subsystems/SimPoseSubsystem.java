package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimPoseSubsystem extends SubsystemBase {
    private static final double kStepMeters = 0.1;
    private static final double kStepRadians = Math.toRadians(5.0);

    private final CommandSwerveDrivetrain drivetrain;
    private final Joystick keyboard = new Joystick(1);

    public SimPoseSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        int pov = keyboard.getPOV();
        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;

        switch (pov) {
            case 0 -> dx = kStepMeters; // Up arrow
            case 180 -> dx = -kStepMeters; // Down arrow
            case 270 -> dy = kStepMeters; // Left arrow (field left)
            case 90 -> dy = -kStepMeters; // Right arrow (field right)
            default -> { }
        }

        if (keyboard.getRawButton(5)) {
            dtheta = kStepRadians;
        } else if (keyboard.getRawButton(6)) {
            dtheta = -kStepRadians;
        }

        if (dx == 0.0 && dy == 0.0 && dtheta == 0.0) {
            return;
        }

        Pose2d pose = drivetrain.getState().Pose;
        Pose2d next = new Pose2d(
            pose.getX() + dx,
            pose.getY() + dy,
            pose.getRotation().plus(new Rotation2d(dtheta))
        );
        drivetrain.resetPose(next);
    }
}
