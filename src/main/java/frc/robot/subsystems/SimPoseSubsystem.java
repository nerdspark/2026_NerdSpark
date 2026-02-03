package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimPoseSubsystem extends SubsystemBase {
    private static final double kStepMeters = 0.1;
    private static final double kStepRadians = Math.toRadians(5.0);
    private static final double kMaxSpeedMetersPerSecond = 2.0;
    private static final double kMaxRotRadPerSecond = Math.toRadians(90.0);
    private static final double kDeadband = 0.05;

    private final CommandSwerveDrivetrain drivetrain;
    private final Joystick keyboard = new Joystick(1);
    private double lastTimestamp = Timer.getFPGATimestamp();

    public SimPoseSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        int pov = keyboard.getPOV();
        double dx = 0.0;
        double dy = 0.0;
        double dtheta = 0.0;

        double xInput = -keyboard.getRawAxis(1);
        double yInput = keyboard.getRawAxis(0);
        if (Math.abs(xInput) < kDeadband) {
            xInput = 0.0;
        }
        if (Math.abs(yInput) < kDeadband) {
            yInput = 0.0;
        }
        dx += xInput * kMaxSpeedMetersPerSecond * dt;
        dy += yInput * kMaxSpeedMetersPerSecond * dt;

        switch (pov) {
            case 0 -> dx = kStepMeters; // Up arrow
            case 180 -> dx = -kStepMeters; // Down arrow
            case 270 -> dy = kStepMeters; // Left arrow (field left)
            case 90 -> dy = -kStepMeters; // Right arrow (field right)
            default -> { }
        }

        if (keyboard.getRawButton(5)) {
            dtheta = kMaxRotRadPerSecond * dt;
        } else if (keyboard.getRawButton(6)) {
            dtheta = -kMaxRotRadPerSecond * dt;
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
