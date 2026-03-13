package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterParams implements Interpolatable<ShooterParams> {
    public final double hoodPose;
    public final double shooterSpeed;

    public ShooterParams(double hoodPose, double shooterSpeed) {
        this.hoodPose = hoodPose;
        this.shooterSpeed = shooterSpeed;
    }

    @Override
    public ShooterParams interpolate(ShooterParams endValue, double t) {
        return new ShooterParams(
            lerp(hoodPose, endValue.hoodPose, t),
            lerp(shooterSpeed, endValue.shooterSpeed, t)
        );
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
