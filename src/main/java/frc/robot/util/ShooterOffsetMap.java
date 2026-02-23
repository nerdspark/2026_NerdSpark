package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterOffsetConstants;

public class ShooterOffsetMap {
    public static final class Offsets {
        public static final Offsets ZERO = new Offsets(0.0, 0.0);

        public final double hoodOffsetDeg;
        public final double motorRpsOffset;

        private Offsets(double hoodOffsetDeg, double motorRpsOffset) {
            this.hoodOffsetDeg = hoodOffsetDeg;
            this.motorRpsOffset = motorRpsOffset;
        }
    }

    private final InterpolatingTreeMap<Double, Double> hoodOffsets =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterOffsetMap::lerp);
    private final InterpolatingTreeMap<Double, Double> motorOffsets =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterOffsetMap::lerp);
    private boolean hasData = false;

    public ShooterOffsetMap() {
        SmartDashboard.setDefaultBoolean(
            ShooterOffsetConstants.enableKey,
            ShooterOffsetConstants.defaultEnable
        );
        SmartDashboard.setDefaultNumberArray(ShooterOffsetConstants.distancesKey, new double[] {});
        SmartDashboard.setDefaultNumberArray(ShooterOffsetConstants.hoodOffsetDegKey, new double[] {});
        SmartDashboard.setDefaultNumberArray(ShooterOffsetConstants.motorRpsOffsetKey, new double[] {});
    }

    public Offsets sample(double distanceMeters) {
        refresh();
        boolean enabled = SmartDashboard.getBoolean(
            ShooterOffsetConstants.enableKey,
            ShooterOffsetConstants.defaultEnable
        );
        if (!enabled || !hasData) {
            return Offsets.ZERO;
        }
        Double hood = hoodOffsets.get(distanceMeters);
        Double motor = motorOffsets.get(distanceMeters);
        if (hood == null || motor == null) {
            return Offsets.ZERO;
        }
        return new Offsets(hood, motor);
    }

    private void refresh() {
        double[] distances = SmartDashboard.getNumberArray(
            ShooterOffsetConstants.distancesKey,
            new double[] {}
        );
        double[] hood = SmartDashboard.getNumberArray(
            ShooterOffsetConstants.hoodOffsetDegKey,
            new double[] {}
        );
        double[] motor = SmartDashboard.getNumberArray(
            ShooterOffsetConstants.motorRpsOffsetKey,
            new double[] {}
        );

        if (distances.length == 0 && hood.length == 0 && motor.length == 0) {
            hoodOffsets.clear();
            motorOffsets.clear();
            hasData = false;
            return;
        }
        if (distances.length != hood.length || distances.length != motor.length) {
            return;
        }

        hoodOffsets.clear();
        motorOffsets.clear();
        boolean inserted = false;
        for (int i = 0; i < distances.length; i++) {
            double d = distances[i];
            double h = hood[i];
            double m = motor[i];
            if (!Double.isFinite(d) || !Double.isFinite(h) || !Double.isFinite(m)) {
                continue;
            }
            hoodOffsets.put(d, h);
            motorOffsets.put(d, m);
            inserted = true;
        }
        hasData = inserted;
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}
