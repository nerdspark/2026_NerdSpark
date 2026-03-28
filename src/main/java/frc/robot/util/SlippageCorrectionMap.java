package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SlippageCorrectionConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Builds a slippage efficiency curve from (commandedRps, observedDistanceMeters) pairs
 * at a fixed characterization hood angle and target height, then uses it to correct
 * the IK solver's theoretical motor RPS output.
 *
 * <h3>Ballistics inversion</h3>
 * With launch angle θ and Δh = charTargetHeight − muzzleHeight:
 * <pre>
 *   V = sqrt( g * D² / (2 * cos²θ * (D*tanθ − Δh)) )
 *   efficiency   = V_actual / (commandedRps * 2π * wheelRadius)
 *   correctedRps = theoreticalRps / efficiency
 * </pre>
 */
public class SlippageCorrectionMap {

    private static final double G = 9.80665;
    private static final double TWO_PI = 2.0 * Math.PI;

    private static boolean refreshed = false;

    private final InterpolatingTreeMap<Double, Double> efficiencyMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SlippageCorrectionMap::lerp);
    private boolean hasData = false;

    // public SlippageCorrectionMap() {
    //     SmartDashboard.setDefaultBoolean(
    //             SlippageCorrectionConstants.enableKey,
    //             SlippageCorrectionConstants.defaultEnable);
    //     SmartDashboard.setDefaultNumber(
    //             SlippageCorrectionConstants.efficiencyOffsetKey,
    //             SlippageCorrectionConstants.defaultEfficiencyOffset);
    //     SmartDashboard.setDefaultNumber(
    //             SlippageCorrectionConstants.efficiencyScaleKey,
    //             SlippageCorrectionConstants.defaultEfficiencyScale);
    //     SmartDashboard.setDefaultNumber(
    //             SlippageCorrectionConstants.charHoodDegKey,
    //             SlippageCorrectionConstants.defaultCharHoodDeg);
    //     SmartDashboard.setDefaultNumber(
    //             SlippageCorrectionConstants.charTargetHeightMetersKey,
    //             SlippageCorrectionConstants.defaultCharTargetHeightMeters);
    //     SmartDashboard.setDefaultNumberArray(
    //             SlippageCorrectionConstants.commandedRpsPointsKey,
    //             SlippageCorrectionConstants.defaultCommandedRpsPoints);
    //     SmartDashboard.setDefaultNumberArray(
    //             SlippageCorrectionConstants.observedDistancePointsKey,
    //             SlippageCorrectionConstants.defaultObservedDistanceMeters);
    //     SmartDashboard.setDefaultNumber(
    //             SlippageCorrectionConstants.sotmPredictionSecondsKey,
    //             SlippageCorrectionConstants.defaultSotmPredictionSeconds);
    // }

    /**
     * Returns the actual ball exit speed (m/s) for a given commanded motor RPS,
     * interpolated from the empirical characterization curve.
     *
     * <p>Formula: {@code actualVelocity = efficiency(rps) * rps * 2π * wheelRadius}
     */
    public double actualVelocityMps(double motorRps) {
        double eff = efficiencyAt(motorRps);
        return eff * motorRps * TWO_PI * TurretConstants.shooterWheelRadius;
    }

    /**
     * Returns the slippage efficiency (0–1] at {@code motorRps}.
     * Returns 1.0 when disabled or no data is available.
     */
    public double efficiencyAt(double motorRps) {
        if (!refreshed) {
            refresh();
            refreshed = true;
        }
        boolean enabled = SlippageCorrectionConstants.defaultEnable;
        if (!enabled || !hasData) {
            return 1.0;
        }
        double scale = SlippageCorrectionConstants.defaultEfficiencyScale;
        Double factor = efficiencyMap.get(motorRps);
        return (factor != null && factor > 0.0) ? factor * scale : 1.0;
    }

    /**
     * Converts a theoretical motor RPS (from IK with efficiency=1) to the corrected
     * RPS that must actually be commanded.
     *
     * <p>{@code correctedRps = theoreticalRps / efficiency(theoreticalRps)}
     */
    public double correctedMotorRps(double theoreticalRps) {
        double eff = efficiencyAt(theoreticalRps);
        if (eff <= 0.0) return theoreticalRps;
        double offset =
            SlippageCorrectionConstants.defaultEfficiencyOffset
        ;
        return (theoreticalRps / eff) + offset;
    }

    private void refresh() {
        double[] rpsPoints =
                SlippageCorrectionConstants.defaultCommandedRpsPoints;
        double[] distPoints =
                SlippageCorrectionConstants.defaultObservedDistanceMeters;

        if (rpsPoints.length == 0 && distPoints.length == 0) {
            efficiencyMap.clear();
            hasData = false;
            return;
        }
        if (rpsPoints.length != distPoints.length) {
            return;
        }

        double charHoodDeg =
                SlippageCorrectionConstants.defaultCharHoodDeg;
        double charTargetHeight =
                SlippageCorrectionConstants.defaultCharTargetHeightMeters;

        double charHoodRad = Math.toRadians(charHoodDeg);
        // Negative for floor shots (ball drops), positive for hub shots (ball rises)
        double deltaH = charTargetHeight - TurretConstants.shooterMuzzleHeightMeters;
        double wheelCircumference = TWO_PI * TurretConstants.shooterWheelRadius;

        efficiencyMap.clear();
        boolean inserted = false;
        for (int i = 0; i < rpsPoints.length; i++) {
            double rps = rpsPoints[i];
            double dist = distPoints[i];
            if (!Double.isFinite(rps) || !Double.isFinite(dist) || rps <= 0.0 || dist <= 0.0) {
                continue;
            }
            double vActual = backCalcExitSpeed(dist, charHoodRad, deltaH);
            if (!Double.isFinite(vActual) || vActual <= 0.0) {
                continue;
            }
            double vTheoretical = rps * wheelCircumference;
            double efficiency = vActual / vTheoretical;
            if (efficiency <= 0.0 || efficiency > 1.5) {
                continue;
            }
            efficiencyMap.put(rps, efficiency);
            inserted = true;
        }
        hasData = inserted;
    }

    /**
     * Back-calculates ball exit speed from observed landing distance, launch angle θ,
     * and height difference Δh = charTargetHeight - muzzleHeight.
     *
     * <pre>V = sqrt( g * D² / (2 * cos²θ * (D*tanθ − Δh)) )</pre>
     */
    private static double backCalcExitSpeed(double distanceMeters, double launchRad, double deltaHeightMeters) {
        double cosTheta = Math.cos(launchRad);
        double tanTheta = Math.tan(launchRad);
        double denominator = 2.0 * cosTheta * cosTheta * (distanceMeters * tanTheta - deltaHeightMeters);
        if (denominator <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt(G * distanceMeters * distanceMeters / denominator);
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}
