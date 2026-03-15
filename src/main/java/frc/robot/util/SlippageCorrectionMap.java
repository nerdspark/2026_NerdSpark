package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SlippageCorrectionConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Builds a slippage efficiency curve from (commandedRps, observedDistanceMeters) pairs
 * at a fixed characterization hood angle, and uses it to correct the IK solver's
 * theoretical motor RPS output.
 *
 * <h3>Protocol</h3>
 * <ol>
 *   <li>Set {@code Slippage/CharHoodDeg} to a fixed hood angle (e.g. 20 deg) and hold
 *       it there via MapTune.</li>
 *   <li>Shoot at RPS steps (e.g. 100, 200, 300, 400). Record where each ball lands.</li>
 *   <li>Enter the RPS steps in {@code Slippage/CommandedRpsPoints} and the landing
 *       distances in {@code Slippage/ObservedDistanceMeters}.</li>
 *   <li>Enable with {@code Slippage/Enable = true}.</li>
 * </ol>
 *
 * <h3>Ballistics inversion — how hood angle is accounted for</h3>
 * With launch angle θ (hood) and height difference Δh = targetHeight - muzzleHeight:
 * <pre>
 *   D     = V * cosθ * t               (horizontal)
 *   Δh    = V * sinθ * t - ½g * t²    (vertical)
 *
 *   solving for V:
 *   V = sqrt( g * D² / (2 * cos²θ * (D*tanθ - Δh)) )
 * </pre>
 * A steeper θ increases tanθ, raising the denominator's bracket and yielding a
 * higher back-calculated V for the same observed distance — which is physically
 * correct, since a steeper ball must have been launched faster to reach the same spot.
 *
 * <h3>Correction applied at runtime</h3>
 * <pre>
 *   efficiency   = V_actual / (commandedRps * 2π * wheelRadius)
 *   correctedRps = theoreticalRps / efficiency
 * </pre>
 */
public class SlippageCorrectionMap {

    private static final double G = 9.80665;
    private static final double TWO_PI = 2.0 * Math.PI;

    private final InterpolatingTreeMap<Double, Double> efficiencyMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SlippageCorrectionMap::lerp);
    private boolean hasData = false;

    public SlippageCorrectionMap() {
        SmartDashboard.setDefaultBoolean(
                SlippageCorrectionConstants.enableKey,
                SlippageCorrectionConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(
                SlippageCorrectionConstants.charHoodDegKey,
                SlippageCorrectionConstants.defaultCharHoodDeg);
        SmartDashboard.setDefaultNumberArray(SlippageCorrectionConstants.commandedRpsPointsKey, new double[] {});
        SmartDashboard.setDefaultNumberArray(SlippageCorrectionConstants.observedDistancePointsKey, new double[] {});
    }

    /**
     * Returns the slippage efficiency (0–1] at {@code motorRps}.
     * Returns 1.0 when disabled or no data is available.
     */
    public double efficiencyAt(double motorRps) {
        refresh();
        boolean enabled = SmartDashboard.getBoolean(
                SlippageCorrectionConstants.enableKey,
                SlippageCorrectionConstants.defaultEnable);
        if (!enabled || !hasData) {
            return 1.0;
        }
        Double factor = efficiencyMap.get(motorRps);
        return (factor != null && factor > 0.0) ? factor : 1.0;
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
        return theoreticalRps / eff;
    }

    private void refresh() {
        double[] rpsPoints = SmartDashboard.getNumberArray(
                SlippageCorrectionConstants.commandedRpsPointsKey, new double[] {});
        double[] distPoints = SmartDashboard.getNumberArray(
                SlippageCorrectionConstants.observedDistancePointsKey, new double[] {});

        if (rpsPoints.length == 0 && distPoints.length == 0) {
            efficiencyMap.clear();
            hasData = false;
            return;
        }
        if (rpsPoints.length != distPoints.length) {
            return;
        }

        double charHoodDeg = SmartDashboard.getNumber(
                SlippageCorrectionConstants.charHoodDegKey,
                SlippageCorrectionConstants.defaultCharHoodDeg);
        double charHoodRad = Math.toRadians(charHoodDeg);
        double deltaH = TurretConstants.targetHeightMeters - TurretConstants.shooterMuzzleHeightMeters;
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
     * Back-calculates ball exit speed from observed landing distance, launch angle,
     * and height difference to the target.
     *
     * <p>Hood angle enters through cos²θ and tanθ — a steeper angle yields a
     * higher back-calculated speed for the same observed distance.
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
