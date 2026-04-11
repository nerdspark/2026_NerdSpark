package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PassTargetConstants;

public class PassTargetSelectorSubsystem extends SubsystemBase {
    private final DoubleArraySubscriber fieldClickSub;

    private boolean enabled = PassTargetConstants.defaultEnable;
    private double cachedX = PassTargetConstants.defaultTargetX;
    private double cachedY = PassTargetConstants.defaultTargetY;

    public PassTargetSelectorSubsystem() {
        fieldClickSub = NetworkTableInstance.getDefault().getTable("SmartDashboard")
            .getDoubleArrayTopic(PassTargetConstants.fieldClickKey).subscribe(new double[] {});

        SmartDashboard.setDefaultBoolean(PassTargetConstants.enableKey, PassTargetConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(PassTargetConstants.targetXKey, cachedX);
        SmartDashboard.setDefaultNumber(PassTargetConstants.targetYKey, cachedY);
    }

    @Override
    public void periodic() {
        // Read enable from SmartDashboard (DS side sets this)
        enabled = SmartDashboard.getBoolean(PassTargetConstants.enableKey, enabled);

        if (!enabled) return;

        // If a new click arrived, update cache
        double[] click = fieldClickSub.get();
        if (click.length >= 2) {
            cachedX = click[0];
            cachedY = click[1];
            SmartDashboard.putNumber(PassTargetConstants.targetXKey, cachedX);
            SmartDashboard.putNumber(PassTargetConstants.targetYKey, cachedY);
        }
    }

    /** Whether the pass target override is active. */
    public boolean isEnabled() {
        return enabled;
    }

    /** The currently selected pass target. Only meaningful when isEnabled() is true. */
    public Translation2d getTarget() {
        return new Translation2d(cachedX, cachedY);
    }
}
