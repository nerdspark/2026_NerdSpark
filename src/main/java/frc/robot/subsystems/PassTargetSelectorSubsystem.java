package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.sim.PassTargetPicker;

public class PassTargetSelectorSubsystem extends SubsystemBase {

    private final PassTargetPicker picker;

    // Subscribers — read what PassTargetCursor publishes
    private final BooleanSubscriber enabledSub;
    private final DoubleSubscriber targetXSub;
    private final DoubleSubscriber targetYSub;

    // Publishers — write to turret target topics
    private final BooleanPublisher turretEnabledPub;
    private final DoublePublisher turretXPub;
    private final DoublePublisher turretYPub;

    private boolean lastEnabled = false;

    public PassTargetSelectorSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

        // Read pass target state written by PassTargetCursor
        enabledSub = table.getBooleanTopic(PassTargetConstants.enableKey)
                          .subscribe(PassTargetConstants.defaultEnable);
        targetXSub = table.getDoubleTopic(PassTargetConstants.targetXKey)
                          .subscribe(PassTargetConstants.defaultTargetX);
        targetYSub = table.getDoubleTopic(PassTargetConstants.targetYKey)
                          .subscribe(PassTargetConstants.defaultTargetY);

        // Write turret target state
        turretEnabledPub = table.getBooleanTopic(turretTargetConstants.enableKey).publish();
        turretXPub       = table.getDoubleTopic(turretTargetConstants.targetXKey).publish();
        turretYPub       = table.getDoubleTopic(turretTargetConstants.targetYKey).publish();

        picker = new PassTargetPicker();
        picker.start();
    }

    @Override
    public void periodic() {
        boolean enabled = enabledSub.get();

        if (!enabled) {
            if (lastEnabled) {
                turretEnabledPub.set(false);
            }
            lastEnabled = false;
            return;
        }

        lastEnabled = true;

        double targetX = targetXSub.get();
        double targetY = targetYSub.get();

        turretEnabledPub.set(true);
        turretXPub.set(targetX);
        turretYPub.set(targetY);
    }
}