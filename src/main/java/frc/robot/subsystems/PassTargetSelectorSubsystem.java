package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.turretTargetConstants;
import frc.robot.sim.PassTargetPicker;

public class PassTargetSelectorSubsystem extends SubsystemBase {
    private final DoubleArraySubscriber fieldClickSub;
    private final PassTargetPicker picker;
    private boolean lastEnabled = false;

    public PassTargetSelectorSubsystem() {
        NetworkTable smart = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        fieldClickSub = smart.getDoubleArrayTopic(PassTargetConstants.fieldClickKey).subscribe(new double[] {});
        picker = new PassTargetPicker();
        picker.start();

        SmartDashboard.setDefaultBoolean(PassTargetConstants.enableKey, PassTargetConstants.defaultEnable);
        SmartDashboard.setDefaultNumber(PassTargetConstants.targetXKey, PassTargetConstants.defaultTargetX);
        SmartDashboard.setDefaultNumber(PassTargetConstants.targetYKey, PassTargetConstants.defaultTargetY);
    }

    @Override
    public void periodic() {
        boolean enabled = SmartDashboard.getBoolean(
            PassTargetConstants.enableKey,
            PassTargetConstants.defaultEnable
        );
        if (!enabled) {
            if (lastEnabled) {
                SmartDashboard.putBoolean(turretTargetConstants.enableKey, false);
            }
            lastEnabled = false;
            return;
        }
        lastEnabled = true;

        double targetX = SmartDashboard.getNumber(
            PassTargetConstants.targetXKey,
            PassTargetConstants.defaultTargetX
        );
        double targetY = SmartDashboard.getNumber(
            PassTargetConstants.targetYKey,
            PassTargetConstants.defaultTargetY
        );

        double[] click = fieldClickSub.get();
        if (click.length >= 2) {
            targetX = click[0];
            targetY = click[1];
            SmartDashboard.putNumber(PassTargetConstants.targetXKey, targetX);
            SmartDashboard.putNumber(PassTargetConstants.targetYKey, targetY);
        }

        SmartDashboard.putBoolean(turretTargetConstants.enableKey, true);
        SmartDashboard.putNumber(turretTargetConstants.targetXKey, targetX);
        SmartDashboard.putNumber(turretTargetConstants.targetYKey, targetY);
    }
}
