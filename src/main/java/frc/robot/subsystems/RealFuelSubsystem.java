package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.turretTargetConstants;

public class RealFuelSubsystem extends SubsystemBase {
    private Translation2d target = Field.blueHub;
    private boolean enabled = false;

    public void enableTargeting(boolean enable) {
        enabled = enable;
        SmartDashboard.putBoolean(turretTargetConstants.enableKey, enabled);
    }

    public void setTarget(Translation2d target) {
        if (target == null) {
            return;
        }
        this.target = target;
        SmartDashboard.putNumber(turretTargetConstants.targetXKey, target.getX());
        SmartDashboard.putNumber(turretTargetConstants.targetYKey, target.getY());
    }

    public void setHubTarget(Alliance alliance) {
        setTarget(alliance == Alliance.Red ? Field.redHub : Field.blueHub);
    }

    @Override
    public void periodic() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (!enabled) {
            return;
        }
        setHubTarget(alliance);
    }
}
