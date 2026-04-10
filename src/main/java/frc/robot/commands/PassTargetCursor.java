package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.PassTargetSelectorSubsystem;

public class PassTargetCursor extends Command {

    private double currentSpeed;

    // private static final double MAX_SPEED_M_PER_S = true ? 20 : 5;
    private static final double DEADBAND = 0.10;
    private static final double SLOW_SPEED = 1.0;
    private static final double FAST_SPEED = 10.0;

    private final CommandXboxController controller;
    private final NetworkTable table;

    private final double loopPeriodS;

    // Publishers are declared as fields but opened/closed in initialize()/end()
    // per WPILib lifecycle guidance (resources should be opened per-scheduling).
    private DoublePublisher cursorXPub;
    private DoublePublisher cursorYPub;
    private DoublePublisher commitXPub;
    private DoublePublisher commitYPub;

    private double cursorX;
    private double cursorY;

    private boolean aPressedLast;

    public PassTargetCursor(CommandXboxController controller, PassTargetSelectorSubsystem passTargetSelector) {
        this(controller, TimedRobot.kDefaultPeriod, passTargetSelector);

        currentSpeed = SLOW_SPEED;
    }

    public PassTargetCursor(CommandXboxController controller, double loopPeriodS,
            PassTargetSelectorSubsystem passTargetSelector) {
        this.controller = controller;
        this.loopPeriodS = loopPeriodS;
        this.table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        addRequirements(passTargetSelector);

        currentSpeed = SLOW_SPEED;
    }

    @Override
    public void initialize() {
        System.out.println("PassTargetCursor initialized, seeding from: " + cursorX + ", " + cursorY);
        // Seed the cursor from the last committed target using typed subscribers.
        // Subscribers MUST be opened and read BEFORE opening publishers on the same
        // topics (targetXKey / targetYKey). If a publisher is opened first, the topic
        // type is "claimed" and a subsequent subscriber.get() may return the
        // publisher's
        // initial default rather than the previously stored value.
        try (DoubleSubscriber targetXSub = table.getDoubleTopic(PassTargetConstants.targetXKey)
                .subscribe(PassTargetConstants.defaultTargetX);
                DoubleSubscriber targetYSub = table.getDoubleTopic(PassTargetConstants.targetYKey)
                        .subscribe(PassTargetConstants.defaultTargetY)) {
            cursorX = targetXSub.get();
            cursorY = targetYSub.get();
        }

        // Open publishers after the seed reads are complete and subscribers are closed.
        // FIX 3: Resources are opened per-scheduling here and closed symmetrically in
        // end().
        cursorXPub = table.getDoubleTopic(PassTargetConstants.cursorXKey).publish();
        cursorYPub = table.getDoubleTopic(PassTargetConstants.cursorYKey).publish();
        commitXPub = table.getDoubleTopic(PassTargetConstants.targetXKey).publish();
        commitYPub = table.getDoubleTopic(PassTargetConstants.targetYKey).publish();

        aPressedLast = controller.getHID().getAButton();

        System.out.println("stickX: " + controller.getRightX() + " stickY: " + controller.getRightY());
        System.out.println("cursorX: " + cursorX + " cursorY: " + cursorY);

        SmartDashboard.putNumber("Turret/Pass/CursorSpeed", currentSpeed);
    }

    public void toggleMaxSpeed() {
        currentSpeed = currentSpeed == SLOW_SPEED ? FAST_SPEED : SLOW_SPEED;
        SmartDashboard.putNumber("Turret/Pass/CursorSpeed", currentSpeed);
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    @Override
    public void execute() {
        System.out.println("PassTargetCursor executing");
        System.out.println("aPressed: " + controller.getHID().getAButton());

        double stickX = applyDeadband(controller.getRightX());
        double stickY = -applyDeadband(controller.getRightY()); // up = +Y

        cursorX = clamp(cursorX + stickX *currentSpeed * loopPeriodS,
                0.0, FieldConstants.fieldLength);
        cursorY = clamp(cursorY + stickY * currentSpeed * loopPeriodS,
                0.0, FieldConstants.fieldWidth);

        cursorXPub.set(cursorX);
        cursorYPub.set(cursorY);

        boolean aPressed = controller.getHID().getAButton();
        boolean aRisingEdge = aPressed && !aPressedLast;
        aPressedLast = aPressed;

        if (aRisingEdge) {
            commitCursor();
        }

        SmartDashboard.getNumber("Turret/Pass/CursorSpeed", currentSpeed);
    }

    @Override
    public boolean isFinished() {
        // FIX 6: This command is intentionally perpetual (runs until interrupted).
        // It should be registered as a default command on whatever subsystem owns
        // the operator controller so it is always rescheduled after a disable/enable
        // cycle. Example in RobotContainer:
        // oi.setDefaultCommand(new PassTargetCursor(operatorController));
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Close all publishers so NetworkTables releases resources immediately
        // rather than waiting for GC.
        if (cursorXPub != null) {
            cursorXPub.close();
            cursorXPub = null;
        }
        if (cursorYPub != null) {
            cursorYPub.close();
            cursorYPub = null;
        }
        if (commitXPub != null) {
            commitXPub.close();
            commitXPub = null;
        }
        if (commitYPub != null) {
            commitYPub.close();
            commitYPub = null;
        }
    }

    private void commitCursor() {
        // Null guard: publishers are only open while the command is scheduled,
        // but guard defensively in case of unexpected calls outside the lifecycle.
        if (commitXPub == null || commitYPub == null)
            return;
        commitXPub.set(cursorX);
        commitYPub.set(cursorY);
    }

    private double applyDeadband(double v) {
        if (Math.abs(v) < DEADBAND)
            return 0.0;
        return (v - Math.signum(v) * DEADBAND) / (1.0 - DEADBAND);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}