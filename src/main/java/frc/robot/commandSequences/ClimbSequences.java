package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb.ShortClimb;
import frc.robot.subsystems.Climb.TallClimb;

//Start of Climb Sequence
//L1-------------------------
//Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to highest position
//Tall Arm pulls down until small arm then small arm hooks on first rung and Tall Arm lets go
//L2-------------------------
//Tall Arm goes to second rung position and hooks on second rung
//Small Arm unhooks from first rung
//Kicker Arm goes out and pushes robot back and allows clearance
//Tall Arm pulls down until small arm then small arm hooks on second rung and Tall Arm lets go
//L3-------------------------
//Tall Arm goes to third rung position and hooks on third rung
//Small Arm unhooks from second rung
//Kicker Arm goes out and pushes robot back and allow clearance
//Tall Arm pulls down until small arm then small arm hooks on third rung and Tall Arm lets go.
//End Climb---------------------

public class ClimbSequences {
    public static Command climbMasterReset(TallClimb tallClimb, ShortClimb shortClimb) {
        return new ParallelCommandGroup(
                tallClimb.tallGoToPosition(() -> 0.0),
                shortClimb.shortGoToPosition(() -> 0.0));
    }

    public static Command climbToL1TeleOp(TallClimb tallClimb, ShortClimb shortClimb) {

        // L1-------------------------
        // Taller Arm goes to 30 inches and hooks on first rung and Small Arm goes to
        // highest position
        // Tall Arm pulls down until small arm then small arm hooks on first rung and
        return new SequentialCommandGroup(
                tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                        tallClimb.tallGoToPosition(() -> 0.0),
                        shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition)));
    }

    public static Command climbToL2(TallClimb tallClimb, ShortClimb shortClimb) {
        return new SequentialCommandGroup(
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                        tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
                        shortClimb.shortGoToPosition(() -> 0.0)),
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                        tallClimb.tallGoToPosition(() -> 0.0),
                        shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        );
    }

    public static Command climbToL3(TallClimb tallClimb, ShortClimb shortClimb) {
        // Tall arm goes to rung
        // Tall arm starts to pull down and Short arm starts to go up
        // By now after Short hooks on we are in L1
        // Now Tall arm goes up and Short arm goes down
        // After tall arm hooks on we are in L2
        // Now tall arm goes down and short arm goes up and hooks on without tall arm
        // letting go
        // voilah we r in l3 now

        return new SequentialCommandGroup(
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                        tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
                        shortClimb.shortGoToPosition(() -> 0.0)),
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                        tallClimb.tallGoToPosition(() -> 0.0),
                        shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        );

    }

    public static Command fullTeleopClimb(TallClimb tallClimb, ShortClimb shortClimb) {
        return new SequentialCommandGroup(
                climbToL1TeleOp(tallClimb, shortClimb),
                climbToL2(tallClimb, shortClimb),
                climbToL3(tallClimb, shortClimb));
    }
}