package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb.ShortClimb;
import frc.robot.subsystems.Climb.TallClimb;

public class ClimbSequences {
        // idk if this is useful
        public static Command climbMasterReset(TallClimb tallClimb, ShortClimb shortClimb) {
                return new ParallelCommandGroup(
                                tallClimb.tallGoToPosition(() -> 0.0),
                                shortClimb.shortGoToPosition(() -> 0.0));
        }

        // public static Command tallArmUp(TallClimb tallClimb, ShortClimb shortClimb) {
        // return new SequentialCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position)
        // );
        // }

        public static Command climbToL1(TallClimb tallClimb, ShortClimb shortClimb) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
                                                shortClimb.shortGoToPosition(() -> ClimbConstants.l1Position)),
                                new WaitCommand(0.3),
                                new ParallelCommandGroup(
                                        tallClimb.tallGoToPosition(() -> 0.0),
                                        shortClimb.shortGoToPosition(() -> 0.0)
                                )
                );
        }

        // public static Command climbToL2(TallClimb tallClimb, ShortClimb shortClimb) {
        // return new SequentialCommandGroup(
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // shortClimb.shortGoToPosition(() -> 0.0)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> 0.0),
        // shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        // );
        // }

        // public static Command climbToL3(TallClimb tallClimb, ShortClimb shortClimb) {
        // // Tall arm goes to rung
        // // Tall arm starts to pull down and Short arm starts to go up
        // // By now after Short hooks on we are in L1
        // // Now Tall arm goes up and Short arm goes down
        // // After tall arm hooks on we are in L2
        // // Now tall arm goes down and short arm goes up and hooks on without tall arm
        // // letting go
        // // voilah we r in l3 now

        // return new SequentialCommandGroup(
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> ClimbConstants.l1Position),
        // shortClimb.shortGoToPosition(() -> 0.0)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
        // tallClimb.tallGoToPosition(() -> 0.0),
        // shortClimb.shortGoToPosition(() -> ClimbConstants.l1ShortPosition))

        // );

        // }
}