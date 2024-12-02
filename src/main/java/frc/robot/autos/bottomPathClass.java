package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.clawConstants;

public class bottomPathClass {
    public Command bottomPath() {

        return new SequentialCommandGroup(
                RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()),
                new ParallelRaceGroup(
                        new WaitCommand(clawConstants.OUTTAKE_SECONDS),
                        RobotContainer.claw.Outtake()
                ),
                RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.doNothing(),
                        new PathPlannerAuto("toBucketBottom")),
                RobotContainer.armSub.goToIntake().alongWith(RobotContainer.claw.Intake()).until(RobotContainer.armSub.isDone()),
                new ParallelRaceGroup( // intakes more after it gets to the bottom
                        new WaitCommand(clawConstants.INTAKE_SECONDS),
                        RobotContainer.claw.Intake()),
                RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.HoldBucket(),
                new PathPlannerAuto("fromBucketBottom")),
                RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()),
                new ParallelRaceGroup(
                        new WaitCommand(clawConstants.OUTTAKE_SECONDS),
                        RobotContainer.claw.Outtake()
                ),
                RobotContainer.claw.doNothing()
        );
    }
}
