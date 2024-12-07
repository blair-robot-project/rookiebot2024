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
                RobotContainer.armSub.goToIntake().until(RobotContainer.armSub.isDone()),
                new ParallelRaceGroup(
                        new WaitCommand(clawConstants.OUTTAKE_SECONDS),
                        RobotContainer.claw.Outtake()
                ),
                RobotContainer.armSub.goToStow().alongWith(RobotContainer.claw.doNothing(),
                        new PathPlannerAuto("toBucketBottom")),
                RobotContainer.armSub.goToIntake().until(RobotContainer.armSub.isDone()).alongWith(RobotContainer.claw.Intake()),
                        new WaitCommand(clawConstants.INTAKE_SECONDS),
                RobotContainer.armSub.goToStow().alongWith(RobotContainer.claw.HoldBucket(),
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
