package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.clawConstants;

public class bottomPathClass {
    public Command bottomPath() {
        return new SequentialCommandGroup(
                RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()).andThen(RobotContainer.claw.Outtake().withTimeout(clawConstants.OUTTAKE_SECONDS)),
                RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.doNothing(),
                        new PathPlannerAuto("toBucketBottom")),
                RobotContainer.armSub.goToIntake().alongWith(RobotContainer.claw.Intake()).until(RobotContainer.armSub.isDone()).andThen(
                        RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.HoldBucket())),
                new PathPlannerAuto("fromBucketBottom"),
                RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()).andThen(RobotContainer.claw.Outtake().withTimeout(clawConstants.OUTTAKE_SECONDS)).andThen(RobotContainer.claw.doNothing())
        );
    }
}
