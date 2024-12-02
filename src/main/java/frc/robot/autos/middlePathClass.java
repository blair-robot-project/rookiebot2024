package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.clawConstants;

public class middlePathClass {
    public Command middlePath() {
        return new SequentialCommandGroup(
                RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()).andThen(RobotContainer.claw.Outtake().withTimeout(clawConstants.OUTTAKE_SECONDS)), ///arm goes half-way down and claw outtakes the bucket first
                RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.doNothing(), ///the claw stops as the arm goes back up
                        new PathPlannerAuto("toBucketMiddle")),///robot follows the path to middle bucket
                RobotContainer.armSub.goToIntake().alongWith(RobotContainer.claw.Intake().until(RobotContainer.armSub.isDone()).andThen( ///arm goes down halfway with the intake running
                                RobotContainer.armSub.goToTop().alongWith(RobotContainer.claw.HoldBucket()).until(RobotContainer.armSub.isDone())),///goes back up
                        new PathPlannerAuto("fromBucketMiddle"),///follows a path back to the staking grid
                        RobotContainer.armSub.goToHalf().until(RobotContainer.armSub.isDone()).andThen(RobotContainer.claw.Outtake().withTimeout(clawConstants.OUTTAKE_SECONDS).andThen(RobotContainer.claw.doNothing())))///arm goes down halfway and outtake
        );
    }
}
