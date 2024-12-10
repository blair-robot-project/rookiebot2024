package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
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
                        RobotContainer.claw.outtake()
                ),
                RobotContainer.armSub.goToStow().alongWith(RobotContainer.claw.doNothing(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("toBucketBottom"))),
                RobotContainer.armSub.goToIntake().until(RobotContainer.armSub.isDone()).alongWith(RobotContainer.claw.intake()),
                        new WaitCommand(clawConstants.INTAKE_SECONDS),
                RobotContainer.armSub.goToStow().alongWith(RobotContainer.claw.holdBucket(),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("fromBucketBottom"))),
                RobotContainer.armSub.goToIntake().until(RobotContainer.armSub.isDone()),
                new ParallelRaceGroup(
                        new WaitCommand(clawConstants.OUTTAKE_SECONDS),
                        RobotContainer.claw.outtake()
                ),
                RobotContainer.claw.doNothing()
        );
    }
}
