package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.claw.clawConstants;


public class taxiPathClass {
    public Command taxiPath() {
        return new SequentialCommandGroup(
                RobotContainer.armSub.goToIntake(),
                new ParallelRaceGroup(
                        new WaitCommand(clawConstants.OUTTAKE_SECONDS),
                        RobotContainer.claw.outtake()
                ),
                RobotContainer.armSub.goToStow().alongWith(RobotContainer.claw.doNothing(),
                        new InstantCommand(() -> RobotContainer.swerveDrive.drive(2.0, 0.0, 0.0, true, 0.02)))
        );
    }
}
