// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.allConstants.armConstants.armBaseValue;
import static frc.robot.allConstants.armConstants.armDesiredValue;
import static frc.robot.allConstants.operatorConstants.DRIVE_CONTROLLER_PORT;
import static frc.robot.allConstants.operatorConstants.MECH_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public SwerveDrive swervee;

    Joystick joystick1 = new Joystick(0);
    Joystick joystick2 = new Joystick(0);

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final ClawSubsystem claw = new ClawSubsystem();
    private final ArmSubsystem armSub = new ArmSubsystem(armDesiredValue, armBaseValue);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(DRIVE_CONTROLLER_PORT);
    private final CommandXboxController mechController = new CommandXboxController(MECH_CONTROLLER_PORT);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        mechController.y().onTrue(armSub.goToTop());
        mechController.x().onTrue(armSub.goToHalf());
        mechController.b().onTrue(armSub.goToHighScore());
        mechController.a().onTrue(armSub.goToTop());

        mechController.rightTrigger().onTrue(claw.Intake()).onFalse(claw.HoldBucket());
        mechController.leftTrigger().onTrue(claw.Outtake()).onFalse(claw.doNothing());

        SmartDashboard.putData(claw);
    }
/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */

public Command taxiPath() {
    return new SequentialCommandGroup(
            armSub.goToHalf().andThen(claw.Outtake()),
            armSub.goToTop().alongWith(claw.doNothing(),
            new PathPlannerAuto("red"))
    );
}

public Command middlePath() {
    return new SequentialCommandGroup(
            armSub.goToHalf().andThen(claw.Outtake()), ///arm goes half-way down and claw outtakes the bucket first
            armSub.goToTop().alongWith(claw.doNothing()).until(armSub.isDone()), ///the claw stops as the arm goes back up
            new PathPlannerAuto("toBucketMiddle"),///robot follows the path to middle bucket
            armSub.goToIntake().alongWith(claw.Intake().andThen( ///arm goes down halfway with the intake running
                            armSub.goToTop().alongWith(claw.HoldBucket()).until(armSub.isDone())),///goes back up
            new PathPlannerAuto("fromBucketMiddle"),///follows a path back to the staking grid
            armSub.goToHalf().until(armSub.isDone()).andThen(claw.Outtake().andThen(claw.doNothing())))///arm goes down halfway and outtake
            ); ///i don't really get what the armsub.isdone() does
}

public Command bottomPath() {
    return new SequentialCommandGroup(
            armSub.goToHalf().andThen(claw.Outtake()),
            armSub.goToTop().alongWith(claw.doNothing()).until(armSub.isDone()),
            new PathPlannerAuto("toBucketBottom"),
            armSub.goToIntake().alongWith(claw.Intake()).andThen(
                    armSub.goToTop().alongWith(claw.HoldBucket())),
            new PathPlannerAuto("fromBucketBottom"),
            armSub.goToHalf().andThen(claw.Outtake()).andThen(claw.doNothing())
    );
}

}