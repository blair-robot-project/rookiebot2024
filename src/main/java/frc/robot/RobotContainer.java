// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        mechController.leftTrigger().onTrue(claw.Outtake()).onFalse(claw.HoldBucket());

        SmartDashboard.putData(claw);
    }
/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */

public Command taxiPath() {
    // An example command will be run in autonomous
    return new ParallelCommandGroup(
            new PathPlannerAuto("red") // placeholder, will be arm
    );
}

public Command middlePath() {
    // An example command will be run in autonomous
    return new ParallelCommandGroup(
            armSub.goToHighScore(),
            claw.Outtake(),
            armSub.goToTop(),
            new PathPlannerAuto("toBucketMiddle"), // placeholder
            armSub.goToHighScore(),
            claw.Intake(),
            armSub.goToTop(),
            new PathPlannerAuto("fromBucketMiddle"),
            armSub.goToHalf().andThen(claw.Outtake())
            );
}
public Command bottomPath() {
    // An example command will be run in autonomous
    return new ParallelCommandGroup(
            new PathPlannerAuto("toBucketBottom"), // placeholder
            new WaitCommand(1), // placeholder
            new PathPlannerAuto("fromBucketBottom"),
            armSub.goToTop().andThen(claw.Intake())

    );
}

}