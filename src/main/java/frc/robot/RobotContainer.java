// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.allConstants.driveConstants;
import frc.robot.commands.Autos;
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

    double xdirection;
    double ydirection;
    double rotation;


    Joystick joystick1 = new Joystick(0);
    Joystick joystick2 = new Joystick(0);

    public RunCommand drive = new RunCommand(() -> {
        xdirection = joystick1.getX() * driveConstants.MAX_SPEED;
        ydirection = joystick1.getY() * driveConstants.MAX_SPEED;

        //joystick2 is for rotation
        rotation = joystick2.getX() * driveConstants.MAX_ANGULAR_SPEED;


        //set the module states based on joystick
        swervee.drive(
                xdirection,
                ydirection,
                rotation,
                driveConstants.fRel,
                driveConstants.pdsec);

    });

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
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
        mechController.x().onTrue(armSub.goToSetpoint());
        mechController.b().onTrue(armSub.goToBase());
        mechController.a().onTrue(armSub.goToHalf());

        mechController.rightTrigger().onTrue(clawSubsystem.Intake()).onFalse(clawSubsystem.HoldBucket());
        mechController.leftTrigger().onTrue(clawSubsystem.Outtake()).onFalse(clawSubsystem.HoldBucket());


    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto(middleB);
    }
}
