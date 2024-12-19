// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.otherConstants.operatorConstants.ARMPORT;
import static frc.robot.otherConstants.operatorConstants.MECH_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    //public SwerveDrive swervee;

    CommandXboxController joystick1 = new CommandXboxController(MECH_CONTROLLER_PORT);

    // The robot's subsystems and commands are defined here...
    public static final ClawSubsystem claw = new ClawSubsystem();
    public static final ArmSubsystem armSub = new ArmSubsystem();
    public static final SwerveDrive swerveDrive = new SwerveDrive();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController mechController = new CommandXboxController(ARMPORT);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        RoutineChooser routineChooser = new RoutineChooser();
        routineChooser.InitializeAutos();
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
        mechController.y().onTrue(armSub.goToStow());
        //mechController.x().onTrue(armSub.goToHalf());
        mechController.b().onTrue(armSub.goToHighScore());
        mechController.a().onTrue(armSub.goToIntake());

        mechController.leftTrigger().onTrue(claw.intake()).onFalse(
                runOnce(() -> {
            mechController.rightTrigger().
                    onTrue(claw.outtake()).
                    onFalse(claw.doNothing());
        }));



        joystick1.start().onTrue(runOnce(() -> {
            swerveDrive.gyro.reset();
        }));

        SmartDashboard.putData("claw data", claw);
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("arm data", armSub);

        SmartDashboard.putData("swerve", swerveDrive);
    }
/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */







}