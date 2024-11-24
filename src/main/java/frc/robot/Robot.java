// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.allConstants.driveConstants;
import frc.robot.subsystems.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


    private Command autonomousCommandTaxi;
    private Command autonomousCommandMiddle;
    private Command autonomousCommandTop;

    private RobotContainer robotContainer;
    private SwerveDrive swerve;



    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        swerve = new SwerveDrive();
        robotContainer = new RobotContainer();
    }

    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        swerve.updateOdometry();
    }

    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override

    public void autonomousInit() {
        autonomousCommandTaxi = robotContainer.taxiPath();
        autonomousCommandMiddle=robotContainer.middlePath();
        autonomousCommandTop= robotContainer.bottomPath();

        // schedule the autonomous command (example)
        if (autonomousCommandTaxi != null) {
            autonomousCommandTaxi.schedule();
        }
        if (autonomousCommandMiddle != null) {
            autonomousCommandMiddle.schedule();
        }
        if (autonomousCommandTop != null) {
            autonomousCommandTop.schedule();
        }
    }

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommandTaxi != null) {
            autonomousCommandTaxi.cancel();
        }
        if (autonomousCommandMiddle != null) {
            autonomousCommandMiddle.cancel();
        }
        if (autonomousCommandTop != null) {
            autonomousCommandTop.cancel();
        }
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double xdirection;
        double ydirection;
        double rotation;

        xdirection = robotContainer.joystick1.getX() * driveConstants.MAX_SPEED;
        ydirection = robotContainer.joystick1.getY() * driveConstants.MAX_SPEED;

        //joystick2 is for rotation
        rotation = robotContainer.joystick2.getX() * driveConstants.MAX_ANGULAR_SPEED;
        //joystick1 is for driving
        swerve.drive(
                xdirection,
                ydirection,
                rotation,
                driveConstants.fRel,
                driveConstants.pdsec);

    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This method is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}