// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.bottomPathClass;
import frc.robot.autos.middlePathClass;
import frc.robot.autos.taxiPathClass;
import frc.robot.subsystems.swerve.driveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.sql.Driver;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private Command autonomousCommandTaxi;
    private Command autonomousCommandMiddle;
    private Command autonomousCommandBottom;

    private RobotContainer robotContainer;
    RoutineChooser routineChooser = new RoutineChooser();
    bottomPathClass bottomPathClass1 = new bottomPathClass();
    middlePathClass middlePathClass1 = new middlePathClass();
    taxiPathClass taxiPathClass1 = new taxiPathClass();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        DriverStation.startDataLog(DataLogManager.getLog());

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
        RobotContainer.swervee.updateOdometry();
    }

    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        RobotContainer.armSub.stop();
    }


    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override

    public void autonomousInit()
    {
        m_autonomousCommand = routineChooser.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        autonomousCommandTaxi = taxiPathClass1.taxiPath();
        autonomousCommandMiddle=middlePathClass1.middlePath();
        autonomousCommandBottom = bottomPathClass1.bottomPath();

        // schedule the autonomous command (example)

    }

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        RobotContainer.armSub.loadPreferences();
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double xdirection;
        double ydirection;
        double rotation;

        xdirection = -robotContainer.joystick1.getLeftY() * driveConstants.MAX_SPEED;

        ydirection = -robotContainer.joystick1.getLeftX() * driveConstants.MAX_SPEED;

        xdirection = MathUtil.applyDeadband(xdirection, 0.2, 1.0);
        ydirection = MathUtil.applyDeadband(ydirection, 0.2, 1.0);

        //joystick2 is for rotation
        rotation = robotContainer.joystick1.getRightX() * driveConstants.MAX_ANGULAR_SPEED;


        if (Math.abs(xdirection) < 0.05 && Math.abs(ydirection) < 0.05 && Math.abs(rotation) < 0.05) {
            RobotContainer.swervee.stopMotors();
        }
        else {
            RobotContainer.swervee.drive(
                    xdirection,
                    ydirection,
                    rotation,
                    driveConstants.fRel,
                    driveConstants.pdsec);
        }

        if (robotContainer.joystick1.getLeftBumper()) {
            RobotContainer.armSub.decreaseDesired();
        }

        if (robotContainer.joystick1.getRightBumper()) {
            RobotContainer.armSub.increaseDesired();
        }
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

    @Override
    public void close() {
        RobotContainer.armSub.close();
        super.close();
    }

    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }

    public void print(Object o) {
        System.out.println(o.toString());
    }
}