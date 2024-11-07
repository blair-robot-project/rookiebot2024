// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ClawSubsystem extends SubsystemBase
{

    /** Creates a new ExampleSubsystem. */
    public ClawSubsystem(int motorId, int inverseFollowMotorId) {


    }


    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command Intake()
    {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public Command Outtake() {
        return runOnce( () -> {

        });
    }

    public Command DoNothing() {

        return runOnce( () -> {

        });
    }

    public Command HoldBucket() {

    return runOnce( () -> {

    });
}


    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition()
    {
        // Query some boolean state, such as a digital sensor.
        return false;
    }


    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
