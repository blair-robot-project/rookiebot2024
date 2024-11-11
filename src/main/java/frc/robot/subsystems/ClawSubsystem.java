// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.allConstants.clawConstants;


public class ClawSubsystem extends SubsystemBase
{
    CANSparkMax motor = new CANSparkMax(clawConstants.clawMotorId, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax inverseFollowMotor = new CANSparkMax(clawConstants.clawFollowerMotorId, CANSparkLowLevel.MotorType.kBrushless);
    /** Creates a new Claw Subsystem. */
    public ClawSubsystem() {
        inverseFollowMotor.follow(motor, true);
    }


    /**
     * Intake command factory method.
     *
     * @return a command
     */
    public Command Intake()
    {
        return runOnce(
                () -> {
                    motor.setVoltage(clawConstants.clawMotorVoltage);
                });
    }
    /**
     * Outtake command factory method.
     *
     * @return a command
     */
    public Command Outtake() {
        return runOnce( () -> {
            motor.setVoltage(clawConstants.clawMotorInvVoltage);
        });
    }
    /**
     * DoNothing command factory method.
     *
     * @return a command
     */
    public Command DoNothing() {

        return runOnce( () -> {
            motor.stopMotor();
        });
    }
    /**
     * HoldBucket command factory method.
     *
     * @return a command
     */
    public Command HoldBucket() {

    return runOnce( () -> {

    });
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
