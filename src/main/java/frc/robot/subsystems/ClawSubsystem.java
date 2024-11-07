// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClawConstants.*;


public class ClawSubsystem extends SubsystemBase
{
    CANSparkMax motor;
    CANSparkMax inverseFollowMotor;
    PIDController Claw_pidController;
    double setpoint_Claw ;
    double Current_Position_Claw;



    /** Creates a new Claw Subsystem. */
    public ClawSubsystem(int motorId, int inverseFollowMotorId) {
        //Needs some editing ; waiting for some values
        motor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
        inverseFollowMotor = new CANSparkMax(inverseFollowMotorId, CANSparkLowLevel.MotorType.kBrushless);
        inverseFollowMotor.follow(motor, true);
        Claw_pidController= new PIDController(CLAW_P,CLAW_I,CLAW_D);
        setpoint_Claw=0.0;
        Current_Position_Claw=0.0;



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
        return runOnce( () -> {
            setpoint_Claw=Claw_Setpoint_Intake;

            /* one-time action goes here */
                });
    }

    public Command Outtake() {
        return runOnce( () -> {;
             //Constants
            setpoint_Claw=Claw_Setpoint_Outtake;


        });
    }

    public Command DoNothing() {

        return runOnce( () -> {
            motor.stopMotor();
        });
    }

    public Command HoldBucket() {

    return runOnce( () -> {
        setpoint_Claw=Claw_Setpoint_Hold;

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
        Current_Position_Claw=motor.getEncoder().getPosition();
        double Voltage=Claw_pidController.calculate(setpoint_Claw,Current_Position_Claw);
        motor.setVoltage(Voltage);

        // This method will be called once per scheduler run
    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
