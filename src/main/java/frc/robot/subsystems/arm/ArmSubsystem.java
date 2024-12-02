// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

    //arm motor
    CANSparkMax armMotor;
    //second arm motor
    CANSparkMax armMotorFollower;
    //establishing kp, ki, and kd
    double kP = armConstants.armKP, kI = armConstants.armKI, kD = armConstants.armKD;
    //pid controller
    PIDController pid = new PIDController(kP, kI, kD);
    /*
    * current is the arm's current position in radians
    * */
    double currentState = armConstants.armBasePosition;
    double desired = armConstants.armHighScorePosition;

    SimpleMotorFeedforward feedForward_a = new SimpleMotorFeedforward(armConstants.armFeedForwardKs, armConstants.armFeedForwardKv, armConstants.armFeedForwardKa);

    public ArmSubsystem() {
        armMotor = new CANSparkMax(armConstants.armMotorIDa, MotorType.kBrushless);
        armMotorFollower= new CANSparkMax(armConstants.armMotorFollowerID, MotorType.kBrushless);
        armMotorFollower.follow(armMotor, false);
    }

    public double getArmF(double des){
        return feedForward_a.calculate(des);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command setVoltage(double voltage) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    armMotor.setVoltage(voltage);
                });
    }

    public Command stopRunning() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    armMotor.setVoltage(0);
                });
    }

    public Command goToStow() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armStowPosition;
                });
    }

    public Command goToHighScore() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition;
                });
    }

    public Command goToHalf() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHighScorePosition / 2;
                });
    }

    public Command goToIntake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armBasePosition;
                });
    }

    public double returnMotorPos() {
        return this.armMotor.getEncoder().getPosition();
    }

    public BooleanSupplier isDone(){
        BooleanSupplier finished = () ->
        returnMotorPos()/armConstants.armGearRatio==this.desired;
        return finished;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Sim Voltage");
        builder.publishConstString("1.0", "Logging stuff");
        //research how to do double supplier in java
        DoubleSupplier motorPos = () -> returnMotorPos();
        builder.addDoubleProperty("1.1 position", motorPos, null);
    }

    @Override
    public void periodic() {
        currentState = returnMotorPos() / armConstants.armGearRatio; // gear ratio maybe somewhere?
        double voltage = pid.calculate(currentState, desired) + getArmF(desired);
        setVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}