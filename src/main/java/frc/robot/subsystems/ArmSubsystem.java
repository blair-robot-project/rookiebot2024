// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.allConstants.armConstants;

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

    DutyCycleEncoder encoder = new DutyCycleEncoder(armConstants.encoderPort);
    /*
    * current is the arm's current position in __
    *
    * */
    double currentState;
    double desired;
    double desiredVal;
    double baseVal;

    ArmFeedforward feedForward_a = new ArmFeedforward (armConstants.armFeedForwardKs, armConstants.armFeedForwardKg, armConstants.armFeedForwardKv);

    public ArmSubsystem(double des, double base) {
        this.armMotor = new CANSparkMax(armConstants.armMotorIDa, MotorType.kBrushless);
        this.armMotorFollower= new CANSparkMax(armConstants.armMotorFollowerID, MotorType.kBrushless);
        armMotorFollower.follow(armMotor, false);
        desiredVal = des;
        baseVal = base;
        currentState = base;
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
                    this.armMotor.setVoltage(voltage);
                });
    }

    public Command stopRunning() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.armMotor.setVoltage(0);
                });
    }

    public Command goToTop() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.desired = armConstants.armTopPositionValue;
                });
    }

    public Command goToHighScore() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.desired = this.desiredVal;
                });
    }

    public Command goToHalf() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.desired = (this.desiredVal+this.baseVal) / 2; // check
                });
    }

    public Command goToIntake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.desired = this.baseVal;
                });
    }

    public double returnMotorPos() {
        return encoder.getPosition();
    }

    public BooleanSupplier isDone(){
        BooleanSupplier finished = () ->
        this.returnMotorPos()/armConstants.armGearRatio==this.desired;
        return finished;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.publishConstString("1.0", "Logging stuff");
        //research how to do double supplier in java
        DoubleSupplier motorPos = () -> this.returnMotorPos();
        builder.addDoubleProperty("1.1 position", motorPos, null);
    }

    @Override
    public void periodic() {
        this.currentState = this.returnMotorPos() / armConstants.armGearing; //.armGearRatio; // gear ratio maybe somewhere?
        double voltage = pid.calculate(this.currentState, this.desired)+getArmF(desired);
        System.out.println(voltage);
        this.armMotor.setVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}