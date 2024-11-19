// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static frc.robot.allConstants.armConstants.*;

public class ArmSubsystem extends SubsystemBase {

    CANSparkMax motor = new CANSparkMax(armMotorID, MotorType.kBrushless);
    CANSparkMax motorFollower = new CANSparkMax(armMotorFollowerID, MotorType.kBrushless);
    double kP = armKP, kI = armKI, kD = armKD;
    PIDController pid = new PIDController(kP, kI, kD);
    double current;
    double desired;
    double desiredVal;
    double baseVal;
    /*
    gearbox - The type of and number of motors in the arm gearbox.
    gearing - The gearing of the arm (numbers greater than 1 represent reductions).
    jKgMetersSquared - The moment of inertia of the arm, can be calculated from CAD software.
    armLengthMeters - The length of the arm.
    minAngleRads - The minimum angle that the arm is capable of.
    maxAngleRads - The maximum angle that the arm is capable of.
    simulateGravity - Whether gravity should be simulated or not.
    startingAngleRads - The initial position of the Arm simulation in radians.
    */
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            armGearbox,
            armGearing,
            armInertia,
            armLength,
            minAngleRads,
            maxAngleRads,
            true,
            minAngleRads
    );

    /**
     * Creates a new ExampleSubsystem.
     */
    public ArmSubsystem(double des, double base) {
        motorFollower.follow(motor, false);
        desiredVal = des;
        baseVal = base;
        current = base;
    }

    public ArmSubsystem(double des) {
        motorFollower.follow(motor, false);
        desiredVal = des;
        baseVal = armDefaultBaseValue;
        current = armDefaultBaseValue;
    }

    public ArmSubsystem() {
        motorFollower.follow(motor, false);
        desiredVal = armDefaultDesiredValue;
        baseVal = armDefaultBaseValue;
        current = armDefaultBaseValue;
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
                    this.motor.setVoltage(voltage);
                });
    }

    public Command stopRunning() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.motor.setVoltage(0);
                });
    }

    public Command goToSetpoint() {
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
                    this.desired = this.desiredVal / 2;
                });
    }

    public Command goToBase() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    this.desired = this.baseVal;
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public double returnMotorPos() {
        return this.motor.getEncoder().getPosition();
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
        this.current = this.returnMotorPos() / armGearRatio; // gear ratio maybe somewhere?
        double voltage = pid.calculate(this.current, this.desired);
        System.out.println(voltage);
        this.motor.setVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}
