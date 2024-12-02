// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.allConstants.armConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {
    //arm motor
    CANSparkMax armMotor;
    //second arm motor
    CANSparkMax armMotorFollower;
    CANSparkMax simMotor;
    SingleJointedArmSim armSim;
    Mechanism2d mech2d;
    //establishing kp, ki, and kd
    double kP = armConstants.armKP, kI = armConstants.armKI, kD = armConstants.armKD;
    //pid controller
    PIDController pid = new PIDController(kP, kI, kD);
    EncoderSim encoderSim;
    SmartDashboard smartDashboard;
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
        if(Robot.isSimulation()){
            armSim = new SingleJointedArmSim(
                    armConstants.armGearbox,
                    armConstants.armGearing,
                    armConstants.armInertia,
                    armConstants.armLength,
                    armConstants.minAngleRads,
                    armConstants.maxAngleRads,
                    armConstants.armSimGrav,
                    armConstants.armBaseValue
            );
            mech2d = new Mechanism2d(60, 60);
            encoderSim = new EncoderSim(armEncoder);
            smartDashboard.putData(mech2d);
            simMotor = new CANSparkMax(armConstants.armSimID, CANSparkLowLevel.MotorType.kBrushless);
        }
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
        builder.addDoubleProperty("1.1 position", this::returnMotorPos, null);
    }

    @Override
    public void periodic() {
        this.currentState = this.returnMotorPos() / armConstants.armGearing; //.armGearRatio; // gear ratio maybe somewhere?
        double voltage = pid.calculate(this.currentState, this.desired)+getArmF(desired);
        this.armMotor.setVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        armSim.setInputVoltage  (simMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setDistance(armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    }

}