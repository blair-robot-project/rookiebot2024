// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;

public class ArmSubsystem extends SubsystemBase {

    //arm motor
    CANSparkMax armMotor;

    //second arm motor
    CANSparkMax armMotorFollower;

    //establishing kp, ki, and kd
    double kP = armConstants.armKP, kI = armConstants.armKI, kD = armConstants.armKD;

    //pid controller
    PIDController pid = new PIDController(kP, kI, kD);

    //current is the arm's current position in radians
    double currentState;

    //encoder position
    double simState = armConstants.armIntakePosition;

    //desired is where the robot wants to go
    double desired;

    //arm feed forward voltage
    double feedForwardVoltage = 0;

    //pid voltage
    double pidVoltage = 0;

    //desired name is the name of the position we want to go to
    String desiredName = "Intake";

    double voltage = 0.0;

    private final DutyCycleEncoder armEncoder;

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private Mechanism2d mech2d;

    private MechanismRoot2d armPivot;

    private MechanismLigament2d armTower;

    private SingleJointedArmSim armSim;

    private MechanismLigament2d armLigament;

    private double RobotControllerBattery = 0.0;

    private double armMotorAppliedOutput = 0.0;

    //feed forward
    ArmFeedforward feedForward_a = new ArmFeedforward(armConstants.armFeedForwardKs, armConstants.armFeedForwardKg, armConstants.armFeedForwardKv);

    public ArmSubsystem() {
        armMotor = new CANSparkMax(armConstants.armMotorIDa, MotorType.kBrushless);

        armEncoder = new DutyCycleEncoder(armConstants.encoderPort);
        armEncoder.reset();
        armEncoder.setPositionOffset(armConstants.offset);
        armEncoder.setDistancePerRotation(armConstants.kArmEncoderDistPerRotation);
        pid.setTolerance(armConstants.armEpsilon);

        armMotorFollower = new CANSparkMax(armConstants.armMotorFollowerID, MotorType.kBrushless);
        armMotorFollower.follow(armMotor, true);

        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotorFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotor.burnFlash();
        armMotorFollower.burnFlash();

        if (Robot.isSimulation()) {

            // Constructs armSim
            armSim = new SingleJointedArmSim(
                    armConstants.armGearbox,
                    armConstants.armGearRatio,
                    armConstants.armInertia,
                    armConstants.armLength,
                    armConstants.minAngleRads,
                    armConstants.maxAngleRads,
                    armConstants.armSimGrav,
                    armConstants.armIntakePosition
            );

            mech2d = new Mechanism2d(60, 60);

            armPivot = mech2d.getRoot("ArmPivot", 30, 30);

            armTower = new MechanismLigament2d("ArmTower", 30, -90);
            armPivot.append(armTower);

            armLigament = new MechanismLigament2d(
                    "Arm",
                    30,
                    Units.radiansToDegrees(armSim.getAngleRads()),
                    6,
                    new Color8Bit(Color.kYellow)
            );
            armPivot.append(armLigament);

            SmartDashboard.putData("Arm Sim", mech2d);
            armTower.setColor(new Color8Bit(Color.kBlue));
        }
        //setting up variables
        desired = armConstants.armStowPosition;
        desiredName = "Stow";
        currentState = armEncoder.getDistance();

        // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
        Preferences.initDouble(armConstants.kArmPositionKey, desired);
        Preferences.initDouble(armConstants.kArmPKey, kP);
    }

    //getters
    public double getVoltage() { return voltage; }
    public double getSetpoint() { return desired; }
    public String getSetpointName() { return desiredName; }
    public double getState() {
        if(Robot.isSimulation()) {
            return armSim.getAngleRads();
        } else {
            return armEncoder.getDistance();
        }
    }
    public double getPidVoltage() { return pidVoltage; }
    public double getFeedForwardVoltage() { return feedForwardVoltage; }
    public double getRobotControllerBattery() { return RobotControllerBattery; }
    public double getArmMotorAppliedOutput() { return armMotorAppliedOutput; }

    public double getArmF (double des) {
        return feedForward_a.calculate(des, 0);
    }

    public void setVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }


    /**
     * Stops arm motor
     *
     * @return command that stops the arm motor
     */
    public Command stopRunning() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    armMotor.stopMotor();
                });
    }

    /**
     * goToShow sets desired position to arm stow position and
     * sets desiredName for logging
     *
     * @return command that sets desired name and desired position to stow position
     */
    public Command goToStow() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    if(Robot.isReal()) {
                        desired = armConstants.armStowPosition;
                    } else {
                        desired = armConstants.armSimStowPosition;
                    }
                    desiredName = "Stow";
                });
    }

    /**
     * goToHighScore sets desired position to arm highScore position and
     * sets desiredName for logging
     *
     * @return command that sets desired name and desired position to highScore position
     */
    public Command goToHighScore() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    if(Robot.isReal()) {
                        desired = armConstants.armHighScorePosition;
                    } else {
                        desired = armConstants.armSimHighPosition;
                    }
                    desiredName = "High Score";
                });
    }
/*
    public Command goToHalf() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    desired = armConstants.armHalfPosition;
                    desiredName = "Half";
                });
    }
 */
    /**
     * goToIntake sets desired position to arm intake position and
     * sets desired name for logging
     *
     * @return command that sets desired name and desired position to intake position
     */
    public Command goToIntake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    if(Robot.isReal()) {
                        desired = armConstants.armIntakePosition;
                    } else {
                        desired = armConstants.armSimIntakePosition;
                    }
                    desiredName = "Intake";
                });
    }

    /**
     * loadPreferences sets desired and kp
     * Sets pid kp to calculated kp from preferences
     *
     * */
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        desired = Preferences.getDouble(armConstants.kArmPositionKey, desired);
        if (kP != Preferences.getDouble(armConstants.kArmPKey, kP)) {
            kP = Preferences.getDouble(armConstants.kArmPKey, kP);
            pid.setP(kP);
        }
    }


    /**
     * Stops arm motor; used in robot line 77 to stop arm subsystem because
     * the other stop motor function in this file returns a command
     */
    public void stop() {
        armMotor.stopMotor();
    }

    /**
     * Closes the robot
     * Used to shut down the robot (called on disableInit in Robot file)
     */
    public void close() {
        armMotor.close();
        mech2d.close();
        armPivot.close();
        pid.close();
        armLigament.close();
    }

    /**
     * Checks if the arm is done moving; used in autos
     *
     * @return a BooleanSupplier for if the arm is done moving
     */
    public BooleanSupplier isDone(){
        BooleanSupplier finished = () -> pid.atSetpoint();
        return finished;
    }

    /**
     * Sets smart dashboard values for logging
     *
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Sim Voltage");
        builder.publishConstString("1.0", "Logging stuff");
        builder.addDoubleProperty("1.1 position", this::getState, null);
        builder.addDoubleProperty( "1.2 setpoint", this::getSetpoint, null);
        builder.addStringProperty("1.3 setpoint name", this::getSetpointName, null);
        builder.addDoubleProperty("1.4 voltage", this::getVoltage, null);
        builder.addDoubleProperty("1.5 internal motor distance", () -> this.armMotor.getEncoder().getPosition(), null);
    }

    /**
     * Sets the voltage for the arm motor from pid calculation
     */
    @Override
    public void periodic() {
        currentState = getState();
        pidVoltage = pid.calculate(currentState, desired);
        feedForwardVoltage = getArmF(desired);
        voltage = pidVoltage + feedForwardVoltage;
        setVoltage(voltage);
    }

    @Override
    /**
     * Run the control loop to reach and maintain the setpoint from preferences.
     * */
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        pidVoltage = pid.calculate(getState(), desired);
        feedForwardVoltage = getArmF(desired);
        double motorVoltage = armConstants.armSimGrav ? pidVoltage + feedForwardVoltage : pidVoltage;
        armMotor.setVoltage(motorVoltage);

        armMotorAppliedOutput = armMotor.getAppliedOutput();
        RobotControllerBattery = RobotController.getBatteryVoltage();
        voltage = armMotorAppliedOutput * RobotControllerBattery;

        //setting voltage input
        armSim.setInput(voltage);

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }

    public void print(Object o) {
        System.out.println(o.toString());
    }

}