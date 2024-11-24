package frc.robot.allConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class armConstants {
    //motor id
    public static final int armMotorID = 1;
    //motor follower id
    public static final int armMotorFollowerID = 2;
    //motor kp, ki, kd
    public static final double armKP = 0.1, armKI = 0, armKD = 0;
    //default desired arm value
    public static final double armDefaultDesiredValue = 0.25;
    //default base arm value
    public static final double armDefaultBaseValue = 0;
    //gear ratio
    public static final double armGearRatio = 1;
    //desired arm value
    public static final double armDesiredValue = 0.25;
    //base arm value
    public static final double armBaseValue = 0;

    //VALUES NEEDED FOR ARM SIM
    //ALL VALUES BELOW ARE FAKE AND TEMPORARY

    //arm dcmotor for arm sim
    public static DCMotor armGearbox = DCMotor.getNEO(1);
    //gearing
    public static double armGearing = 1;
    //moment inertia of the arm NEED FROM CAD (jkg per meters squared)
    public static double armInertia = 200;
    //arm length (units?)
    public static double armLength = Units.inchesToMeters(30);
    //minAngle (radians)
    public static double minAngleRads = Units.degreesToRadians(-60);
    //maxAngle (rads)
    public static double maxAngleRads = Units.degreesToRadians(120);
    //whether or not to simualte gravity
    public static boolean armSimGrav = true;

    //Encoder Channels
    public static int encoderAChannel = 0;
    public static int encoderBChannel = 1;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    public static double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";
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
}
