package frc.robot.allConstants;

import edu.wpi.first.math.system.plant.DCMotor;

public class armConstants {
    //motor id
    public static final int armMotorID = 11;
    //motor follower id
    public static final int armMotorFollowerID = 12;
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

    //VALUES NEEDED FOR ARM SIM
    //ALL VALUES BELOW ARE FAKE AND TEMPORARY

    public static final double armBaseValue = 0;
    //gearbox
    public static final DCMotor armGearbox = null;
    //gearing
    public static double armGearing = 0;
    //moment inertia of the arm NEED FROM CAD
    public static double armInertia = 1;
    //arm length (units?)
    public static double armLength = 5;
    //minAngle (radians)
    public static double minAngleRads = 1;
    //maxAngle (rads)
    public static double maxAngleRads = 3;
    //whether or not to simualte gravity
    public static boolean armSimGrav = true;
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
