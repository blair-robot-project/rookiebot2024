

package frc.robot.subsystems.swerve;


public final class driveConstants {
    public static final double MAX_SPEED = 8.0; // can be changed
    public static final double MAX_ANGULAR_SPEED = 3*Math.PI; // can be changed, 3/2 rotations per second (in radians, so pi radians is 1/2 of a rotation)

    public static final double WHEEL_RADIUS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI; // placeholder
    // idk if we'll need these ^^^
    //location of the wheels relative to the center of the robot
    public static final double moduleDistanceX = 0.3495;
    public static final double moduleDistanceY = 0.324;
    public static final double moduleDistanceDiagonal=Math.sqrt(Math.pow(moduleDistanceX,2)+Math.pow(moduleDistanceY,2));

    //PID calculator values for swerve

    public static final double drivePIDkp = 0.25; // placeholder
    public static final double drivePIDki = 0.0; // placeholder
    public static final double drivePIDkd = 0.0; // placeholder

    public static final double turnPIDkp = 0.25; // placeholder
    public static final double turnPIDki = 0.0; // placeholder
    public static final double turnPIDkd = 0.0; // placeholder

    //feedForward calc values for swerve
    public static final double swerveFeedForwardDriveKs = 1; // placeholder
    public static final double swerveFeedForwardDriveKv = 1; // placeholder
    public static final double swerveFeedForwardDriveKa = 1; // placeholder

    public static final double swerveFeedForwardTurnKs = 1; // placeholder
    public static final double swerveFeedForwardTurnKv = 1; // placeholder
    public static final double swerveFeedForwardTurnKa = 1; // placeholder

    //8 motor ids, 4 encoder ids
    //front left
    public static final int driveMotor1 = 5;
    public static final int turnMotor1 = 6;
    public static final int turnEncoderChannel1 = 8;

//front right
    public static final int driveMotor2 = 7;
    public static final int turnMotor2 = 8;
    public static final int turnEncoderChannel2 = 6;
//back left
    public static final int driveMotor3 = 9;
    public static final int turnMotor3 = 10;
    public static final int turnEncoderChannel3 = 5;
//back right
    public static final int driveMotor4 = 11;
    public static final int turnMotor4 = 12;
    public static final int turnEncoderChannel4 = 7;

    public static final boolean driveMotor1Inverted = false;
    public static final boolean turnMotor1Inverted = false;
    public static final boolean turnEncoder1Inverted = false;

    public static final boolean driveMotor2Inverted = false;
    public static final boolean turnMotor2Inverted = false;
    public static final boolean turnEncoder2Inverted = false;

    public static final boolean driveMotor3Inverted = false;
    public static final boolean turnMotor3Inverted = false;
    public static final boolean turnEncoder3Inverted = false;

    public static final boolean driveMotor4Inverted = false;
    public static final boolean turnMotor4Inverted = false;
    public static final boolean turnEncoder4Inverted = false;

    public static final double FLturnOffset = 0.0; // placeholder
    public static final double FRturnOffset = 0.0; // placeholder
    public static final double BLturnOffset = 0.0; // placeholder
    public static final double BRturnOffset = 0.0; // placeholder


    //the interavals in which the motor loops
    public static final double pdsec = 0.02; // always the same I think

    //no matter the robot position forward is forward (fRel = field relative)
    public static final boolean fRel = true; // probably will stay this way


    public static final double driveGearing = 6.75; // This is 6.75 according to John J
    public static final double turnGearing = 10; // placeholder

    public static final double autoTranKp = 0.5; ///placeholder
    public static final double autoTranKi = 0.0;///placeholder
    public static final double autoTranKd = 0.0;///placeholder

    public static final double autoRotKp = 0.5; ///placeholder
    public static final double autoRotKi = 0.0;///placeholder
    public static final double autoRotKd = 0.0;///placeholder

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int TURN_CURRENT_LIMIT = 30;

}