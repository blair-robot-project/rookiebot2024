

package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class driveConstants {
    public static final double MAX_SPEED = 4.0; // can be changed
    public static final double MAX_ANGULAR_SPEED = Math.PI; // can be changed, 3/2 rotations per second (in radians, so pi radians is 1/2 of a rotation)

    public static final double WHEEL_RADIUS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI; // placeholder
    // idk if we'll need these ^^^
    //location of the wheels relative to the center of the robot
    public static final double moduleDistanceX = 0.3495;
    public static final double moduleDistanceY = 0.324;
    public static final double moduleDistanceDiagonal=Math.sqrt(Math.pow(moduleDistanceX,2)+Math.pow(moduleDistanceY,2));

    //PID calculator values for swerve

    public static final double drivePIDkp = 0.475; // placeholder
    public static final double drivePIDki = 0.0; // placeholder
    public static final double drivePIDkd = 0.0; // placeholder

    public static final double turnPIDkp = 10.0; // placeholder
    public static final double turnPIDki = 0.0; // placeholder
    public static final double turnPIDkd = 0.0; // placeholder

    //feedForward calc values for swerve
    public static final double swerveFeedForwardDriveKs = 0; // placeholder
    public static final double swerveFeedForwardDriveKv = 2.3887 + 0.2 + 0.0935; // placeholder
    public static final double swerveFeedForwardDriveKa = 0.43365 + 0.035 + 0.0185; // placeholder

    public static final double swerveFeedForwardTurnKs = 0; // placeholder
    public static final double swerveFeedForwardTurnKv = 0; // placeholder
    public static final double swerveFeedForwardTurnKa = 0; // placeholder

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
    public static final int turnEncoderChannel4 = 0;

    public static final boolean driveMotor1Inverted = true;
    public static final boolean turnMotor1Inverted = true;
    public static final boolean turnEncoder1Inverted = false;

    public static final boolean driveMotor2Inverted = true;
    public static final boolean turnMotor2Inverted = true;
    public static final boolean turnEncoder2Inverted = false;

    public static final boolean driveMotor3Inverted = true;
    public static final boolean turnMotor3Inverted = true;
    public static final boolean turnEncoder3Inverted = false;

    public static final boolean driveMotor4Inverted = true;
    public static final boolean turnMotor4Inverted = true;
    public static final boolean turnEncoder4Inverted = false;

    public static final double FLturnOffset = 1.159170; // placeholder
    public static final double FRturnOffset = 2.912143; // placeholder
    public static final double BLturnOffset = 2.301405; // placeholder
    public static final double BRturnOffset = -2.377702; // placeholder


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
    public static final double xShift=0.0;
    public static final Pose2d robotInitialPose = new Pose2d(0.0,0.0, Rotation2d.fromDegrees(0));


}