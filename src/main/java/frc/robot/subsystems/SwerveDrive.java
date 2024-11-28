package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.allConstants.driveConstants;

import static frc.robot.allConstants.driveConstants.*;


public class SwerveDrive extends SubsystemBase {
    // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)
    // There should probably be a constant for these distance values otherwise it could be confusing.
    //the initial position of the four wheels and

    private final Translation2d frontLeftLocation = new Translation2d(driveConstants.moduleDistanceX, driveConstants.moduleDistanceY);
    private final Translation2d frontRightLocation = new Translation2d(driveConstants.moduleDistanceX,-1*driveConstants.moduleDistanceY);
    private final Translation2d backLeftLocation = new Translation2d(-1*driveConstants.moduleDistanceX, driveConstants.moduleDistanceY);
    private final Translation2d backRightLocation = new Translation2d(-1*driveConstants.moduleDistanceX,-1*driveConstants.moduleDistanceY);


    public final SwerveModule frontLeft = new SwerveModule(driveConstants.driveMotor1, driveConstants.turnMotor1, driveConstants.turnEncoderChannel1,driveConstants.driveMotor1Inverted,driveConstants.turnMotor1Inverted,driveConstants.turnEncoder1Inverted);
    private final SwerveModule frontRight = new SwerveModule(driveConstants.driveMotor2,driveConstants.turnMotor2, driveConstants.turnEncoderChannel2,driveConstants.driveMotor2Inverted,driveConstants.turnMotor2Inverted,driveConstants.turnEncoder2Inverted);
    private final SwerveModule backLeft = new SwerveModule(driveConstants.driveMotor3,driveConstants.turnMotor3, driveConstants.turnEncoderChannel3,driveConstants.driveMotor3Inverted,driveConstants.turnMotor3Inverted,driveConstants.turnEncoder3Inverted);
    private final SwerveModule backRight = new SwerveModule(driveConstants.driveMotor4,driveConstants.turnMotor4, driveConstants.turnEncoderChannel4,driveConstants.driveMotor4Inverted,driveConstants.turnMotor4Inverted,driveConstants.turnEncoder4Inverted);

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


    public SwerveDrive() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier

                this::resetPoseGiven, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(autoTranKp, autoTranKi, autoTranKd), // Translation PID constants
                        new PIDConstants(autoRotKp, autoRotKi, autoRotKd), // Rotation PID constants
                        driveConstants.MAX_SPEED, // Max module speed, in m/s
                        driveConstants.moduleDistanceDiagonal, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );


    }


    private final SwerveDriveOdometry odometry =
            new SwerveDriveOdometry(
                    kinematics,
                    gyro.getRotation2d(),
                    new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                    });

    public SwerveModulePosition[] positions(){
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public Rotation2d gyroAngle(){
        return new Rotation2d(gyro.getAngle());
    }
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetPoseGiven(Pose2d p) {
        odometry.resetPosition(gyroAngle(),positions(),p);
       /// gyro.reset();
    }

    //joystick info stuff
    public void drive(
            double xSpeed,double ySpeed,double rot, boolean fieldRelative, double periodSeconds
    ){
        desiredSpeeds = getSetSpeeds(xSpeed,ySpeed,rot, fieldRelative, periodSeconds);
        SwerveDriveWheelStates swerveModuleStates = kinematics.toWheelSpeeds(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates.states, driveConstants.MAX_SPEED);
        frontLeft.SetDesired(swerveModuleStates.states[0]);
        frontRight.SetDesired(swerveModuleStates.states[1]);
        backLeft.SetDesired(swerveModuleStates.states[2]);
        backRight.SetDesired(swerveModuleStates.states[3]);
    }

    public void updateOdometry(){
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });
    }
    public ChassisSpeeds getSetSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds){
        var chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        if (fieldRelative) {
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d());
        }
        // forward, sideways, angular, period
        ChassisSpeeds.discretize(xSpeed,ySpeed,rot,periodSeconds);
        return chassisSpeeds;
    }

    public ChassisSpeeds getSpeeds() {
        return currentSpeeds;
    }

    public void driveSpeeds(ChassisSpeeds givenSpeeds){
        desiredSpeeds = givenSpeeds;
        SwerveDriveWheelStates swerveModuleStates = kinematics.toWheelSpeeds(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates.states, driveConstants.MAX_SPEED);
        frontLeft.SetDesired(swerveModuleStates.states[0]);
        frontRight.SetDesired(swerveModuleStates.states[1]);
        backLeft.SetDesired(swerveModuleStates.states[2]);
        backRight.SetDesired(swerveModuleStates.states[3]);
    }

    public boolean isRed() { // returns true if alliance is red
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public void periodic(){
        updateOdometry();
        currentSpeeds=kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );
    }
















}