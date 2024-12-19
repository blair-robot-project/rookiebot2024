package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.swerve.driveConstants.*;


public class SwerveDrive extends SubsystemBase {
    // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)
    // There should probably be a constant for these distance values otherwise it could be confusing.
    //the initial position of the four wheels and

    private final Translation2d frontLeftLocation = new Translation2d(driveConstants.moduleDistanceX, driveConstants.moduleDistanceY);
    private final Translation2d frontRightLocation = new Translation2d(driveConstants.moduleDistanceX,-1*driveConstants.moduleDistanceY);
    private final Translation2d backLeftLocation = new Translation2d(-1*driveConstants.moduleDistanceX, driveConstants.moduleDistanceY);
    private final Translation2d backRightLocation = new Translation2d(-1*driveConstants.moduleDistanceX,-1*driveConstants.moduleDistanceY);

    public final double wheelBase=moduleDistanceX*2;
    public final double trackWidth=moduleDistanceY*2;
    Pose2d robotInitialPose = driveConstants.robotInitialPose;
    private final Field2d field=new Field2d();
    private SwerveDrivePoseEstimator poseEstimator;


    public final SwerveModule frontLeft = new SwerveModule(driveConstants.driveMotor1, driveConstants.turnMotor1, driveConstants.turnEncoderChannel1,driveConstants.driveMotor1Inverted,driveConstants.turnMotor1Inverted,driveConstants.turnEncoder1Inverted,driveConstants.FLturnOffset);
    private final SwerveModule frontRight = new SwerveModule(driveConstants.driveMotor2,driveConstants.turnMotor2, driveConstants.turnEncoderChannel2,driveConstants.driveMotor2Inverted,driveConstants.turnMotor2Inverted,driveConstants.turnEncoder2Inverted,driveConstants.FRturnOffset);
    private final SwerveModule backLeft = new SwerveModule(driveConstants.driveMotor3,driveConstants.turnMotor3, driveConstants.turnEncoderChannel3,driveConstants.driveMotor3Inverted,driveConstants.turnMotor3Inverted,driveConstants.turnEncoder3Inverted,driveConstants.BLturnOffset);
    private final SwerveModule backRight = new SwerveModule(driveConstants.driveMotor4,driveConstants.turnMotor4, driveConstants.turnEncoderChannel4,driveConstants.driveMotor4Inverted,driveConstants.turnMotor4Inverted,driveConstants.turnEncoder4Inverted,driveConstants.BRturnOffset);
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();




    public final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


    Pose2d pose;

    private final SwerveDriveOdometry odometry =
            new SwerveDriveOdometry(
                    kinematics,
                    gyroAngle(),
                    new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                    });
    private SwerveDrive swerveDrive;

    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }



    public SwerveDrive() {

        gyro.reset();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                gyroAngle(),
                positions(),
                robotInitialPose
        );
        List<SwerveModule> modules = new ArrayList<>();
        modules.add(frontLeft);
        modules.add(frontRight);
        modules.add(backLeft);
        modules.add(backRight);

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

                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );


    }

    public Rotation2d gyroAngle(){
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose(){
        return this.poseEstimator.getEstimatedPosition();
    }

    public void SetPose(Pose2d value){
        this.poseEstimator.resetPosition(
                gyroAngle(),
                positions(),
                value
        );
    }

    public void resetPoseGiven(Pose2d pose) {
        odometry.resetPosition(gyroAngle(),positions(), pose);
        gyro.reset();
    }


    //joystick info stuff
    public void drive(
            double forwards, double sideways, double rot,
            boolean fieldRelative, double periodSeconds
    ) {

        desiredSpeeds = getSetSpeeds(forwards, sideways, rot, fieldRelative, periodSeconds);
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, driveConstants.MAX_SPEED);
        frontLeft.SetDesired(swerveModuleStates[0]);
        frontRight.SetDesired(swerveModuleStates[1]);
        backLeft.SetDesired(swerveModuleStates[2]);
        backRight.SetDesired(swerveModuleStates[3]);

    }

    public void updateOdometry(){
        odometry.update(gyroAngle(), positions());
    }

    public ChassisSpeeds getSetSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds){

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyroAngle());
        }
        else {
            chassisSpeeds=ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, gyroAngle());
            DriverStation.reportError("Robot is Robot Relative ", true);
        }

        // forward, sideways, angular, period
        // chassisSpeeds = ChassisSpeeds.discretize(xSpeed,ySpeed,rot,periodSeconds);
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

    public void stopMotors() {
        frontLeft.driveMotor.stopMotor();
        frontLeft.turnMotor.stopMotor();
        frontRight.driveMotor.stopMotor();
        frontRight.turnMotor.stopMotor();
        backLeft.driveMotor.stopMotor();
        backLeft.turnMotor.stopMotor();
        backRight.driveMotor.stopMotor();
        backRight.turnMotor.stopMotor();

    }

    public void periodic() {
        updateOdometry();

        currentSpeeds = kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );


        if(poseEstimator!=null){this.poseEstimator.update(
                gyroAngle(),
                positions()
        );}
        else{
            DriverStation.reportError("poseEstimator is null",false);
        }

        setRobotPose();
    }


    protected void setRobotPose(){
        if (this.pose!=null) {
            this.field.setRobotPose(this.pose);

            Pose2d frontLeftPose = this.pose.plus(new Transform2d(
                    new Translation2d(wheelBase / 2 - xShift, trackWidth / 2), positions()[1].angle));
            this.field.getObject("FrontLeft").setPose(frontLeftPose);

            Pose2d frontRightPose = this.pose.plus(new Transform2d(new Translation2d(
                    wheelBase / 2 - xShift, -trackWidth / 2), positions()[1].angle));
            this.field.getObject("frontRightPose").setPose(frontRightPose);

            Pose2d backLeftPose = this.pose.plus(new Transform2d(new Translation2d(
                    -wheelBase / 2 - xShift, trackWidth / 2), positions()[2].angle));
            this.field.getObject("backLeftPose").setPose(backLeftPose);

            Pose2d backRightPose = this.pose.plus(new Transform2d(new Translation2d(
                    -wheelBase / 2 - xShift, -trackWidth / 2), positions()[3].angle));
            this.field.getObject("backRightPose").setPose(backRightPose);
        }
        else{
            return;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder){

        builder.setSmartDashboardType("swerve");

        builder.addDoubleProperty("speed x",() -> desiredSpeeds.vxMetersPerSecond,null);
        builder.addDoubleProperty("speed y",() -> desiredSpeeds.vyMetersPerSecond,null);
        builder.addDoubleProperty("speed rot",() -> desiredSpeeds.omegaRadiansPerSecond,null);

        builder.addDoubleProperty("frontLeftVoltage", frontLeft::getVoltage,null);
        builder.addDoubleProperty("frontRightVoltage", frontRight::getVoltage,null);
        builder.addDoubleProperty("backLeftVoltage", backLeft::getVoltage,null);
        builder.addDoubleProperty("backRightVoltage", backRight::getVoltage,null);

        builder.addDoubleProperty("set drive speed front left", frontLeft::getDesiredSpeed, null);
        builder.addDoubleProperty("set drive speed front right", frontRight::getDesiredSpeed, null);
        builder.addDoubleProperty("set drive speed back left", backLeft::getDesiredSpeed, null);
        builder.addDoubleProperty("set drive speed back right", backRight::getDesiredSpeed, null);

        builder.addDoubleProperty("set turn angle front left", frontLeft::getDesiredAngle, null);
        builder.addDoubleProperty("set turn angle front right", frontRight::getDesiredAngle, null);
        builder.addDoubleProperty("set turn angle back left", backLeft::getDesiredAngle, null);
        builder.addDoubleProperty("set turn angle back right", backRight::getDesiredAngle, null);
        builder.addDoubleProperty("frontleft Heading", frontLeft::getTurnPosition,null);
        builder.addDoubleProperty("frontRight Heading", frontRight::getTurnPosition,null);
        builder.addDoubleProperty("BackLeft Heading", backLeft::getTurnPosition,null);
        builder.addDoubleProperty("BackRight Heading",backRight::getTurnPosition,null);
        builder.addDoubleProperty("frontleft velocity",frontLeft::getVelocity,null);
        builder.addDoubleProperty("frontRight velocity",frontRight::getVelocity,null);
        builder.addDoubleProperty("backLeft velocity",backLeft::getVelocity,null);
        builder.addDoubleProperty("backRight velocity",backRight::getVelocity,null);

        builder.addDoubleProperty("set frontleft heading",frontLeft::getDesiredAngle,null);
        builder.addDoubleProperty("set frontRight heading",frontRight::getDesiredAngle,null);
        builder.addDoubleProperty("set backLeft heading",backLeft::getDesiredAngle,null);
        builder.addDoubleProperty("set backRight heading",backRight::getDesiredAngle,null);

        builder.addDoubleProperty("gyro", () -> MathUtil.angleModulus(gyroAngle().getRadians()), null);

    }
}