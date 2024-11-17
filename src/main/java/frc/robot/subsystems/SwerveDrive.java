package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.allConstants.driveConstants;
import com.kauailabs.navx.frc.AHRS;


public class SwerveDrive extends SubsystemBase {
    // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)
    // There should probably be a constant for these distance values otherwise it could be confusing.
    //the initial position of the four wheels and

    private final Translation2d frontLeftLocation = new Translation2d(driveConstants.moduleDistanceX, driveConstants.moduleDistanceY);
    private final Translation2d frontRightLocation = new Translation2d(-1*driveConstants.moduleDistanceX,driveConstants.moduleDistanceY);
    private final Translation2d backLeftLocation = new Translation2d(driveConstants.moduleDistanceX, -1*driveConstants.moduleDistanceY);
    private final Translation2d backRightLocation = new Translation2d(-1*driveConstants.moduleDistanceX,-1*driveConstants.moduleDistanceY);

    public final SwerveModule frontLeft = new SwerveModule(driveConstants.driveMotor1, driveConstants.turnMotor1);
    private final SwerveModule frontRight = new SwerveModule(driveConstants.driveMotor2,driveConstants.turnMotor2);
    private final SwerveModule backLeft = new SwerveModule(driveConstants.driveMotor3,driveConstants.turnMotor3);
    private final SwerveModule backRight = new SwerveModule(driveConstants.driveMotor4,driveConstants.turnMotor4);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

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

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public Rotation2d gyroAngle(){
        return new Rotation2d(gyro.getAngle());
    }
    public SwerveModulePosition[] positions(){
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    // the current gyro angle, an array of the current module positions
    // (as in the constructor and update method), and the new field-relative pose
    public void resetPose(){
        odometry.resetPosition(gyroAngle(),positions(),getPose());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public ChassisSpeeds getCurrentSpeeds(double xSpeed,double ySpeed,double rot, boolean fieldRelative, double periodSeconds){
        var chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        if (fieldRelative) {
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d());
        }
        // forward, sideways, angular, period
        ChassisSpeeds.discretize(xSpeed,ySpeed,rot,periodSeconds);
        return chassisSpeeds;
    }
    //joystick info stuff
    public void drive(
            double xSpeed,double ySpeed,double rot, boolean fieldRelative, double periodSeconds
    ){
        SwerveDriveWheelStates swerveModuleStates = kinematics.toWheelSpeeds(getCurrentSpeeds(xSpeed, ySpeed, rot,
        fieldRelative, periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates.states, driveConstants.MAX_SPEED);
        frontLeft.SetDesired(swerveModuleStates.states[0]);
        frontRight.SetDesired(swerveModuleStates.states[1]);
        backLeft.SetDesired(swerveModuleStates.states[2]);
        backRight.SetDesired(swerveModuleStates.states[3]);
    }

    public boolean isRed() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public void updateOdometry(){
        //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });
    }


}
