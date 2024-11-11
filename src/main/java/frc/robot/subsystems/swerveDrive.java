package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.allConstants.driveConstants;


public class swerveDrive extends SubsystemBase {
    // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)
    // There should probably be a constant for these distance values otherwise it could be confusing.
    //the initial position of the four wheels and

    private final Translation2d m_frontLeftLocation = new Translation2d(driveConstants.frontLeftLocationX, driveConstants.frontLeftLocationY);
    private final Translation2d m_frontRightLocation = new Translation2d(driveConstants.frontRightLocationX,driveConstants.frontRightLocationY);
    private final Translation2d m_backLeftLocation = new Translation2d(driveConstants.backLeftLocationX, driveConstants.backLeftLocationY);
    private final Translation2d m_backRightLocation = new Translation2d(driveConstants.backRightLocationX,driveConstants.backRightLocationY);

    public final SwerveModule m_frontLeft = new SwerveModule(driveConstants.driveMotor1, driveConstants.turnMotor1);
    private final SwerveModule m_frontRight = new SwerveModule(driveConstants.driveMotor2,driveConstants.turnMotor2);
    private final SwerveModule m_backLeft = new SwerveModule(driveConstants.driveMotor3,driveConstants.turnMotor3);
    private final SwerveModule m_backRight = new SwerveModule(driveConstants.driveMotor4,driveConstants.turnMotor4);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
            new SwerveDriveOdometry(
                    m_kinematics,
                    m_gyro.getRotation2d(),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_backLeft.getPosition(),
                            m_backRight.getPosition()
                    });

    public void Drivetrain() {
        m_gyro.reset();
    }
    //joystick info stuff
    public void drive(
            double xSpeed,double ySpeed,double rot, boolean fieldRelative, double periodSeconds
    ){

        var chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
        if (fieldRelative) {
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,m_gyro.getRotation2d());
        }
        // forward, sideways, angular, period
        ChassisSpeeds.discretize(xSpeed,ySpeed,rot,periodSeconds);
        SwerveDriveWheelStates swerveModuleStates = m_kinematics.toWheelSpeeds(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates.states, driveConstants.kMaxSpeed);
        m_frontLeft.SetDesired(swerveModuleStates.states[0]);
        m_frontRight.SetDesired(swerveModuleStates.states[1]);
        m_backLeft.SetDesired(swerveModuleStates.states[2]);
        m_backRight.SetDesired(swerveModuleStates.states[3]);
    }

    public void updateOdometry(){
        m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        });
    }
    public void periodic(){
        updateOdometry();
    }
}
