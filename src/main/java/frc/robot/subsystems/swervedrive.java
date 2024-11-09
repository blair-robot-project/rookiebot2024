package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.AnalogGyro;

public class swervedrive {
    public static final double kMaxSpeed=3.0; // 3 m/s
    public static final double kMaxAngularSpeed=Math.PI; // 1/2 rotation per second (in radians, so pi radians is 1/2 of a rotation)
    // There should probably be a constant for these distance values otherwise it could be confusing.
    //the initial position of the four wheels and
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public final swervemodule m_frontLeft = new swervemodule(1, 2);
    private final swervemodule m_frontRight = new swervemodule(1,2);
    private final swervemodule m_backLeft = new swervemodule(1,2);
    private final swervemodule m_backRight = new swervemodule(1,2);

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
            chassisSpeeds.toRobotRelativeSpeeds(m_gyro.getRotation2d());
        }
        chassisSpeeds.discretize(periodSeconds);
        var swerveModuleStates = m_kinematics.toWheelSpeeds(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        SwerveDriveWheelStates swerveModuleStates = m_kinematics.toWheelSpeeds(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates.states, kMaxSpeed);
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

}
