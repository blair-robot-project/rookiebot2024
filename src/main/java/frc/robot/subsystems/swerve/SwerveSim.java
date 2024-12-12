package frc.robot.subsystems.swerve;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.lang.reflect.Field;

import static frc.robot.subsystems.swerve.driveConstants.*;


public class SwerveSim extends SwerveDrive {
    public final SimpleMotorFeedforward simfeedForward_d = new SimpleMotorFeedforward(swerveFeedForwardDriveKs, swerveFeedForwardDriveKv, swerveFeedForwardDriveKa);
    public final SimpleMotorFeedforward simfeedForward_t = new SimpleMotorFeedforward(swerveFeedForwardTurnKs, swerveFeedForwardTurnKv, swerveFeedForwardTurnKa);

    public final double simcurrentPos = 0;
    public boolean FieldOriented;
    public double gyroSim;
    SwerveDriveKinematics kinematics;
    Field2d field2d;
    double trackWidth;
    double wheelBase;
    double xShift;
    double maxRotSpeed;
    double maxLinearSpeed;
    SwerveModulePosition positions;
    public SwerveDrivePoseEstimator poseEstimator;
     public Field2d field;
    Pose2d pose2d;
    Rotation2d currHeading;
    Pose2d odoPose;
    ChassisSpeeds desiredSpeeds;
    double currTime;
    double lasTime;
    SwerveDriveOdometry odometry;
    public SendableBuilder builder;



    // private AnalogGyro gyro = new AnalogGyro(1); // 1 is a filler value, not yet sure what to put into the Analog Gyro
    // private AnalogGyroSim simGyro = new AnalogGyroSim(gyro);
    public SwerveSim() {


        lasTime = Timer.getFPGATimestamp();
        Pose2d odoPose = new Pose2d();
        Rotation2d currHeading = new Rotation2d();
        field2d = new Field2d();
        desiredSpeeds = super.desiredSpeeds;
        currTime = Timer.getFPGATimestamp();
        odometry =
                new SwerveDriveOdometry(
                            kinematics,
                            currHeading,
                            super.positions(),
                            robotInitialPose
                    );

        SmartDashboard.putData(field2d);


        }



        public void snapToDesired (SwerveModuleState desiredState){


        }

        @Override
        protected void setRobotPose () {
            super.setRobotPose();
        }



        public void initSendable (SendableBuilder builder){
            builder.setSmartDashboardType("Swerve Sim");
        }


        public void resSetPose (Pose2d value){
            this.odometry.resetPosition(
                    currHeading,
                    positions(),
                    value
            );
        }


        @Override
        public void periodic () {


            currentSpeeds = super.currentSpeeds;

            this.odometry.update(
                    currHeading,
                    positions()
            );

            odoPose = odometry.update(
                    currHeading,
                    positions()
            );

            setRobotPose();
            field2d.getObject("Odo Pose").setPose(odoPose);

            this.lasTime = this.currTime;


        }

}

