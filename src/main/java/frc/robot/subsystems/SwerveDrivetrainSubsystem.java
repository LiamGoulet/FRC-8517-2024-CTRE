// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class SwerveDrivetrainSubsystem extends SwerveDrivetrain implements Subsystem, ISubsystem {
  EDriveMode m_driveMode = EDriveMode.FIELD_CENTRIC;
  Rotation2d m_targetAngle = new Rotation2d();



  public SwerveDrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

  }

  public SwerveDrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public void changeDriveMode() {
    switch (m_driveMode) {
      case FIELD_CENTRIC_FACINGANGLE:
        m_driveMode = EDriveMode.FIELD_CENTRIC;
        break;
      case FIELD_CENTRIC:
        m_driveMode = EDriveMode.ROBOT_CENTRIC;
        break;
      case ROBOT_CENTRIC:
        m_driveMode = EDriveMode.FIELD_CENTRIC_FACINGANGLE;
        break;
      default:
        break;
    }
  }
  public EDriveMode getDriveMode() {
    return m_driveMode;
  }
  public Rotation2d getDriveTargetAngle(double _x, double _y){
    double midAngle = k.DRIVEBASE.ROTATE_DISCRETE_ANGLE_deg;
    if(_x > 0.8 && _y > 0.8){
      double ang = Math.toDegrees(Math.atan2(_y,_x));
      if(ang >= -22.5 && ang <= 22.5){
        m_targetAngle = new Rotation2d(Math.toRadians(0));
      }else if(ang >= -67.5 && ang < -22.5){
        m_targetAngle = new Rotation2d(Math.toRadians(-midAngle));
      }else if(ang >= -112.5 && ang < -67.5){
        m_targetAngle = new Rotation2d(Math.toRadians(-90));
      }else if(ang >= -157.5 && ang < -112.5){
        m_targetAngle = new Rotation2d(Math.toRadians(-(180-midAngle)));
      }else if((ang >= 157.5 && ang <= 180.0) || (ang <= -157.5 && ang > -179.99)){
        m_targetAngle = new Rotation2d(Math.toRadians((180)));
      }else if(ang >= 67.5 && ang < 22.5){
        m_targetAngle = new Rotation2d(Math.toRadians(midAngle));
      }else if(ang >= 112.5 && ang < 67.5){
        m_targetAngle = new Rotation2d(Math.toRadians(90));
      }else if(ang >= 157.5 && ang < 112.5){
        m_targetAngle = new Rotation2d(Math.toRadians((180-midAngle)));
      }
    }
    return m_targetAngle;
  }
  /** Update the dashboard with Drivetrain information.
   *  The robot Pose is handled in the telemetry and odometry of the base class
   * 
   */
  public void updateDashboard(){
    SmartDashboard.putString(k.DRIVE.T_DRIVER_MODE, m_driveMode.toString());
    SmartDashboard.putNumber("Drive Target Angle", m_targetAngle.getDegrees());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
