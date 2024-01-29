// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.SwerveTuningConstants;

public class DriveDefaultCommand extends Command {
  SwerveDrivetrainSubsystem m_drive;
  private double MaxSpeed = SwerveTuningConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
   private final SwerveRequest.FieldCentric driveFieldCentricOpenLoop = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.FieldCentric driveFieldCentricVelocity = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric driving in velocity mode
  private final SwerveRequest.FieldCentricFacingAngle driveFieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric driving in velocity mode
  
  /** Creates a new DriveDefaultCommand. */
  public DriveDefaultCommand(SwerveDrivetrainSubsystem _drivetrain) {
    addRequirements(_drivetrain);
    m_drive = _drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhoenixPIDController rotatePID = new PhoenixPIDController(1.6,0.32,0); // Radians are used in calc.
    rotatePID.setTolerance(0.1);
    /** TODO this PID is internal to Control Request. The PID tuning values, tolerance and Max/Min integral are all that are available. 
     *  There is no way to control speed of the turn other than the P term. I wish we could limit the applied velocity.
     * 
    */
    driveFieldCentricFacingAngle.HeadingController = rotatePID;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    switch (m_drive.getDriveMode()) {
      case FIELD_CENTRIC:
        m_drive.setControl(driveFieldCentricOpenLoop
        .withVelocityX(MathUtil.applyDeadband(RobotContainer.s_driverController.getLeftY(),0.08, 1.0) * MaxSpeed)
        .withVelocityY(MathUtil.applyDeadband(RobotContainer.s_driverController.getLeftX(),0.08, 1.0) * MaxSpeed)
        .withRotationalRate(MathUtil.applyDeadband(RobotContainer.s_driverController.getRightX(),0.08, 1.0) * MaxAngularRate)
        );

        // m_drive.applyRequest(() -> driveFieldCentricOpenLoop
        // .withVelocityX(MathUtil.applyDeadband(-RobotContainer.s_driverController.getLeftY(),0.08, 1.0) * MaxSpeed)
        // .withVelocityY(MathUtil.applyDeadband(-RobotContainer.s_driverController.getLeftX(),0.08, 1.0) * MaxSpeed)
        // .withRotationalRate(MathUtil.applyDeadband(-RobotContainer.s_driverController.getRightX(),0.08, 1.0) * MaxAngularRate)
        // ).ignoringDisable(true);
        
        break;
      case FIELD_CENTRIC_FACINGANGLE:
        m_drive.setControl(driveFieldCentricFacingAngle
        .withVelocityX(MathUtil.applyDeadband(RobotContainer.s_driverController.getLeftY(),0.08, 1.0) * MaxSpeed)
        .withVelocityY(MathUtil.applyDeadband(RobotContainer.s_driverController.getLeftX(),0.08, 1.0) * MaxSpeed)
        .withTargetDirection(m_drive.getDriveTargetAngle(RobotContainer.s_driverController.getRightX(), RobotContainer.s_driverController.getRightY()))
        );
        break;
      case ROBOT_CENTRIC:
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
