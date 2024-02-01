// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class ShotCommand extends InstantCommand {
  SwerveDrivetrainSubsystem m_drive;
  ShooterSubsystem m_shooter;
  double m_angleDrive;
  double m_angleShooter;
  public ShotCommand(SwerveDrivetrainSubsystem _drive, ShooterSubsystem _shooter, double _angleDrive, double _angleShooter) {
    addRequirements(_drive, _shooter);
    m_drive = _drive;
    m_shooter = _shooter;
    m_angleDrive = _angleDrive;
    m_angleShooter = _angleShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setTargetAngle(m_angleDrive);
    m_shooter.setShooterAngle(m_angleShooter);
  }
}
