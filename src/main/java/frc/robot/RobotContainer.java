// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.Drive.DriveDefaultCommand;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.lib.k;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.SwerveTuningConstants;
import frc.robot.subsystems.DriveTelemetry;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static Set<ISubsystem> subsystems = new HashSet<>();
  public final SwerveDrivetrainSubsystem m_drivetrain = SwerveTuningConstants.DriveTrain;
  private final DriveTelemetry logger = new DriveTelemetry();

  public static final CommandPS5Controller s_driverController = new CommandPS5Controller(k.OI.DRIVER_CONTROLLER_PORT);
  //public static final CommandPS5Controller s_operatorController = new CommandPS5Controller(1);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Notifier m_telemetry;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

      // Add all autonomous command groups to the list on the Smartdashboard
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData("Autonomous Play",autoChooser);
    // Setup the dashboard notifier that runs at a slower rate than our main robot periodic.
    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);
  }
  private void updateDashboard(){
    SmartDashboard.putString("RobotMode", GD.G_RobotMode.toString());
    Iterator<ISubsystem> it = subsystems.iterator();
    while(it.hasNext()){
      it.next().updateDashboard(); // Comment this line out if you want ALL smartdashboard data to be stopped.
    }
  }
  private void configureBindings() {
    //driveFieldCentricFacingAngle.HeadingController.setPID(0,0,0);
    m_drivetrain.setDefaultCommand(new DriveDefaultCommand(m_drivetrain));

    // reset the field-centric heading on touchpad press
    s_driverController.touchpad().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));
    s_driverController.square().onTrue(new InstantCommand(m_drivetrain::changeDriveMode, m_drivetrain));
    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
