// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.lib.Directions;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem{
  private TalonFX m_shooterLeft = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT);
  private TalonFX m_shooterRight = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT);
  private CANSparkMax m_shooterRotate = new CANSparkMax(k.ROBORIO_CAN_IDS.SHOOTER_ROTATE, MotorType.kBrushless);

  private CANSparkMax m_intakeSpin = new CANSparkMax(k.ROBORIO_CAN_IDS.INTAKE_SPIN, MotorType.kBrushless);
  private CANSparkMax m_intakeRotate = new CANSparkMax(k.ROBORIO_CAN_IDS.INTAKE_ROTATE, MotorType.kBrushless);

  private double m_intakeAngle = 0;
  private double m_shooterAngle = 0;
  private Directions m_currentIntakeLocation = Directions.IN;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /** The intake will go up or down.
   * When it goes up it is dependent on the angle of the shooter so it can be used as a 
   * shooter loader if needed or just be out of the way.
   * 
   * @param _angle
   */
  public void setIntakeLocation(Directions _location){
    m_currentIntakeLocation = _location;
    switch (_location) {
      case UP: // To shoot at the Speaker
        break;
      case DOWN: // Same as OUT
      case OUT: // Go to the Floor
        break;
      case IN: // Move into frame perimeter but deal with current shooter angle.
        break;
      default:
        
        break;

    }
  }
  public void setIntakeSpeed(double _speed){

  }
  public void setShooterAngle(double _angle){

  }
  public void setShooterSpeed(double _speed){

  }
  @Override
  public void updateDashboard() {
    
  }
}
