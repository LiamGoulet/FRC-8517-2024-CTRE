// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Directions;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem{
  // private TalonFX m_shooterLeft = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT);
  // private TalonFX m_shooterRight = new TalonFX(k.ROBORIO_CAN_IDS.SHOOTER_LEFT);
  // private CANSparkMax m_shooterRotate = new CANSparkMax(k.ROBORIO_CAN_IDS.SHOOTER_ROTATE, MotorType.kBrushless);

  // private CANSparkMax m_intakeSpin = new CANSparkMax(k.ROBORIO_CAN_IDS.INTAKE_SPIN, MotorType.kBrushless);
  // private CANSparkMax m_intakeRotate = new CANSparkMax(k.ROBORIO_CAN_IDS.INTAKE_ROTATE, MotorType.kBrushless);

  private double m_intakeAngle = 0;
  ArmFeedforward m_intakeFF = new ArmFeedforward(.5, 1, 0);
  PIDController m_intakeRotatePID = new PIDController(1, 0, 0);

  private double m_shooterAngle = 0;
  private double m_requestedShooterAngle = 0;
  ArmFeedforward m_shooterFF = new ArmFeedforward(.5, 1, 0);
  PIDController m_shooterRotatePID = new PIDController(1, 0, 0);
  Follower m_shooterRightFollower = new Follower(k.ROBORIO_CAN_IDS.SHOOTER_RIGHT, true);  

  private Directions m_currentIntakeLocation = Directions.IN;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //m_shooterRight.setControl(m_shooterRightFollower);
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
      // Assuming UP does not interfere with the shooter if the shooter was all the way up.
      setIntakeAngle(k.INTAKE.UP_ANGLE);
        break;
      case DOWN: // Same as OUT
        
      case OUT: // Go to the Floor
        setIntakeAngle(k.INTAKE.DOWN_ANGLE);
        break;
      case IN: // Move into frame perimeter but deal with current shooter angle.
        // TODO calibrate these numbers and fix calculations
        setIntakeAngle(k.INTAKE.IN_ANGLE - m_shooterAngle- k.INTAKE.IN_ANGLE_OFFSET);
        break;
      default:
        
        break;

    }
  }
  public void setIntakeSpeed(double _speed){
   // m_intakeSpin.set(_speed);
  }
  public void setIntakeAngle(double _angle){
    // TODO: copy from setShooterAngle when it is tested
    // double intakeAngle = m_intakeRotate.getEncoder().getPosition() * k.INTAKE.ROTATE_DEG_PER_REV;
    // m_intakeAngle = intakeAngle;
  }
  public double getShooterRequestedAngle(){
    return m_requestedShooterAngle;
  }
  public void setShooterAngle(double _angle){
    m_requestedShooterAngle = _angle;
    // double shooterAngle = m_shooterRotate.getEncoder().getPosition() * k.SHOOTER.ROTATE_DEG_PER_REV; 
    // m_shooterAngle = shooterAngle;

    // double shooterPID = m_shooterRotatePID.calculate(shooterAngle,_angle);

    // shooterPID *= Math.toRadians(shooterPID) * 0.1; // Fudge factor to get rad to rad/sec as velocity.
    // shooterPID = MathUtil.clamp(shooterPID, -1, 1);

    // double volts = m_shooterFF.calculate(shooterAngle,shooterPID); // Velocity is Radians/Sec Kv needs to be Volts*Sec/Rad
    // m_shooterRotate.setVoltage(volts);

  }
  public void setShooterSpeed(double _speed){
    // m_shooterLeft.setVoltage(_speed);
    // m_shooterRight.setControl(m_shooterRightFollower);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Shooter Requested Angle", m_requestedShooterAngle);
    SmartDashboard.putNumber("Intake Angle", m_intakeAngle);
  }
}
