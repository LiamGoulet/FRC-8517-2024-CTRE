// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class TargetAngle {
    // BEGIN Class Data
    private Rotation2d m_targetAngle;
    private double m_actualAngle;
    private double m_hyp;
    private double m_discreteAngle = 45;
    
    // END  Class Data

    // BEGIN Class Constructors 
    public TargetAngle(){

    }
    public TargetAngle(double _ang){
      m_discreteAngle = _ang;
      m_targetAngle = new Rotation2d(0);
    }

    // END Class Constructors

    // BEGIN Class Methods
    /** Set the target angle by giving a x and y value and creating a Rotation 2D object.
     * A Rotation2d object is just a fancy class for holding an angle.
     * 
     * @param _x The x value that usually comes from the joystick
     * @param _y The y value that usually comes from the joystick
     */
    public Rotation2d setTargetAngle(double _x, double _y){
        double midAngle = m_discreteAngle;
        m_hyp = Math.hypot(_x, _y);
        if(Math.abs(m_hyp) > 0.8){
          m_actualAngle = Math.toDegrees(Math.atan2(_y,_x));
          if(m_actualAngle >= -22.5 && m_actualAngle <= 22.5){
            m_targetAngle = new Rotation2d(Math.toRadians(0));
          }else if(m_actualAngle >= -67.5 && m_actualAngle < -22.5){
            m_targetAngle = new Rotation2d(Math.toRadians(-midAngle));
          }else if(m_actualAngle >= -112.5 && m_actualAngle < -67.5){
            m_targetAngle = new Rotation2d(Math.toRadians(-90));
          }else if(m_actualAngle >= -157.5 && m_actualAngle < -112.5){
            m_targetAngle = new Rotation2d(Math.toRadians(-(180-midAngle)));
          }else if((m_actualAngle >= 157.5 && m_actualAngle <= 180.0) || (m_actualAngle <= -157.5 && m_actualAngle > -179.99)){
            m_targetAngle = new Rotation2d(Math.toRadians((180)));
          }else if(m_actualAngle <= 67.5 && m_actualAngle > 22.5){
            m_targetAngle = new Rotation2d(Math.toRadians(midAngle));
          }else if(m_actualAngle <= 112.5 && m_actualAngle > 67.5){
            m_targetAngle = new Rotation2d(Math.toRadians(90));
          }else if(m_actualAngle <= 157.5 && m_actualAngle > 112.5){
            m_targetAngle = new Rotation2d(Math.toRadians((180-midAngle)));
          }else {
            m_targetAngle = new Rotation2d();
          }
        }
        return m_targetAngle;
    }
    public void setTargetAngle(double _angle){
      m_targetAngle = new Rotation2d(Math.toRadians(_angle));
    }
    public Rotation2d getTargetAngle(){
        return m_targetAngle;
    }
    public double getActualAngle(){
        return m_actualAngle;
    }
    public double getHyp(){
        return m_hyp;
    }
    // END Class Constrcutors
}
