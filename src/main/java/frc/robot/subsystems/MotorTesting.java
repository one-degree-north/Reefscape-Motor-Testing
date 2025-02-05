// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

@Logged
public class MotorTesting extends SubsystemBase {
  private TalonFX m_Master;
  private TalonFX m_Slave;
  private TalonFX m_motorIndependent;
  private VoltageOut voltageOut = new VoltageOut(0);
  private double voltageInput = 0.5; //CHANGE THIS TO WHATEVER VOLTAGE YOU WANT TO TEST (MUST BE POSITIVE)
  
  public MotorTesting() {
    m_Master = new TalonFX(19, "rio");
    m_Slave = new TalonFX(17, "rio");
    m_motorIndependent = new TalonFX(4, "rio");
    configureClimb();
  }

  private void configureClimb(){
     TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500")) //SET MOTOR TYEP
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
    .withFeedback(MotorConfigs.getFeedbackConfigs(6/1)); //SET GEAR RATIO

    Follower followerConfig = new Follower(m_Master.getDeviceID(), true); // CHANGE DEPENDING ON GEARBOX
    m_Slave.setControl(followerConfig);

    m_Master.getConfigurator().apply(motorConfigs);
    m_Slave.getConfigurator().apply(motorConfigs);
    m_motorIndependent.getConfigurator().apply(motorConfigs);
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
 }

  public void voltageUp(){
    setControl(m_Master, voltageOut.withOutput(voltageInput));
  }

  public void voltageDown(){
    setControl(m_Master, voltageOut.withOutput(-voltageInput));
  }

  public void stopMotor(){
    setControl(m_Master, voltageOut.withOutput(0));
    setControl(m_motorIndependent, voltageOut.withOutput(0));
  }

  public Command runIndependent(boolean runOpposite){
    if (runOpposite){
      return Commands.run(()-> setControl(m_motorIndependent, voltageOut.withOutput(-3)));
    } else{
      return Commands.run(()-> setControl(m_motorIndependent, voltageOut.withOutput(3)));
    }
  }

  public double getVelocity(){
    return m_Master.getVelocity().getValueAsDouble();
  }

  public Command getCommand(mechanismStates wantedState){
    return Commands.run(() -> mechanismTransitionHandler(wantedState));
  }

  public void mechanismTransitionHandler(mechanismStates state){
    switch(state){
      case VOLTAGEUP:
        voltageUp();
        break;
      case VOLTAGEDOWN:
        voltageDown();
        break;
    }
  }

  
  @Override
  public void periodic() {
  }

  public enum mechanismStates{
    VOLTAGEUP,VOLTAGEDOWN
  }
}
