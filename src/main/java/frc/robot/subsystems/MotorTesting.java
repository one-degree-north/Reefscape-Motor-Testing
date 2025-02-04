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
  private VoltageOut voltageOut = new VoltageOut(0);
  private double voltageInput = 0; //CHANGE THIS TO WHATEVER VOLTAGE YOU WANT TO TEST (MUST BE POSITIVE)
  
  public MotorTesting() {
    m_Master = new TalonFX(1, "rio");
    m_Slave = new TalonFX(2, "rio");
    configureClimb();
  }

  private void configureClimb(){
     TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
    .withCurrentLimits(MotorConfigs.getCurrentLimitConfig("Falcon500")) //SET MOTOR TYEP
    .withMotorOutput(MotorConfigs.getMotorOutputConfigs(
      NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
    .withFeedback(MotorConfigs.getFeedbackConfigs(192/1)); //SET GEAR RATIO

    Follower followerConfig = new Follower(m_Master.getDeviceID(), true); // CHANGE DEPENDING ON GEARBOX
    m_Slave.setControl(followerConfig);

    m_Master.getConfigurator().apply(motorConfigs);
    m_Slave.getConfigurator().apply(motorConfigs);
  }

  private void setControl(TalonFX motor, ControlRequest req) {
    if (motor.isAlive()) {
      motor.setControl(req);
    }
 }

  public void climbUp(){
    setControl(m_Master, voltageOut.withOutput(voltageInput));
  }

  public void climbDown(){
    setControl(m_Master, voltageOut.withOutput(-voltageInput));
  }

  public void stopClimb(){
    setControl(m_Master, voltageOut.withOutput(0));
  }

  public double getVelocity(){
    return m_Master.getVelocity().getValueAsDouble();
  }

  public Command getCommand(ClimbStates wantedState){
    return Commands.runOnce(() -> climbTransitionHandler(wantedState));
  }

  public void climbTransitionHandler(ClimbStates state){
    switch(state){
      case VOLTAGEUP:
        climbUp();
        break;
      case VOLTAGEDOWN:
        climbDown();
        break;
    }
  }

  
  @Override
  public void periodic() {
  }

  public enum ClimbStates{
    VOLTAGEUP,VOLTAGEDOWN
  }
}
