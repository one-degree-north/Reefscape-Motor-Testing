// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorTesting;
import frc.robot.subsystems.MotorTesting.mechanismStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class mechanismCommands extends Command {
  private final MotorTesting s_Mechanism;
  private final mechanismStates commandType;
  private Command commandToRun;
  
  public mechanismCommands(MotorTesting s_Mechanism, mechanismStates commandType) {
    this.s_Mechanism = s_Mechanism;
    this.commandType = commandType;
    addRequirements(s_Mechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      commandToRun = s_Mechanism.getCommand(commandType);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandToRun.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Mechanism.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandToRun.isFinished();
  }
}
