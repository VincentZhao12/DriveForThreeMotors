// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class DrivingFor3Meters extends CommandBase {
  /** Creates a new DrivingFor3Meters. */

  Drivetrain drivetrain;
  RobotContainer robotContainer;

  public DrivingFor3Meters(Drivetrain drivetrainIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drivetrainIn;
    robotContainer = new RobotContainer();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.startMotors(drivetrain.getMotorSpeed(3.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !robotContainer.joystickButton.get();
  }
}
