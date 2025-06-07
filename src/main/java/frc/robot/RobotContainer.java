// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisCmd;
import frc.robot.subsystems.SwerveChassis;

public class RobotContainer {
  public SwerveChassis chassis = new SwerveChassis();
  public Joystick joystick = new Joystick(0);
  public double SpeedMode = 0.4;
  public SendableChooser<Command> autoChooser;

  @SuppressWarnings("unchecked")
  public RobotContainer() {
    chassis.setDefaultCommand(new ChassisCmd(
      chassis, 
      () -> joystick.getRawAxis(1)*DrivetrainConstants.MaxVelocity.in(MetersPerSecond)*SpeedMode,
      () -> joystick.getRawAxis(0)*DrivetrainConstants.MaxVelocity.in(MetersPerSecond)*SpeedMode,
      () -> joystick.getRawAxis(2)*DrivetrainConstants.MaxOmega.in(RadiansPerSecond)*(SpeedMode)));

    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> joystick.getRawButton(5)).and(() -> SpeedMode > 0)
      .onTrue(Commands.runOnce(() -> SpeedMode -= 0.1));
    new Trigger(() -> joystick.getRawButton(6)).and(() -> SpeedMode < 1)
      .onTrue(Commands.runOnce(() -> SpeedMode += 0.1));
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAutoChooser().getSelected();
  }
}
