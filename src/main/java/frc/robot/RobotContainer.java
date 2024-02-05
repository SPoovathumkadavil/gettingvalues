// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {

  private CommandJoystick flightSim;
  private CANSparkMax shooterRightMotor;
  private CANSparkMax shooterLeftMotor;
  private CANSparkMax shooterAngleMotor;
  private TalonSRX armExtensionMotor;

  public RobotContainer() {
    configureBindings();
    setupShooter();
    setupArm();
    setupSystem();
  }

  private void configureBindings() {
    flightSim = new CommandJoystick(1);
  }

  private void setupSystem() {
    double ampAngle = 6.95;
    double ampArm = 2000;
    double speakerAngle = 21.8;
    flightSim.button(9).onTrue(
      new InstantCommand(
        () -> {
          armExtensionMotor.set(ControlMode.Position, ampArm);
          shooterAngleMotor.getPIDController().setReference(ampAngle, ControlType.kPosition);
        }
      )
    );
    flightSim.button(7).onTrue(
      new InstantCommand(
        () -> {
          armExtensionMotor.set(ControlMode.Position, 100);
          shooterAngleMotor.getPIDController().setReference(speakerAngle, ControlType.kPosition);
        }
      )
    );
  }

  private void setupArm() {
    armExtensionMotor = new TalonSRX(12);
    armExtensionMotor.config_kP(0, 1);
    armExtensionMotor.configPeakOutputForward(0.6);
    armExtensionMotor.configPeakOutputReverse(0.6);

    flightSim.button(10).onTrue(
      new InstantCommand(
        () -> {
          armExtensionMotor.set(ControlMode.Position, 2000);
        }
      )
    );

    Shuffleboard.getTab("Debug").addDouble("Arm Position", () -> armExtensionMotor.getSelectedSensorPosition());
  }

  private void setupShooter() {

    /* Flywheels */

    shooterRightMotor = new CANSparkMax(13, MotorType.kBrushless);
    shooterLeftMotor = new CANSparkMax(14, MotorType.kBrushless);

    flightSim.button(1).onTrue(
      new InstantCommand(() -> {
        shooterRightMotor.set(-1.0); // Working, RIGHT(OF ROBOT)(ID 13) -1.00, LEFT(OF ROBOT)(ID 14) -0.40
        shooterLeftMotor.set(-0.4);
      })
    ).onFalse(
      new InstantCommand(() -> {
        shooterRightMotor.set(0);
        shooterLeftMotor.set(0);
      })
    );

    flightSim.button(2).onTrue(
      new InstantCommand(() -> {
        shooterRightMotor.set(1);
        shooterLeftMotor.set(1);
      })
    ).onFalse(
      new InstantCommand(() -> {
        shooterRightMotor.set(0);
        shooterLeftMotor.set(0);
      })
    );

    Shuffleboard.getTab("Debug").addDouble("speed", () -> shooterRightMotor.get());

    /* Angle */
    // 28.2 limit

    shooterAngleMotor = new CANSparkMax(15, MotorType.kBrushless);
    shooterAngleMotor.getPIDController().setP(0.5);
    shooterAngleMotor.getPIDController().setOutputRange(-0.3, 0.3);
    shooterAngleMotor.setSoftLimit(SoftLimitDirection.kForward, 28);
    shooterAngleMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    double flatShoot = 21.8;
    double diveShoot = 25.0;

    flightSim.button(5).onTrue(
      new InstantCommand(
        () -> shooterAngleMotor.getPIDController().setReference(flatShoot, ControlType.kPosition)
      )
    );

    flightSim.button(6).onTrue(
      new InstantCommand(
        () -> shooterAngleMotor.getPIDController().setReference(diveShoot, ControlType.kPosition)
      )
    );

    Shuffleboard.getTab("Debug").addDouble("Shooter Angle Position", () -> shooterAngleMotor.getEncoder().getPosition());
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
