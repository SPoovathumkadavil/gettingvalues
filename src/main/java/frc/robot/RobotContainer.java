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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {

  private CommandJoystick flightSim;
  private CANSparkMax shooterRightMotor;
  private CANSparkMax shooterLeftMotor;
  private CANSparkMax shooterAngleMotor;
  private CANSparkMax loaderMotor;
  private CANSparkMax intakeTiltMotor;
  private CANSparkMax intakeOutputMotor;
  private TalonSRX armExtensionMotor;
  private AnalogInput sensor;

  public RobotContainer() {
    configureBindings();
    setupShooter();
    setupArm();
    setupSensor();
    setupIntake();
    setupSystem();
  }

  public void teleopInit() {
    intakeTiltMotor.set(0);
  }

  private void setupIntake() {
    intakeTiltMotor = new CANSparkMax(11, MotorType.kBrushless);
    intakeOutputMotor = new CANSparkMax(10, MotorType.kBrushless);

    intakeTiltMotor.getPIDController().setOutputRange(-0.4, 0.4);
    intakeTiltMotor.getPIDController().setP(1);

    Shuffleboard.getTab("Debug").addDouble("Intake Position", () -> intakeTiltMotor.getEncoder().getPosition());
  }

  private void setupSensor() {
    sensor = new AnalogInput(1);
    Shuffleboard.getTab("Debug").addInteger("Sensor Measurement", () -> sensor.getValue());
    Shuffleboard.getTab("Debug").addBoolean("Has Note", () -> sensor.getValue() < 2200);
  }

  private void configureBindings() {
    flightSim = new CommandJoystick(1);
  }

  private void setupSystem() {

    double ampAngle = -18;
    double ampArm = 3000;
    double speakerAngle = -1.;
    double intakeAngle = -35.; // TODO
    double readyHandoffAngle = -6.15; // TODO
    double intakeHandoffAngle = 5;
    double intakeStowAngle = -8.5;

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

     flightSim.button(4).onTrue(
      Commands.runOnce(
        () -> {
          shooterLeftMotor.set(-0.40); //-0.4
          shooterRightMotor.set(-0.40);
        }
      ).andThen(
        new WaitCommand(0.2)
      ).andThen(
        Commands.runOnce(
          () -> {
            loaderMotor.set(-1);
          }
        )
      )
    ).onFalse(
      new InstantCommand(
          () -> {
            shooterLeftMotor.set(0);
            shooterRightMotor.set(0);
            loaderMotor.set(0);
          }
        )
    );


    flightSim.button(1).onTrue(
      Commands.runOnce(
        () -> {
          shooterLeftMotor.set(-1.0);
          shooterRightMotor.set(-1.0);
        }
      ).andThen(
        new WaitCommand(1.6)
      ).andThen(
        Commands.runOnce(
          () -> {
            loaderMotor.set(-1);
          }
        )
      )
    ).onFalse(
      new InstantCommand(
          () -> {
            shooterLeftMotor.set(0);
            shooterRightMotor.set(0);
            loaderMotor.set(0);
          }
        )
    );

    // flightSim.button(2).onTrue(
    //   Commands.run(
    //     () -> {
    //       loaderMotor.set(-0.25);
    //     }
    //   ).until(
    //     () -> sensor.getValue() < 2200
    //   ).withTimeout(1.5).andThen(
    //     new InstantCommand(
    //       () -> {
    //         loaderMotor.set(0);
    //       }
    //     )
    //   )
    // );

    flightSim.button(2).onTrue(
      Commands.runOnce(
        () -> intakeTiltMotor.getPIDController().setReference(intakeAngle, ControlType.kPosition)
      ).andThen(
        new InstantCommand(
          () -> intakeOutputMotor.set(0.3)
        )
      )
    ).onFalse(
      Commands.runOnce(
        () -> {
          intakeOutputMotor.set(0);
          intakeTiltMotor.getPIDController().setReference(intakeHandoffAngle, ControlType.kPosition);
          shooterAngleMotor.getPIDController().setReference(readyHandoffAngle, ControlType.kPosition);
          armExtensionMotor.set(ControlMode.Position, 100);
        }
      ).andThen(
        new WaitCommand(1.5)
      ).andThen(
        () -> {
          loaderMotor.set(-0.25);
          intakeOutputMotor.set(-0.3);
        }
      ).andThen(
        Commands.waitUntil(() -> sensor.getValue() < 2200).withTimeout(4.0)
      ).andThen(
        new InstantCommand(
          () -> {
            loaderMotor.set(0);
            intakeOutputMotor.set(0);
          }
        )
      ).andThen(
        new InstantCommand(
          () -> intakeTiltMotor.getPIDController().setReference(intakeStowAngle, ControlType.kPosition)
        )
      )
    );

    flightSim.button(12).onTrue(Commands.runOnce(() -> intakeTiltMotor.getEncoder().setPosition(0)));

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

    shooterRightMotor.setSmartCurrentLimit(40);
    shooterLeftMotor.setSmartCurrentLimit(40);

    // flightSim.button(1).onTrue(
    //   new InstantCommand(() -> {
    //     shooterRightMotor.set(-1.0); // Working, RIGHT(OF ROBOT)(ID 13) -1.00, LEFT(OF ROBOT)(ID 14) -0.40
    //     shooterLeftMotor.set(-0.4);
    //   })
    // ).onFalse(
    //   new InstantCommand(() -> {
    //     shooterRightMotor.set(0);
    //     shooterLeftMotor.set(0);
    //   })
    // );

    // flightSim.button(2).onTrue(
    //   new InstantCommand(() -> {
    //     shooterRightMotor.set(1);
    //     shooterLeftMotor.set(1);
    //   })
    // ).onFalse(
    //   new InstantCommand(() -> {
    //     shooterRightMotor.set(0);
    //     shooterLeftMotor.set(0);
    //   })
    // );

    Shuffleboard.getTab("Debug").addDouble("speed", () -> shooterRightMotor.get());

    /* Angle */
    // 28.2 limit

    shooterAngleMotor = new CANSparkMax(15, MotorType.kBrushless);
    shooterAngleMotor.getPIDController().setP(0.5);
    shooterAngleMotor.getPIDController().setOutputRange(-0.3, 0.3);
    shooterAngleMotor.setSoftLimit(SoftLimitDirection.kForward, 28);
    shooterAngleMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    double flatShoot = -3.0;
    double diveShoot = -4.0;

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
  
    /* Loader */

    loaderMotor = new CANSparkMax(16, MotorType.kBrushless);

    loaderMotor.setSmartCurrentLimit(40);

    Shuffleboard.getTab("Debug").addDouble("Loader Speed", () -> loaderMotor.get());
  
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");//q
  }
}
