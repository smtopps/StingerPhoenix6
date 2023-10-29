// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.PDBalanceOnChargeStation;
import frc.robot.commands.ArmCommands.DropConeAgain;
import frc.robot.commands.ArmCommands.FlipCone;
import frc.robot.commands.ArmCommands.PlaceOnPosition;
import frc.robot.commands.ArmCommands.ReleaseAndRetract;
import frc.robot.commands.ArmCommands.ComplexMethod.MoveArmExtensionToCubePickup;
import frc.robot.commands.IntakeCommands.ReverseConveyor;
import frc.robot.commands.IntakeCommands.ReverseIntake;
import frc.robot.commands.IntakeCommands.RunConveyor;
import frc.robot.commands.IntakeCommands.RunIntakeCone;
import frc.robot.commands.IntakeCommands.RunIntakeCube;
import frc.robot.commands.IntakeCommands.SideStationIntake;
import frc.robot.commands.IntakeCommands.StopConveyor;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Classifier;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Arm arm = new Arm();
  private final Pincher pincher = new Pincher();
  private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();
  private final Classifier classifier = new Classifier();

  public static final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.kOperatorControllerPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem,
      poseEstimator,
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> -driverController.getRightX(),
      () -> GlobalVariables.fieldRelative,
      () -> GlobalVariables.maxSpeed));
      
    configureBindings();
  }

  private void configureBindings() {
    driverController.back().onTrue(new InstantCommand( () -> poseEstimator.setPose(new Pose2d()), poseEstimator));
    driverController.x().onTrue(new InstantCommand( () -> GlobalVariables.fieldRelative = !GlobalVariables.fieldRelative));
    driverController.b().onTrue(new InstantCommand( () -> swerveSubsystem.lock(), swerveSubsystem));
    driverController.a().whileTrue(new PDBalanceOnChargeStation(pigeon2Subsystem, swerveSubsystem, poseEstimator));
    driverController.rightBumper().onTrue(new RunIntakeCone(intake).alongWith(new RunConveyor(conveyor)));
    driverController.rightTrigger(0.5).onTrue(new ReverseIntake(intake).alongWith(new ReverseConveyor(conveyor)));
    driverController.rightBumper().onFalse(new StopIntake(intake).andThen(new WaitCommand(1).andThen(new StopConveyor(conveyor))));
    driverController.rightTrigger(0.5).onFalse(new StopIntake(intake).alongWith(new StopConveyor(conveyor)));
    driverController.leftBumper().onTrue(new RunIntakeCube(intake).alongWith(new RunConveyor(conveyor)));
    driverController.leftBumper().onFalse(new StopIntake(intake).andThen(new WaitCommand(1).andThen(new StopConveyor(conveyor))));
    driverController.y().onTrue(new SideStationIntake(intake).alongWith(new RunConveyor(conveyor)));
    driverController.y().onFalse(new StopIntake(intake).andThen(new StopConveyor(conveyor)));

    //operatorController.x().onTrue(new GripAndHold(pincher, arm));
    //operatorController.x().onTrue(new Toggle(pincher));
    operatorController.x().onTrue(new MoveArmExtensionToCubePickup(arm, pincher, classifier));
    operatorController.a().onTrue(new PlaceOnPosition(arm, pincher, () -> (GlobalVariables.upDownPosition)));
    operatorController.b().onTrue(new ReleaseAndRetract(pincher, arm, () -> (GlobalVariables.upDownPosition)));
    operatorController.y().onTrue(new DropConeAgain(arm, pincher));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> GlobalVariables.isCone = !GlobalVariables.isCone).alongWith(new InstantCommand(() -> candleSubsystem.setGamePiece())));
    //operatorController.rightTrigger(0.5).onTrue(new RunConveyor(conveyor)).onFalse(new StopConveyor(conveyor));
    operatorController.rightTrigger(0.5).onTrue(new SideStationIntake(intake).alongWith(new RunConveyor(conveyor))).onFalse(new StopConveyor(conveyor).alongWith(new StopIntake(intake)));
    operatorController.leftTrigger(0.5).onTrue(new ReverseConveyor(conveyor)).onFalse(new StopConveyor(conveyor));
    operatorController.rightBumper().onTrue(new FlipCone(arm, pincher));
    
    operatorController.povUp().onTrue(new InstantCommand(() -> arm.moveGridTargetIf(true), arm));
    operatorController.povDown().onTrue(new InstantCommand(() -> arm.moveGridTargetIf(false), arm));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
