// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.PDBalanceOnChargeStation;
import frc.robot.commands.PIDBalanceOnChargeStation;
import frc.robot.commands.PIDDriveOnChargeStation;
import frc.robot.commands.ArmCommands.DropConeAgain;
import frc.robot.commands.ArmCommands.FlipCone;
import frc.robot.commands.ArmCommands.PlaceOnPosition;
import frc.robot.commands.ArmCommands.ReleaseAndRetract;
import frc.robot.commands.ArmCommands.AutoSpecific.GripConeAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.GripCubeAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.PlaceOnPositionAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.ReleaseOnPositionAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.RetractFromPositionAuto;
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

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

    m_chooser.setDefaultOption("Nothing", "Nothing");
    m_chooser.addOption("Score", "Score");
    m_chooser.addOption("Level Right", "LevelRight");
    m_chooser.addOption("Score Level Left", "ScoreLevelLeft");
    m_chooser.addOption("Score Level Middle", "ScoreLevelMiddle");
    m_chooser.addOption("Score Level Right", "ScoreLevelRight");
    m_chooser.addOption("Score Cross Level Middle", "ScoreCrossLevelMiddle");
    m_chooser.addOption("Score Cross Pickup Level Left Middle", "ScoreCrossPickupLevelLeftMiddle");
    m_chooser.addOption("Score Cross Pickup Level Right Middle", "ScoreCrossPickupLevelRightMiddle");
    m_chooser.addOption("Score Pickup Level Left", "ScorePickupLevelLeft");
    m_chooser.addOption("Score Pickup Level Right", "ScorePickupLevelRight");
    m_chooser.addOption("Two Left", "TwoLeft");
    m_chooser.addOption("Two Right", "TwoRight");
    m_chooser.addOption("Two Right Bump", "TwoRightBump");
    m_chooser.addOption("Two Left Level", "TwoLeftLevel");
    m_chooser.addOption("Three Left", "ThreeLeft");

    SmartDashboard.putData(m_chooser);
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
    
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("RunIntakeCone", new RunIntakeCone(intake));
    eventMap.put("RunIntakeCube", new RunIntakeCube(intake));
    eventMap.put("RunConveyor", new RunConveyor(conveyor));
    eventMap.put("StopIntake", new StopIntake(intake));
    eventMap.put("StopConveyor", new StopConveyor(conveyor));
    eventMap.put("SideIntake", new SideStationIntake(intake));
    eventMap.put("Level", new PIDBalanceOnChargeStation(pigeon2Subsystem, swerveSubsystem, poseEstimator));
    eventMap.put("PassLevel", new PIDDriveOnChargeStation(pigeon2Subsystem, swerveSubsystem, poseEstimator));
    eventMap.put("Stop", new InstantCommand(() -> swerveSubsystem.stop(), swerveSubsystem));
    eventMap.put("GripCone", new GripConeAuto(pincher, arm));
    eventMap.put("GripCube", new GripCubeAuto(pincher, arm));
    eventMap.put("Place3", new PlaceOnPositionAuto(arm, pincher, () -> 2));
    eventMap.put("Place2", new PlaceOnPositionAuto(arm, pincher, () -> 1));
    eventMap.put("Release3", new ReleaseOnPositionAuto(arm, pincher, () -> 2));
    eventMap.put("Release2", new ReleaseOnPositionAuto(arm, pincher, () -> 1));
    eventMap.put("Retract", new RetractFromPositionAuto(arm));


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      poseEstimator::getPose, // Pose2d supplier
      poseEstimator::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.SwerveConstants.KINEMATICS, // SwerveDriveKinematics
      new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true,
      swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // An example command will be run in autonomous
    List<PathPlannerTrajectory> trajectories;
    if(m_chooser.getSelected() == "Nothing") {
      return null;
    }else if(m_chooser.getSelected() == "TwoRightBump") {
      trajectories = PathPlanner.loadPathGroup(m_chooser.getSelected(), 2, 2);
      return autoBuilder.fullAuto(trajectories);
    }else{
      trajectories = PathPlanner.loadPathGroup(m_chooser.getSelected(), 3, 2.5);//vel 3, accel 2.5
      return autoBuilder.fullAuto(trajectories);
    }
  }

  public void autoModeSettings() {
    arm.setBrakeMode();
    //pigeon2Subsystem.zeroPitch();
    GlobalVariables.pigeonPitch = pigeon2Subsystem.getPigeonPitch();
  }

  public void teleopModeSettings() {
    //arm.setCoastMode();
  }
}
