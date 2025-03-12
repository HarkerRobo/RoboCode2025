// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.LimelightSimulation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = RobotContainer.getInstance();
  }

  @Override
  public void robotInit() {
    if (Utils.isSimulation()) {
      LimelightSimulation limelightSim = new LimelightSimulation(
                Constants.Vision.kCamera1Name, Constants.Vision.kRobotToCam1);

      // Back Limelight
        LimelightSimulation limelight2Sim = new LimelightSimulation(
                Constants.Vision.kCamera2Name, Constants.Vision.kRobotToCam2);

      // Show field visualizations
      // SmartDashboard.putData("LL1 Field", limelightSim.getField2d());
      // SmartDashboard.putData("LL2 Field", limelight2Sim.getField2d());
    }

    LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.kCamera1Name, 
      Constants.Vision.kRobotToCam1.getX(), Constants.Vision.kRobotToCam1.getY(), Constants.Vision.kRobotToCam1.getZ(),
      Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getZ()));

    LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.kCamera2Name, 
      Constants.Vision.kRobotToCam2.getX(), Constants.Vision.kRobotToCam2.getY(), Constants.Vision.kRobotToCam2.getZ(),
      Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getZ()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateTelemetry();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    // SignalLogger.stop();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {

  }
}