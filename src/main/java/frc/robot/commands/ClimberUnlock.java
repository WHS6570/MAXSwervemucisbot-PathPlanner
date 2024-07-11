package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SASubsystem;

public class ClimberUnlock extends Command {
private final ClimberSubsystem m_robotClimber;
double distance;    

  public ClimberUnlock(double targdist, ClimberSubsystem climb) {
    distance = targdist;
    m_robotClimber = climb;
    addRequirements(m_robotClimber);
  }
  @Override
  public void execute() {
    //m_robotIntake.engulf(-1);
    m_robotClimber.unlock();
  }
  @Override
 public boolean isFinished() {
  return false;
 }
  //@Override
  //public void end(boolean interrupted) {
  //  m_robotIntake.engulf(-1);
  //  wait(100);
  //  m_robotIntake.engulf(0);
   // }
}
