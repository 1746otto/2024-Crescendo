package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.kLeftMasterID);
    private WPI_VictorSPX leftSlave = new WPI_VictorSPX(DrivetrainConstants.kLeftSlaveID);
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.kRightMasterID);
    private WPI_VictorSPX rightSlave = new WPI_VictorSPX(DrivetrainConstants.kRightSlaveID);
    private double rotationalMotion;
    private double translationalMotion;
    private double sumMotions;

    DifferentialDrive m_drive = new DifferentialDrive(leftMaster, rightMaster);
  
  public DrivetrainSubsystem() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  public void ArrrrrrrcadeDrive(double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {

  }
}
