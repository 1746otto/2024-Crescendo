package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystemtest extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LEDSubsystemtest() {
    led = new AddressableLED(LEDConstants.PWMPortLeft);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LEDLength);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLEDToCube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, LEDConstants.cubeHValue, LEDConstants.cubeSValue,
          LEDConstants.cubeVValue);
    }
    led.setData(ledBuffer);
  }

  public void setToHue(int hue) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, 255, 130);
    }
    led.setData(ledBuffer);
  }

  public void setLedtoCone() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, LEDConstants.coneHValue, LEDConstants.coneSValue,
          LEDConstants.coneVValue);
    }
    led.setData(ledBuffer);
  }

  public void setLedOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("led", ledBuffer.getLength());
  }

}
