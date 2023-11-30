package org.firstinpires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

@Autonomous
public class AutoBlue extends RobotConfig {
  @Override
  public void runOpMode() throws InterruptedException {
    
    this.initializeHardware();
    waitForStart();
    
    if (opModeIsActive()) {      
      servoClose();      
      forward(22);
      turnLeft(40);
      strafeLeft(20);  
      backward(6);
      autoArmUp();
      autoArmDown();
    }
  }
}
