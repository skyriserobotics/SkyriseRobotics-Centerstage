package org.firstinpires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

@Autonomous
public class AutoRed extends RobotConfig {
  @Override
  public void runOpMode() throws InterruptedException {
    
    this.initializeHardware();
    waitForStart();
    
    if (opModeIsActive()) {      
      servoClose();      
      forward(22);
      turnRight(40);
      strafeRight(20);  
      backward(11);
      autoArmUp();
      autoArmDown();
    }
  }
}
