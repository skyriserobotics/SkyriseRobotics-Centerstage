package org.firstinpires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

@Autonomous
public class Auto extends RobotConfig {
    
    
    private int runCount = 1;
    private ElapsedTime  runTime = new ElapsedTime();
    
    @Override
    public void runOpMode() throws InterruptedException {
        this.initializeHardware();
        
        waitForStart();
       
      
        //close left and right servo claws TODO
       if (opModeIsActive()) {
        
        servoClose();
        Forward(22);
        
        TurnLeftAngel(40);
        StrafeLeft(20);  
        Backward(6);
        autoArmUp(); 
      
        autoArmDown();
        
       }
    }
}
