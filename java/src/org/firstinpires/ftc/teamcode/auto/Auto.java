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
        runTime.reset();
        while (opModeIsActive() && runTime.seconds() <= 7 ) {
      
        //close left and right servo claws TODO
       
        Forward(22);
        
        sleep(500);
        TurnLeftAngel(27);
         sleep(1000);
        StrafeLeft(40);  
         sleep(1000);
              autoArmUp(); 
              //runCount = runCount + 1;
             //telemetry.addData("counter", runCount);
             //telemetry.update
        }
    }
}
