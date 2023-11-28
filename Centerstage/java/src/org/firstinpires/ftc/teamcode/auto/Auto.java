package org.firstinpires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.common.RobotConfig;

@Autonomous
public class Auto extends RobotConfig {
    
    @Override
    public void runOpMode() throws InterruptedException {
        this.initializeHardware();        
        waitForStart();
        
        //close left and right servo claws TODO
        //servoClose()
        //sleep(1000);
        forward(24);        
        sleep(1000);
        turnLeft(27);
        sleep(500);
        strafeLeft(40);  
        sleep(1000);
        // Backward(5);
        autoArmUp();
        // servoOpen();
        // autoArmDown();
    }
}
