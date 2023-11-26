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
        
        Forward(24);
        
       // this.initializeHardware();
        
        sleep(1000);
        // TurnLeft(10);

        TurnLeftAngel(27);sleep(500);
        StrafeLeft(40);  
        sleep(1000);
        // Backward(5);
        autoArmUp();
        // servoOpen();
    }
}
