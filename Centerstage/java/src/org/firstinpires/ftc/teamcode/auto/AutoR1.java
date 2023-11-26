// package org.firstinpires.ftc.teamcode.auto;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// import org.firstinspires.ftc.teamcode.common.RobotConfig;
// import org.firstinspires.ftc.teamcode.common.SleeveDetection;

// @Autonomous
// public class AutoR1 extends RobotConfig {
//     @Override
//     public void runOpMode() throws InterruptedException {
//         this.initializeHardware();

//         // while (!isStarted()) {
//         //     telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
//         //     telemetry.update();
//         // }

//         waitForStart();
//         // SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
//         // camera.stopStreaming();
//         int forwardDistance = 25;
//         int strafeDistance = 30;
//         StrafeRight(strafeDistance);
//         //boolean isExit = false;
//         // while(opModeIsActive() && !isExit) {
//         //     Forward(11);
//         //     TurnRight(37);
//         //     Forward(5);
//         //     moveToLow();
//         //     //Forward(5);
//         //     clawOpen();
//         //     // Backward(2);
//         //     // moveToGround();
//         //     // TurnRight(5);
//         //     // if (position == SleeveDetection.ParkingPosition.CENTER) {
//         //     //     Forward(forwardDistance);
//         //     // }
//         //     // else if (position == SleeveDetection.ParkingPosition.LEFT) {
//         //     //     Forward(forwardDistance);
//         //     //     StrafeLeft(strafeDistance);
//         //     // } else {
//         //     //     Forward(forwardDistance);
//         //     //     StrafeRight(strafeDistance);
//         //     // }30
//         //     isExit = true;
//         // }
//     }
// }
