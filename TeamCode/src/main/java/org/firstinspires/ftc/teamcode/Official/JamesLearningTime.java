package org.firstinspires.ftc.teamcode.Official;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LearningTeleOp",group="TeleOp")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class JamesLearningTime extends LinearOpMode {
    Mark11 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark11(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.a){
                robot.leftGrabber.setPower(0.1);
                robot.rightGrabber.setPower(0.1);

            }
            else if (gamepad1.x){
                robot.leftGrabber.setPower(-0.1);
                robot.rightGrabber.setPower(-0.1);
            }
            else{
                robot.leftGrabber.setPower(0);
                robot.rightGrabber.setPower(0);
            }
        }

    }

}
