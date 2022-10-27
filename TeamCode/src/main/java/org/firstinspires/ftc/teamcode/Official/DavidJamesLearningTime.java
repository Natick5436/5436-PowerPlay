package org.firstinspires.ftc.teamcode.Official;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LearningTeleOp",group="TeleOp")
public class DavidJamesLearningTime extends LinearOpMode {
    Mark12 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark12(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.left_trigger!=0){
                robot.lF.setPower(gamepad1.left_trigger*drivePower);
                robot.lB.setPower(gamepad1.left_trigger*drivePower);
            }else{
                robot.lF.setPower(0);
                robot.lB.setPower(0);
            }
            if(gamepad1.right_trigger!=0){
                robot.rF.setPower(gamepad1.right_trigger*drivePower);
                robot.rB.setPower(gamepad1.right_trigger*drivePower);
            }else{
                robot.rF.setPower(0);
                robot.rB.setPower(0);
            }
            if(gamepad1.a){
                robot.lF.setPower(0.2);
            }else{
                robot.lF.setPower(0);
            }
            if(gamepad1.b){
                robot.lB.setPower(0.2);
            }else{
                robot.lB.setPower(0);
            }
            if(gamepad1.x){
                robot.rF.setPower(0.2);
            }else{
                robot.rF.setPower(0);
            }
            if(gamepad1.y){
                robot.rB.setPower(0.2);
            }else{
                robot.rB.setPower(0);
            }
            robot.disableBrakes();


        }

    }

}
