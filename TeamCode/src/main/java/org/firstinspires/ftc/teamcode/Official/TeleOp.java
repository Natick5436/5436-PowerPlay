package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;
import org.firstinspires.ftc.teamcode.Robots.Mark13;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp",group="TeleOp")
public class TeleOp extends LinearOpMode {
    Mark13 robot;

    public void runOpMode() {
        robot = new Mark13(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;

        long angleTime = System.currentTimeMillis();
        long liftTime = System.currentTimeMillis();

        robot.disableBrakes();



        waitForStart();


        while (opModeIsActive()) {

            /**GENERAL MOVEMENT**/
            if (fastMode) {
                drivePower = 0.2;
            } else {

                drivePower = 0.5;
            }
            if (!gamepad1.right_bumper && rightBumper) fastMode = !fastMode;
            rightBumper = gamepad1.right_bumper;
            if (gamepad1.left_bumper) {
                if (Math.abs(gamepad1.right_stick_x) < 0.25) {
                    robot.angleStrafe(drivePower * Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                } else {
                    robot.turningStrafe(drivePower * Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);
                }
                bumperDown = true;
            } else if (gamepad1.right_trigger>0 || gamepad1.left_trigger>0){
                //robot.SimpleStrafe(drivePower * (gamepad1.right_trigger - gamepad1.left_trigger));
                if(gamepad1.right_trigger>0){
                    robot.lF.setPower(gamepad1.right_trigger * drivePower);
                    robot.rF.setPower(gamepad1.right_trigger * drivePower);
                    robot.lB.setPower(-gamepad1.right_trigger * drivePower);
                    robot.rB.setPower(-gamepad1.right_trigger * drivePower);
                }else if(gamepad1.left_trigger>0){
                    robot.lF.setPower(-gamepad1.left_trigger * drivePower);
                    robot.rF.setPower(-gamepad1.left_trigger * drivePower);
                    robot.lB.setPower(gamepad1.left_trigger * drivePower);
                    robot.rB.setPower(gamepad1.left_trigger * drivePower);
                }
            } else {
                if (!bumperDown) {
                    robot.lF.setPower(drivePower * gamepad1.left_stick_y);
                    robot.lB.setPower(drivePower * gamepad1.left_stick_y);
                    robot.rF.setPower(drivePower * gamepad1.right_stick_y);
                    robot.rB.setPower(drivePower * gamepad1.right_stick_y);
                    robot.setStatus(Mecanum_Drive.Status.DRIVING);
                } else {
                    if
                    (Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) < 0.1 && Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) < 0.1) {
                        bumperDown = false;
                    }
                }
            }
//            if (gamepad1.back && !backDown) {
//                // robot.getPositionTracker().resetPosition(0.2032, 3.40995);
//                //robot.getAngleTracker().resetAngle(Math.PI/2);.
//                backDown = true;
//            }
//            backDown = gamepad1.back;


            /*if(gamepad2.dpad_left){
                robot.leftAxis.setTargetPosition(robot.leftAxis.getTargetPosition() + 3);
                robot.rightAxis.setTargetPosition(robot.rightAxis.getTargetPosition() + 3);
            }else if(gamepad2.dpad_right){
                robot.leftAxis.setTargetPosition(robot.leftAxis.getTargetPosition() - 3);
                robot.rightAxis.setTargetPosition(robot.rightAxis.getTargetPosition() - 3);
            }*/

            if (System.currentTimeMillis() - angleTime > 700) {
                //int armEncoderDiff = robot.getArmEncoderDiff();
                int leftAxisPosition = robot.leftAxis.getTargetPosition();
                int rightAxisPosition = robot.rightAxis.getTargetPosition();
                if(gamepad2.dpad_left && leftAxisPosition < 12000 /*+ armEncoderDiff*/){
                    //Change the 400s change how far the axis goes each time
                    robot.leftAxis.setTargetPosition(leftAxisPosition + 400);
                    robot.rightAxis.setTargetPosition(rightAxisPosition + 400);
                    angleTime = System.currentTimeMillis();

                }else if(gamepad2.dpad_right && leftAxisPosition > -100 /* + armEncoderDiff*/){
                    robot.leftAxis.setTargetPosition(leftAxisPosition - 400);
                    robot.rightAxis.setTargetPosition(rightAxisPosition - 400);
                    angleTime = System.currentTimeMillis();

                }
            }

            if (System.currentTimeMillis() - liftTime > 500) {
                //int armEncoderDiff = robot.getArmEncoderDiff();
                int leftPulleyPosition = robot.leftAxis.getTargetPosition();
                int rightPulleyPosition = robot.rightAxis.getTargetPosition();
                if(gamepad2.dpad_up /*&& leftPulleyPosition < 12000 + armEncoderDiff*/){
                    //change the 400s to change how much to pulleys goe each time
                    robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() + 400);
                    robot.rightPulley.setTargetPosition(rightPulleyPosition + 400);
                    liftTime = System.currentTimeMillis();

                }else if(gamepad2.dpad_down && leftPulleyPosition > -100 /* + armEncoderDiff*/){
                    robot.leftPulley.setTargetPosition(leftPulleyPosition - 400);
                    robot.rightPulley.setTargetPosition(rightPulleyPosition - 400);
                    liftTime = System.currentTimeMillis();

                }
            }

            /*
            if(gamepad2.dpad_up){
                robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() + 40);
                robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() + 40);
            }else if(gamepad2.dpad_down){
                robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() - 40);
                robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() - 40);
                if (robot.leftPulley.getTargetPosition()==0){
                    robot.pulleyServo.setPosition(robot.pulleyServo.getPosition() + 10);
                }
            }*/





            if(gamepad2.left_bumper){
                robot.leftGrabber.setPower(-0.3);
                robot.rightGrabber.setPower(-0.3);
            }else if(gamepad2.right_bumper){
                robot.leftGrabber.setPower(0.3);
                robot.rightGrabber.setPower(0.3);
            }else{
                robot.leftGrabber.setPower(0);
                robot.rightGrabber.setPower(0);
            }


            if(gamepad2.left_trigger>0){
                robot.leftClawSpinner.setPosition(robot.leftClawSpinner.getPosition()+0.006);
                robot.rightClawSpinner.setPosition(robot.rightClawSpinner.getPosition()+0.006);
            }else if(gamepad2.right_trigger>0){
                robot.leftClawSpinner.setPosition(robot.leftClawSpinner.getPosition()-0.006);
                robot.rightClawSpinner.setPosition(robot.rightClawSpinner.getPosition()-0.006);
            }



            if(gamepad2.x){
                robot.centerServo.setPower(0.4);
            }else if(gamepad2.b){
                robot.centerServo.setPower(-0.4);
            }else{
                robot.centerServo.setPower(0);
            }


            if(gamepad1.a && gamepad1.b){
                for(int i=0; i<5;i++){
                    robot.lF.setPower(0.15);
                    robot.lB.setPower(0.15);
                    robot.rF.setPower(-0.15);
                    robot.rB.setPower(-0.15);
                    sleep(500);
                    robot.lF.setPower(0.15);
                    robot.lB.setPower(0.15);
                    robot.rF.setPower(-0.15);
                    robot.rB.setPower(-0.15);
                }

            }


            //telemetry.addData("Raw IMU", ((REV_IMU)robot.getAngleTracker()).imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS));


            telemetry.addData("Horizontal", robot.getX());
            telemetry.addData("Vertical", robot.getY());
            telemetry.addData("Angle", robot.getAngle());
            telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());

            telemetry.addData("fastMode",fastMode);
            telemetry.addData("lb", robot.lB.getPower());
            telemetry.addData("rb", robot.rB.getPower());
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("rF", robot.rF.getPower());

            telemetry.addData("left pulley", robot.leftPulley.getTargetPosition());
            telemetry.addData("right pulley", robot.rightPulley.getTargetPosition());
            telemetry.addData("lift time", liftTime);

            telemetry.addData("height", robot.leftPulley.getCurrentPosition());

            //telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }
}

//BACKUP CODE
/*
            if (System.currentTimeMillis() - liftTime > 700) {
                int armEncoderDiff = robot.getArmEncoderDiff();
                int liftPosition = robot.liftMotor.getTargetPosition();
                if(gamepad2.dpad_up && liftPosition < 12000 + armEncoderDiff){
                    robot.liftMotor.setTargetPosition(liftPosition + 400);
                    liftTime = System.currentTimeMillis();
                    //sleep(500);
                }else if(gamepad2.dpad_down && liftPosition > 1 + armEncoderDiff){
                    robot.liftMotor.setTargetPosition(liftPosition - 400);
                    liftTime = System.currentTimeMillis();
                    //sleep(500);
                };
            }

            if (System.currentTimeMillis() - liftTime > 250) {
                int armPosition = robot.liftMotor.getTargetPosition();
                if (gamepad2.a) {
                    robot.setArmEncoderDiff(20);
                    robot.liftMotor.setTargetPosition(armPosition + 20);
                    liftTime = System.currentTimeMillis();
                } else if (gamepad2.b && !gamepad2.start) {
                    robot.setArmEncoderDiff(-20);
                    robot.liftMotor.setTargetPosition(armPosition - 20);
                    liftTime = System.currentTimeMillis();
                }
            }*/


/***
 if (System.currentTimeMillis() - angleTime > 700) {c
 int armEncoderDiff = robot.getArmEncoderDiff();
 int anglePosition = robot.angleMotor.getTargetPosition();
 if(gamepad2.dpad_left && anglePosition < 30000 /*+ armEncoderDiff){
 robot.angleMotor.setTargetPosition(anglePosition + 100);
 angleTime = System.currentTimeMillis();
 //sleep(500);
 }else if(gamepad2.dpad_right && anglePosition > 1 /*+ armEncoderDiff){
 robot.angleMotor.setTargetPosition(anglePosition - 100);
 angleTime = System.currentTimeMillis();
 //sleep(500);
 };
 }
 ***/
/*
            if (System.currentTimeMillis() - angleTime > 250) {
                int armPosition = robot.liftMotor.getTargetPosition();
                if (gamepad2.a) {
                    robot.setArmEncoderDiff(20);
                    robot.liftMotor.setTargetPosition(armPosition + 20);
                    liftTime = System.currentTimeMillis();
                } else if (gamepad2.b && !gamepad2.start) {
                    robot.setArmEncoderDiff(-20);
                    robot.liftMotor.setTargetPosition(armPosition - 20);
                    liftTime = System.currentTimeMillis();
                }
            }


*/
