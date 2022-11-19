package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp",group="TeleOp")
public class TeleOp extends LinearOpMode {
    Mark12 robot;

    public void runOpMode() {
        robot = new Mark12(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;

        long liftTime = System.currentTimeMillis();
        long angleTime = System.currentTimeMillis();

        robot.liftMotor.setPower(0.8);
        robot.angleMotor.setPower(0.15);
        robot.disableBrakes();

        //Pat's test code
        //leftServoPosition = Range.clip(leftServoPosition, robot.ARM_MIN_RANGE,robot.ARM_MAX_RANGE);
        //robot.leftServo.setPosition(leftServoPosition);

        waitForStart();


        while (opModeIsActive()) {
            if (fastMode) {
                drivePower = 0.3;
            } else {
                drivePower = 0.15;
            }
            if (!gamepad1.right_bumper && rightBumper) fastMode = !fastMode;
            rightBumper = gamepad1.right_bumper;
            if (gamepad1.left_bumper) {
                if (Math.abs(gamepad1.right_stick_x) < 0.25) {
                    //robot.angleStrafe(drivePower * Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                } else {
                    //robot.turningStrafe(drivePower * Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);
                }
                bumperDown = true;
            } else if ((gamepad1.right_trigger - gamepad1.left_trigger) != 0) {
                robot.SimpleStrafe(drivePower * (gamepad1.right_trigger - gamepad1.left_trigger));
            } else {
                if (!bumperDown) {
                    robot.lF.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.lB.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.rF.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.rB.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.setStatus(Mecanum_Drive.Status.DRIVING);
                } else {
                    if (Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) < 0.1 && Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) < 0.1) {
                        bumperDown = false;
                    }
                }
            }
            if (gamepad1.back && !backDown) {
                // robot.getPositionTracker().resetPosition(0.2032, 3.40995);
                //robot.getAngleTracker().resetAngle(Math.PI/2);.
                backDown = true;
            }
            backDown = gamepad1.back;

            //5080 before
            if(gamepad1.dpad_up && robot.liftMotor.getTargetPosition()<12080){
                robot.liftMotor.setTargetPosition(robot.liftMotor.getTargetPosition()+90);
            }if(gamepad1.dpad_down && robot.liftMotor.getTargetPosition()>0){

                robot.liftMotor.setTargetPosition(robot.liftMotor.getTargetPosition()-90);
            }
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

            if(gamepad2.dpad_up){
                robot.angleMotor.setTargetPosition(robot.angleMotor.getTargetPosition()+13);
            }if(gamepad2.dpad_down){
                robot.angleMotor.setTargetPosition(robot.angleMotor.getTargetPosition()-13
                );
            }

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
                robot.centerServo.setPower(0.25);
            }else if(gamepad2.right_trigger>0){
                robot.centerServo.setPower(-0.25);
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


            telemetry.addData("Horizontal", robot.odo.h.distanceTraveled());
            telemetry.addData("Vertical", robot.odo.v.distanceTraveled());
            telemetry.addData("Angle", robot.getAngle());
            telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());

            telemetry.addData("fastMode",fastMode);
            telemetry.addData("lb", robot.lB.getPower());
            telemetry.addData("rb", robot.rB.getPower());
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("rF", robot.rF.getPower());

//            telemetry.addData("leftTrigger", gamepad2.left_trigger);
//            telemetry.addData("leftBumper", gamepad2.left_bumper);
//            telemetry.addData("power", robot.centerServo.getPower());

            telemetry.addData("armPos", robot.liftMotor.getTargetPosition());
            telemetry.addData("armPos", robot.liftMotor.getCurrentPosition());



            //telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }
}
