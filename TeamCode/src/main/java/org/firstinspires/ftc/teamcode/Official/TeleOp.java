package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        boolean adjustment = false;

        long angleTime = System.currentTimeMillis();
        long liftTime = System.currentTimeMillis();
        long speedTime = System.currentTimeMillis();
        long adjustmentTime = System.currentTimeMillis();
        long bumperTime = System.currentTimeMillis();
        long clawTime = System.currentTimeMillis();

        robot.disableBrakes();

        robot.leftPulley.setPower(0.5);
        robot.rightPulley.setPower(0.5);

        waitForStart();

        robot.rB.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lB.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftAxis.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightAxis.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.rightPulley.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftPulley.setDirection(DcMotorSimple.Direction.FORWARD);


        while (opModeIsActive()) {

            /**GENERAL MOVEMENT**/
            if (fastMode) {
                drivePower = 0.7;
            } else {

                drivePower = 0.5;
            }

            if (gamepad1.right_bumper && System.currentTimeMillis() - speedTime > 700){
                fastMode = !fastMode;
                speedTime = System.currentTimeMillis();
            }
            if (gamepad1.left_bumper) {
                if (Math.abs(gamepad1.right_stick_x) < 0.25) {
                    robot.angleStrafe(drivePower * Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                } else {
                    robot.turningStrafe(drivePower * Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);
                }
                bumperDown = true;
                //CHANGES
            } else if (gamepad1.right_trigger>0 || gamepad1.left_trigger>0){
                //robot.SimpleStrafe(drivePower * (gamepad1.right_trigger - gamepad1.left_trigger));
                robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rF.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);
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
                robot.rB.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.lB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                if (!bumperDown) {
                    robot.lF.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.lB.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.rF.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.rB.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.setStatus(Mecanum_Drive.Status.DRIVING);
                } else {
                    if
                    (Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) < 0.1 && Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) < 0.1) {
                        bumperDown = false;
                    }
                }
            }

            if(gamepad2.y && gamepad2.a && adjustmentTime >1000){
                adjustment = !adjustment;
                adjustmentTime = System.currentTimeMillis();
            }



            //Put this in adjustment



            if(!adjustment){


                if(System.currentTimeMillis() - bumperTime > 700){
                    if(gamepad2.left_bumper){
                        //robot.leftGrabber.setPower(-0.3);
                        //robot.rightGrabber.setPower(-0.3);
                        robot.grabber.setPosition(robot.grabber.getPosition() + .5);
                    }else if(gamepad2.right_bumper){
                        //robot.leftGrabber.setPower(0.3);
                        //robot.rightGrabber.setPower(0.3);
                        robot.grabber.setPosition(robot.grabber.getPosition() - .5);
                    }
                }


                if(gamepad2.left_trigger>0){
                    robot.clawSpinner.setPosition(robot.clawSpinner.getPosition()+0.025);
                }else if(gamepad2.right_trigger>0){
                    robot.clawSpinner.setPosition(robot.clawSpinner.getPosition()-0.025);
                }



                if(System.currentTimeMillis() - clawTime > 700){
                    if(gamepad2.x){
                        robot.centerServo.setPosition(robot.centerServo.getPosition() + 0.3);
                    }else if(gamepad2.b){
                        robot.centerServo.setPosition(robot.centerServo.getPosition() - 0.3);
                    }
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

                if (System.currentTimeMillis() - liftTime > 500) {
                    //int armEncoderDiff = robot.getArmEncoderDiff();
                    if(gamepad2.dpad_up /*&& leftPulleyPosition <= 11000 /*+ armEncoderDiff*/){
                        //change the 400s to change how much to pulleys goe each time
                        robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() + 400);
                        robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() + 400);
                        liftTime = System.currentTimeMillis();

                    }else if(gamepad2.dpad_down /*&& leftPulleyPosition >= 20 /* + armEncoderDiff*/){
                        robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() - 400);
                        robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() - 400);
                        liftTime = System.currentTimeMillis();

                    }
                }
                if (System.currentTimeMillis() - angleTime > 700) {
                    //int armEncoderDiff = robot.getArmEncoderDiff();
                    int leftAxisPosition = robot.leftAxis.getTargetPosition();
                    int rightAxisPosition = robot.rightAxis.getTargetPosition();
                    if(gamepad2.dpad_left ){//&& leftAxisPosition < 12000 /*+ armEncoderDiff*/){
                        //Change the 400s change how far the axis goes each time
                        robot.leftAxis.setTargetPosition(leftAxisPosition + 200);
                        robot.rightAxis.setTargetPosition(rightAxisPosition + 200);
                        angleTime = System.currentTimeMillis();

                    }else if(gamepad2.dpad_right){ //&& leftAxisPosition > -100 /* + armEncoderDiff*/){
                        robot.leftAxis.setTargetPosition(leftAxisPosition - 200);
                        robot.rightAxis.setTargetPosition(rightAxisPosition - 200);
                        angleTime = System.currentTimeMillis();

                    }
                }

            } else if(adjustment){
                if(gamepad2.dpad_up){
                    robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() + 40);
                    robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() + 40);
                }else if(gamepad2.dpad_down){
                    robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() - 40);
                    robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() - 40);
                }

                if(gamepad2.left_bumper){
                    //robot.leftGrabber.setPower(-0.3);
                    //robot.rightGrabber.setPower(-0.3);
                    robot.grabber.setPosition(robot.grabber.getPosition()+0.04);
                }else if(gamepad2.right_bumper){
                    //robot.leftGrabber.setPower(0.3);
                    //robot.rightGrabber.setPower(0.3);
                    robot.grabber.setPosition(robot.grabber.getPosition()-0.04);
                }

                if(gamepad2.x){
                    robot.centerServo.setPosition(robot.centerServo.getPosition()+0.04);
                }else if(gamepad2.b){
                    robot.centerServo.setPosition(robot.centerServo.getPosition()-0.04);
                }

                if(gamepad2.left_trigger>0){
                    robot.clawSpinner.setPosition(robot.clawSpinner.getPosition()+0.025);
                }else if(gamepad2.right_trigger>0){
                    robot.clawSpinner.setPosition(robot.clawSpinner.getPosition()-0.025);
                }


                    if(gamepad2.dpad_left ){//&& leftAxisPosition < 12000 /*+ armEncoderDiff*/){
                        robot.leftAxis.setTargetPosition(robot.leftAxis.getTargetPosition() + 20);
                        robot.rightAxis.setTargetPosition(robot.rightAxis.getTargetPosition() + 200);
                        angleTime = System.currentTimeMillis();

                    }else if(gamepad2.dpad_right){
                        robot.leftAxis.setTargetPosition(robot.leftAxis.getTargetPosition() - 20);
                        robot.rightAxis.setTargetPosition(robot.rightAxis.getTargetPosition() - 200);
                        angleTime = System.currentTimeMillis();

                    }

            }

            if (gamepad2.a && gamepad2.dpad_up){
                robot.clawSpinner.setPosition(0);
                robot.rightPulley.setTargetPosition(11400);
                robot.leftPulley.setTargetPosition(11400);
            }

            if (gamepad2.a && gamepad2.dpad_down) {
                robot.clawSpinner.setPosition(0);
                robot.rightPulley.setTargetPosition(3200);
                robot.leftPulley.setTargetPosition(3200);
            }
            if (gamepad2.a && gamepad2.dpad_right) {
                robot.rightAxis.setTargetPosition(2200);
                robot.leftAxis.setTargetPosition(2200);
            }
            if (gamepad2.a && gamepad2.dpad_left) {
                robot.rightAxis.setTargetPosition(800);
                robot.leftAxis.setTargetPosition(800);
            }
//            if (gamepad2.y) {
//                robot.leftPulley.setTargetPosition(0);
//                robot.rightPulley.setTargetPosition(0);
//                robot.leftAxis.setTargetPosition(0);
//                robot.rightAxis.setTargetPosition(0);
//            }
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

            telemetry.addData("lF", robot.lF.getDirection());
            telemetry.addData("rF", robot.rF.getDirection());
            telemetry.addData("lB", robot.lB.getDirection());
            telemetry.addData("rB", robot.rB.getDirection());
            telemetry.addData("Arm pos", robot.leftAxis.getCurrentPosition());
            telemetry.addData("claw servo", robot.grabber.getPosition());
            telemetry.addData("hand rotation", robot.clawSpinner.getPosition());
            telemetry.addData("Right direction", robot.rightAxis.getTargetPosition());
            telemetry.addData("Left direction", robot.leftAxis.getTargetPosition());
            telemetry.addData("Adjustment", adjustment);


            //telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }
}

//BACKUP CODE
/*


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
