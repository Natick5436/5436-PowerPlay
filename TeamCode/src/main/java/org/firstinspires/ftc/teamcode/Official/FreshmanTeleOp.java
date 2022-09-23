package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.FreshmanBot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FreshmanTeleOp",group="TeleOp")
public class FreshmanTeleOp extends LinearOpMode {
    FreshmanBot robot;

    public void runOpMode() {
        robot = new FreshmanBot(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;

        waitForStart();


        while (opModeIsActive()) {
            if (fastMode) {
                drivePower = 0.5;
            } else {
                drivePower = 0.3;
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
                //robot.getAngleTracker().resetAngle(Math.PI/2);
                backDown = true;
            }
            backDown = gamepad1.back;





            //telemetry.addData("Raw IMU", ((REV_IMU)robot.getAngleTracker()).imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS));


            //telemetry.addData("Horizontal", robot.odo.h.distanceTraveled());
            //telemetry.addData("Vertical", robot.odo.v.distanceTraveled());
            //telemetry.addData("Angle", robot.getAngle());
            //telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());
            telemetry.addData("down",gamepad1.dpad_down);
            telemetry.addData("up",gamepad1.dpad_up);
            telemetry.addData("fastMode",fastMode);
            telemetry.addData("lb", robot.lB.getPower());
            telemetry.addData("rb", robot.rB.getPower());
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("rF", robot.rF.getPower());
            //telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }
}
