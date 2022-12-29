package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark13;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Reset",group="TeleOp")
public class ResetProgram extends LinearOpMode {
    Mark13 robot;

    public void runOpMode() {
        robot = new Mark13(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;

        long liftTime = System.currentTimeMillis();
        long angleTime = System.currentTimeMillis();


        //Pat's test code
        //leftServoPosition = Range.clip(leftServoPosition, robot.ARM_MIN_RANGE,robot.ARM_MAX_RANGE);
        //robot.leftServo.setPosition(leftServoPosition);

        waitForStart();


        while (opModeIsActive()) {

            if(gamepad2.dpad_up){
                robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() + 5);
                robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() + 5);

            }else if(gamepad2.dpad_down){
                robot.leftPulley.setTargetPosition(robot.leftPulley.getTargetPosition() - 5);
                robot.rightPulley.setTargetPosition(robot.rightPulley.getTargetPosition() - 5);

            }

            //telemetry.addData("Raw IMU", ((REV_IMU)robot.getAngleTracker()).imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS));


            //telemetry.addData("Horizontal", robot.odo.h.distanceTraveled());
            //telemetry.addData("Vertical", robot.odo.v.distanceTraveled());
            //telemetry.addData("Angle", robot.getAngle());
            //telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());

            telemetry.addData("fastMode",fastMode);
            telemetry.addData("lb", robot.lB.getPower());
            telemetry.addData("rb", robot.rB.getPower());
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("rF", robot.rF.getPower());

            telemetry.addData("leftTrigger", gamepad2.left_trigger);
            telemetry.addData("leftBumper", gamepad2.left_bumper);
            //telemetry.addData("power", robot.centerServo.getPower());

            telemetry.addData("armPos", robot.leftPulley.getTargetPosition());




            //telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }

    }
}
