package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

public class Mark13 extends Mecanum_Drive{
    public static final double driveWheelRadius = 0.05;
    public static final double driveLengthX = 0.336;
    public static final double driveLengthY = 0.39625;
    public static final double driveMotorMaxRPM = 369.75;

    //4
    public final double OUTTAKECLOSED = 0.40;
    public final double OUTTAKEMIDDLE = 0.30;
    public final double OUTTAKEOPEN = 0;

    public boolean[][] obstacles;
    public TwoWheelOdometry odo;

    public DcMotor leftAxis;
    String leftAxisInit = "leftAxis";

    public DcMotor rightAxis;
    String rightAxisInit = "rightAxis";

    public DcMotor leftPulley;
    String leftPulleyInit = "leftPulley";

    public DcMotor rightPulley;
    String rightPulleyInit = "rightPulley";



    public CRServo centerServo;
    String centerServoInit = "centerServo";

    public CRServo leftGrabber;
    String leftGrabberInit = "leftGrabber";

    public CRServo rightGrabber;
    String rightGrabberInit = "rightGrabber";

    public Servo pulleyServo;
    String pulleyServoInit = "pulleyServo";

    public final static double ARM_HOME = 0.0;
    public final static double ARM_MIN_RANGE = 0.0;
    public final static double ARM_MAX_RANGE = 1.0;


    public static int armEncoderDiff;
    public static int angleEncoderDiff;;

    public Mark13(LinearOpMode ln, double initialX, double initialY, double initialAngle) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_USING_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        switchReverseSide();
        attachLinearOpMode(ln);

        REV_IMU imu = new REV_IMU(ln, "imu", 1, initialAngle);

        File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");
        File verticalRadiusFile = AppUtil.getInstance().getSettingsFile("verticalRadius.txt");
        double horizontalRadius = Double.parseDouble(ReadWriteFile.readFile(horizontalRadiusFile).trim());
        double verticalRadius = Double.parseDouble(ReadWriteFile.readFile(verticalRadiusFile).trim());


        //armEncoderDiff = 0; //Integer.parseInt(ReadWriteFile.readFile(armEncoderFile).trim());
        odo = new TwoWheelOdometry(ln, new DeadWheel(rF, 0.0508, 8192, -1), new DeadWheel(rB, 0.0508, 8192, -1), imu, horizontalRadius, verticalRadius, initialX, initialY);
        odo.start();
        attachAngleTracker(imu);
        attachPositionTracker(odo);
        enableBrakes();

        leftAxis = ln.hardwareMap.dcMotor.get(leftAxisInit);
        rightAxis = ln.hardwareMap.dcMotor.get(rightAxisInit);
        leftPulley = ln.hardwareMap.dcMotor.get(leftPulleyInit);
        rightPulley = ln.hardwareMap.dcMotor.get(rightPulleyInit);

        centerServo =ln.hardwareMap.crservo.get(centerServoInit);
        leftGrabber =ln.hardwareMap.crservo.get(leftGrabberInit);
        rightGrabber = ln.hardwareMap.crservo.get(rightGrabberInit);
        pulleyServo = ln.hardwareMap.servo.get(pulleyServoInit);


        /***Sets all motors to run on encoders***/
        leftAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAxis.setTargetPosition(0);
        leftAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftAxis.setDirection(DcMotorSimple.Direction.REVERSE);

        rightAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAxis.setTargetPosition(0);
        rightAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPulley.setTargetPosition(0);
        leftPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftPulley.setDirection(DcMotorSimple.Direction.REVERSE);

        rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPulley.setTargetPosition(0);
        rightPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftAxis.setPower(0.4);
        rightAxis.setPower(0.4);
        leftPulley.setPower(0.4);
        rightPulley.setPower(0.4);

        armEncoderDiff = 0;
        angleEncoderDiff = 0;


    }

    public void SimpleForward(double power){
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(power);
    }

    public void SimpleStrafe(double power){
        lF.setPower(Range.clip(-power, -0.5, 0.5));
        lB.setPower(Range.clip(power, -0.5, 0.5));
        rF.setPower(Range.clip(power, -0.5, 0.5));
        rB.setPower(Range.clip(power, -0.5, 0.5));
    }



    public int getArmEncoderDiff() {
        return armEncoderDiff;
    }

    public void setArmEncoderDiff(int change) {
        armEncoderDiff += change;
        //ReadWriteFile.writeFile(armEncoderFile,Integer.toString(armEncoderDiff));
    }

    public int getAngleEncoderDiff() {
        return angleEncoderDiff;
    }

    public void setAngleEncoderDiff(int change) {
        angleEncoderDiff += change;
        //ReadWriteFile.writeFile(armEncoderFile,Integer.toString(armEncoderDiff));
    }

}
