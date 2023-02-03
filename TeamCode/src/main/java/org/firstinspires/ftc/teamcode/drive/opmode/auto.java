package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")//lblfrbrf
public class auto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory goForward = dt.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(100).build();
        dt.followTrajectory(goForward);

        Trajectory lineToPosition = dt.trajectoryBuilder(new Pose2d(10,10,0))
                .lineTo(new Vector2d(0,0))
                .build();
        dt.followTrajectory(lineToPosition);
    }
}
