## docker commands

```bash
docker build -t nvidia_gemini_r1 .
```

```bash
docker run -it --name nvidia_gemini_r1 -e DISPLAY=host.docker.internal:0.0 nvidia_gemini_r1
```

```bash
docker exec -it nvidia_gemini_r1 bash
```


roslaunch mecanum_bot gazebo_test_robot.launch
rosrun mecanum_bot test_mecanum_robot.py 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
