# OmniCommand Robot

## 🤖 Overview
OmniCommand Robot is an intelligent 4-omni-wheeled robot platform that combines ROS1 with advanced LLM capabilities for natural language command processing. The robot can execute various movement commands through natural language input, making it ideal for research, education, and prototyping applications.

### Core Capabilities
- Natural language command processing (e.g., "Move forward 10 units", "Make a square of 10 units")
- Object following with computer vision
- Real-time velocity control via ROS topics
- Remote monitoring and visualization through Foxglove

### Technical Stack
- **Base Platform**: 4-omni-wheeled robot
- **Operating System**: Ubuntu 20.04 (Containerized)
- **Framework**: ROS1 Noetic
- **Command Processing**: Gemini 1.5 Flash Large Language Model
- **Hardware Target**: NVIDIA Jetson Nano
- **Visualization**: Foxglove Studio

## 🚀 Quick Start

### Development Environment

```bash
# Build development image
docker build -t nvidia_gemini_r1 .

# Run development container
docker run -it \
    --name nvidia_gemini_r1 \
    -e DISPLAY=host.docker.internal:0.0 \
    -v $(pwd)/src:/workspace/src \
    nvidia_gemini_r1

# Core development commands
roslaunch mecanum_bot gazebo_test_robot.launch  # Launch simulation
rosrun mecanum_bot test_mecanum_robot.py        # Run robot controller
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  # Manual control
```

### Deployment (Jetson Nano)

```bash
# Build deployment image
docker build -f deploy_dockerfile -t nvidia_gemini_r1_deploy .

# Run deployment container
docker run -it \
    --name nvidia_gemini_r1_deploy \
    --runtime nvidia \
    --network host \
    -e DISPLAY=:0 \
    nvidia_gemini_r1_deploy

# Launch robot
roslaunch mecanum_bot gazebo_test_robot.launch
rosrun mecanum_bot test_mecanum_robot.py
```

## 🔧 Development Tools

### Docker Container Management

```bash
# Access running container
docker exec -it nvidia_gemini_r1 bash

# View container logs
docker logs nvidia_gemini_r1 -f

# Container lifecycle management
docker stop nvidia_gemini_r1
docker rm nvidia_gemini_r1
docker system prune -f  # Clean unused resources
```

### Debugging Tools

```bash
# Process inspection
docker exec -it nvidia_gemini_r1 ps aux

# Container details
docker inspect nvidia_gemini_r1

# Interactive session
docker attach nvidia_gemini_r1
```

## 🌟 Key Features

### Command Processing Pipeline
1. Natural language commands are received through a ROS topic
2. Gemini 1.5 Flash LLM processes and interprets the commands
3. Interpreted commands are converted to velocity vectors
4. Velocity commands are published to `/cmd_vel` topic
5. Robot's omni-wheel system executes the movement

### Visualization & Monitoring
- Real-time robot state visualization in Foxglove
- Command execution monitoring
- Sensor data visualization
- Performance metrics tracking

## 🛠️ Future Development

### Short-term Goals
- [ ] LLM fine-tuning for improved command interpretation
- [ ] Basic obstacle avoidance implementation
- [ ] Performance optimization for Jetson Nano

### Long-term Vision
- [ ] Migration to ROS2 Humble
- [ ] Advanced perception model integration
- [ ] Multi-robot coordination capabilities
- [ ] Cloud-based command processing

## 👥 Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS coding standards
- Include unit tests for new features
- Update documentation as needed
- Maintain compatibility with Jetson Nano

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

- GitHub Issues: Bug reports and feature requests
- Discussions: General questions and community interaction
- Wiki: Detailed documentation and guides
