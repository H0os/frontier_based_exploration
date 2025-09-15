# Frontier-Based Exploration (ROS 2 Jazzy)

[![CI](https://github.com/H0os/frontier_based_exploration/actions/workflows/ci.yaml/badge.svg?branch=reproducible-setup)](https://github.com/H0os/frontier_based_exploration/actions/workflows/ci.yaml)

Reproducible ROS 2 stack for **frontier-based autonomous exploration** with:
- Frontier detection (map utilities)
- Exploration controller
- Optional **Bayesian optimisation** for decision-making
- One-command run, containerised with ROS 2 **Jazzy** + TurtleBot 4 Gazebo sim

> **What this repo gives you**  
> A clean, one-command baseline to run exploration in simulation.
---

## Features
- ✅ **ROS 2 Jazzy** base image (official `osrf/ros:jazzy-desktop`)
- ✅ **TurtleBot 4 Gazebo** simulator + iRobot create nodes
- ✅ **One-command run** via Docker
- ✅ **CI + smoke test badge** (builds container, verifies ROS tools)

---

## Quickstart

### Docker (recommended for portability)
First, install Docker and Docker Compose using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with NVIDIA GPU support, you can optionally install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).


First, clone this repository and go into the top-level folder:

```
git clone https://github.com/H0os/frontier_based_exploration.git
cd frontier_based_exploration
```

Build the Docker images.
This will take a while and requires approximately 5 GB of disk space.

```
docker compose build
```

---

## Basic Usage

We use [Docker Compose](https://docs.docker.com/compose/) to automate building, as shown above, but also for various useful entry points into the Docker container once it has been built.
**All `docker compose` commands below should be run from your host machine, and not from inside the container**.

To enter a Terminal in the overlay container:

```
docker compose run overlay bash
```

Once inside the container, you can verify that display in Docker works by starting a Gazebo simulation with Nav2 support:

```
ros2 launch tb_worlds tb_demo_world.launch.py
```

Alternatively, you can use the pre-existing `sim` service to do this in a single line:

```
docker compose up demo-world
```

## Frontier Based Exploration Demo

To start the full demo in non-ml mode, copy and paste the following block of commands:
```
docker compose up -d demo-world
docker compose up -d demo-explore
docker compose up -d demo-frontier-detector-grid
```

Once the simulation starts you can use the following commands to connect to the container:
```
docker exec -it frontier_based_exploration-demo-frontier-detector-grid-1 bash
```

And then to view metrics/visualisation:
```
eog graph_performance_metrics.png
eog result.jpg
```

For the image based ml detector use:
```
docker compose up -d demo-world
docker compose up -d demo-explore
docker compose up -d demo-frontier-detector-ml
```

Once the simulation starts you can use the following commands to connect to the container:
```
docker exec -it frontier_based_exploration-demo-frontier-detector-ml-1 bash
```

And then to view metrics/visualisation:
```
eog graph_performance_metrics.png
eog result.jpg
```

To close the simulation use:
```
docker compose down
```