# ROS Topic Example

This repository provides a basic example of **ROS topic communication** using custom **Publisher** and **Subscriber** nodes.  
The goal is to create a simple workspace, write Publisher and Subscriber code, and communicate between them via ROS topics.

![Nodes, Topics, and Services in ROS](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

> 📃 For more details on this topic, please refer to the document
https://share.google/PKZuHiYb0PJ5hXBNY

---

## 1. Create Workspace

First, create a workspace in your home directory:

```bash
mkdir ros_lecture
```

Move into the workspace and create the `src` directory:

```bash
cd ros_lecture
mkdir src
cd src
```

---

## 2. Create Package

Inside the `src` directory, create a ROS 2 package. Since we are using Python, run:

```bash
ros2 pkg create my_rclpy_package --build-type ament_python --dependencies rclpy std_msgs
```

The created package structure will look like this:

```
my_rclpy_package
├── my_rclpy_package
│   ├── helloworld_publisher.py
│   ├── helloworld_subscriber.py
│   └── __init__.py
├── package.xml
├── resource
│   └── my_rclpy_package
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## 3. Build Workspace

Return to the workspace root (`~/ros_lecture`) and build the workspace:

```bash
cd ~/ros_lecture
colcon build --symlink-install
```

---

## 4. Write Publisher and Subscriber Code

Move into the package directory:

```bash
cd ~/ros_lecture/src/my_rclpy_package/my_rclpy_package
```

Create and edit the publisher node:

```bash
code helloworld_publisher.py
```

Create and edit the subscriber node:

```bash
code helloworld_subscriber.py
```

---

## 5. Configure `setup.py`

After writing the nodes, go back to the package root and edit `setup.py`:

```bash
code setup.py
```

Add **entry points** to register the nodes:

```python
entry_points={
        'console_scripts': [
            'helloworld_publisher = my_rclpy_package.helloworld_publisher:main',
            'helloworld_subscriber = my_rclpy_package.helloworld_subscriber:main'
        ]
}
```

Full example of `setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'my_rclpy_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinhan',
    maintainer_email='jinhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld_publisher = my_rclpy_package.helloworld_publisher:main',
            'helloworld_subscriber = my_rclpy_package.helloworld_subscriber:main'
        ],
    },
)
```

---

## 6. Final Build

Return to the workspace root and rebuild:

```bash
cd ~/ros_lecture
colcon build --symlink-install
```

After this, the Publisher and Subscriber nodes can be run to communicate over ROS topics.
