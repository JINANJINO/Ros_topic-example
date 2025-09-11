# Ros-topic-example
ROS topic과 관련된 기초 예제와 관련된 코드로 Publisher와 Subscriber를 직접 구현해 본 기초예제를 만들어보고 topic으로 통신해본 코드이다.
가장 먼저 workspace를 생성해야 한다. Home directory에서 다음 명령어를 통해서 Work Space를 생성하자

```mkdir ros_lecture```

이후에 만든 work space 공간으로 이동한 이후에 src 파일을 만들어야 한다.

1. ```cd ros_lecture```

2. ```mkdir src```

3. ```cd src```

src 디렉토리로 이동한 이후에 패키지를 생성해야한다. ros에서 패키지 생성 명령은 다음과 같다.

```ros2 pkg create [패키지명] [빌드 타입] [의존성 파일]```

여기서는 python언어로 작성할 것이므로 다음과 같은 명령어를 사용하면 된다.

```ros2 pkg create my_rclpy_package --build-type ament_python --dependencies rclpy std_msgs```

해당 패키지를 만들면 다음과 같은 구조가 만들어짐을 알 수 있다.
```
my_rclpy_package
    ├── my_rclpy_package
    │   ├── helloworld_publisher.py
    │   ├── helloworld_subscriber.py
    │   └── __init__.py
    ├── package.xml
    ├── resource
    │   └── my_rclpy_package
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
````        
이후에 다시 work space 공간으로 돌아와서 build를 해주어야 한다.(반드시 work space 공간에서 build를 시켜야 한다.)

```cd ~/ros_lecture```

```colcon build --symlink-install```

이제 Publisher와 Subscriber code를 작성해야 한다.
먼저 Publisher Code를 작성해보자. Publisher Code를 작성하기 위해서는 package로 들어가서 동일한 이름의 디렉토리에서 코드를 작성해야 한다.

```cd ~/ros_lecture/src/my_rclpy_package/my_rclpy_package```

이후  terminal창에 ```code helloworld_publisher.py``` 를 통해서 ```helloworld_publisher.py```코드를 작성하여 넣고  ```code helloworld_subscriber.py```를 통해서 ```helloworld_subscriber.py``` 코드를 작성하여 넣은 후
다시 기존의 package 디렉토리로 들어와서 ```setup.py``` 파이썬 파일을 ```code setup.py```를 통해서 파이썬 파일을 연다.
이후 ```entry point```를 작성하여 넣어주어야 한다. ```entry point```는 Build 이전에 진행해주어야 하며 ```Node 이름 = file의 경로```순으로 작성해주면 된다.
file의 경로는 ```패키지명.파이썬 파일명:main```순으로 작성해주며 되며,
해당 파일에서는 다음과 같이 작성하면 된다.
```
entry_points={
        'console_scripts': [
            'helloworld_publisher = my_rclpy_package.helloworld_publisher:main',
            'helloworld_subscriber = my_rclpy_package.helloworld_subscriber:main'
        ]
```

전체적인 ```setup.py```파일은 다음과 같다.
```
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

최종적으로 다시 workspace 공간으로 돌아와서 ```colcon build --symlink-install```을 통해서 Build를 작업해주면 된다.
