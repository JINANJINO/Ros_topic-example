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

만약 VSCode를 설치하였다면 ```code helloworld_publisher.py```를 통해서 파이썬 파일을 만들자.
