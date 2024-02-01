# widowx-project

Стек софта для работы с манипулятором WIDOWX-MKII (Trossen Robotics)

<p align="center">
    <img src="https://www.trossenrobotics.com/images/PImages/widowx-a.jpg" width=300>
</p>


## список исходников репозитория

[пакет ROS для манипулятора WidowX-MKII (ветка `master`)](https://github.com/Interbotix/widowx_arm)

[пакет ROS для платы ArbotixM (ветка `noetic-devel`)](https://github.com/vanadiumlabs/arbotix_ros)

[архив с прошивкой платы](https://github.com/trossenrobotics/arbotix/archive/master.zip)

## начало работы

* в папке `src` выполним команду, которая установит все зависимости

    ```bash
    rosdep install -y -r \
        --from-paths src/ \
        --ignore-src \
        --rosdistro noetic
    ```

* соберем пакеты командой

    ```bash
    catkin build
    ```

* добавим устойство `ttyUSB_WIDOWX`

    ```bash
    sudo cp $(rospack find widowx_arm_controller)/config/58-widowx.rules /etc/udev/rules.d
    ```

    ```bash
    sudo service udev reload
    ```

    ```bash
    sudo service udev restart
    ```

    ```bash
    sudo udevadm trigger
    ```
