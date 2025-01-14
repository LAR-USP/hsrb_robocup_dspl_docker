About the HSR
--------------------

Please refer to the following to obtain information about Human Support Robot (HSR) and use it in your paper.

https://robomechjournal.springeropen.com/articles/10.1186/s40648-019-0132-3

docker installation
--------------------

In order to run the simulator, docker and docker-compose are necessary.

In the case of a Windows or Mac environment, please install docker for Windows or Mac respectively.

In the case of Linux, please input the following commands and install docker.

```sh
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sh get-docker.sh
```

If you input the following command, even regular users will be able to execute the docker command.

```sh
$ sudo usermod -aG docker <USERNAME>
```

After executing the above command, log out then log in again.

Input the following command, then verify that docker can execute correctly.

```sh
$ docker info
```

Input the following commands and install docker-compose.
As the docker-compose that can be installed via apt-get is old,
please input all of the following commands to install the newest version of docker-compose.

```sh
$ sudo apt-get remove docker-compose
$ COMPOSE_VERSION=$(wget https://api.github.com/repos/docker/compose/releases/latest -O - | grep 'tag_name' | cut -d\" -f4)
$ sudo wget https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-`uname -s`-`uname -m` -O /usr/local/bin/docker-compose
$ sudo chmod 755 /usr/local/bin/docker-compose
```

Usage
------

Download all of the images necessary for running the simulator.
As you will be downloading a large amount of data,
please execute the following command in an environment that is connected to a high speed network.

```sh
$ docker-compose pull
```

Starting the simulator
----------------------

Please input the following command and start the simulator.

```sh
$ docker-compose up --build
```

Please open each of the following URLs in a browser, then move on to development.

- The simulator's screen http://localhost:3000
- IDE http://localhost:3001
- jupyter notebook http://localhost:3002


Please open each of the following URLs in a browser, then move on to development.

- The simulator's screen http://localhost:3000
- IDE http://localhost:3001
- jupyter notebook http://localhost:3002

Operation within the docker host PC
-----------------------------------

In order to communicate from the host PC that is running the docker image with the simulator's roscore,
it is necessary that ROS_MASTER_URI is set appropriately.
If you source the script that is located directly under this package as illustrated below,
then it is possible to set ROS_MASTER_URI.

```sh
$ source ./set-rosmaster.sh
```

After starting the simulator, please check that ROS communication is working using the host PC.

Authors
---------------
 * Yosuke Matsusaka

Contact
---------------
 * HSR Support <xr-hsr-support@mail.toyota.co.jp>

LICENSE
---------------
This software is released under the BSD 3-Clause Clear License, see LICENSE.txt.
