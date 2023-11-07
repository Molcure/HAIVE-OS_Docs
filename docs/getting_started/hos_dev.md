# Coding in HAIVE-OS

## Start
To develop on HAIVE-OS, we are using [Bitbucket](https://bitbucket.org/molcure/workspace/repositories/). Bitbucket is used for Kanban tracking since it's link to Jira, for pull request purpose and so on

HAIVE have then 2 repository :

- [HAIVE-OS github](https://github.com/Molcure/HAIVE-OS)
- [HAIVE-OS bitbucket](https://bitbucket.org/molcure/haive-os/src)

Developers should regulary push on the bitbucket repository
if you already have the HAIVE-OS repository somewhere you can have both remote repository by doing this command :
```shell
git remote add source https://<UserApp>@bitbucket.org/molcure/haive-os.git
```

please check bitbucket guidance for User App creation, they are here to replace ssh_keys.

if you create a new feature, please create a branch in feature/ folder
if you are fixing a bug, please create a branch inside bug/ folder

if you want to update your files on the rasberry Pi, there is several solution : 
-use the Visual Studio Code ssh plugin
or
```shell
scp -r <your_folder> ubuntu@192.168.X.X:<destination>
```
or
- you're using github on your Raspberry Pi:
for pulling a branch :
```shell
git fetch --all 
```
#this will update both remote repositories (github and bitbucket remote are updated locally, UserApp password might be asked)
```shell
git pull
``` 
(UserApp password might be asked)

If you want to push your update from Rasberry Pi to a repository :
for github :
```shell
git push <branch_name>
```
for bitbucket :
```shell
git push source HEAD:<branch_name>
```

## Service

When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a .srv file.

Currently each firmware APICall (recorded in Airtable) are service between HAIVE-OS and the device.

Whenever you call a service in ROS2 you need to fulfill those requirement :
 - use async_call (not necessary but avoid lot of trouble)
 - ensure that your node are threaded, that is to say just in an other script for ROS, in a launch file, or threaded with python Thread package
 - When those node are create, please create each time a MultiThreadExecutor() and add_add() method for each node (each of them need a different executor)
 - By default create all your ros_event (subscriber, publisher, client, service ,...) with a  ReentrantCallbackGroup() unless you want some particular behavior, please check ROS2 documentation about callback groups.
 - use spin_once() or spin_until_future() when you need the result of some service call and then come back to your protocol.