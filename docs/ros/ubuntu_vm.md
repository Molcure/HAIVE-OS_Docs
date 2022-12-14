# Multipass VM

This describes setting up a virtual Ubuntu 22.04 instance running on multipass.

## Setup

Download and install multipass: https://multipass.run/install.
Launch the instance with the following command:
```shell
multipass launch 22.04 -n primary
```
Use the `shell` command to open a shell prompt on the instance:
```shell
multipass shell primary
```

## Sharing Folders

To share a folder with the instance, from your host system type:
```shell
multipass mount <host-path> primary:<instance-path>
```

## Managing Instances
Stop the instance:
```shell
multipass stop <instance-name>
```
Start the instance:
```shell
multipass start <instance-name>
```
Use docs to see a complete overview of multipass: https://multipass.run/docs

## Setup SSH
Login to the instance
```shell
multipass shell primary
```
Set a password
```shell
sudo passwd ubuntu
```
Enable ssh by setting `PasswordAuthentication yes` in
```shell
sudo nano /etc/ssh/sshd_config
```
Restart `sshd` service
```shell
service sshd restart
```
You can find your instance's IP address by typing
```shell
hostname -I
```
Logout and copy your host machhine ssh key to the instance
```shell
ssh-copy-id ubuntu@<ip-address>
```

## Remote Development using SSH
In VS Code, select `Remote-SSH: Connect to Host...` from the Command Palette (`F1`, `⇧⌘P`) and type
```shell
ubuntu@<ip-address>
```
You may have to install `Remote - SSH` extension first.
