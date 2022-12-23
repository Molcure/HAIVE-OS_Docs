# Raspberry Pi Setup

This describes setting up a Raspberry Pi to run a headless version of Ubuntu.

- Get Raspberry Pi 4 Model B
- Get MicroSD card 16GB
- Download [Ubuntu Server 22.04.1 LTS](https://ubuntu.com/download/raspberry-pi/thank-you?version=22.04.1&architecture=server-arm64+raspi)
- Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- Set the following in advanced settings before flashing (Wifi settings will be set later):

![Raspberry Pi Imager Settings](img/ubuntu_setup.png 'Raspberry Pi Imager Settings')

After booting Ubuntu, we setup Wifi with a static IP. Open the following file:

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

Put these contents:

```bash
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: false
      optional: true
      addresses: [192.168.2.250/24]
      routes:
        - to: default
          via: 192.168.2.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "<wifi-name>":
          password: "<wifi-password>"
```

Save the file and run the following command:

```bash
sudo netplan apply
```

Confirm the connection by typing:

```bash
ping www.google.com
```

Try to `ssh` into the Raspberry Pi from a machine on the same network:

```bash
ssh ubuntu@192.168.2.250
```
