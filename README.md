# rostoyo

This is an ros2 wrapper package for the Mitutoyo USB-ITN cable demonstration by Richard Bryn (https://github.com/rabryan/pytuyo).

## Installation
To install the package, do the following steps:

Install `pyusb` with:
```
sudo pip install pyusb
```

Clone the wrapper-repository into your colcon workspace with:
```
git clone --recurse-submodules https://github.com/DLR-MO/rostoyo
```

Now build package using colcon:
```
colcon build
```

To avoid the need of the `sudo` comand modify the devicerules in `/etc/udev/rules.d/` by using the following command and replug the dial gauge.
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0fe7", ATTRS{idProduct}=="4001", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/60-mitutoyo_usb.rules
```
This is also described by Richard Bryn (https://github.com/rabryan/pytuyo).



## Run Example

Get sure you have sourced your setup-file with
```source install/local_setup.bash```
or if you using `zsh` 
```source install/local_setup.zsh```

Than start the example node with
```
ros2 run rostoyo rostoyo_node
```

You also can subscribe the topic to process the data. 
A we provide an example subsriber that can be started with:
```
ros2 run rostoyo rostoyo_subscriber.py
```

If you only want to request the value sometimes, you can use a service. The node will read the dial gauge value ones and send back the result. 
We provide an exemplay client that you can start with:
```
ros2 run rostoyo rostoyo_client.py
```

# Thanks
Thanks to Oliver Mümken for the original implementation of the package.
