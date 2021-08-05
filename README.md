# opencvcompetition2021 | AI CYCLIST GUARDIAN

Here are all the project code including: python script, arduino scripts and the stls 3d files.

## main-python script

In the folder main-python is the python code to work with the OAK-D and make the vehicles detection and the logic to create the formatted data to send to the bracelets... the core of the project.

The project is running in the raspberry pi3B+ raspbian OS with Python 3.7.3. and if you want to run in windows 10 you need Python 3.9.5

**Libraries**
Run the install_requirements.py script to install all the libraries needed for AOK to work
Here is the luxonis official documentation page 

[docs.luxonis.com/en/latest/](https://docs.luxonis.com/en/latest/)


Youy need also install PyBluez to run the bluetooth connection

	pip install pybluez


[https://github.com/pybluez/pybluez](https://github.com/pybluez/pybluez)


###### Run the Python Script

	python3 cyclist_guardian-v5.py


**Arguments**
| Shor param | param | Description |
|     :---:      |     :---:      |:---         |
| -r     | --record  | If you want to save a OAK-D and reference video in videos folder |
| -f     | --recordframes  | If record Videos enable, set the frame rate ( default is 4 for better performance ) |
| -b     | --bluetoothname  | The name of the bluetooth device to connect ( default is D-Gloves ) |
| -i     | --invertbracelets  | Invert Bracelets set signal left/right |


## bracelets-arduino

Here is the arduino code to load on ESP32 lolin, one for the master and one for the slave (client)

You can make the full functionality bracelets with the follow diagram:

![bracelets diagram](https://github.com/vecnostudioar/opencvcompetition2021/blob/main/bracelets-diagram.jpg)

