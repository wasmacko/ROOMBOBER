# ROOMBOBER (temporary dev name)
Project of autonomic robot for scanning indoor areas\
Built with ESP-IDF v6.1.0

## Build it by yourself:
* Clone this repository
* Open it with VSCode with ESP-IDF v6.1.0 installed
* Run in IDF terminal (there is a button on bottom of VSCode):
```
idf.py reconfigure
```
* Change in 'sdkconfig' 'CONFIG_FREERTOS_HZ' value from 100 to 1000
* Compile build:
```
idf.py all
```
* Flash it to device:
```
idf.py flash
```