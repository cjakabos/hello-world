Prerequisites:
[Follow this tutorial](https://github.com/chalmers-revere/opendlv-tutorial-kiwi/tree/master/opendlv-perception-helloworld-python)

# Processing image data with openCV for single images or full folders

```bash
python3 python-perception-dev.py <image_path>
python3 python-perception-dev.py <folder_path>
```

# Processing image data with OpenCV and for rec files

* Start the h264 replay and web app as follows. Remember to change the filename in the yml file. The replay will start automatically when the program starts, including a video stream put in shared memory, and you can use the web app to see data UDP multicast data. You should see a video running on your screen before continuing.

```bash
docker-compose -f h264-replay-viewer.yml up
```

* Open another terminal. Then run the Python (note that you need version 3) module from the folder `python-perception-dev` (don't forget to run 'make' once for the OpenDLV message set):
```bash
python3 opendlv-perception-dev.py
```

The application should start and wait for images to come in. (This is a lie as far as I can tell...)

You can stop the Python application by pressing `Ctrl-C`.

---
