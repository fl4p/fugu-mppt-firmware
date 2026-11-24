# web app editor with upload

Write a command-line tool to upload, download and view firmware configuration files.
Take a look at download.py .
The tool uses discover_scope_servers() to discover chips on the network.
Chips can be filtered by command line argument (regex matching against host or ip).

It then downloads the files on the chip with `download_ftp_tree()`.

The user can provide a local directory to diff the downloaded configurations with.
The user can see the entries with different value and can chose which fields to accept.
Existing comments in the files are preserved.
The script then uploads the changed files to the chip.

Example command:

```
conf-tool.py --hosts '.+' --local-conf config/fmetal
```

# web version

Create a web app to edit a set of ConfFiles. Single page HTML `conf-editor.html`.
Users can upload *.zip files or folders which follow the structure like ../../config/fmetal .
Each .conf file gets its own tab. Inspect each config value, try to find it in the firmware code and figure out unit (if
applicable) and a short description.
Display unit and description in the input form.
When users are done editing, they can download a zip file with the updated configuration files.

## general logic

* show all fields, even if they are missing in the file. inputs have a background text `<not set>` then.
* `<not set>` fields are not written to the file
* there is a way to add new entries (input form below)
* when loading new data (either from an uploaded file/folder or from serial port or Bluetooth), all existing data is
  cleared

## field inputs

* the input fields have a reasonable width
* show the data type of each field (string, byte, long, float, ...)
* display the unit inside the input elements, right aligned
    * GPIO pins have the unit "GPIO"
* there is a x -button next to the input that set the value to `<not set>` removing the entry from the file
    * the x-button is only visible if a value is present
* when changing or deleting a value, display its original value loaded from the file below the input (`was: ...`)
    * notice that `<not set>` and 0 is *not* the same. Make sure to compare with `===` operator
    * when deleting a field with the value 0, make sure that `was: ...` is still shown
    * strings can be empty, which is different from `<not set>`
* show the raw contents of the .conf file below
* when a value in a tab has been changed, show a small dot next to the tab name (changed marker)
    * when the user reverts all fields in a tab to its original values the changed marker disappears

# testing

* write a test for every feature described before
    * you can choose the language, put the test code under './test'
* test the page headlessly (jsdom or similar), if you can't report it before continuing
    * run test with a single .conf file, a folder and a zip file (create one)
* use regression testing

# serial port

An alternative way to the upload: users can feed in data directly from the device over a serial console.
Add a button that lets the user connect to a serial port (baud-rate 115200).
Once connected, the values can be read with the `get-config <file>` command, for example:

```
> get-config board.conf
I (267894) main: received serial command: 'get-config board.conf'
Conf '/littlefs/conf/board.conf:mcu' = 'esp32s3'
Conf '/littlefs/conf/board.conf:skip_assert' = '1'
Conf '/littlefs/conf/board.conf:adc_freq' = '3000'
Conf '/littlefs/conf/board.conf:pwm_freq' = '39000'
I (267988) main: OK: get-config board.conf
```

make sure to wait for the final confirmation `main: OK: get-config <file>`

To read the device's name use `hostname`.

## Serial port dev advice

First, try the communication with the device directly to get an understanding.
If the port is held by a `idf.py monitor` session, kill that session. If the port is held by another process, stop and
report.
You find the device serial port with cu.usbmodem* or cu.usbserial*, or "USB JTAG/serial debug unit"
Then headlessly test it with the Web Serial API. If you can't test, stop and report why.
If you cannot automate the native serial port-chooser dialog and there is no way to grant a physical serial port,
drive it with a mock navigator.serial whose port replays the real device's exact captured output.

# Bluetooth connection

An additional way to read (and write) config files is a BLE GATT Nordic UART Service (NUS) console.
It uses the same protocol as the wired serial console described earlier.
Put another button next to the Serial Connect button to scan for a BLE device, connect to the NUS console and fetch the
files.


# mqtt
Another way to communicate with the device is mqtt. See `scan_mqtt()` in `fugu_console.py` to learn how to discover host.
Implement the MQTT in JS, the browser will connect to the broker directly. Then use `MqttTransport` as reference on how
to communicate with the device. It uses the same protocol as the other transports (serial, BLE).
Add another button that opens a modal and scans the MQTT broker for devices and show them in a list in the modal.
The user can then chose one device to connect to, edit the files and upload (same procedure as serial, BLE)


# console
the user can open the device log, the one that opens when uploading changes to BLE/Serial/MQTT. 
The user can send commands on with a text input below the log.
the log container scroll follows new log lines when scrolled to the bottom. when the user scrolls up, it halts following.
the log lines have ANSI color codes. use this parser https://github.com/rburns/ansi-to-html/blob/master/lib/ansi_to_html.js to display the text in the right
color.
lines in the log should not wrap.
# upload

Add an upload button that uploads *only changed* fields to the device.
Use the `set-config <file> <key> <value>` command to upload values.
Use `del-config <file> <key>` command to remove values that were previously set on the device.
Send this command to either serial or Bluetooth connection, depending on which was used last.
*Before submitting*, show a confirmation dialog to the user listing all commands that will be sent.
Display the commands sent and the device response during upload.
After each upload, the input field is considered unchanged (this also propagates to the tab changed marker). 

# cron

scan the firmware code for ConfFile keys and update the html tool


show all tabs for all known files, regardless of their existence in the load (allow user can create a new files).


