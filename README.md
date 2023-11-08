# TVMD Avionic Functionality Test

This project test the basic functionalities including IMU, and barometers.

## Getting Started
To upload the code, you need to hold the `Reset` and `Boot` buttons on the Avionic board, then release the `Reset` button first before esp_tool.py starts uploading the code. When you see the message `Connecting...`, you can release the `Boot` button.

## Troubleshooting
When you see the ICM throws an device ID error, you can try to power off the board and power on again. If the error still exists, you can try to re-solder the ICM chip.