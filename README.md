## Usage

The monitor app is only available at pepper's tablet. There is no external access.

The app subscribes to a ROS topic named ```/pepper_monitor/message``` with type of ```std_msgs/String```. It shows the user defined messages after the topic gets published from ROS. A message contains media (emoji expression, image or camera stream) or/and dialog (text, input or buttons). The data published by topic ```/pepper_monitor/message``` is a list of messages for displaying. Each message should have a unique name in order to identify them. They will be displayed in order and each of them will last a certain amount of time that is configurable. There should be at least one message when publishing on the topic.

The app will publish the message name on topic ```/pepper_monitor/status``` to ROS once the message is displayed. The message is in following format: 
 
```json
{
  "status": "name of the message"
}
```

If the displayed message enables input or buttons, it will also publish a feedback with the message name and a value comes from the input or button on topic ```/pepper_monitor/feedback```. The message is in following format: 

```json
{
  "name": "name of the message",
  "value": "a value from a input or button"
}
```

Following is a description of the object used to define a display message.

- `name` -- Name of the message. Will be used for identify feedback or status sent from monitor app. 
- `layout` -- Display layout. Only matters when showing both media and dialog. Use `column` to place media on top of the dialog. Use `row` to put media on the left and dialog on the right. Defaults to `column`.
- `timeout` -- How long will this message last in second. Defaults to 60. 
- `media` -- A object for displaying graphical message:
  - `type` -- `image`, `emoji` or `camera`.
  - `src` -- Source of the graphical message: 
    - For `image`, it should be the name of the image file (with extension) in `/home/nao/monitor/assets/images/`. Add new images if needed to this directory via `scp` or any SFTP client.
    - For `emoji`, there are 13 animated smiley expressions available: *angry*, *applaud*, *bye*, *complacent*, *delicious*, *disappointed*, *giggle*, *kiss*, *laugh*, *love*, *sad*, *shock* and *shy*
    - For `camera`, it should be the name of a ROS topic of type `sensor_msgs/Image`. e.g. `/pepper/camera/front/image_raw`
- `dialog` -- A object for displaying text, input or buttons.
  - `text` -- A text message. 
  - `input` -- Set to `True` for showing a input box. The input value will be sent to ROS as a feedback message with the name of current message.
  - `buttons` -- A list of objects that define buttons. The object has following attributes:
    - `color`: Available colors: *red*, *blue* and *green*. 
    - `label`: Label of the button
    - `value`: When button is clicked, this will be sent to ROS as a feedback message with the name of current message.
 
Before publishing the display messages to monitor app, in your python program, you need to use `json` module to stringify a python list:
```python
import rospy
from std_msgs.msg import String
import json

rospy.init_node('pepper_monitor')
pub = rospy.Publisher('pepper_monitor/message', String, queue_size=1)
messages = [...] # a list of message dicts
pub.publish(json.dumps(messages))
```
Before processing topic published from monitor app, in your python program, you need to deserialize topic message to a python object:
```python
def callback(message):
    rospy.loginfo(json.loads(message.data))

rospy.init_node('pepper_monitor')
status_sub = rospy.Subscriber('pepper_monitor/status', String, callback)
feedback_sub = rospy.Subscriber('pepper_monitor/feedback', String, callback)
```