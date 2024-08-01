Enables interaction between dynamic reconfigure nodes and topics and other dr nodes.

# rust node

```
ROS_PACKAGE_PATH=`rospack find dynamic_reconfigure` cargo run --example example_server --release
ROS_PACKAGE_PATH=`rospack find dynamic_reconfigure` cargo run --release dynrec_server foo bar2
```
