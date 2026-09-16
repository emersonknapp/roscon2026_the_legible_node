$ ros2 nodl describe /talker
nodl_version: 2
publishers:
- name: /chatter
  qos:
    depth: 7
    durability: VOLATILE
    history: KEEP_LAST
    liveliness: AUTOMATIC
    reliability: RELIABLE
  type: example_interfaces/msg/String
