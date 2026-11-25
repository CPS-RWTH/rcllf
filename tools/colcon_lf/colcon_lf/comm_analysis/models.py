# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Data models for the inter-node communication analysis layer."""

from __future__ import annotations

from dataclasses import dataclass, field

# What a connection's endpoint strings (publishers, subscribers, server,
# clients) name. NODE_KEY: the launch node key, one per node instance, the key
# of ``SystemGraph.nodes``. NODE_NAME: ``NodeInfo.name``, the node
# name in the code, which every instance of one class shares; system.json files
# from an older analyzer carry no ``endpoint_ids`` field and mean this.
NODE_KEY = "node_key"
NODE_NAME = "node_name"


@dataclass
class TopicConnection:
    """A pub/sub topic with all publisher and subscriber node names."""

    topic: str              # Fully resolved topic name (e.g. "/robot_1/sensor_data")
    message_type: str       # Fully-qualified C++ type from the publisher
    publishers: list[str]   # node keys (see SystemCommunication.endpoint_ids)
    subscribers: list[str]  # node keys (see SystemCommunication.endpoint_ids)
    fields: list[dict] | None = None  # [{"type": "float64", "name": "temperature"}, ...]
    # Declared logical-time offset (LF `after` delay, LET-style) for this link,
    # in nanoseconds. None = use generator default. This is a contract on when
    # the value becomes visible in logical time, not a latency measurement.
    # Measured DDS WCET or a user override may inform the choice of value;
    # not extracted here. See docs/concepts/time-and-coordination.md.
    after_delay_ns: int | None = None


@dataclass
class ServiceConnection:
    """A ROS2 service with its server and all client node names."""

    service: str            # Fully resolved service name
    service_type: str       # Fully-qualified C++ type
    server: str             # node key (see SystemCommunication.endpoint_ids)
    clients: list[str]      # node keys (see SystemCommunication.endpoint_ids)
    request_fields: list[dict] | None = None
    response_fields: list[dict] | None = None


@dataclass
class ActionConnection:
    """A ROS2 action with its server and all client node names."""

    action: str             # Fully resolved action name
    action_type: str        # Fully-qualified C++ type
    server: str             # node key (see SystemCommunication.endpoint_ids)
    clients: list[str]      # node keys (see SystemCommunication.endpoint_ids)
    goal_fields: list[dict] | None = None
    result_fields: list[dict] | None = None
    feedback_fields: list[dict] | None = None


@dataclass
class SystemCommunication:
    """All inter-node communication connections in a ROS2 system."""

    topics: list[TopicConnection] = field(default_factory=list)
    services: list[ServiceConnection] = field(default_factory=list)
    actions: list[ActionConnection] = field(default_factory=list)
    # NODE_KEY or NODE_NAME; the default reads a file written before endpoints carried the node key as it was written.
    endpoint_ids: str = NODE_NAME
