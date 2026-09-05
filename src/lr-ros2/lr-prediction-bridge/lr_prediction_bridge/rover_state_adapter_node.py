"""Compatibility executable backed by the shared prediction bridge."""

from .prediction_bridge_node import PredictionBridgeNode, run


class RoverStateAdapterNode(PredictionBridgeNode):
    def __init__(self, **kwargs):
        super().__init__(role="rover_state", **kwargs)


def main(argv=None):
    run(role="rover_state", argv=argv)


if __name__ == "__main__":
    main()
