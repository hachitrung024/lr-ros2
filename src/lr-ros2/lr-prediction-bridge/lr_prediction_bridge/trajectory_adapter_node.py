"""Compatibility executable backed by the shared prediction bridge."""

from .prediction_bridge_node import PredictionBridgeNode, run


class TrajectoryAdapterNode(PredictionBridgeNode):
    def __init__(self, **kwargs):
        super().__init__(role="trajectory", **kwargs)


def main(argv=None):
    run(role="trajectory", argv=argv)


if __name__ == "__main__":
    main()
