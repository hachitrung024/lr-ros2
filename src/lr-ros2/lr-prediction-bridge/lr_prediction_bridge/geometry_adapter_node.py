"""Compatibility executable backed by the shared prediction bridge."""

from .prediction_bridge_node import PredictionBridgeNode, run


class GeometryAdapterNode(PredictionBridgeNode):
    def __init__(self, **kwargs):
        super().__init__(role="geometry", **kwargs)


def main(argv=None):
    run(role="geometry", argv=argv)


if __name__ == "__main__":
    main()
