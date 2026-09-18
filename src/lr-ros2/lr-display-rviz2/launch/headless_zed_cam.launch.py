# Copyright 2026 Landfill Rover
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the complete ZED/LR pipeline without a local RViz process."""

import importlib.util
from pathlib import Path


def _load_pipeline_launch():
    path = Path(__file__).with_name('display_zed_cam.launch.py')
    spec = importlib.util.spec_from_file_location('lr_display_zed_cam_pipeline', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Cannot load the display pipeline launch: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    """Reuse the complete pipeline while making RViz impossible to enable."""
    pipeline = _load_pipeline_launch()
    return pipeline.generate_launch_description(
        start_rviz_default='false',
        start_rviz_choices=('false',),
    )
