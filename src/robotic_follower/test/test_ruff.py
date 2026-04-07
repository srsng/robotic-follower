# Copyright 2017 Open Source Robotics Foundation, Inc.
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

# from ament_ruff.main import main
import pytest


@pytest.mark.ruff
@pytest.mark.linter
def test_ruff():
    """Run ruff as a pytest plugin."""
    # 使用 pytest 的主函数，并通过 --ruff 参数来执行代码风格检查
    # -q 参数用于安静模式，让输出更简洁
    result_code = pytest.main(["--ruff", "-q", __file__])
    assert result_code == 0, "Ruff linter found code style issues."


# @pytest.mark.ruff
# @pytest.mark.linter
# def test_ruff():
#     rc, errors = main(argv=[])
#     assert rc == 0, "Found %d code style errors / warnings:\n" % len(
#         errors
#     ) + "\n".join(errors)
