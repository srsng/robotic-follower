"""测试 CGNL 模块。"""

import torch
import pytest
from perception.detection.models.cgnl import CGNLBlock, CGNLNeck


def test_cgnl_block_forward():
    """测试 CGNL 块前向传播。"""
    batch_size = 2
    in_channels = 64
    num_points = 1024

    block = CGNLBlock(in_channels=in_channels, reduction=2)
    x = torch.randn(batch_size, in_channels, num_points)

    output = block(x)

    assert output.shape == x.shape
    assert not torch.isnan(output).any()


def test_cgnl_neck_forward():
    """测试 CGNL Neck 前向传播。"""
    batch_size = 2
    in_channels = [64, 128]
    out_channels = [128, 256]
    num_points = 1024

    neck = CGNLNeck(
        in_channels=in_channels,
        out_channels=out_channels,
        num_blocks=2
    )

    inputs = [
        torch.randn(batch_size, in_channels[0], num_points),
        torch.randn(batch_size, in_channels[1], num_points)
    ]

    outputs = neck(inputs)

    assert len(outputs) == len(inputs)
    assert outputs[0].shape == (batch_size, out_channels[0], num_points)
    assert outputs[1].shape == (batch_size, out_channels[1], num_points)


if __name__ == '__main__':
    test_cgnl_block_forward()
    test_cgnl_neck_forward()
    print("✓ 所有 CGNL 测试通过")
