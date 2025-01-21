import pytest
import jax
from jax import numpy as jp
import numpy as np
from pupperv3_mjx.utils import (
    activation_fn_map,
    circular_buffer_push_back,
    circular_buffer_push_front,
    sample_lagged_value,
)


def test_relu():
    fn = activation_fn_map("relu")
    input_val = jp.array([-1.0, 0.0, 1.0])
    expected_output = jp.array([0.0, 0.0, 1.0])
    np.testing.assert_array_equal(fn(input_val), expected_output)


def test_sigmoid():
    fn = activation_fn_map("sigmoid")
    input_val = jp.array([-1.0, 0.0, 1.0])
    expected_output = 1 / (1 + jp.exp(-input_val))
    np.testing.assert_array_almost_equal(fn(input_val), expected_output)


def test_elu():
    fn = activation_fn_map("elu")
    input_val = jp.array([-1.0, 0.0, 1.0])
    expected_output = jax.nn.elu(input_val)
    np.testing.assert_array_almost_equal(fn(input_val), expected_output)


def test_tanh():
    fn = activation_fn_map("tanh")
    input_val = jp.array([-1.0, 0.0, 1.0])
    expected_output = jp.tanh(input_val)
    np.testing.assert_array_almost_equal(fn(input_val), expected_output)


def test_softmax():
    fn = activation_fn_map("softmax")
    input_val = jp.array([1.0, 2.0, 3.0])
    expected_output = jax.nn.softmax(input_val)
    np.testing.assert_array_almost_equal(fn(input_val), expected_output)


def test_invalid_activation():
    with pytest.raises(KeyError):
        activation_fn_map("invalid")


def test_circular_buffer_push_back():
    buffer = jp.array([[1, 2, 3], [4, 5, 6]])
    new_value = jp.array([7, 8])
    expected_output = jp.array([[2, 3, 7], [5, 6, 8]])
    output = circular_buffer_push_back(buffer, new_value)
    np.testing.assert_array_equal(output, expected_output)


def test_circular_buffer_push_front():
    buffer = jp.array([[1, 2, 3], [4, 5, 6]])
    new_value = jp.array([7, 8])
    expected_output = jp.array([[7, 1, 2], [8, 4, 5]])
    output = circular_buffer_push_front(buffer, new_value)
    np.testing.assert_array_equal(output, expected_output)


def test_sample_lagged_value():
    # The action sample from 3 time steps ago will always be sampled
    latency_distribution = jp.array([0, 0, 0, 1])
    buffer = jp.zeros((12, 4), dtype=float)

    expected_value = jp.arange(12)
    buffer = buffer.at[:, -2].set(expected_value)
    new_value = jp.zeros(12)

    # Sample action buffer
    latency_key = jax.random.PRNGKey(1)
    sampled_value, buffer = sample_lagged_value(
        latency_key, buffer, new_value, latency_distribution
    )

    # Check that the sampled action is within the expected range
    assert jp.allclose(sampled_value, expected_value, atol=1e-5)

    # Check buffer is updated
    expected_buffer = jp.zeros((12, 4), dtype=float)
    expected_buffer = expected_buffer.at[:, 0].set(new_value)
    expected_buffer = expected_buffer.at[:, -1].set(expected_value)
    assert jp.allclose(buffer, expected_buffer, atol=1e-5)


def test_sample_lagged_value_buffer_size_one():
    # Buffer size of 1 means no lag
    latency_distribution = jp.array([0])
    buffer = jp.zeros((12, 1), dtype=float)
    new_value = jp.ones(12)

    # Sample action buffer
    latency_key = jax.random.PRNGKey(1)
    sampled_value, buffer = sample_lagged_value(
        latency_key, buffer, new_value, latency_distribution
    )
    expected_value = jp.ones(12)

    # Check that the sampled action is within the expected range
    assert jp.allclose(sampled_value, expected_value, atol=1e-5)
