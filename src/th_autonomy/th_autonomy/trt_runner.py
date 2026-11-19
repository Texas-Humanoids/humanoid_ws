"""Runs an ONNX model on the GPU with TensorRT (FP16), via cuda-python for memory.

The first run builds an engine for this GPU and TensorRT version and caches it
next to the model (`<model>.trt<version>.fp16.engine`); that takes a few minutes
on a Jetson Orin Nano, then starts take a second. Outputs whose size depends on
the data (the model's own NMS) are allocated as TensorRT reports their shape.

Needs TensorRT's Python bindings and cuda-python, which are in the GPU Docker
image (Dockerfile.gpu); pose_detector falls back to the CPU without them.
"""

from pathlib import Path

import numpy as np
import tensorrt as trt

try:  # cuda-python 12.6+ / 13
    from cuda.bindings import runtime as cudart
except ImportError:  # older cuda-python
    from cuda import cudart


def _check(result):
    err, *rest = result if isinstance(result, tuple) else (result,)
    if int(err) != 0:
        raise RuntimeError(f'CUDA error {err}')
    return rest[0] if len(rest) == 1 else rest


class _OutputAllocator(trt.IOutputAllocator):
    """Device memory for an output whose shape is only known after running."""

    def __init__(self):
        super().__init__()
        self.ptr = 0
        self.size = 0
        self.shape = None

    def reallocate_output(self, tensor_name, memory, size, alignment):
        if size > self.size:
            if self.ptr:
                _check(cudart.cudaFree(self.ptr))
            self.ptr = int(_check(cudart.cudaMalloc(max(size, 1))))
            self.size = size
        return self.ptr

    def reallocate_output_async(self, tensor_name, memory, size, alignment, stream):
        return self.reallocate_output(tensor_name, memory, size, alignment)

    def notify_shape(self, tensor_name, shape):
        self.shape = tuple(shape)


class TensorRTRunner:

    backend = 'tensorrt'

    def __init__(self, model_path, fp16=True, log=print):
        self.logger = trt.Logger(trt.Logger.WARNING)
        model_path = Path(model_path)
        engine_path = model_path.with_name(
            f'{model_path.stem}.trt{trt.__version__}.{"fp16" if fp16 else "fp32"}.engine')
        if not engine_path.is_file():
            log(f'Building a TensorRT engine for {model_path.name} (first run only, a few minutes)')
            engine_path.write_bytes(self._build(model_path, fp16))
            log(f'Saved {engine_path}')
        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self.engine is None:
            raise RuntimeError(f'Could not load {engine_path}; delete it to rebuild')
        self.context = self.engine.create_execution_context()
        self.stream = _check(cudart.cudaStreamCreate())
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inputs = [n for n in names
                       if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        self.outputs = [n for n in names if n not in self.inputs]
        self.dtypes = {n: np.dtype(trt.nptype(self.engine.get_tensor_dtype(n))) for n in names}
        self.buffers = {}     # name -> (device pointer, bytes) for fixed-size tensors
        self.allocators = {}  # name -> _OutputAllocator for data-dependent outputs

    def _build(self, model_path, fp16):
        builder = trt.Builder(self.logger)
        network = builder.create_network(0)
        parser = trt.OnnxParser(network, self.logger)
        if not parser.parse(model_path.read_bytes()):
            errors = '; '.join(str(parser.get_error(i)) for i in range(parser.num_errors))
            raise RuntimeError(f'TensorRT could not parse {model_path}: {errors}')
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
        if fp16:
            config.set_flag(trt.BuilderFlag.FP16)
        profile = builder.create_optimization_profile()
        for i in range(network.num_inputs):
            tensor = network.get_input(i)
            shape = tuple(1 if d < 0 else d for d in tensor.shape)  # batch of one
            profile.set_shape(tensor.name, shape, shape, shape)
        config.add_optimization_profile(profile)
        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            raise RuntimeError(f'TensorRT could not build an engine for {model_path}')
        return bytes(serialized)

    def _device(self, name, nbytes):
        ptr, size = self.buffers.get(name, (0, 0))
        if nbytes > size:
            if ptr:
                _check(cudart.cudaFree(ptr))
            ptr = int(_check(cudart.cudaMalloc(nbytes)))
            self.buffers[name] = (ptr, nbytes)
        return ptr

    def __call__(self, tensor):
        """Outputs (in engine order) for one input array."""
        tensor = np.ascontiguousarray(tensor, self.dtypes[self.inputs[0]])
        name = self.inputs[0]
        self.context.set_input_shape(name, tensor.shape)
        ptr = self._device(name, tensor.nbytes)
        _check(cudart.cudaMemcpyAsync(ptr, tensor.ctypes.data, tensor.nbytes,
                                      cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
        self.context.set_tensor_address(name, ptr)
        shapes = {}
        for out in self.outputs:
            shape = tuple(self.context.get_tensor_shape(out))
            if any(d < 0 for d in shape):
                allocator = self.allocators.setdefault(out, _OutputAllocator())
                self.context.set_output_allocator(out, allocator)
            else:
                shapes[out] = shape
                nbytes = int(np.prod(shape)) * self.dtypes[out].itemsize
                self.context.set_tensor_address(out, self._device(out, max(nbytes, 1)))
        if not self.context.execute_async_v3(int(self.stream)):
            raise RuntimeError('TensorRT inference failed')
        results = []
        for out in self.outputs:
            if out in shapes:
                shape, ptr = shapes[out], self.buffers[out][0]
            else:
                allocator = self.allocators[out]
                shape, ptr = allocator.shape, allocator.ptr
            host = np.empty(shape, self.dtypes[out])
            if host.nbytes:
                _check(cudart.cudaMemcpyAsync(host.ctypes.data, ptr, host.nbytes,
                                              cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                                              self.stream))
            results.append(host)
        _check(cudart.cudaStreamSynchronize(self.stream))
        return results if len(results) != 2 else self._ordered(results)

    def _ordered(self, results):
        # The pose model's outputs by name: dets, then keypoints.
        by_name = dict(zip(self.outputs, results))
        if set(by_name) == {'dets', 'keypoints'}:
            return by_name['dets'], by_name['keypoints']
        return results
