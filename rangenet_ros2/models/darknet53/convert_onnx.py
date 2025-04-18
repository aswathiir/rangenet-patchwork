#!/usr/bin/env python3
import tensorrt as trt

def convert_onnx_to_trt(onnx_path, trt_path):
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    
    # Load ONNX model
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            print("ERROR: Failed to parse ONNX file")
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return False
    
    # Build TensorRT engine
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB
    serialized_engine = builder.build_serialized_network(network, config)
    
    # Save engine
    with open(trt_path, "wb") as f:
        f.write(serialized_engine)
    return True

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Input ONNX model path")
    parser.add_argument("--output", required=True, help="Output TensorRT engine path")
    args = parser.parse_args()
    
    if convert_onnx_to_trt(args.model, args.output):
        print(f"Successfully converted to TensorRT engine: {args.output}")
    else:
        print("Conversion failed")
