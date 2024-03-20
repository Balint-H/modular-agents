from brax.io import model
import onnx
import numpy as np
from onnx import helper, numpy_helper

from absl import app, flags



FLAGS = flags.FLAGS
flags.DEFINE_string(
  'obs_name', 'obs_0', 'typically obs_0, but it can be different.')
flags.DEFINE_integer('obs_size', 336, 'the size of the observation array.')
#flags.register_validator(
#  'env', lambda env: env in ['ShadowEnv', 'Humanoid', 'HumanoidDeepMimic'],
#  message='--env must be Humanoid or ShadowEnv'
#)
flags.DEFINE_integer('actuator_size', 21, 'the size of the actuation outputs.')
flags.DEFINE_string('input', 'mjx_brax_policy.pickle', 'The weights of the neural net to convert.')
flags.DEFINE_string('output', 'feedforward_model.onnx', 'The name of the .onnx file you get as output, with the extension. By default it is feedforward_model.onnx')
flags.DEFINE_bool('normalize', True, 'Do you want normalisation?')
flags.DEFINE_bool('convert4unity', False, 'the resulting .onnx will be used in unity?')

#  convert_to_feedforward_onnx_model([("obs_0", 336)], 21, "./mjx_brax_policy.pickle", save_path="feedforward_model.onnx", normalize=True)


def convert_to_feedforward_onnx_model(named_input_size_tuples, output_size, brax_path,
                                      save_path="feedforward_model.onnx",
                                      normalize=True, opset=19,
                                      output_random=True, output_deterministic=True,
                                      add_mlagents_outputs=True,
                                      output_hidden=False,
                                      activation="swish"):
    normalize_params, layer_params = get_brax_params(brax_path)

    print("creating onnx layers...")
    # Create an ONNX graph
    graph = helper.make_graph(nodes=[], name='feedforward_model', inputs=[], outputs=[], initializer=[])

    # Add input tensor to the graph
    input_tensor = [helper.make_tensor_value_info(name, onnx.TensorProto.FLOAT, [1, input_size])
                    for name, input_size in named_input_size_tuples]
    graph.input.extend(input_tensor)

    concat_node = helper.make_node(
        'Concat',
        [name for name, _ in named_input_size_tuples],
        [f'input'],
        name=f'Concatenated Inputs',
        axis=-1
    )
    graph.node.append(concat_node)
    input_size = sum([size for _, size in named_input_size_tuples])

    processed_start = "input"
    if normalize:
        processed_start = "normalized"
        means = onnx.helper.make_tensor(name="norm_mean_vals", data_type=onnx.TensorProto.FLOAT, dims=[1, input_size],
                                        vals=normalize_params.mean)
        stds = onnx.helper.make_tensor(name="norm_std_vals", data_type=onnx.TensorProto.FLOAT, dims=[1, input_size],
                                        vals=normalize_params.std)
        mean_node = onnx.helper.make_node("Constant", inputs=[], outputs=["norm_mean"], name="norm_mean", value=means)
        std_node = onnx.helper.make_node("Constant", inputs=[], outputs=["norm_std"], name="norm_std", value=stds)
        sub_node = helper.make_node(
            'Sub',
            ['input', f'norm_mean'],
            [f'subbed'],
            name=f'Sub'
        )
        div_node = helper.make_node(
            'Div',
            ['subbed', f'norm_std'],
            [f'normalized'],
            name=f'Div'
        )
        graph.node.extend([mean_node, std_node, sub_node, div_node])
    if output_hidden:
        get_output(graph, "normalized", [1, 336])
    # Iterate through each hidden layer
    for i, layer_key in enumerate(layer_params.keys()):
        layer_info = layer_params[layer_key]

        # Create ONNX tensor for the kernel
        weight_tensor = numpy_helper.from_array(layer_info['kernel'], name=f'{layer_key}_weight')
        graph.initializer.extend([weight_tensor])

        # Create ONNX tensor for the bias
        bias_tensor = numpy_helper.from_array(layer_info['bias'], name=f'{layer_key}_bias')
        graph.initializer.extend([bias_tensor])

        # MatMul for the weights of the linear layer
        matmul_node = helper.make_node(
            'MatMul',
            [f'hidden_output_{i}_activated' if i > 0 else processed_start, f'{layer_key}_weight'],
            [f'hidden_output_{i + 1}'],
            name=f'MatMul_{i}'
        )
        # Add operation for bias
        add_node = helper.make_node(
            'Add',
            [f'hidden_output_{i + 1}', f'{layer_key}_bias'],
            [f'hidden_output_{i + 1}_biased'],
            name=f'Add_{i}'
        )
        graph.node.extend([matmul_node, add_node])

        # Don't apply activation fn for last layer
        if i >= len(layer_params)-1:
            break

        if activation.upper() == "RELU":
            act_node = helper.make_node(
                'Relu',
                [f'hidden_output_{i + 1}_biased'],
                [f'hidden_output_{i + 1}_activated'],
                name=f'ReLU_{i}'
            )
        elif activation.upper() == "SWISH" or activation.upper() == "SILU":
            sigm_node = helper.make_node(
                'Sigmoid',
                [f'hidden_output_{i + 1}_biased'],
                [f'hidden_output_{i + 1}_sigmoid'],
                name=f'Sigm_{i}'
            )
            act_node = helper.make_node(
                'Mul',
                [f'hidden_output_{i + 1}_sigmoid', f'hidden_output_{i + 1}_biased'],
                [f'hidden_output_{i + 1}_activated'],
                name=f'Swish_{i}'
            )
            graph.node.append(sigm_node)
        else:
            act_node = helper.make_node(
                'Identity',
                [f'hidden_output_{i + 1}_biased'],
                [f'hidden_output_{i + 1}_activated'],
                name=f'Identity_{i}'
            )
        graph.node.append(act_node)
        if output_hidden:
        
            get_output(graph, f'hidden_output_{i + 1}_activated', [1, output_size])

    if output_hidden:

        
        get_output(graph, f'hidden_output_{len(layer_params)}_biased', [1, output_size*2])
        
        
        # We output unprocessed action distribution means and stds, need to split it
        # For some reason, Unity expects you to use the `split` argument with value `[output_size, output_size]`
        # but `onnxruntime` complains if you use the `split`.
        # To have the model load in unity num_outputs= 2 should be replaced by split =[output_size, output_size]
        # To have the model load in onnxruntime use num_outputs= 2 instead of split =[output_size, output_size]
        
        
    if(FLAGS.convert4unity):
        print("onnx formatted for import in unity.......")
        split_node = helper.make_node(
        'Split',
        [f'hidden_output_{len(layer_params)}_biased'],
        ['loc', 'scale'],
        axis=-1,
        split =[output_size, output_size],
        name='Split Action'
        )

    else:
        print("onnx formatted for import using onnxruntime.......")
    
        split_node = helper.make_node(
        'Split',
        [f'hidden_output_{len(layer_params)}_biased'],
        ['loc', 'scale'],
        axis=-1,
        num_outputs=2,
        name='Split Action'
        )
    graph.node.extend([split_node])

    if output_hidden:
        get_output(graph, f'loc', [1, output_size])

    if output_random:
        # We apply softplus and add a small value to the std so its guaranteed positive
        min_std = onnx.helper.make_tensor(name="min_std_val", data_type=onnx.TensorProto.FLOAT, dims=[1, 1],
                                          vals=[0.001])
        min_std_node = onnx.helper.make_node("Constant", inputs=[], outputs=["min_std"], name="min_std",
                                             value=min_std)

        softplus_node = helper.make_node(
            'Softplus',
            [f'scale'],
            ['softplus_scale'],
            name='Softplus Scale'
        )

        softplus_epsilon_node = helper.make_node(
            'Add',
            [f'softplus_scale', 'min_std'],
            ['softplus_scale_epsilon'],
            name='Softplus Scale Epsilon'
        )
        # Sample for the action, then scale with std and offset with mean
        normal_node = helper.make_node(
            'RandomNormalLike',
            [f'loc'],
            ['normal_noise'],
            name='Normal Distribution Samples'
        )

        scaled_noise_node = helper.make_node(
            'Mul',
            [f'softplus_scale_epsilon', 'normal_noise'],
            ['scaled_noise'],
            name='Scaled Noise'
        )

        scaled_offset_noise_node = helper.make_node(
            'Add',
            [f'scaled_noise', 'loc'],
            ['scaled_offset_noise'],
            name='Scaled Offset Noise'
        )

        # Bound between -1 and 1
        tanh_noise_node = helper.make_node(
            'Tanh',
            [f'scaled_offset_noise'],
            ['continuous_actions'],
            name='Actions'
        )
        graph.node.extend(
            [softplus_node, min_std_node, softplus_epsilon_node, normal_node, scaled_noise_node,
             scaled_offset_noise_node, tanh_noise_node])
        output_tensor = helper.make_tensor_value_info('continuous_actions', onnx.TensorProto.FLOAT, [1, output_size])
        graph.output.append(output_tensor)

    if output_deterministic:
        tanh_deterministic_node = helper.make_node(
            'Tanh',
            [f'loc'],
            ['deterministic_continuous_actions'],
            name='Deterministic Actions'
        )
        graph.node.extend([tanh_deterministic_node])
        deterministic_output_tensor = helper.make_tensor_value_info('deterministic_continuous_actions', onnx.TensorProto.FLOAT, [1, output_size])
        graph.output.append(deterministic_output_tensor)

    if add_mlagents_outputs:
        add_constant_output(graph, "version_number", [3, 0, 0])
        add_constant_output(graph, "memory_size", [0])
        add_constant_output(graph, "continuous_action_output_shape", [output_size])

    # Create the ONNX model
    compiled_model = helper.make_model(graph, producer_name='feedforward_model')
    if save_path is not None and save_path != "":
        onnx.save_model(set_onnx_opset(compiled_model, target_opset=opset), save_path)
    return compiled_model


def add_constant_output(graph, name, value, dtype=onnx.TensorProto.INT32):
    val = onnx.helper.make_tensor(name=name+"_val", data_type=dtype, dims=np.array(value).shape,
                                  vals=value)
    const = onnx.helper.make_node("Constant",inputs=[], outputs=[name], name=name, value=val)
    info = helper.make_tensor_value_info(name, dtype, np.array(value).shape)
    graph.node.append(const)
    graph.output.append(info)



def get_brax_params(param_path):
    model_path = param_path
    params = model.load_params(model_path)
    normalize_params = params[0]
    params = params[1]["params"]
    for key in params.keys():
        params[key] = {param_key: np.array(val) for param_key, val in params[key].items()}
    return normalize_params, params


def get_output(graph, name, shape, dtype=onnx.TensorProto.FLOAT):
    output_tensor = helper.make_tensor_value_info(name, dtype, shape)
    graph.output.append(output_tensor)

def set_onnx_opset(onnx_model, target_opset=19):
    del onnx_model.opset_import[:]
    opset = onnx_model.opset_import.add()
    opset.domain = ''
    opset.version = target_opset
    return onnx_model

def main(unused_argv):
    if FLAGS.normalize:
        print("normalisation is true")
    else:
        print("normalisation is false")

    print("Launching conversion....")
    #convert_to_feedforward_onnx_model([("obs_0", 336)], 21, "./mjx_brax_policy.pickle", save_path="feedforward_model.onnx", normalize=True)
    convert_to_feedforward_onnx_model([(FLAGS.obs_name, FLAGS.obs_size)], FLAGS.actuator_size,FLAGS.input, save_path=FLAGS.output, normalize=FLAGS.normalize)

    print("saved output in: " + FLAGS.output)

if __name__ == "__main__":
 
    app.run(main)
