#pragma once
#include "common.h"

enum class NEURAL_NET_LAYER_TYPE
{
	LINEAR, // passthrough
	SIGMOID, // activated by the logistic sigmoid function
	SELU // activated by a scaled exponential linear unit function
};

class InputNormalizer
{
	int num_inputs = 0;
	double* mean = nullptr;
	double* standard_deviation = nullptr;
public:
	InputNormalizer(FILE* file, const int inputs)
	{
		num_inputs = inputs;
		mean = static_cast<double*>(malloc(inputs * sizeof(double)));
		standard_deviation = static_cast<double*>(malloc(inputs * sizeof(double)));
		for (auto i = 0; i < inputs; i++)
		{
			fscanf_s(file, " %le %le", mean + i, standard_deviation + i);
		}
	}
	~InputNormalizer()
	{
		free(mean);
		free(standard_deviation);
	}

	void Normalize(double* input) const
	{
		for (auto i = 0; i < num_inputs; i++)
		{
			input[i] = (input[i] - mean[i]) / standard_deviation[i];
		}
	}
};

class OutputNormalizer
{
	int num_outputs = 0;
	double* mean = nullptr;
	double* standard_deviation = nullptr;
public:
	OutputNormalizer(FILE* file, const int outputs)
	{
		num_outputs = outputs;
		mean = static_cast<double*>(malloc(outputs * sizeof(double)));
		standard_deviation = static_cast<double*>(malloc(outputs * sizeof(double)));
		for (auto i = 0; i < outputs; i++)
		{
			fscanf_s(file, " %le %le", mean + i, standard_deviation + i);
		}
	}

	~OutputNormalizer()
	{
		free(mean);
		free(standard_deviation);
	}

	void Normalize(double* output) const
	{
		for (auto i = 0; i < num_outputs; i++)
		{
			output[i] = (output[i] * standard_deviation[i]) + mean[i];
		}
	}
};

class NeuralNetworkLayer
{
	NEURAL_NET_LAYER_TYPE type = NEURAL_NET_LAYER_TYPE::LINEAR; // The type of layer this will be
	int inputs = 0; // The number of inputs this layer will receive
	int size = 0; // The number of nodes in this layer
	double** weights = nullptr; // The weights for each node (cardinality is size * inputs)
	double* biases = nullptr; // The bias weights for each node (cardinality is size)
	double* values = nullptr; // The current values for each node -- should be provided as empty

	double activation(const double value) const
	{
		switch (type)
		{
		case NEURAL_NET_LAYER_TYPE::LINEAR:
			return value;
		case NEURAL_NET_LAYER_TYPE::SIGMOID:
			return 1 / (1 + exp(-value));
		case NEURAL_NET_LAYER_TYPE::SELU:
			const auto scale = 1.05070098;
			const auto alpha = 1.67326324;
			return (value >= 0) ? scale * value : scale * alpha * (exp(value) - 1);
		}
		return 0; // Unreachable
	}

public:
	explicit NeuralNetworkLayer(FILE* file)
	{
		fscanf_s(file, " %d %d %d", &type, &inputs, &size);
		weights = static_cast<double**>(malloc(size * sizeof(double*)));
		for (auto i = 0; i < size; i++)
		{
			weights[i] = static_cast<double*>(malloc(inputs * sizeof(double)));
			for (auto j = 0; j < inputs; j++)
			{
				fscanf_s(file, " %le", &weights[i][j]);
			}
		}
		biases = static_cast<double*>(malloc(size * sizeof(double)));
		for (auto i = 0; i < size; i++)
		{
			fscanf_s(file, " %le", &biases[i]);
		}
		values = static_cast<double*>(malloc(size * sizeof(double)));
	}

	~NeuralNetworkLayer()
	{
		for (auto i = 0; i < size; i++)
		{
			free(weights[i]);
		}
		free(weights);
		free(biases);
		free(values);
	}

	NEURAL_NET_LAYER_TYPE Type() const { return type; }
	int Size() const { return size; }
	int Inputs() const { return inputs; }
	double Weight(const int node, const int input) const { return weights[node][input]; }
	double Bias(const int node) const { return biases[node]; }
	double* Values() const { return values; }

	void Propagate(const double* input) const
	{
		for (auto i = 0; i < size; i++)
		{
			auto sum = biases[i];
			for (auto j = 0; j < inputs; j++)
			{
				sum += weights[i][j] * input[j];
			}
			values[i] = activation(sum);
		}
	}
};

class NeuralNetwork
{
	int num_layers = 0;
	NeuralNetworkLayer** layers = nullptr;
	InputNormalizer* input_normalizer = nullptr;
	OutputNormalizer* output_normalizer = nullptr;
public:
	explicit NeuralNetwork(FILE* file)
	{
		fscanf_s(file, "%d", &num_layers);
		layers = new NeuralNetworkLayer * [num_layers];
		for (auto i = 0; i < num_layers; i++)
		{
			layers[i] = new NeuralNetworkLayer(file);
		}
		input_normalizer = new InputNormalizer(file, (*layers[0]).Inputs());
		output_normalizer = new OutputNormalizer(file, (*layers[num_layers - 1]).Size());
	}
	~NeuralNetwork()
	{
		for (auto i = 0; i < num_layers; i++)
		{
			delete layers[i];
		}
		delete layers;
		delete input_normalizer;
		delete output_normalizer;
	}

	// Predicts output given inputs
	double* Predict(double* inputs) const
	{
		input_normalizer->Normalize(inputs);
		for (auto i = 0; i < num_layers; i++)
		{
			auto* input = (i == 0) ? inputs : layers[i - 1]->Values();
			layers[i]->Propagate(input);
		}
		auto* output = layers[num_layers - 1]->Values();
		output_normalizer->Normalize(output);
		return output;
	}
};