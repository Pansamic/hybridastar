// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "dubins_curve.h"
#define _USE_MATH_DEFINES
#include <limits>
#include <cmath>
#include <cassert>

const std::array<std::array<uint8_t, 3>, 6> DubinsCurve::dirdata_ =
{{
    {L_SEG, S_SEG, L_SEG},
    {L_SEG, S_SEG, R_SEG},
    {R_SEG, S_SEG, L_SEG},
    {R_SEG, S_SEG, R_SEG},
    {R_SEG, L_SEG, R_SEG},
    {L_SEG, R_SEG, L_SEG}
}};

DubinsCurve::DubinsCurve()
{
    initial_config_[0] = 0.0f;
    initial_config_[1] = 0.0f;
    initial_config_[2] = 0.0f;
    path_params_[0] = 0.0f;
    path_params_[1] = 0.0f;
    path_params_[2] = 0.0f;
    turning_radius_ = 1.0f;
    path_type_ = LSL; // default to LSL
}

DubinsCurve::~DubinsCurve()
{
    // Destructor implementation
}

DubinsCurve::ErrorCode DubinsCurve::init(const std::array<double, 3> &initial_config, const std::array<double, 3> &target_config, double turning_radius)
{
    if (turning_radius <= 0.0f)
    {
        return DUBINS_ERR_BADRHO;
    }

    double dx = target_config[0] - initial_config[0];
    double dy = target_config[1] - initial_config[1];
    double distance = sqrtf(dx * dx + dy * dy);
    double normalized_distance = distance / turning_radius;

    double theta = mod2pi(atan2f(dy, dx));
    double alpha = mod2pi(initial_config[2] - theta);
    double beta = mod2pi(target_config[2] - theta);

    for (int i = 0; i < 3; i++)
    {
        initial_config_[i] = initial_config[i];
    }
    turning_radius_ = turning_radius;

    return initNormalised(alpha, beta, normalized_distance);
}

DubinsCurve::ErrorCode DubinsCurve::initNormalised(double alpha, double beta, double d)
{
    double best_cost = std::numeric_limits<double>::infinity();
    int best_word = -1;

    for (int i = 0; i < 6; i++)
    {
        std::array<double, 3> params;
        int err;

        switch (i)
        {
        case LSL:
            err = solveLSL(alpha, beta, d, params);
            break;
        case LSR:
            err = solveLSR(alpha, beta, d, params);
            break;
        case RSL:
            err = solveRSL(alpha, beta, d, params);
            break;
        case RSR:
            err = solveRSR(alpha, beta, d, params);
            break;
        case RLR:
            err = solveRLR(alpha, beta, d, params);
            break;
        case LRL:
            err = solveLRL(alpha, beta, d, params);
            break;
        default:
            err = ErrorCode::DUBINS_ERR_NOPATH;
            break;
        }

        if (err == ErrorCode::DUBINS_ERR_OK)
        {
            double cost = params[0] + params[1] + params[2];
            if (cost < best_cost)
            {
                best_word = i;
                best_cost = cost;
                path_params_[0] = params[0];
                path_params_[1] = params[1];
                path_params_[2] = params[2];
                path_type_ = static_cast<DubinsPathType>(i);
            }
        }
    }

    if (best_word == -1)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    path_type_ = static_cast<DubinsPathType>(best_word);
    return ErrorCode::DUBINS_ERR_OK;
}

double DubinsCurve::getPathLength() const
{
    double length = 0.0f;
    length += path_params_[0];
    length += path_params_[1];
    length += path_params_[2];
    length = length * turning_radius_;
    return length;
}

DubinsCurve::DubinsPathType DubinsCurve::getPathType() const
{
    return path_type_;
}

DubinsCurve::ErrorCode DubinsCurve::pathSample(double distance_param, std::array<double, 3> &output_config) const
{
    if (distance_param < 0 || distance_param >= getPathLength())
    {
        // error, parameter out of bounds
        return ErrorCode::DUBINS_ERR_PARAM;
    }

    // tprime is the normalised variant of the parameter t
    double normalized_distance = distance_param / turning_radius_;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    std::array<double, 3> translated_initial_config = {0.0f, 0.0f, initial_config_[2]};

    // Generate the target configuration
    const std::array<uint8_t, 3>& segment_types = dirdata_[path_type_];
    double segment_1_length = path_params_[0];
    double segment_2_length = path_params_[1];
    std::array<double, 3> end_segment_1; // end-of segment 1
    std::array<double, 3> end_segment_2; // end-of segment 2
    segment(segment_1_length, translated_initial_config, end_segment_1, segment_types[0]);
    segment(segment_2_length, end_segment_1, end_segment_2, segment_types[1]);
    if (normalized_distance < segment_1_length)
    {
        segment(normalized_distance, translated_initial_config, output_config, segment_types[0]);
    }
    else if (normalized_distance < (segment_1_length + segment_2_length))
    {
        segment(normalized_distance - segment_1_length, end_segment_1, output_config, segment_types[1]);
    }
    else
    {
        segment(normalized_distance - segment_1_length - segment_2_length, end_segment_2, output_config, segment_types[2]);
    }

    // scale the target configuration, translate back to the original starting point
    output_config[0] = output_config[0] * turning_radius_ + initial_config_[0];
    output_config[1] = output_config[1] * turning_radius_ + initial_config_[1];
    output_config[2] = mod2pi(output_config[2]);

    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::pathSampleMany(std::function<DubinsCurve::ErrorCode(std::array<double, 3> &, double, void *)> callback_func, double step_size, void *user_data) const
{
    double x = 0.0f;
    double length = getPathLength();
    while (x < length)
    {
        std::array<double, 3> sample;
        pathSample(x, sample);
        ErrorCode retcode = callback_func(sample, x, user_data);
        if (retcode != 0)
        {
            return retcode;
        }
        x += step_size;
    }
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::pathEndpoint(std::array<double, 3> &endpoint_config) const
{
    // Use a small epsilon to avoid numerical issues at the endpoint
    const double epsilon = 1e-6f;
    return pathSample(getPathLength() - epsilon, endpoint_config);
}

DubinsCurve::ErrorCode DubinsCurve::extractSubpath(double distance_param, DubinsCurve &output_path) const
{
    // calculate the true parameter
    double normalized_distance = distance_param / turning_radius_;

    // copy most of the data
    output_path.initial_config_[0] = initial_config_[0];
    output_path.initial_config_[1] = initial_config_[1];
    output_path.initial_config_[2] = initial_config_[2];
    output_path.turning_radius_ = turning_radius_;
    output_path.path_type_ = path_type_;

    // fix the parameters
    output_path.path_params_[0] = std::fmin(path_params_[0], normalized_distance);
    output_path.path_params_[1] = std::fmin(path_params_[1], normalized_distance - output_path.path_params_[0]);
    output_path.path_params_[2] = std::fmin(path_params_[2], normalized_distance - output_path.path_params_[0] - output_path.path_params_[1]);
    return ErrorCode::DUBINS_ERR_OK;
}

// Private solver functions
DubinsCurve::ErrorCode DubinsCurve::solveLSL(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double tmp0 = d + sa - sb;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if (p_squared < 0)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double tmp1 = atan2f((cb - ca), tmp0);
    double t = mod2pi(-alpha + tmp1);
    double p = sqrtf(p_squared);
    double q = mod2pi(beta - tmp1);

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::solveRSR(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double tmp0 = d - sa + sb;
    double p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
    if (p_squared < 0)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double tmp1 = atan2f((ca - cb), tmp0);
    double t = mod2pi(alpha - tmp1);
    double p = sqrtf(p_squared);
    double q = mod2pi(-beta + tmp1);

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::solveLSR(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
    if (p_squared < 0)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double p = sqrtf(p_squared);
    double tmp2 = atan2f((-ca - cb), (d + sa + sb)) - atan2f(-2.0f, p);
    double t = mod2pi(-alpha + tmp2);
    double q = mod2pi(-mod2pi(beta) + tmp2);

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::solveRSL(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
    if (p_squared < 0)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double p = sqrtf(p_squared);
    double tmp2 = atan2f((ca + cb), (d - sa - sb)) - atan2f(2.0f, p);
    double t = mod2pi(alpha - tmp2);
    double q = mod2pi(beta - tmp2);

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::solveRLR(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double tmp_rlr = (6.0f - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0f;
    if (fabsf(tmp_rlr) > 1.0f)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double p = mod2pi(2 * M_PI - acosf(tmp_rlr));
    double t = mod2pi(alpha - atan2f(ca - cb, d - sa + sb) + mod2pi(p / 2.0f));
    double q = mod2pi(alpha - beta - t + mod2pi(p));

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

DubinsCurve::ErrorCode DubinsCurve::solveLRL(double alpha, double beta, double d, std::array<double, 3> &outputs)
{
    double sa = sinf(alpha);
    double sb = sinf(beta);
    double ca = cosf(alpha);
    double cb = cosf(beta);
    double c_ab = cosf(alpha - beta);

    double tmp_lrl = (6.0f - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.0f;
    if (fabsf(tmp_lrl) > 1.0f)
    {
        return ErrorCode::DUBINS_ERR_NOPATH;
    }
    double p = mod2pi(2 * M_PI - acosf(tmp_lrl));
    double t = mod2pi(-alpha - atan2f(ca - cb, d + sa - sb) + p / 2.0f);
    double q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));

    outputs[0] = t;
    outputs[1] = p;
    outputs[2] = q;
    return ErrorCode::DUBINS_ERR_OK;
}

// Private utility functions
double DubinsCurve::mod2pi(double theta) const
{
    return fmodr(theta, 2 * M_PI);
}

double DubinsCurve::fmodr(double x, double y) const
{
    return x - y * floorf(x / y);
}

void DubinsCurve::segment(double distance, const std::array<double, 3> &input_config, std::array<double, 3> &output_config, int segment_type) const
{
    assert(segment_type == L_SEG || segment_type == S_SEG || segment_type == R_SEG);

    if (segment_type == L_SEG)
    {
        output_config[0] = input_config[0] + sinf(input_config[2] + distance) - sinf(input_config[2]);
        output_config[1] = input_config[1] - cosf(input_config[2] + distance) + cosf(input_config[2]);
        output_config[2] = input_config[2] + distance;
    }
    else if (segment_type == R_SEG)
    {
        output_config[0] = input_config[0] - sinf(input_config[2] - distance) + sinf(input_config[2]);
        output_config[1] = input_config[1] + cosf(input_config[2] - distance) - cosf(input_config[2]);
        output_config[2] = input_config[2] - distance;
    }
    else if (segment_type == S_SEG)
    {
        output_config[0] = input_config[0] + cosf(input_config[2]) * distance;
        output_config[1] = input_config[1] + sinf(input_config[2]) * distance;
        output_config[2] = input_config[2];
    }
}