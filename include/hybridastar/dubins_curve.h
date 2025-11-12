/*!
  \file dubins_curve.h
   \brief A dubins path class for finding analytical solutions to the problem of the shortest path.

  Converted from C implementation to modern C++ OOP design.
  \author Andrew Walker (original C implementation)
  \author Converted to modern C++ OOP
*/
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

#ifndef DUBINS_CURVE_H
#define DUBINS_CURVE_H

#include <functional>
#include <array>
#include <cstdint>

class DubinsCurve
{
public:
    // The three segment types a path can be made up of
    enum SegmentType
    {
        L_SEG = 0,
        S_SEG = 1,
        R_SEG = 2
    };

    // Public path type enum
    enum DubinsPathType
    {
        LSL = 0,
        LSR = 1,
        RSL = 2,
        RSR = 3,
        RLR = 4,
        LRL = 5
    };

    enum ErrorCode
    {
        DUBINS_ERR_OK = 0,    // No error
        DUBINS_ERR_COCONFIGS, // Colocated configurations
        DUBINS_ERR_PARAM,     // Path parameterisitation error
        DUBINS_ERR_BADRHO,    // the rho value is invalid
        DUBINS_ERR_NOPATH     // no connection between configurations with this word
    };

    // Constructor
    DubinsCurve();

    // Destructor
    ~DubinsCurve();

    /**
     * Generate a path from an initial configuration to
     * a target configuration, with a specified maximum turning
     * radii
     *
     * A configuration is (x, y, theta), where theta is in radians, with zero
     * along the line x = 0, and counter-clockwise is positive
     *
     * @param initial_config    - Initial configuration as [x, y, theta]
     * @param target_config     - Target configuration as [x, y, theta]
     * @param turning_radius    - Turning radius of the vehicle (forward velocity divided by maximum angular velocity)
     * @return                  - non-zero on error
     */
    ErrorCode init(const std::array<double, 3> &initial_config, const std::array<double, 3> &target_config, double turning_radius);

    /**
     * Calculate the length of an initialised path
     *
     * @return - the length of the path
     */
    double getPathLength() const;

    /**
     * Extract an integer that represents which path type was used
     *
     * @return - one of LSL, LSR, RSL, RSR, RLR or LRL (ie/ 0-5 inclusive)
     */
    DubinsPathType getPathType() const;

    /**
     * Calculate the configuration along the path, using the parameter t
     *
     * @param distance_param    - a length measure, where 0 <= t < getPathLength()
     * @param output_config     - the configuration result [x, y, theta]
     * @returns                 - non-zero if 't' is not in the correct range
     */
    ErrorCode pathSample(double distance_param, std::array<double, 3> &output_config) const;

    /**
     * Walk along the path at a fixed sampling interval, calling the
     * callback function at each interval
     *
     * @param callback_func     - the callback function to call for each sample
     * @param step_size         - the distance along the path for subsequent samples
     * @param user_data         - optional information to pass on to the callback
     */
    ErrorCode pathSampleMany(std::function<ErrorCode(std::array<double, 3> &, double, void *)> callback_func, double step_size, void *user_data) const;

    /**
     * Convenience function to identify the endpoint of a path
     *
     * @param endpoint_config   - the configuration result [x, y, theta]
     */
    ErrorCode pathEndpoint(std::array<double, 3> &endpoint_config) const;

    /**
     * Convenience function to extract a subset of a path
     *
     * @param distance_param    - a length measure, where 0 < t < getPathLength()
     * @param output_path       - the resultant path (output)
     */
    ErrorCode extractSubpath(double distance_param, DubinsCurve &output_path) const;

    // Public accessors
    const std::array<double, 3> &getInitialConfig() const { return initial_config_; }
    double getRho() const { return turning_radius_; }
    const std::array<double, 3> &getParams() const { return path_params_; }

private:
    // Private member variables (with underscore suffix)
    std::array<double, 3> initial_config_; // the initial configuration [x, y, theta]
    std::array<double, 3> path_params_;    // the lengths of the three segments [p1, p2, p3]
    double turning_radius_;                // model forward velocity / model angular velocity
    DubinsPathType path_type_;                       // path type. one of LSL, LSR, ...
    static const std::array<std::array<uint8_t, 3>, 6> dirdata_;

    // Private helper functions
    ErrorCode initNormalised(double alpha, double beta, double d);
    void segment(double distance, const std::array<double, 3> &input_config, std::array<double, 3> &output_config, int segment_type) const;

    // Private solver functions for each path type
    ErrorCode solveLSL(double alpha, double beta, double d, std::array<double, 3> &outputs);
    ErrorCode solveLSR(double alpha, double beta, double d, std::array<double, 3> &outputs);
    ErrorCode solveRSL(double alpha, double beta, double d, std::array<double, 3> &outputs);
    ErrorCode solveRSR(double alpha, double beta, double d, std::array<double, 3> &outputs);
    ErrorCode solveRLR(double alpha, double beta, double d, std::array<double, 3> &outputs);
    ErrorCode solveLRL(double alpha, double beta, double d, std::array<double, 3> &outputs);

    // Private utility functions
    double mod2pi(double theta) const;
    double fmodr(double x, double y) const;
};

#endif // DUBINS_CURVE_H