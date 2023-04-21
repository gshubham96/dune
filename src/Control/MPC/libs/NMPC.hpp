// NMPC.h
#pragma once

#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"


namespace NMPC
{
    class CourseController
    {
    private:
        bool Initialized;
        // in [rad]
        double desired_course;
        // state is [psi, u, v, r]
        std::vector<double> mpc_state;
        // params are [Vc, beta_c, Vw, beta_w, k1, k2]
        std::vector<double> mpc_params;
        // config vector is <Ts, Tp, solver, >
        std::vector<double> config;
        // convert state, config and param identifiers to indices
        std::map<std::string, int> state_idx;
        std::map<std::string, int> config_idx;
        std::map<std::string, int> params_idx;
        std::map<std::string, int> solver_idx;

    public:



        // input: contains the key-value pair information on the parameter to set. if a key is missing, use default parameters
        // output: feedback on whether the problem configuration was a success or not
        // purpose: interacts with the acados compiled library and returns the MPC object
        bool defineMpcProblem(std::map<std::string, double> &config);
            // config contains: model_type, cost_type, Tp, Ts, 

        // input: contains the key-value pair information on the parameter to update (during runtime).
        // output: feedback on whether the state updates was a success or not
        // purpose: updates the internal mpc_params vector variable.
        bool updateMpcParameters(std::map<int, double> &params)

        // input: contains the state vector.
        // output: feedback on whether the state updates was a success or not
        // purpose: updates the internal mpc_state vector variable.
        bool updateMpcState(std::vector<double> &state);

        // input: contains the desired reference angle.
        // output: feedback on whether the reference updates was a success or not
        // purpose: updates the internal mpc_ref vector variable.
        bool updateMpcState(double &reference);

        // input: args
        // output: feedback on whether the solver was a success or not
        // purpose: calls the acados object to optimize the problem given current vehicle and env state
        bool optimizeMpcProblem(std::vector &args);

        // input: time elapsed since last state update
        // output: returns the optimal control value given time
        // purpose: time_elapsed contains the info on when the state was last updated. 
        //          If the last state update was immediate, then return first control input but if the last state update happened a while back, 
        //          do not optimize the problem again but send subsequent control data
        double getOptimalRudderAngle(double &time_elapsed);

        // input: vars passed as reference to store mpc prediction
        // output: 
        // purpose: This functions saves the state and input prediction information over the control horizon to the passed variables.
        void getMpcPrediction(std::vector<float> &state_p, std::vector<float> &input_p);

        // input: vector of length 7, environmental state parameters
        // output: vector of length 2, condensed coefficients for surge model
        // purpose: reduces 7 params to 2
        void getCondensedSurgeModel(std::vector<double> &env_state, std::vector<double> &surge_coefficients);

    };
}