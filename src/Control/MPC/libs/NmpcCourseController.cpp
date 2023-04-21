// NmpcCourseController.cpp
// compile with: cl /c /EHsc MathLibrary.cpp
// post-build command: lib MathLibrary.obj

#include "NMPC.h"

namespace NMPC
{
    CourseController::CourseController(std::string model){

        // ask user which model to load as the argument
        switch(model_idx[model]){
            case 0:
            case 1: std::map<std::string, int> state_idx = {
                        {"psi", 0}, {"u", 1}, {"v", 2}, {"r", 3}, {"delta", 4}
                    };
                    break;
            case 2:
            case 3: std::map<std::string, int> state_idx = {
                        {"psi", 0}, {"v", 1}, {"r", 2}, {"delta", 3}
                    };
                    break;
            // TODO: Throw error and fail
            default: std::cout << "error, no model found!" << std::endl;            
        }

        // load the default controller parameters
        this->config = loadDefaultConfiguration();

        // define identifier to indice maps
        std::map<std::string, int> config_idx = {
            {"Ts", 0}, {"Tp", 1}, {"solver", 2}, {"cost_Q", 3}, {"cost_R", 4}, {"cost_S", 5} 
        };
        std::map<std::string, int> params_idx = {
            {"Vc", 0}, {"beta_c", 1}, {"Vw", 2}, {"beta_w", 3}, {"Hs", 4}, {"omega_p", 5}, {"gamma_w", 6}
        };
        std::map<std::string, int> model_idx = {
            {"nonlinear4", 0}, {"linear4", 1}, {"nonlinear3", 2}, {"linear3", 3}
        };
    }

    bool defineMpcProblem(std::map<std::string, double> &config)
    {
        // Updates MPC Parameters
        for (auto i : config){
            this->config[config_idx[i.first]] = i.second;
        }

        int Ts = this->config[0];
        int Tp = this->config[1];
        int solver = this->config[2];

        // TODO: multiple solvers or models
        acados_ocp_capsule = x8_ocp_ERK_gryte_acados_create_capsule();
        status = x8_ocp_ERK_gryte_acados_create(acados_ocp_capsule, horizon_length);


        return a + b;
    }

}