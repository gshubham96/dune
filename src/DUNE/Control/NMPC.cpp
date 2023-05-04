#include <DUNE/Control/NMPC.hpp>

namespace DUNE{
    namespace CONTROL{
        namespace NMPC{

        // wrap the angle between [-pi, pi]
        double Dynamics::ssa(double diff) {
            diff -= (2*PI) * floor((diff + PI) * (1 / 2*PI));
            return diff;
        }

        // wrap the angle between [-pi, pi] for SX Symbolics
        casadi::SX Dynamics::ssa(casadi::SX diff) {
            return diff;
            diff -= (2*PI) * floor((diff + PI) * (1 / 2*PI));
            return diff;
        }

        // Function to define and compile the NLP Optimization Problem
        bool Dynamics::defineDynamicsProblem(bool compile){

            // ################################################
            // ###----------------SETUP PARAMS--------------###
            // ################################################
            // mpc params
            nx = 4; nu = 1; np = 14;                                                    // constants as their is no plan to allow multiple models yet

            // read system params from file
            if(!loadDefaultsFromFile(file_path, system_)){
                ERROR_STRING = "COULD NOT LOAD SYSTEM PARAMETERS FROM FILE";
                return false;
            }

            // system params
            const double D11 = system_["D11"], R11 = system_["R11"], INV_M11 = system_["INV_M11"];
            const double D22 = system_["D22"], R22 = system_["R21"], INV_M22 = system_["INV_M22"], INV_M23 = system_["INV_M23"];
            const double D33 = system_["D33"], R33 = system_["R31"], INV_M32 = system_["INV_M32"], INV_M33 = system_["INV_M33"];
            const double CR12 = system_["CR12"], CR21 = system_["CR21"];
            const double W11 = system_["W11"], W21 = system_["W21"], W31 = system_["W31"];

            // ################################################
            // ###----------------SETUP CONTROL DYNAMICS----###
            // ################################################

            // named symbolica vars
            casadi::SX psi = casadi::SX::sym("psi", 1),
                    u = casadi::SX::sym("u", 1),
                    v = casadi::SX::sym("v", 1),
                    r = casadi::SX::sym("r", 1),
                    delta = casadi::SX::sym("delta", 1);

            // optim vars for each shooting period
            casadi::SX sym_x = vertcat(psi, u, v, r);
            casadi::SX sym_u = delta;
            casadi::SX sym_p = casadi::SX::sym("p", np);

            // environmental parameters that are constant over a given horizon
            casadi::SX
                chi_d   = sym_p(nx),
                Vc      = sym_p(nx+1),
                beta_c  = sym_p(nx+2),
                Vw      = sym_p(nx+3),
                beta_w  = sym_p(nx+4),
                Hs      = sym_p(nx+5),
                omega_p = sym_p(nx+6),
                gamma_p = sym_p(nx+7),
                Q       = sym_p(nx+8),
                R       = sym_p(nx+9);

            // hard coded speed model. 
            // TODO load from FILE
            speed_model = {0.116392998053662, 0.214487083945715, 0.0880678632611925, -0.00635496887217675, 0.0937464223577265, 0.238364678400396 };        


            // surge coefficients
            casadi::SX
                k_1 = speed_model[0]*Hs + speed_model[1]*omega_p + speed_model[2]*cos(gamma_p) + speed_model[4]*Vc*cos(beta_c) + speed_model[5],
                k_2 = speed_model[3]*Vw;

            // detived states
            casadi::SX 
                u_e = u + EPS,
                u_c = Vc * cos(beta_c - psi),
                v_c = Vc * sin(beta_c - psi),
                u_r = u_e - u_c,
                v_r = v - v_c,
                nu_r = vertcat(u_r, v_r, r),
                U_r2 = pow(u_r, 2) + pow(v_r, 2);

            // ################################################
            // ###----------------DYNAMIC EQUATIONS---------###
            // ################################################

            // CURRENTS
            casadi::SX
                nu_c_dot_u = v_c * r,
                nu_c_dot_v = -u_c * r;

            // DAMPING
            casadi::SX 
                damping_u  = D11,
                damping_v  = D22,
                damping_r  = D33;

            // CORIOLIS 
            // Add only if model is nonlinear
            casadi::SX coriolis_u = 0, coriolis_v = 0;
            if(model_type == 0){
                    coriolis_u = CR12 * r,
                    coriolis_v = CR21 * r;
            }

            // WAVE FOILS
            casadi::SX tau_foil_u = (k_1 + k_2*cos(psi - beta_w - PI)) * D11;

            // RUDDER
            casadi::SX tau_rudr_u, tau_rudr_v, tau_rudr_r;
            // If the model is nonlinear, consider nonlinear dynamics of the rudder
            if(model_type == 0){
                casadi::SX alpha_r = delta - atan(v_r/u_r);
                tau_rudr_u = R11 * U_r2 * sin(alpha_r) * sin(delta) ;
                tau_rudr_v = R22 * U_r2 * sin(alpha_r) * cos(delta) ;
                tau_rudr_r = R33 * U_r2 * sin(alpha_r) * cos(delta) ;
            }
            // else consider the approximated linear equations
            else if(model_type == 1){
                tau_rudr_u = R11 * U_r2 * delta * delta ;
                tau_rudr_v = R22 * U_r2 * delta * 0.5 ;
                tau_rudr_r = R33 * U_r2 * delta * 0.5 ;
            }

            // WIND
            // Add only if model is nonlinear
            casadi::SX tau_wind_u = 0, tau_wind_v = 0, tau_wind_r = 0;
            if(model_type == 0){
                casadi::SX u_rw = u_e - Vw * cos(beta_w - psi);
                casadi::SX v_rw = v - Vw * sin(beta_w - psi);
                casadi::SX V_rw2 = pow(u_rw, 2) + pow(v_rw, 2);
                casadi::SX gamma = -atan2(v_rw, u_rw);

                tau_wind_u = W11 * V_rw2 * cos(gamma);
                tau_wind_v = W21 * V_rw2 * sin(gamma);
                tau_wind_r = W31 * V_rw2 * sin(2*gamma);
            }

            // dynamics of yaw
            casadi::SX yaw_dot = r;

            // dynamics of surge
            casadi::SX u_dot = nu_c_dot_u + INV_M11*(tau_wind_u + tau_foil_u + tau_rudr_u - damping_u*u_r - coriolis_u*v_r);

            // dynamics of sway
            casadi::SX v_dot = nu_c_dot_v 
                                + INV_M22*(tau_wind_v + tau_rudr_v - damping_v*v_r - coriolis_v*u_r)
                                + INV_M23*(tau_wind_v + tau_rudr_r - damping_r*r);

            // dynamics of yaw rate
            casadi::SX r_dot = 0 
                                + INV_M32*(tau_wind_r + tau_rudr_v - damping_v*v_r - coriolis_v*u_r)
                                + INV_M33*(tau_wind_r + tau_rudr_r - damping_r*r);

            casadi::SX nu_dot = vertcat(yaw_dot, u_dot, v_dot, r_dot);
            x_dot = casadi::Function("x_dot", {sym_x, sym_u, sym_p}, {nu_dot});

            // Compiles the function for faster compute
            // TODO: Add JIT Compilation using clang?
            solver.generate_dependencies("nlp.c");
            if (compile) {
                // Compile the c-code
                int flag = system("gcc -fPIC -shared -O3 x_dot.c -o x_dot.so");
                casadi_assert(flag==0, "Compilation failed");

                // Create a new NLP solver instance from the compiled code
                x_dot = casadi::external("x_dot", "x_dot.so");
            }
            return true;
        }

        // reads data from file and stores in passed arg
        bool Dynamics::loadDefaultsFromFile(const std::string &file_name, std::map<std::string, double> &data_from_file){

            // get file name + path
            std::string full_file_name = fs::current_path().parent_path().string() + "/dune/etc/autonaut-mpc/" + file_name;

            std::ifstream myFile(full_file_name);
            std::string line;

            if (myFile.fail()){
                std::cerr << "ERROR: FILE OPEN FAILED. " << myFile.is_open() << std::endl;
                std::cerr << "ERROR: LOOKING AT: " << full_file_name << std::endl;
                return false;               
            }

            while (std::getline(myFile, line)) {

                // create a stringstream to read the data
                std::istringstream iss(line);

                // create key and value variables to store the data
                std::string key;
                double value;

                // skip this line if unable to read both key and value
                if (!(iss >> key >> value))
                    continue;                   

                // store the key and value to map 
                data_from_file[key] = value;
            }
            myFile.close();        

            return true;    
        }

        bool Dynamics::simulateDynamics(const std::vector<double> &state_init, const double u0, const std::map<std::string, double> &params, std::vector<double> &state_next){

            std::map<std::string, casadi::DM> arg, f_eval;

            x_dot = casadi::Function("x_dot", {sym_x, sym_u, sym_p}, {nu_dot});

            // set input
            arg["i1"] = u0;

            // set parameters
            if(!areParamsSane(params_)){
                ERROR_STRING = "PARAMETERS ARE NOT SANE";
                return false;
            }
            std::vector<double> param_vector(np, 0);
            param_vector[5] = params["Vc"];
            param_vector[6] = ssa(params["beta_c"]);
            param_vector[7] = params["Vw"];
            param_vector[8] = ssa(params["beta_w"]);
            param_vector[9] = params["Hs"];
            param_vector[10] = params["omega_p"];
            param_vector[11] = params["gamma_p"];
            arg["i2"] = param_vector;

            // set initial state
            std::vector<double> p0 = state_init;
            arg["i0"] = p0;

            // RUNGE KUTTA STAGE 1
            f_eval = x_dot(args);
            std::vector<double> rk1(f_eval["o0"]);

            // RUNGE KUTTA STAGE 2
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + 0.5*Ts*rk1[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk2(f_eval["o0"]);

            // RUNGE KUTTA STAGE 3
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + 0.5*Ts*rk2[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk3(f_eval["o0"]);

            // RUNGE KUTTA STAGE 4
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + Ts*rk3[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk4(f_eval["o0"]);

            // NEXT STATE
            std::vector<double> x_rk4(4,0);
            for(int i=0; i<nx; i++){
                rk2[i] *= 2;
                rk3[i] *= 2;

                x_rk4[i] = p0[i] + (Ts/6) * (rk1[i] + rk2[i] + rk3[i] + rk4[i]);
            }

            state_next = x_rk4;
            return true;
        }

        // performs sanity check of config params
        bool Dynamics::areParamsSane(const std::map<std::string, double> &mapped_dict){
            // check for the correct number of configuration paramters
            if((int)mapped_dict.size() != np-nx-1){
                ERROR_STRING = "PARAMETER NOT OF RIGHT LENGTH!";
                return false;
            }
            
            // checks for all keys 
            int sum = mapped_dict.count("Vc") + mapped_dict.count("beta_c") + mapped_dict.count("Vw") + mapped_dict.count("beta_w")
                    + mapped_dict.count("Hs") + mapped_dict.count("omega_p") + mapped_dict.count("gamma_p") + mapped_dict.count("Q") + mapped_dict.count("R");
            if(sum != np-nx-1){
                ERROR_STRING = "ALL RUNTIME PARAMETERs NOT PRESENT!";
                return false;
            }
            
            return true;
        }

        // returns the string that stores errors
        void Dynamics::getErrorString(std::string &err){
            err = ERROR_STRING;
            ERROR_STRING = "";
        }

        // allow user to skip configuration
        Dynamics(const std::string &model_type, const double &Tp, const &double Ts):Tp_(Tp), Ts_(Ts){
            // get config params
            if (model_type.compare("nonlinear"))
                model_type_ = 0;
            else if (model_type.compare("linear"))
                model_type_ = 1;
            else
                ERROR_STRING = "model_type NOT FOUND. CAN ONLY BE <linear> or <nonlinear>"

            // define dynamics
            if(defineDynamicsProblem(compile))
                ERROR_STRING = "PROBLEM DEFINED CORRECTLY";
            else
                ERROR_STRING = "ERROR DEFINING PROBLEM";
        }

        // Destructor
        Dynamics::~Dynamics() { 
        }

    }
}
