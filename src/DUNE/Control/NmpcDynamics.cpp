#include <DUNE/Control/NmpcDynamics.hpp>

namespace DUNE{
    namespace Control{
        // wrap the angle between [-pi, pi]
        double NmpcDynamics::ssa(double diff) {
            diff -= (2*PI) * floor((diff + PI) * (1 / 2*PI));
            return diff;
        }

        // wrap the angle between [-pi, pi] for SX Symbolics
        casadi::SX NmpcDynamics::ssa(casadi::SX diff) {
            return diff;
            diff -= (2*PI) * floor((diff + PI) * (1 / 2*PI));
            return diff;
        }

        // Function to define and compile the NLP Optimization Problem
        bool NmpcDynamics::defineDynamicsProblem(bool compile){

            // ################################################
            // ###----------------SETUP PARAMS--------------###
            // ################################################

            // read system params from file
            if(!loadDefaultsFromFile(file_path_, system_)){
                ERROR_STRING_ = "COULD NOT LOAD SYSTEM PARAMETERS FROM FILE";
                std::cout << ERROR_STRING_ << std::endl;
                return false;
            }

            // system params
            const double D11 = system_.at("D11"), R11 = system_.at("R11"), INV_M11 = system_.at("INV_M11");
            const double D22 = system_.at("D22"), R22 = system_.at("R21"), INV_M22 = system_.at("INV_M22"), INV_M23 = system_.at("INV_M23");
            const double D33 = system_.at("D33"), R33 = system_.at("R31"), INV_M32 = system_.at("INV_M32"), INV_M33 = system_.at("INV_M33");
            const double CR12 = system_.at("CR12"), CR21 = system_.at("CR21");
            const double W11 = system_.at("W11"), W21 = system_.at("W21"), W31 = system_.at("W31");

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
            sym_x = vertcat(psi, u, v, r);
            sym_u = delta;
            sym_p = casadi::SX::sym("p", np);

            // environmental parameters that are constant over a given horizon
            casadi::SX
                Vc      = sym_p(nx+1),
                beta_c  = sym_p(nx+2),
                Vw      = sym_p(nx+3),
                beta_w  = sym_p(nx+4),
                Hs      = sym_p(nx+5),
                omega_p = sym_p(nx+6),
                gamma_p = sym_p(nx+7);

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
            if(model_type_ == 0){
                    coriolis_u = CR12 * r,
                    coriolis_v = CR21 * r;
            }

            // WAVE FOILS
            casadi::SX tau_foil_u = (k_1 + k_2*cos(psi - beta_w - PI)) * D11;

            // RUDDER
            casadi::SX tau_rudr_u, tau_rudr_v, tau_rudr_r;
            // If the model is nonlinear, consider nonlinear dynamics of the rudder
            if(model_type_ == 0){
                casadi::SX alpha_r = delta - atan(v_r/u_r);
                tau_rudr_u = R11 * U_r2 * sin(alpha_r) * sin(delta) ;
                tau_rudr_v = R22 * U_r2 * sin(alpha_r) * cos(delta) ;
                tau_rudr_r = R33 * U_r2 * sin(alpha_r) * cos(delta) ;
            }
            // else consider the approximated linear equations
            else if(model_type_ == 1){
                tau_rudr_u = R11 * U_r2 * delta * delta ;
                tau_rudr_v = R22 * U_r2 * delta * 0.5 ;
                tau_rudr_r = R33 * U_r2 * delta * 0.5 ;
            }

            // WIND
            // Add only if model is nonlinear
            casadi::SX tau_wind_u = 0, tau_wind_v = 0, tau_wind_r = 0;
            if(model_type_ == 0){
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
            if (compile) {
                x_dot.generate_dependencies("x_dot.c");
                // Compile the c-code
                int flag = system("gcc -fPIC -shared -O3 x_dot.c -o x_dot.so");
                casadi_assert(flag==0, "Compilation failed");

                // Create a new NLP solver instance from the compiled code
                x_dot = casadi::external("x_dot", "x_dot.so");
            }
            return true;
        }

        // reads data from file and stores in passed args
        bool NmpcDynamics::loadDefaultsFromFile(const std::string &file_name, std::map<std::string, double> &data_from_file){

            // get file name + path
            // std::string full_file_name = fs::current_path().parent_path().string() + "/dune/etc/autonaut-mpc/" + file_name;
            // std::string full_file_name = file_name + "/system.csv";
            std::string full_file_name = file_name;

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

        bool NmpcDynamics::simulateDynamics(const std::vector<double> &state_init, const double u0, const std::map<std::string, double> &params, std::vector<double> &state_next){

            std::map<std::string, casadi::DM> args, f_eval;

            // set input
            args["i1"] = u0;

            // set parameters
            if(!areParamsSane(params)){
                ERROR_STRING_ = "PARAMETERS ARE NOT SANE";
                return false;
            }
            std::vector<double> param_vector(np, 0);
            param_vector[5] = params.at("Vc");
            param_vector[6] = ssa(params.at("beta_c"));
            param_vector[7] = params.at("Vw");
            param_vector[8] = ssa(params.at("beta_w"));
            param_vector[9] = params.at("Hs");
            param_vector[10] = params.at("omega_p");
            param_vector[11] = params.at("gamma_p");
            args["i2"] = param_vector;

            // set initial state
            std::vector<double> p0 = state_init;
            args["i0"] = p0;

            // RUNGE KUTTA STAGE 1
            f_eval = x_dot(args);
            std::vector<double> rk1(f_eval["o0"]);

            // RUNGE KUTTA STAGE 2
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + 0.5*Ts_*rk1[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk2(f_eval["o0"]);

            // RUNGE KUTTA STAGE 3
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + 0.5*Ts_*rk2[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk3(f_eval["o0"]);

            // RUNGE KUTTA STAGE 4
            for(int i=0; i<nx; i++)
                p0[i] = state_init[i] + Ts_*rk3[i];
            args["i0"] = p0;
            f_eval = x_dot(args);
            std::vector<double> rk4(f_eval["o0"]);

            // NEXT STATE
            std::vector<double> x_rk4(4,0);
            for(int i=0; i<nx; i++){
                rk2[i] *= 2;
                rk3[i] *= 2;

                x_rk4[i] = p0[i] + (Ts_/6) * (rk1[i] + rk2[i] + rk3[i] + rk4[i]);
            }

            state_next = x_rk4;

            std::cout << "state init: " << state_init << std::endl;
            std::cout << "input     : " << u0 << std::endl;
            std::cout << "params    : " << params << std::endl;
            std::cout << "state next: " << state_next << std::endl;
            std::cout << "-----------------------\n";

            return true;
        }

        // performs sanity check of config params
        bool NmpcDynamics::areParamsSane(const std::map<std::string, double> &mapped_dict){

            std::cout << "mapped_dict --" << mapped_dict << std::endl;
            
            // check for the correct number of configuration paramters
            if((int)mapped_dict.size() != 7){
                ERROR_STRING_ = "PARAMETER NOT OF RIGHT LENGTH!";
                std::cout << "## --" << ERROR_STRING_ << std::endl;
                return false;
            }
            
            // checks for all keys 
            int sum = mapped_dict.count("Vc") + mapped_dict.count("beta_c") + mapped_dict.count("Vw") + mapped_dict.count("beta_w")
                    + mapped_dict.count("Hs") + mapped_dict.count("omega_p") + mapped_dict.count("gamma_p");
            if(sum != np-nx-1){
                ERROR_STRING_ = "ALL RUNTIME PARAMETERs NOT PRESENT!";
                std::cout << "## --" << ERROR_STRING_ << std::endl;
                return false;
            }
            
            return true;
        }

        // returns the string that stores errors
        void NmpcDynamics::getErrorString(std::string &err){
            err = ERROR_STRING_;
            ERROR_STRING_ = "";
        }

        // allow user to skip configuration
        bool NmpcDynamics::configureDynamics(const std::string model_type, const std::string &file_path, const double &Ts, const bool &compile){
            // Time step
            Ts_ = Ts;

            file_path_ = file_path;

            // get config params
            if (model_type.compare("nonlinear"))
                model_type_ = 0;
            else if (model_type.compare("linear"))
                model_type_ = 1;
            else{
                ERROR_STRING_ = "model_type NOT FOUND. CAN ONLY BE <linear> or <nonlinear>";
                return false;
            }

            // define dynamics
            if(!defineDynamicsProblem(compile)){
                ERROR_STRING_ = "ERROR DEFINING PROBLEM";
                return false;
            }

            return true;
        }

        // Constructor
        NmpcDynamics::NmpcDynamics() { 
            // mpc params
            // TODO constants as their is no plan to allow multiple models yet
            nx = 4; nu = 1; np = 14;                                                    
        }

        // Destructor
        NmpcDynamics::~NmpcDynamics() { 
        }
    }
}
