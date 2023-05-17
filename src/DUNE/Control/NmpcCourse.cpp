#include <DUNE/Control/NmpcCourse.hpp>

namespace DUNE{
    namespace Control{
        // Function to define and compile the NLP Optimization Problem
        bool NmpcCourse::defineMpcProblem(bool compile){

            // ################################################
            // ###----------------SETUP LOOP----------------###
            // ################################################
            N = Tp_/Ts_;

            casadi::SX
                chi_d   = sym_p(4),
                Q       = sym_p(12),
                R       = sym_p(13);

            // trajectory / motion planning
            casadi::SX chi_t_dot, chi_t = chi_d;

            // optimization variables
            casadi::SX
                X = casadi::SX::sym("X", nx, N+1),
                U = casadi::SX::sym("U", N),
                optims = casadi::SX::sym("optims", nx*(N+1) + nu*N);

            // objective function, equlity constraints
            casadi::SX 
                obj = 0,
                g = casadi::SX::sym("g", nx*(N+1));

            // casadi loop helper vars
            casadi::SX sym_du, sym_dx = casadi::SX::sym("sym_dx", nx);

            // set initial state
            for(int j = 0; j < nx; j++)
                sym_dx(j) = X(j,0) - sym_p(j);
            sym_dx(0) = ssa(sym_dx(0));

            // fill in the constraint vector
            for(int j = 0; j < nx; j++)
                g(j) = sym_dx(j);

            // optimization loop
            for(int i = 0; i < N; i++){

                // assign current state
                for(int j = 0; j < nx; j++)
                    sym_x(j) = X(j,i);

                // assign current input or difference in input
                sym_u = U(i);
                if(i > 0)
                    sym_du = U(i) - U(i-1);
                else
                    sym_du = U(i);

                // assign states for readibility
                casadi::SX
                    psi_p = sym_x(0),
                    u_p = sym_x(1) + EPS,
                    v_p = sym_x(2),
                    r_p = sym_x(3);

                casadi::SX SOG = sqrt( pow(u_p,2) + pow(v_p,2) );

                // trajectory
                chi_t_dot = ssa(chi_d - chi_t);
                chi_t = ssa(chi_t + Ts_ * chi_t_dot);

                casadi::SX delta_x;
                // minimizes error in course angle
                if(cost_type_ == 0)
                    delta_x = ssa(chi_t - psi_p - asin(v_p / SOG));

                // minimizes error in course vector                    
                else if(cost_type_ == 1){
                    casadi::SX vec_chi_p = casadi::SX::sym("vec_chi_p", 2);
                    vec_chi_p(0) = 1/SOG * (u_p * cos(psi_p) - v_p * sin(psi_p));
                    vec_chi_p(1) = 1/SOG * (u_p * sin(psi_p) + v_p * cos(psi_p));

                    casadi::SX vec_chi_d = casadi::SX::sym("vec_chi_d", 2);
                    vec_chi_d(0) = cos(chi_t);
                    vec_chi_d(1) = sin(chi_t);

                    delta_x = 1 - mtimes(vec_chi_d.T(), vec_chi_p);
                }

                // minimizes error in heading angle
                else if(cost_type_ == 2)
                    delta_x = ssa(chi_t - psi_p);
                
                // 
                casadi::SX cost_x  = delta_x * Q * delta_x;
                casadi::SX cost_u  = sym_du * R * sym_du;
                obj = obj + cost_u + cost_x;

                // multiple shooting using Runge-Kutta4
                casadi::SXDict args, f_eval;
                // Stage 1
                args["i0"] = sym_x;
                args["i1"] = sym_u;
                args["i2"] = sym_p;
                f_eval = x_dot(args);
                casadi::SX rk1 = f_eval["o0"];

                // Stage 2
                args["i0"] = sym_x + 0.5*Ts_*rk1;
                args["i1"] = sym_u;
                f_eval = x_dot(args);
                casadi::SX rk2 = f_eval["o0"];

                // Stage 3
                args["i0"] = sym_x + 0.5*Ts_*rk2;
                args["i1"] = sym_u;
                f_eval = x_dot(args);
                casadi::SX rk3 = f_eval["o0"];

                // Stage 4
                args["i0"] = sym_x + Ts_*rk3;
                args["i1"] = sym_u;
                f_eval = x_dot(args);
                casadi::SX rk4 = f_eval["o0"];

                // next state
                casadi::SX sym_x_rk4 = sym_x + (Ts_/6) * (rk1 + 2*rk2 + 2*rk3 + rk4);

                // introduce dynamics to constraints
                for(int j = 0; j < nx; j++)
                    sym_dx(j) = X(j,i+1) - sym_x_rk4(j);
                sym_dx(0) = ssa(sym_dx(0));

                for(int j = 0; j < nx; j++)
                    g(nx*(i+1) + j) = sym_dx(j);

                // push into main vector being optimized
                for(int j = 0; j < nx; j++)
                    optims(nx*i + j) = sym_x(j);
                optims(nx*(N+1) + i) = sym_u;

            }
            for(int j = 0; j < nx; j++)
                optims(nx*N + j) = X(j,N);

            // ################################################
            // ###----------------SETUP NLP PROBLEM---------###
            // ################################################
            casadi::SXDict nlp = {{"x", optims}, {"f", obj}, {"g", g}, {"p", sym_p}};

            // nlp options
            casadi::Dict opts;
            opts["ipopt.max_iter"] = 300;
            opts["ipopt.print_level"] = 0;
            opts["ipopt.acceptable_tol"] = 1e-8;
            opts["ipopt.acceptable_obj_change_tol"] = 1e-6;
            opts["ipopt.warm_start_init_point"] = "yes";
            // opts_dict["ipopt.sb"] = "yes";
            opts["print_time"] = 0;

            solver_ = casadi::nlpsol("solver", "ipopt", nlp, opts);
            if (compile) {
                solver_.generate_dependencies("solver.c");
                // Compile the c-code
                int flag = system("gcc -fPIC -shared -O3 solver.c -o solver.so");
                casadi_assert(flag==0, "Compilation failed");

                // Create a new NLP solver instance from the compiled code
                solver_ = casadi::nlpsol("solver", "ipopt", "solver.so");
            }

            // define state bounds
            std::vector<double> ubx, lbx, ubg, lbg;
            for(int i = 0; i < nx*(N+1); i++){
                lbx.push_back(-casadi::inf);
                ubx.push_back(casadi::inf);
                lbg.push_back(0);
                ubg.push_back(0); 
            }
            for(int i = nx*(N+1); i < nx*(N+1)+nu*N; i++){
                lbx.push_back(-DEG2RAD(40));
                ubx.push_back(DEG2RAD(40));
            }

            // setup lower and upper bounds for constraints as well as warm start
            args_["lbx"] = lbx;
            args_["ubx"] = ubx;
            args_["lbg"] = lbg;
            args_["ubg"] = ubg;

            args_["x0"] = generateRandomVector(nx*(N+1)+nu*N);
            args_["lam_x0"] = generateRandomVector(nx*(N+1)+nu*N);
            args_["lam_g0"] = generateRandomVector(nx*(N+1));

            initialized_=0;
            return true;
        }

        // generates random vector for warm start
        std::vector<double> NmpcCourse::generateRandomVector(int n) {
            std::vector<double> result(n);
            std::random_device rd;              // obtain a random seed from the OS
            std::mt19937 gen(rd());             // seed the generator
            std::uniform_real_distribution<> distr(EPS, 1.0); // define the range
            for (int i = 0; i < n; ++i) {
                result[i] = distr(gen);         // generate the random number and assign it to the vector
            }
            return result;
        }

        // updates parameters such as wind, currents, etc
        // need to do it atlease once
        bool NmpcCourse::updateMpcParams(const std::map<std::string, double> &param){
            // Controller is ready to run when parameters are set
            switch(initialized_){
                case -1: ERROR_STRING_ = "configure problem first!\n"; return false;
                case  0: initialized_++; break;
                default: break;
            }

            // Update defaul parameter list
            for (auto it = param.begin(); it != param.end(); it++)            
                params_[it->first] = it->second;

            return true;
        }

        // updates mpc state
        bool NmpcCourse::updateMpcState(const std::map<std::string, double> &state){

            // flag to check if state was updated
            switch(initialized_){
                case -1: ERROR_STRING_ = "configure problem first!\n"; return false;
                case  0: ERROR_STRING_ = "update parameters first!\n"; return false;
                case  1: initialized_++; break;
                default: break;
            }

            // Update vehicle state
            for (auto i : state) 
                state_[i.first] = i.second;

            return true;
        }

        // updates course reference angle [rad]
        bool NmpcCourse::updateMpcReference(const double &reference){
            // Update vehicle course reference
            reference_ = reference;
            return true;
        }

        // cosntructs a mpc-friendly format parameter vector
        std::vector<double> NmpcCourse::reWriteParams(){
            
            std::vector<double> param_vector(np, 0);

            // set initial state
            param_vector[0] = ssa(state_["psi"]);
            param_vector[1] = state_["u"];
            param_vector[2] = state_["v"];
            param_vector[3] = state_["r"];
            // set desired state
            param_vector[4] = reference_;
            // set env params                
            param_vector[5] = params_["Vc"];
            param_vector[6] = ssa(params_["beta_c"]);
            param_vector[7] = params_["Vw"];
            param_vector[8] = ssa(params_["beta_w"]);
            param_vector[9] = params_["Hs"];
            param_vector[10] = params_["omega_p"];
            param_vector[11] = params_["gamma_p"];
            // set costs
            param_vector[12] = params_["Q"];
            param_vector[13] = params_["R"];

            return param_vector;
        }

        bool NmpcCourse::optimizeMpcProblem(){

            // flag to check if nlp was set up and parameters were updated
            switch(initialized_){
                case -1: ERROR_STRING_ = "configure problem first!"; return false;
                case  0: ERROR_STRING_ = "update parameters first!"; return false;
                case  1: ERROR_STRING_ = "update state first!"; return false;
                default: break;
            }

            std::map<std::string, casadi::DM> arg, res;
            // set state and input constraints
            arg["lbx"] = args_["lbx"];
            arg["ubx"] = args_["ubx"];
            arg["lbg"] = args_["lbg"];
            arg["ubg"] = args_["ubg"];

            // set Mpc parameters
            // checks if params are sane
            if(areParamsSane(params_)){
                std::vector p = reWriteParams();
                arg["p"] = p;           
            }
            else
                return false;
            
            // set initial trajectory for warm start
            arg["x0"] = args_["x0"];
            arg["lam_x0"] = args_["lam_x0"];
            arg["lam_g0"] = args_["lam_g0"];


            res = solver_(arg);
            solution_exists_ = true;

            // get optimal input trajectory
            optimized_vars_.clear();
            optimized_vars_ = std::vector<double>(res.at("x"));                

            input_traj_.clear();
            input_traj_.assign(optimized_vars_.begin()+nx*(N+1), optimized_vars_.end());
            
            // update variables for warm start
            args_["x0"]  = optimized_vars_;
            args_["lam_x0"]  = std::vector<double>(res.at("lam_x"));
            args_["lam_g0"]  = std::vector<double>(res.at("lam_g"));

            std::cout << "parameters         : " << arg["p"] << std::endl;
            // print_details();

            initialized_--;
            return true;
        }

        // get solution from the latest optimized input trajectory
        bool NmpcCourse::getOptimalInput(double &u_star, const double &t_elapsed){

            // fail if problem is not yet initialized_
            switch(initialized_){
                case -1: ERROR_STRING_ = "PROBLEM NOT YET CONFIGURED!"; return false;
                default: break;
            }

            // fail if NLP hasn't been solved even once 
            if(!solution_exists_){
                ERROR_STRING_ = "VALID SOLUTION DOES NOT EXISTS";
                return false;
            }

            // fail if NLP has not been run for a long time
            if(t_elapsed > 0.25*Tp_){
                ERROR_STRING_ = "TIME SINCE LAST NLP RUN EXCEEDS THRESHOLD";
                solution_exists_ = false;
                return false;
            }

            // otherwise, find the closest time index and send that input
            int t_ind = round(t_elapsed/Ts_);
            u_star = input_traj_[t_ind];

            // for(int j = 0; j < 25; j++)
            //     std::cout << "traj at " << j << " is " << input_traj_[j] << ", ";
            // std::cout << std::endl;

            return true;
        }

        // 
        void NmpcCourse::saveTrajectoryToFile(){
            if(filecount_ == -1){
                // open a file
                filename_ = fs::current_path().parent_path().string();
                filename_ = filename_ + "/results/course_gen.m";
                file_.open(filename_.c_str());
                file_ << "% Results file_ from " __FILE__ << std::endl;
                file_ << "% Generated " __DATE__ " at " __TIME__ << std::endl << std::endl;
                file_.close();
                filecount_++;
            }
            else if(filecount_ > -1){
                // save trajectory to file
                file_.open(filename_.c_str(), std::ios::app);
                file_<< "chi_d" << filecount_ << " = " << reference_ << ";" << std::endl;
                file_<< "optims" << filecount_++ << " = " << optimized_vars_ << ";" << std::endl;
                file_.close();
            }
        }

        // 
        void NmpcCourse::print_details(){

            std::cout.precision(3);
            double psi = optimized_vars_[nx*N];
            double u = optimized_vars_[nx*N+1];
            double v = optimized_vars_[nx*N+2];
            double beta = atan(v/u);
            double chi = psi + beta;

            // prints out desired and current states
            std::cout << "desired angle      : " << reference_ << std::endl;
            std::cout << "final heading angle: " << psi << std::endl;
            std::cout << "final course  angle: " << chi  << std::endl;

            // prints out the trajectory
            // for(int i = N-4; i < N; i++){
            //     std::cout << "N: " << i << ", st: ";
            //     for(int j = 0; j < nx; j++)
            //         std::cout << optimized_vars_[nx * i + j] << ", ";                    
            //     std::cout << "cn: " << optimized_vars_[nx*(N+1)+i] << std::endl;                    
            // }
            // std::cout << "N: " << N << ", st: ";
            // for(int j = 0; j < nx; j++)
            //     std::cout << optimized_vars_[nx * N + j] << ", ";                    

            // for(int j = 0; j < 10; j++)
            //     std::cout << "input traj at " << j << " is " << input_traj_[j] << std::endl;

            std::cout << "\n##################################\n";
        }

        // 
        void NmpcCourse::reset(){
            // To be implemented
        }

        // checks if the problem is configured properly. If not, configure it
        bool NmpcCourse::isProblemConfigured(){
            // returns true if initalized is greater than -1.
            if (initialized_ > -1)
                return true;
            return false;
        }

        // allow user to skip configuration
        bool NmpcCourse::configureSolver(std::string model_type, std::string cost_type, std::string file_path, double Tp, double Ts, bool compile){
            // set init flag
            initialized_ = -1;
            // set file count flag
            filecount_ = -1;
            // set solution flag
            solution_exists_ = false;

            // configure Dynamics
            if(!configureDynamics(model_type, file_path, Ts, false)){
                ERROR_STRING_ = "ERROR CONFIGURING DYNAMICS";
                return false;
            };

            // Prediction Horizon for MPC
            Tp_ = Tp;

            // get config params
            if (cost_type.compare("chi_d"))
                cost_type_ = 0;
            else if (cost_type.compare("dotv"))
                cost_type_ = 1;
            else if (cost_type.compare("psi_d"))
                cost_type_ = 2;
            else{
                ERROR_STRING_ = "cost_type_ NOT FOUND. CAN ONLY BE <chi_d>, <dotv> or <psi_d>";
                return false;
            }

            // define solver
            if(!defineMpcProblem(compile)){
                ERROR_STRING_ = "ERROR DEFINING SOLVER";
                return false;
            }

            // initialize logging
            saveTrajectoryToFile();

            return true;
        }

        // Default Constructor
        NmpcCourse::NmpcCourse(){
            // set init flag
            initialized_ = -1;
        }

        // Destructor
        NmpcCourse::~NmpcCourse() { 
            // std::cout << "My class is destroyed here. :(" << std::endl; 
        }
    }
}
