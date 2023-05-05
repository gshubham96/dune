//***************************************************************************
// Copyright 2007-2022 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: gshubham96                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
using DUNE_NAMESPACES;

namespace Simulators
{
  //! Vehicle SIMulator for DUNE.
  //! %VSIM is responsible for multiple vehicle simulation.
  //! In the present, it is able to simulate
  //! Unmanned Underwater Vehicles
  //! and Autonomous Surface Vehicles.
  //!
  //! @author Bruno Terra
  //! @author José Braga
  namespace VSIMCasadi
  {
    //! %Task arguments.
    struct Arguments
    {
      //! Entity label of the stream velocity source.
      std::string svlabel, cvlabel, wvlabel;
      //! Simulation time multiplier
      double time_multiplier;
      //! time step
      double Ts;
      //! model type
      std::string model_type;
    };

    //! Simulator task.
    struct Task: public Tasks::Periodic
    {
      //! Simulation vehicle.
      NmpcDynamics m_sim;
      //! Simulated position (X,Y,Z).
      IMC::SimulatedState m_state;
      //! Environment Forces
      //! Task arguments.
      Arguments m_args;
      //! Stream velocity.
      double m_svel[3];

      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx)
      {
        param("Time Multiplier", m_args.time_multiplier)
        .defaultValue("1.0")
        .description("Simulation time multiplier");

        param("Entity Label - Stream Velocity Source", m_args.svlabel)
            .defaultValue("Stream Velocity Simulator")
            .description("Entity label of the stream velocity source.");

        // Register handler routines.
        bind<IMC::GpsFix>(this);
        bind<IMC::ServoPosition>(this);
        bind<IMC::SetThrusterActuation>(this);
        bind<IMC::EstimatedStreamVelocity>(this);
      }

      void
      onUpdateParameters(void)
      {
      }

      //! Release allocated resources.
      void
      onResourceRelease(void)
      {
      }

      //! Initialize resources and add vehicle to the world.
      void
      onResourceInitialization(void)
      {
      }

      void
      consume(const IMC::GpsFix* msg)
      {
      }

      void
      consume(const IMC::ServoPosition* msg)
      {
      }

      void
      consume(const IMC::SetThrusterActuation* msg)
      {
      }

      void
      consume(const IMC::EstimatedStreamVelocity* msg)
      {
      }

      void
      task(void)
      {

      }
    };
  }
}

DUNE_TASK
