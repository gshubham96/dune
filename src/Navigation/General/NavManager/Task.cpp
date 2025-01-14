//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Alberto Dallolio                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Navigation
{
  namespace General
  {
    //! Navigation Manager.
    //!
    //! @author Alberto Dallolio
    namespace NavManager
    {
      struct Arguments
      {
        //! GPS entity label.
        std::string elabel_gps;
        //! IMU entity label.
        std::string elabel_imu;
        //! Convert height to geoid height (MSL)
        bool convert_msl;
        //! Main unit.
        std::string main_unit;
        //! Secondary unit.
        std::string secondary_unit;
        //! Third unit.
        std::string third_unit;
        //! Number of seconds without data before reporting an error.
        double inp_tout;
      };

      struct Task: public DUNE::Tasks::Task
      {
        //! GPS entity eid.
        int m_gps_eid;
        //! Height offset.
        float m_offset;
        //! Offset flag.
        bool m_offset_flag;
        //! Estimated state.
        IMC::EstimatedState m_estate;
        //! Main GPS entity eid.
        int m_main_unit_eid;
        //! Secondary GPS entity eid.
        int m_secondary_unit_eid;
        //! Third GPS entity eid.
        int m_third_unit_eid;
        //! Time without main GPS.
        Counter<double> m_time_without_main_unit;
        //! Time without secondary GPS.
        Counter<double> m_time_without_secondary_unit;
        //! Time without third GPS.
        Counter<double> m_time_without_third_unit;
        //! GPS fix rejection.
        IMC::GpsFixRejection m_main_rej, m_secondary_rej, m_third_rej;
        //! True if preferred GPS is not working.
        bool main_unit_lost;
        //! True if secondary GPS is not working.
        bool secondary_unit_lost;
        //! True if third GPS is not working.
        bool third_unit_lost;
        //! Iridium warning for main GPS.
        bool main_unit_ir_sent;
        //! Iridium warning for secondary GPS.
        bool secondary_unit_ir_sent;
        //! Iridium warning for third GPS.
        bool third_unit_ir_sent;
        //! Transmission request id
        int m_reqid;
        //! Fix validity.
        bool m_main_valid, m_secondary_valid, m_third_valid;
        //! Main system GpsFix message.
        IMC::GpsFix m_sys_fix;
        //! Task arguments.
        Arguments m_args;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          main_unit_lost(false),
          secondary_unit_lost(false),
          third_unit_lost(false),
          main_unit_ir_sent(false),
          secondary_unit_ir_sent(false),
          third_unit_ir_sent(false),
          m_reqid(0),
          m_main_valid(true),
          m_secondary_valid(true),
          m_third_valid(true)
        {
          // Define configuration parameters.
          param("Entity Label - GPS", m_args.elabel_gps)
          .description("Entity label of 'GpsFix' and 'GroundVelocity' messages");

          param("Entity Label - IMU", m_args.elabel_imu)
          .description("Entity label of 'EulerAngles' and 'AngularVelocity' messages");

          param("Convert Height to Geoid Height", m_args.convert_msl)
          .defaultValue("false")
          .description("Convert WGS84 height to geoid height (mean sea level) height");
          
          param("Main unit", m_args.main_unit)
          .description("Name of preferred navigation unit.");

          param("Secondary unit", m_args.secondary_unit)
          .description("Name of secondary preferred navigation unit.");

          param("Third unit", m_args.third_unit)
          .description("Name of third preferred navigation unit.");

          param("Input timeout", m_args.inp_tout)
          .units(Units::Second)
          .defaultValue("30.0")
          .minimumValue("0.0")
          .description("Input timeout before error is thrown.");

          m_estate.clear();
          m_offset = 0.0f;
          m_offset_flag = false;

          // Register callbacks
          bind<IMC::AngularVelocity>(this);
          bind<IMC::EulerAngles>(this);
          bind<IMC::GpsFix>(this);
        }

        void
        onUpdateParameters(void)
        {
          if(!m_args.convert_msl)
          {
            m_offset = 0.0f;
            m_offset_flag = false;
          }
        }

        void
        onEntityResolution(void)
        {
          try
          {
            m_gps_eid = resolveEntity(m_args.elabel_gps);
          }
          catch (...)
          {
            m_gps_eid = 0;
          }
        }

        void
        onResourceAcquisition(void)
        {
          // Navigation enters error mode without valid GPS data.
          m_time_without_main_unit.setTop(m_args.inp_tout);
          m_time_without_secondary_unit.setTop(m_args.inp_tout);
          m_time_without_third_unit.setTop(m_args.inp_tout);
        }

        void
        consume(const IMC::AngularVelocity* msg)
        {
          if(resolveEntity(msg->getSourceEntity()).c_str() == m_args.elabel_imu)
          {
            m_estate.p = msg->x;
            m_estate.q = msg->y;
            m_estate.r = msg->z;
          }
        }

        void
        consume(const IMC::EulerAngles* msg)
        {
          if (msg->getSourceEntity() == m_gps_eid)
          {
            //debug("IMC::EulerAngles from %s",resolveEntity(msg->getSourceEntity()).c_str());
            m_estate.phi = msg->phi;
            m_estate.theta = msg->theta;
            m_estate.psi = msg->psi;
          }
        }

        void
        consume(const IMC::GpsFix* msg)
        {
          debug("GPS FIX FROM: %s",resolveEntity(msg->getSourceEntity()).c_str());
          if(m_args.main_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0)
          {
            m_time_without_main_unit.reset();
            main_unit_lost = false;

            if((msg->validity & IMC::GpsFix::GFV_VALID_POS) == 0)
            {
              debug("Got INVALID fix from main unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_main_valid = false;
              m_main_rej.reason = IMC::GpsFixRejection::RR_INVALID;
              dispatch(m_main_rej, DF_KEEP_TIME);
            }
            else
            {
              trace("Got VALID fix from main unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_main_valid = true;
            }
          } else if(m_args.secondary_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0)
          {
            m_time_without_secondary_unit.reset();
            secondary_unit_lost = false;

            if((msg->validity & IMC::GpsFix::GFV_VALID_POS) == 0)
            {
              debug("Got INVALID fix from secondary unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_secondary_valid = false;
              m_secondary_rej.reason = IMC::GpsFixRejection::RR_INVALID;
              dispatch(m_secondary_rej, DF_KEEP_TIME);
            }
            else
            {
              trace("Got VALID fix from secondary unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_secondary_valid = true;
            }
          } else if(m_args.third_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0)
          {
            m_time_without_third_unit.reset();
            third_unit_lost = false;

            if((msg->validity & IMC::GpsFix::GFV_VALID_POS) == 0)
            {
              debug("Got INVALID fix from third unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_third_valid = false;
              m_third_rej.reason = IMC::GpsFixRejection::RR_INVALID;
              dispatch(m_third_rej, DF_KEEP_TIME);
            }
            else
            {
              trace("Got VALID fix from third unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
              m_third_valid = true;
            }
          }

          //! Logic for unit choice.
          if(m_args.main_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0 && m_main_valid)
          {
            debug("Using GpsFix from main unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
            m_sys_fix = *msg;
            sendFix();
            return;
          }

          if(m_args.secondary_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0 && m_secondary_valid && (main_unit_lost || !m_main_valid)) //m_time_without_main_unit.overflow()
          {
            debug("Using GpsFix from secondary unit: %s",resolveEntity(msg->getSourceEntity()).c_str());            
            m_sys_fix = *msg;
            sendFix();
            return;
          }

          if(m_args.third_unit.compare(resolveEntity(msg->getSourceEntity()).c_str())==0 && m_third_valid && (main_unit_lost || !m_main_valid) && (secondary_unit_lost || !m_secondary_valid))
          {
            debug("Using GpsFix from third unit: %s",resolveEntity(msg->getSourceEntity()).c_str());
            m_sys_fix = *msg;
            sendFix();
          }
        }

        void
        sendFix()
        {
          if((m_sys_fix.validity & IMC::GpsFix::GFV_VALID_POS) == 0)
          {
            IMC::GpsFixRejection m_sys_fix_rej;
            m_sys_fix_rej.reason = IMC::GpsFixRejection::RR_INVALID;
            dispatch(m_sys_fix_rej, DF_KEEP_TIME);
            return;
          }
          // Received valid GPS data.
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          // Dispatch IMC::GpsFix.
          m_sys_fix.setSourceEntity(getEid("Navigation Manager"));
          dispatch(m_sys_fix);

          // Fill out IMC::EstimatedState as well.
          m_estate.lat = m_sys_fix.lat;
          m_estate.lon = m_sys_fix.lon;

          if(m_args.convert_msl && !m_offset_flag)
          {
            m_offset_flag = true;
            Coordinates::WMM wmm(m_ctx.dir_cfg);
            m_offset = wmm.height(m_estate.lat, m_estate.lon);
          }

          m_estate.height = m_sys_fix.height - m_offset;

          // Decompose velocity vector.
          m_estate.vx = std::cos(m_sys_fix.cog) * m_sys_fix.sog;
          m_estate.vy = std::sin(m_sys_fix.cog) * m_sys_fix.sog;
          //m_estate.u = msg->sog;
          m_estate.u = m_estate.vx*std::cos(m_estate.psi) + m_estate.vy*std::sin(m_estate.psi);
          m_estate.v = -m_estate.vx*std::sin(m_estate.psi) + m_estate.vy*std::cos(m_estate.psi);
          dispatch(m_estate);
        }

        unsigned
        getEid(std::string label)
        {
          unsigned eid = 0;
          try
          {
            eid = resolveEntity(label);
          }
          catch (Entities::EntityDataBase::NonexistentLabel& e)
          {
            (void)e;
            eid = reserveEntity(label);
          }
          return eid;
        }

        //! Helper function to get a certain token from a string  
        std::string getToken(std::string toParse, std::string delim, int n_token = 1)
        {
          std::string token;
          for (int i = 0; i<n_token; i++)
          {
              token = toParse.substr(0, toParse.find(delim));
              toParse.erase(0, toParse.find(delim) + delim.length());
          }
          return token;
        }

        void
        sendIridium(std::string msg)
        {
          IMC::TransmissionRequest req;
          req.setDestination(m_ctx.resolver.id());
          req.data_mode = TransmissionRequest::DMODE_TEXT;
          req.txt_data = msg;
          req.deadline = Clock::getSinceEpoch() + 60;
          req.req_id = ++m_reqid;

          req.comm_mean = TransmissionRequest::CMEAN_SATELLITE;
          req.destination = "";
          inf("Sending via Iridium: '%s'", req.txt_data.c_str());
          dispatch(req);
        }

        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);

            if(m_time_without_main_unit.overflow() && !main_unit_lost)
            {
              debug("Main Gps disappeared");
              main_unit_lost = true;
            }

            if(m_time_without_secondary_unit.overflow() && !secondary_unit_lost)
            {
              debug("Secondary Gps disappeared");
              secondary_unit_lost = true;
            }

            if(m_time_without_third_unit.overflow() && !third_unit_lost)
            {
              debug("Third Gps disappeared");
              third_unit_lost = true;
            }

            // Raise ERROR MODE if all GPSs are out of service.
            if((m_time_without_main_unit.overflow() && m_time_without_secondary_unit.overflow() && m_time_without_main_unit.overflow()) || (!m_main_valid && !m_secondary_valid && !m_third_valid))
              setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_WAIT_GPS_FIX);
            else if((main_unit_lost || !m_main_valid) && !main_unit_ir_sent)
            {
              debug("Main GPS not working!");
              
              // Send iridium message.
              sendIridium("Main GPS not working!");
              main_unit_ir_sent = true;
              
            } else if((secondary_unit_lost || !m_secondary_valid) && !secondary_unit_ir_sent)
            {
              debug("Secondary GPS not working!");

              // Send iridium message.
              sendIridium("Secondary GPS not working!");
              secondary_unit_ir_sent = true;

            } else if((third_unit_lost || !m_third_valid) && !third_unit_ir_sent)
            {
              debug("Third GPS not working!");

              // Send iridium message.
              sendIridium("Third GPS not working!");
              third_unit_ir_sent = true;
            }
          }
        }
      };
    }
  }
}

DUNE_TASK
