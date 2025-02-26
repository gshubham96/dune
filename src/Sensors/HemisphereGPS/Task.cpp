//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <sstream>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Reader.hpp"

namespace Sensors
{
  //! Device driver for NMEA capable %GPS devices.
  namespace HemisphereGPS
  {
    using DUNE_NAMESPACES;

    //! Maximum number of initialization commands.
    static const unsigned c_max_init_cmds = 14;
    //! Timeout for waitReply() function.
    static const float c_wait_reply_tout = 4.0;
    //! Minimum number of fields of PUBX,00 sentence.
    static const unsigned c_pubx00_fields = 21;
    //! Minimum number of fields of GGA sentence.
    static const unsigned c_gga_fields = 15;
    //! Minimum number of fields of VTG sentence.
    static const unsigned c_vtg_fields = 9;
    //! Minimum number of fields of ZDA sentence.
    static const unsigned c_zda_fields = 7;
    //! Minimum number of fields of HDT sentence.
    static const unsigned c_hdt_fields = 3;
    //! Minimum number of fields of HDM sentence.
    static const unsigned c_hdm_fields = 3;
    //! Minimum number of fields of ROT sentence.
    static const unsigned c_rot_fields = 3;
    //! Minimum number of fields of ROT sentence.
    static const unsigned c_hev_fields = 2;
    //! Minimum number of fields of PSATHPR sentence.
    static const unsigned c_psathpr_fields = 7;
    //! Power on delay.
    static const double c_pwr_on_delay = 5.0;
    //! Command reply timeout.
    static const float c_timeout = 3.0f;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Order of sentences.
      std::vector<std::string> stn_order;
      //! Input timeout in seconds.
      float inp_tout;
      //! Output timeout in seconds.
      float out_tout;
      //! Initialization commands.
      std::string init_cmds[c_max_init_cmds];
      //! Initialization replies.
      std::string init_rpls[c_max_init_cmds];
      //! Power channels.
      std::vector<std::string> pwr_channels;
      //! Rate of Turn time constant for v104s GPS compass.
      double hrtau;
      //! Course time constant for v104s GPS compass.
      double cogtau;
      //! Heading time constant for v104s GPS compass.
      double htau;
    };

    struct Task: public Tasks::Task
    {
      //! Serial port handle.
      IO::Handle* m_handle;
      //! GPS Fix message.
      IMC::GpsFix m_fix;
      //! Euler angles message.
      IMC::EulerAngles m_euler;
      //! Angular velocity message.
      IMC::AngularVelocity m_agvel;
      //! Heave message.
      IMC::Heave m_heave;
      //! Task arguments.
      Arguments m_args;
      //! Input watchdog.
      Time::Counter<float> m_wdog;
      //! Output watchdog.
      Time::Counter<float> m_out_timer;
      //! Sent new config to GPS.
      bool m_cmd_sent;
      //! True if we have heave.
      bool m_has_heave;
      //! True if we have angular velocity.
      bool m_has_agvel;
      //! True if we have euler angles.
      bool m_has_euler;
      //! Last initialization line read.
      std::string m_init_line;
      //! Reader thread.
      Reader* m_reader;
      //! Buffer forEntityState
      char m_bufer_entity[64];

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_handle(NULL),
        m_cmd_sent(false),
        m_has_heave(false),
        m_has_agvel(false),
        m_has_euler(false),
        m_reader(NULL)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("4800")
        .description("Serial port baud rate");

        param("Input Timeout", m_args.inp_tout)
        .units(Units::Second)
        .defaultValue("4.0")
        .minimumValue("0.0")
        .description("Input timeout");

        param("Output Timeout", m_args.out_tout)
        .units(Units::Second)
        .defaultValue("5.0")
        .minimumValue("1.0")
        .description("Timeout before a configuration is set.");

        param("Power Channel - Names", m_args.pwr_channels)
        .defaultValue("")
        .description("Device's power channels");

        param("Sentence Order", m_args.stn_order)
        .defaultValue("")
        .description("Sentence order");

        param("ROT Time Constant", m_args.hrtau)
        .defaultValue("2.0")
        .description("Rate of Turn time constant for v104s GPS compass, GPROT message.");

        param("COG Time Constant", m_args.cogtau)
        .defaultValue("0.0")
        .description("Course over ground time constant for v104s GPS compass, GPVTG.");

        param("Heading Time Constant", m_args.htau)
        .defaultValue("20.0")
        .description("Heading time constant for v104s GPS compass, GPVTG.");

        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          std::string cmd_label = String::str("Initialization String %u - Command", i);
          param(cmd_label, m_args.init_cmds[i])
          .defaultValue("");

          std::string rpl_label = String::str("Initialization String %u - Reply", i);
          param(rpl_label, m_args.init_rpls[i])
          .defaultValue("");
        }

        // Initialize messages.
        clearMessages();

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
      }

      void
      onUpdateParameters(void)
      {
        if(paramChanged(m_args.hrtau))
          sendCommand("ROT");
        if(paramChanged(m_args.cogtau))
          sendCommand("COG");
        if(paramChanged(m_args.htau))
          sendCommand("H");
      }

      void
      onResourceAcquisition(void)
      {
        if (m_args.pwr_channels.size() > 0)
        {
          IMC::PowerChannelControl pcc;
          pcc.op = IMC::PowerChannelControl::PCC_OP_TURN_ON;
          for (size_t i = 0; i < m_args.pwr_channels.size(); ++i)
          {
            pcc.name = m_args.pwr_channels[i];
            dispatch(pcc);
          }
        }

        Counter<double> timer(c_pwr_on_delay);
        while (!stopping() && !timer.overflow())
          waitForMessages(timer.getRemaining());

        try
        {
          if (!openSocket())
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);

          m_reader = new Reader(this, m_handle);
          m_reader->start();
        }
        catch (...)
        {
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }
      }

      bool
      openSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.uart_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_handle = sock;
        return true;
      }

      void
      onResourceRelease(void)
      {
        if (m_reader != NULL)
        {
          m_reader->stopAndJoin();
          delete m_reader;
          m_reader = NULL;
        }

        Memory::clear(m_handle);
      }

      void
      onResourceInitialization(void)
      {
        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          if (m_args.init_cmds[i].empty())
            continue;

          std::string cmd = String::unescape(m_args.init_cmds[i]);
          m_handle->writeString(cmd.c_str());

          if (!m_args.init_rpls[i].empty())
          {
            std::string rpl = String::unescape(m_args.init_rpls[i]);
            if (!waitInitReply(rpl))
            {
              err("%s: %s", DTR("no reply to command"), m_args.init_cmds[i].c_str());
              throw std::runtime_error(DTR("failed to setup device"));
            }
          }
        }

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        m_wdog.setTop(m_args.inp_tout);
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        spew("%s", sanitize(msg->value).c_str());

        if (getEntityState() == IMC::EntityState::ESTA_BOOT)
          m_init_line = msg->value;
        else
          processSentence(msg->value);
      }

      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      //! Send command to device.
      //! @param[in] cmd command to send.
      //! @param[in] ack wait for ack.
      //! @return true if command succeeded, false otherwise.
      void
      sendCommand(const std::string& cmd)
      {
        if (m_reader == NULL)
          return;

        // Create an output string stream.
        std::ostringstream stream;
        // Set Fixed -Point Notation.
        stream << std::fixed;
        // Set precision to 1 digit.
        stream << std::setprecision(1);
        
        if(cmd.compare("ROT") == 0)
        {
          //Add double to stream
          stream << m_args.hrtau;
          // Get string from output string stream
          std::string to_send = stream.str();
          std::string bfr("$JATT,HRTAU," + to_send + "\r\n");
          m_handle->writeString(bfr.c_str());
          trace("sent: '%s'", sanitize(bfr).c_str());
        } else if(cmd.compare("COG") == 0)
        {
          //Add double to stream
          stream << m_args.cogtau;
          // Get string from output string stream
          std::string to_send = stream.str();
          std::string bfr("$JATT,COGTAU," + to_send + "\r\n");
          m_handle->writeString(bfr.c_str());
          trace("sent: '%s'", sanitize(bfr).c_str());
        } else if(cmd.compare("H") == 0)
        {
          //Add double to stream
          stream << m_args.htau;
          // Get string from output string stream
          std::string to_send = stream.str();
          std::string bfr("$JATT,HTAU," + to_send + "\r\n");
          m_handle->writeString(bfr.c_str());
          trace("sent: '%s'", sanitize(bfr).c_str());
        }

        m_cmd_sent = true;
        m_out_timer.setTop(m_args.out_tout);
      }

      //! Read input and checks if a given sequence is received.
      int
      checkConfirmation(std::string line)
      {
        std::string sub = line.substr(line.length() - 4,2);
        std::string sub_ = line.substr(0,line.size() - 2);
        if(line.at(1) == '>' && sub.compare("OK") == 0)
          return 1;
        else if(line.at(1) == '>' && sub_.compare("$> Save Complete") == 0)
          return 2;
        else
          return 3;
      }

      void
      clearMessages(void)
      {
        m_euler.clear();
        m_agvel.clear();
        m_fix.clear();
      }

      //! Wait reply to initialization command.
      //! @param[in] stn string to compare.
      //! @return true on successful match, false otherwise.
      bool
      waitInitReply(const std::string& stn)
      {
        Counter<float> counter(c_wait_reply_tout);
        while (!stopping() && !counter.overflow())
        {
          waitForMessages(counter.getRemaining());
          if (m_init_line == stn)
          {
            m_init_line.clear();
            return true;
          }
        }

        return false;
      }

      //! Read time from string.
      //! @param[in] str string.
      //! @param[out] dst time.
      //! @return true if successful, false otherwise.
      bool
      readTime(const std::string& str, float& dst)
      {
        unsigned h = 0;
        unsigned m = 0;
        unsigned s = 0;
        double sfp = 0;

        if (std::sscanf(str.c_str(), "%02u%02u%lf", &h, &m, &sfp) != 3)
        {
          if (std::sscanf(str.c_str(), "%02u%02u%02u", &h, &m, &s) != 3)
            return false;
        }

        dst = (h * 3600) + (m * 60) + s + sfp;

        return true;
      }

      //! Read latitude from string.
      //! @param[in] str input string.
      //! @param[in] h either North (N) or South (S).
      //! @param[out] dst latitude.
      //! @return true if successful, false otherwise.
      bool
      readLatitude(const std::string& str, const std::string& h, double& dst)
      {
        int degrees = 0;
        double minutes = 0;

        if (std::sscanf(str.c_str(), "%02d%lf", &degrees, &minutes) != 2)
          return false;

        dst = Angles::convertDMSToDecimal(degrees, minutes);

        if (h == "S")
          dst = -dst;

        return true;
      }

      //! Read longitude from string.
      //! @param[in] str input string.
      //! @param[in] h either West (W) or East (E).
      //! @param[out] dst longitude.
      //! @return true if successful, false otherwise.
      double
      readLongitude(const std::string& str, const std::string& h, double& dst)
      {
        int degrees = 0;
        double minutes = 0;

        if (std::sscanf(str.c_str(), "%03d%lf", &degrees, &minutes) != 2)
          return false;

        dst = Angles::convertDMSToDecimal(degrees, minutes);

        if (h == "W")
          dst = -dst;

        return true;
      }

      //! Read decimal from input string.
      //! @param[in] str input string.
      //! @param[out] dst decimal.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readDecimal(const std::string& str, T& dst)
      {
        unsigned idx = 0;
        while (str[idx] == '0')
          ++idx;

        return castLexical(std::string(str, idx), dst);
      }

      //! Read number from input string.
      //! @param[in] str input string.
      //! @param[out] dst number.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readNumber(const std::string& str, T& dst)
      {
        return castLexical(str, dst);
      }

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
        bool discard_current = false;

        // Discard leading noise.
        size_t sidx = 0;
        for (sidx = 0; sidx < line.size(); ++sidx)
        {
          if (line[sidx] == '$')
            break;
        }

        if(m_cmd_sent)
        {
          int confirmed = checkConfirmation(line);
          if(confirmed==1)
          {
            trace("'%s' received from GPS, configuration set!", line.c_str());
            // Configuration was applied, now can save.
            std::string endline = "\r\n";
            std::string save_str = "$JSAVE" + endline;
            m_handle->writeString(save_str.c_str());
            trace("sent: '%s'", sanitize(save_str).c_str());

            discard_current = true;
          } else if(confirmed==2)
          {
            m_out_timer.reset();
            m_cmd_sent = false;
            trace("'%s' received from GPS, configuration saved!", line.c_str());
            discard_current = true;
          }
        }

        if(!discard_current)
        {
          // Discard trailing noise.
          size_t eidx = 0;
          for (eidx = line.size() - 1; eidx > sidx; --eidx)
          {
            if (line[eidx] == '*')
              break;
          }

          if (sidx >= eidx)
            return;

          // Compute checksum.
          uint8_t ccsum = 0;
          for (size_t i = sidx + 1; i < eidx; ++i)
            ccsum ^= line[i];

          // Validate checksum.
          unsigned rcsum = 0;
          if (std::sscanf(&line[0] + eidx + 1, "%02X", &rcsum) != 1)
            return;

          // Split sentence
          std::vector<std::string> parts;
          String::split(line.substr(sidx + 1, eidx - sidx - 1), ",", parts);

          if (std::find(m_args.stn_order.begin(), m_args.stn_order.end(), parts[0]) != m_args.stn_order.end())
            interpretSentence(parts);
        }
      }

      //! Interpret given sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretSentence(std::vector<std::string>& parts)
      {
        if (parts[0] == m_args.stn_order.front())
        {
          clearMessages();
          m_fix.setTimeStamp();
          m_euler.setTimeStamp(m_fix.getTimeStamp());
          m_agvel.setTimeStamp(m_fix.getTimeStamp());
          m_heave.setTimeStamp(m_fix.getTimeStamp());
        }

        if (hasNMEAMessageCode(parts[0], "ZDA"))
        {
          interpretZDA(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "GGA"))
        {
          interpretGGA(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "VTG"))
        {
          interpretVTG(parts);
        }
        else if (parts[0] == "PSAT")
        {
          if (parts[1] == "HPR")
            interpretPSATHPR(parts);
        }
        else if (parts[0] == "PUBX")
        {
          if (parts[1] == "00")
            interpretPUBX00(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HDM"))
        {
          interpretHDM(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HDT"))
        {
          interpretHDT(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "ROT"))
        {
          interpretROT(parts);
        }
        else if (hasNMEAMessageCode(parts[0], "HEV"))
        {
          interpretHEV(parts);
        }

        if (parts[0] == m_args.stn_order.back())
        {
          debug("Dispatching fix");
          m_wdog.reset();
          m_fix.setDestinationEntity(resolveEntity("Navigation Manager"));
          dispatch(m_fix);

          if (m_has_euler)
          {
            dispatch(m_euler);
            m_has_euler = false;
          }
          
          if (m_has_heave)
          {
            dispatch(m_heave);
            m_has_heave = false;
          }

          if (m_has_agvel)
          {
            dispatch(m_agvel);
            m_has_agvel = false;
          }

          std::memset(&m_bufer_entity, '\0', sizeof(m_bufer_entity));
          if (m_fix.validity & IMC::GpsFix::GFV_VALID_POS)
          {
            std::sprintf(m_bufer_entity, "active - hdop: %.2f , Sat: %d", m_fix.hdop, m_fix.satellites);
            setEntityState(IMC::EntityState::ESTA_NORMAL, Utils::String::str(DTR(m_bufer_entity)));
          }
          else
          {
            std::sprintf(m_bufer_entity, "wait gps fix - hdop: %.2f , Sat: %d", m_fix.hdop, m_fix.satellites);
            setEntityState(IMC::EntityState::ESTA_NORMAL, Utils::String::str(DTR(m_bufer_entity)));
          }
        }
      }

      bool
      hasNMEAMessageCode(const std::string& str, const std::string& code)
      {
        return (String::startsWith(str, "G") || String::startsWith(str, "H")) && String::endsWith(str, code);
      }

      //! Interpret ZDA sentence (UTC date and time).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretZDA(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_zda_fields)
        {
          war(DTR("invalid ZDA sentence"));
          return;
        }

        // Read time.
        if (readTime(parts[1], m_fix.utc_time))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_TIME;

        // Read date.
        if (readDecimal(parts[2], m_fix.utc_day)
            && readDecimal(parts[3], m_fix.utc_month)
            && readDecimal(parts[4], m_fix.utc_year))
        {
          m_fix.validity |= IMC::GpsFix::GFV_VALID_DATE;
        }
      }

      //! Interpret GGA sentence (GPS fix data).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretGGA(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_gga_fields)
        {
          war(DTR("invalid GGA sentence"));
          return;
        }

        int quality = 0;
        readDecimal(parts[6], quality);
        if (quality == 1)
        {
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else if (quality == 2)
        {
          m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }

        if (readLatitude(parts[2], parts[3], m_fix.lat)
            && readLongitude(parts[4], parts[5], m_fix.lon)
            && readNumber(parts[9], m_fix.height)
            && readDecimal(parts[7], m_fix.satellites))
        {
          // Convert altitude above sea level to altitude above ellipsoid.
          double geoid_sep = 0;
          if (readNumber(parts[11], geoid_sep))
            m_fix.height += geoid_sep;

          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
          debug("lat: %f, lon: %f",m_fix.lat,m_fix.lon);
        }
        else
        {
          m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
        }

        if (readNumber(parts[8], m_fix.hdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HDOP;
      }

      //! Interpret PUBX00 sentence (navstar position).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPUBX00(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pubx00_fields)
        {
          war(DTR("invalid PUBX,00 sentence"));
          return;
        }

        if (parts[8] == "G3" || parts[8] == "G2")
        {
          m_fix.type = IMC::GpsFix::GFT_STANDALONE;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else if (parts[8] == "D3" || parts[8] == "D2")
        {
          m_fix.type = IMC::GpsFix::GFT_DIFFERENTIAL;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }

        if (readLatitude(parts[3], parts[4], m_fix.lat)
            && readLongitude(parts[5], parts[6], m_fix.lon)
            && readNumber(parts[7], m_fix.height)
            && readDecimal(parts[18], m_fix.satellites))
        {
          // Convert coordinates to radians.
          m_fix.lat = Angles::radians(m_fix.lat);
          m_fix.lon = Angles::radians(m_fix.lon);
          m_fix.validity |= IMC::GpsFix::GFV_VALID_POS;
        }
        else
        {
          m_fix.validity &= ~IMC::GpsFix::GFV_VALID_POS;
        }

        if (readNumber(parts[9], m_fix.hacc))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HACC;

        if (readNumber(parts[10], m_fix.vacc))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_VACC;

        if (readNumber(parts[15], m_fix.hdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_HDOP;

        if (readNumber(parts[16], m_fix.vdop))
          m_fix.validity |= IMC::GpsFix::GFV_VALID_VDOP;
      }

      //! Interpret VTG sentence (course over ground).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretVTG(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_vtg_fields)
        {
          war(DTR("invalid VTG sentence"));
          return;
        }

        if (readNumber(parts[1], m_fix.cog))
        {
          m_fix.cog = Angles::normalizeRadian(Angles::radians(m_fix.cog));
          m_fix.validity |= IMC::GpsFix::GFV_VALID_COG;
        }

        if (readNumber(parts[7], m_fix.sog))
        {
          m_fix.sog *= 1000.0f / 3600.0f;
          m_fix.validity |= IMC::GpsFix::GFV_VALID_SOG;
        }
      }

      //! Interpret HDT sentence (true heading).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHDT(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hdt_fields)
        {
          war(DTR("invalid HDT sentence"));
          return;
        }

        if (readNumber(parts[1], m_euler.psi))
          m_euler.psi = Angles::normalizeRadian(Angles::radians(m_euler.psi));
      }

      //! Interpret HDM sentence (Magnetic heading of
      //! the vessel derived from the true heading calculated).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHDM(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hdm_fields)
        {
          war(DTR("invalid HDM sentence"));
          return;
        }

        if (readNumber(parts[1], m_euler.psi_magnetic))
        {
          m_euler.psi_magnetic = Angles::normalizeRadian(Angles::radians(m_euler.psi_magnetic));
          m_has_euler = true;
        }
      }

      //! Interpret ROT sentence (rate of turn).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretROT(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_rot_fields)
        {
          war(DTR("invalid ROT sentence"));
          return;
        }

        if (readNumber(parts[1], m_agvel.z))
        {
          m_agvel.z = Angles::radians(m_agvel.z) / 60.0;
          m_has_agvel = true;
        }
      }

      //! Interpret HEV sentence (rate of turn).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretHEV(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_hev_fields)
        {
          war(DTR("invalid HEV sentence"));
          return;
        }

        if (readNumber(parts[1], m_heave.value))
        {
          m_has_heave = true;
        }
      }

      //! Interpret PSATHPR sentence (Proprietary NMEA message that
      //! provides the heading, pitch, roll, and time in a single message).
      //! @param[in] parts vector of strings from sentence.
      void
      interpretPSATHPR(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_psathpr_fields)
        {
          war(DTR("invalid PSATHPR sentence"));
          return;
        }

        if (readNumber(parts[4], m_euler.theta))
        {
          m_euler.theta = Angles::normalizeRadian(Angles::radians(m_euler.theta));
          m_has_euler = true;
        }

        if (readNumber(parts[5], m_euler.phi))
        {
          m_euler.phi = Angles::normalizeRadian(Angles::radians(m_euler.phi));
          m_has_euler = true;
        }
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);

          if (m_wdog.overflow()) //|| m_out_timer.overflow()
          {
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
            throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
          }
        }
      }
    };
  }
}

DUNE_TASK
