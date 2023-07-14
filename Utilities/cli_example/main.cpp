/**
  ******************************************************************************
  * @file    main.c
  * @author  SRA
  * @brief   source code of command line example of High Speed Datalog
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */
#include "main.h"

#include <unordered_map>
#include <iomanip>
#include <sstream>
#include "json.hpp"

using namespace std;

int main(int argc, char *argv[])
{

    unsigned int sID;
    unsigned int ssID;

    unsigned int mlc_sID = (unsigned int)(-1);
    unsigned int mlc_ssID = (unsigned int)(-1);

    /* ---------------------------- Parse command line options ---------------------------- */

    InputParser input(argc, argv);

    if(input.cmdOptionExists("-h"))
    {
        cout << "Welcome to HSDatalog Command Line Interface example" << endl << endl;
        cout << "Usage: " << endl << " cli_example [-COMMAND [ARGS]]" << endl << endl;
        cout << "Commands:" << endl << endl;
        cout << " -h\t\t: Show this help" << endl;
        cout << " -f <filename>\t: Device Configuration file (JSON)" << endl;
        cout << " -u <filename>\t: UCF Configuration file for MLC" << endl;
        cout << " -t <seconds>\t: Duration of the current acquisition (seconds) " << endl;
        cout << " -g\t\t: Get current Device Configuration, save it to file <DeviceConfig.json> and return." << endl;
        cout << " -y\t\t: Directly start the acquisition without waiting for user confirmation" << endl;
        cout << "   \t\t  All other parameters are ignored! " << endl;
        cout << endl;

        exit(0);
    }

    if(input.cmdOptionExists("-g"))
    {
        if(hs_datalog_open() != ST_HS_DATALOG_OK)
        {
            cout << "Error occurred while initializing datalog\n";
            cout << "Press any key to exit \n";
            getchar();
            return -1;
        }

        int nDevices = 0;
        if(hs_datalog_get_device_number(&nDevices) != ST_HS_DATALOG_OK)
        {
            cout << "Error occurred while retrieving device number\n";
            cout << "Press any key to exit \n";
            getchar();
            return -1;

        }

        if(nDevices == 0)
        {
            cout << "No devices, exiting\n";
            cout << "Press any key to exit \n";
            getchar();
            return -1;
        }

        char *tmp1;
        if(hs_datalog_get_device(0, &tmp1) != ST_HS_DATALOG_OK)
        {
            cout << "Error occurred while retrieving device configuration\n";
            cout << "Press any key to exit \n";
            getchar();
            return -1;
        }

        auto json = nlohmann::json::parse(tmp1);

        ofstream currentConfigFile;
        currentConfigFile.open("DeviceConfig.json", ios::out|ios::binary);
        currentConfigFile << json.dump(1);
        currentConfigFile.close();

        cout << "Current Device Configuration has been saved.\n";
        hs_datalog_free(tmp1);
        exit(0);
    }


    const std::string &fileNameParam = input.getCmdOption("-f");
    ifstream configFile;

    if(input.cmdOptionExists("-f"))
    {
        if (!fileNameParam.empty())
        {
            string configFileName = fileNameParam;

            configFile.open(configFileName, ios::in|ios::binary);

            if (!configFile.is_open()) // Needed in linux in case of local file???
            {
                configFileName ="./" + fileNameParam;
                configFile.open(configFileName, ios::in|ios::binary);
            }
            if (!configFile.is_open())
            {
                cout << "Device configuration file not found" << endl << endl;
                exit(1);
            }
        }
        else
        {
            cout << "Please specify a file name after -f command" << endl << endl;
            exit(1);
        }
    }

    const std::string &fileUCFParam = input.getCmdOption("-u");
    ifstream ucfFile;
    if(input.cmdOptionExists("-u"))
    {
        if (!fileUCFParam.empty())
        {
            string ucfFileName = fileUCFParam;

            ucfFile.open(ucfFileName, ios::in|ios::binary);

            if (!ucfFile.is_open()) // Needed in linux in case of local file???
            {
                ucfFileName ="./" + fileUCFParam;
                ucfFile.open(ucfFileName, ios::in|ios::binary);
            }
            if (!ucfFile.is_open())
            {
                cout << "UCF configuration file not found" << endl << endl;
                exit(1);
            }
        }
        else
        {
            cout << "Please specify a file name after -u command" << endl << endl;
            exit(1);
        }
    }

    const std::string &timeoutSecondsParam = input.getCmdOption("-t");
    unsigned long timeoutSeconds = 0;

    if(input.cmdOptionExists("-t"))
    {
        if (!timeoutSecondsParam.empty())
        {
            try
            {
                timeoutSeconds = std::stoul(timeoutSecondsParam);
            }
            catch (const std::invalid_argument& ia)
            {
                cout << "The specified timeout argument is not a valid number: " << timeoutSecondsParam << endl;
                exit(1);
            }
        }
        else
        {
            cout << "Please specify a timeout after -t command" << endl << endl;
            exit(1);
        }
    }

    /* ---------------------------- Retrieve device information  ---------------------------- */

    char * version;
    hs_datalog_get_version(&version);

    cout << "USB HS-Datalog Command Line Interface example\n";
    cout << "Version: 2.5.0\n";
    cout << "Based on : ";
    cout << version;
    cout << "\n";

    if(hs_datalog_open() != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while initializing datalog\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    int nDevices = 0;
    if(hs_datalog_get_device_number(&nDevices) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while retrieving device number\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;

    }

    if(nDevices == 0)
    {
        cout << "No devices, exiting\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    /* If multiple devices are present, address only device with id = 0 */
    int deviceID = 0;
    int nSensors = 0;

    char * tempDev;

    if(hs_datalog_get_device_descriptor(deviceID, &tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while retrieving device descriptor\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    auto json = nlohmann::json::parse(tempDev);
    cout << "Device information: \n";
    cout << json.dump(1) << "\n";

    /* Free memory */
    if(hs_datalog_free(tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while freeing memory\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    if(hs_datalog_get_sensor_number(deviceID, &nSensors) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while retrieving sensor number\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    char acq_name[] = "testName";
    char acq_description[] = "descriptionTest";
    hs_datalog_set_acquisition_param(deviceID, acq_name, acq_description);

    /* -------------------- Load device configuration from JSON file (if requested) or use default configuration  -------------------- */

    std::vector<vector<bool>> SubsensorIsActive;

    /* Try and open JSON config file on the hard disk*/
    bool configFromFile = false;

    /* JSON config file is present */
    if (configFile.is_open())
    {
        configFromFile = true;
        cout << endl <<"Configuration imported from Json file " << endl << endl;

        configFile.seekg (0, configFile.end);
        long long size = configFile.tellg();
        configFile.seekg (0, configFile.beg);

        /* Read the whole file */
        char * jsonChar = new char [size+1];
        configFile.seekg (0, ios::beg);
        configFile.read (jsonChar, static_cast<int>(size));
        configFile.close();
        jsonChar[size]=0;  // JSON lib needs a '\0' at the end of the string

        /* Parse JSON and extract sensors */
        auto configJson = nlohmann::json::parse(jsonChar);
        auto jSensors = configJson.at("device").at("sensor");

        /* Create messages to set sensor status as described in the file and send them to the device*/
        for (size_t j = 0; j < jSensors.size(); j++ )
        {
            auto jSubSensorsStatus = jSensors.at(j).at("sensorStatus").at("subSensorStatus");
            nlohmann::json cmd;
            cmd["command"] = "SET";
            cmd["sensorId"] = jSensors.at(j).at("id");

            SubsensorIsActive.push_back(std::vector<bool>());
            for(size_t k = 0; k< jSubSensorsStatus.size(); k++ )
            {
                nlohmann::json params;
                auto subId = jSensors.at(j).at("sensorDescriptor").at("subSensorDescriptor").at(k).at("id");
                params["id"]= subId;

                auto it = jSubSensorsStatus.at(k).begin();
                auto end = jSubSensorsStatus.at(k).end();
                for(; it!=end; it++)
                {
                    params[it.key()]=it.value();
                }
                cmd["subSensorStatus"].push_back(params);
                SubsensorIsActive[j].push_back(jSensors.at(j).at("sensorStatus").at("subSensorStatus").at(k).at("isActive"));
            }
            if(hs_datalog_send_message(deviceID, const_cast <char *>(cmd.dump().data()), static_cast<int>(cmd.dump().length()), nullptr, nullptr) != ST_HS_DATALOG_OK)
            {
                cout << "Error occurred while sending message\n";
                cout << "Press any key to exit \n";
                getchar();
                return -1;
            }
        }
        /* Update tag list */
        auto tag_hw = configJson.at("device").at("tagConfig").at("hwTags");
        auto tag_sw = configJson.at("device").at("tagConfig").at("swTags");

        auto it= tag_hw.begin();
        while(it !=tag_hw.end() )
        {
            int id=it->at("id");
            string label=it->at("label");
            hs_datalog_set_hw_label(deviceID, id, &label[0]);

            bool status =it->at("enabled");
            hs_datalog_enable_hw_tag(deviceID, id, status);
            it++;
        }

        it= tag_sw.begin();
        while(it !=tag_sw.end() )
        {
            int id=it->at("id");
            string label=it->at("label");
            hs_datalog_set_sw_label(deviceID, id, &label[0]);
            it++;
        }


        delete[] jsonChar;
    }
    else
    {   /* if JSON config file is not specified, simply get all the sensors status with their default configuration */

        cout << endl <<"Using default configuration stored in the device" << endl << endl;
    }

    /* send UCF file to MLC if present */
    if (ucfFile.is_open())
    {
        ucfFile.seekg (0, ucfFile.end);
        long long size_ucf = ucfFile.tellg();
        ucfFile.seekg (0, ucfFile.beg);

        /* Read the whole file */
        char * ucfData = new char [size_ucf+1];
        ucfFile.seekg (0, ios::beg);
        ucfFile.read (ucfData, static_cast<int>(size_ucf));

        for(sID = 0; sID < static_cast<unsigned int>(nSensors); sID++)
        {
            int MaxSub;
            hs_datalog_get_sub_sensor_number(deviceID, static_cast<int>(sID), &MaxSub);
            for (ssID = 0; ssID < static_cast<unsigned int>(MaxSub) ; ssID++)
            {
                char *sub_name;
                hs_datalog_get_sub_sensor_name(0, sID, ssID, &sub_name);

                if( strcmp(sub_name, "MLC") == 0)
                {
                    mlc_sID = sID;
                    mlc_ssID = ssID;
                }
            }
        }

        /* number 5 states for ISM330DHCX sensor and 2 states for MLC sub sensor*/
        hs_datalog_send_UCF_to_MLC(deviceID, mlc_sID, (uint8_t *) ucfData, size_ucf);
    }

    bool taggingEnabled=true;

    if(!input.cmdOptionExists("-y"))
    {
        cout << "Press any key to start logging\n";
        getchar();
    }

    std::map<std::tuple<int,string>, bool> tags;
    if(taggingEnabled)
    {
        char *temp_t;
        hs_datalog_get_available_tags(0, &temp_t);

        using my_json = nlohmann::json;
        auto tag_list = my_json::parse(std::string(temp_t));
        auto tag_it = tag_list.at("swTags").begin();
        auto tag_end = tag_list.at("swTags").end();

        while(tag_it !=  tag_end)
        {
            auto ta = std::make_tuple ( tag_it->at("id"), tag_it->at("label"));
            tags.insert(std::pair<std::tuple<int,string>,bool>(ta,false));
            tag_it++;
        }
        if(hs_datalog_free(temp_t) != ST_HS_DATALOG_OK)
        {
         cout << "Error occurred while freeing memory\n";
         cout << "Press any key to exit \n";
         getchar();
         return -1;
        }
    }

    /* -------------------- Open raw data files and allocate buffers  -------------------- */
    std::vector<vector<string>> names;
    std::vector<vector<FILE *>> pFiles;
    std::vector<vector<int>> packetsReceived;

    /*Get current time as char buffer to create the directory */
    time_t rawtime;
    struct tm * timeinfo;
    char time_buff[256] = "./";
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(&time_buff[2],sizeof(time_buff) - 2,"%Y%m%d_%H_%M_%S",timeinfo);
#ifdef __linux__
    mkdir(time_buff, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#elif _WIN32
    mkdir(time_buff);
#endif

    /* save UCF file in Acquisition folder and enable MLC */
    if (ucfFile.is_open())
    {
        int index = fileUCFParam.find_last_of("/\\");
        std::string input_trace_filename = fileUCFParam.substr(index+1);

        std::string name=  string(time_buff) + "/";
        name.append(input_trace_filename);

        std::ofstream  dst(name,   std::ios::binary);
        ucfFile.seekg (0, ucfFile.beg);
        dst << ucfFile.rdbuf();
        ucfFile.close();

        hs_datalog_set_sub_sensor_active(deviceID, mlc_sID, mlc_ssID, true);
    }

    int MaxSub;    
    char * subsensor_status;

    for (sID = 0; sID < static_cast<unsigned int>(nSensors); sID++)
    {
        names.push_back(std::vector<string>());
        pFiles.push_back(std::vector<FILE*>());
        char * tmp_sName;
        if(hs_datalog_get_sensor_name(deviceID, static_cast<int>(sID), &tmp_sName) != ST_HS_DATALOG_OK)
        {
            cout << "Error occurred while retrieving sensor " << sID << " name"<< endl;
            cout << "Press any key to exit \n";
            getchar();
            return -1;
        }

        hs_datalog_get_sub_sensor_number(deviceID, static_cast<int>(sID), &MaxSub);
        SubsensorIsActive.push_back(std::vector<bool>());

        for (ssID = 0; ssID < static_cast<unsigned int>(MaxSub) ; ssID++)
        {
            char *subname;
            hs_datalog_get_sub_sensor_name(deviceID, static_cast<int>(sID), static_cast<int>(ssID), &subname);
            names[sID].push_back(string(tmp_sName)+ "_" +string(subname));

            /* Retrive the subsensor status from the device */
            if(hs_datalog_get_subsensor_status(deviceID, sID, ssID, &subsensor_status) != ST_HS_DATALOG_OK)
            {
                cout << "Error occurred while getting sub sensor status " << sID << endl;
                cout << "Press any key to exit \n";
                getchar();
                return -1;
            }
            auto json_subSensorStatus = nlohmann::json::parse(subsensor_status);
            /* Save the status */
            SubsensorIsActive[sID].push_back(json_subSensorStatus.at("isActive"));
            hs_datalog_free(subsensor_status);

            FILE * tmp_f = nullptr;
            if(SubsensorIsActive[sID][ssID])
            {
                std::string name=  string(time_buff) + "/";
                name.append(names[sID][ssID]);
                name.append(".dat");
                tmp_f = fopen(name.data(), "wb+");
            }
            pFiles[sID].push_back(tmp_f);
        }
    }


    /* -------------------- Start logging and wait for user command or timeout  -------------------- */

    /* Start logging */
    hs_datalog_start_log(deviceID);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>( start_time - start_time ).count();

    for(sID = 0; sID < static_cast<unsigned int>(nSensors); sID++)
    {
        int MaxSub;
        hs_datalog_get_sub_sensor_number(deviceID, static_cast<int>(sID), &MaxSub);
        packetsReceived.push_back(std::vector<int>());
        for (ssID = 0; ssID < static_cast<unsigned int>(MaxSub) ; ssID++)
        {
            packetsReceived[sID].push_back(0);

            char *sub_name;
            hs_datalog_get_sub_sensor_name(0, sID, ssID, &sub_name);

            if( strcmp(sub_name, "MLC") == 0)
            {
                mlc_sID = sID;
                mlc_ssID = ssID;
            }
        }

    }

    uint8_t MLC_val[8];
    double  MLC_time=0;
    bool exit = false;
    string dev_alias = json.at("deviceInfo").at("alias");
    while(!exit)
    {
#ifdef __linux__
        sleep(1);
#elif _WIN32
        Sleep(1000);
        system("CLS");
#endif
        cout << "+--------------HSDatalog CLI----------------+"  << endl;
        cout << "| Streaming from: ";
        cout << right << setw(25) << dev_alias << " |" << endl;

        auto current_time = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>( current_time - start_time ).count();
        cout << right << "| Elapsed: " << setw(5) << round(elapsed_time/1000.f) << "s";

        if(timeoutSeconds!=0)
        {
            auto remainingTime = timeoutSeconds-(elapsed_time/1000.f);
            stringstream tmp;
            tmp << "Remaining: " << right << setw(5) << round(((remainingTime>0)?remainingTime:0)) << "s |";
            cout << right << setw(28) << tmp.str();
            if(timeoutSeconds<elapsed_time/1000.f)
            {
                exit = true;
            }
        }
        else
        {
            cout << right << setw(28) << "|";

        }
        cout << endl;

        cout << "+--------------Received Data----------------+"  << endl;
        for(sID = 0; sID < static_cast<unsigned int>(nSensors); sID++)
        {
            int MaxSub;
            hs_datalog_get_sub_sensor_number(deviceID, static_cast<int>(sID), &MaxSub);
            for (ssID = 0; ssID < static_cast<unsigned int>(MaxSub) ; ssID++)
            {
                if(SubsensorIsActive[sID][ssID])
                {
                    int size = 0, actual = 0, MLC_counter;
                    char *sub_name;
                    hs_datalog_get_sub_sensor_name(0, sID, ssID, &sub_name);

                    hs_datalog_get_available_data_size(0, static_cast<int>(sID), static_cast<int>(ssID), &size);
                    cout.setf(ios :: left, ios :: adjustfield);
                    cout << "| " << setw(17) << names[sID][ssID];
                    cout.setf(ios :: right, ios :: adjustfield);
                    cout << setw(18) << packetsReceived[sID][ssID] << " Bytes |\n";

                    if(size > 0)
                    {
                        uint8_t * data = new uint8_t[size];
                        hs_datalog_get_data(0, static_cast<int>(sID), static_cast<int>(ssID), data, size, &actual);


                        if( strcmp(sub_name, "MLC") == 0)
                        {
                            for (MLC_counter = 0; MLC_counter < 8; MLC_counter++)
                            {
                                MLC_val[MLC_counter]= data[MLC_counter + size - 16];
                            }
                            MLC_time = *reinterpret_cast<double *>(&data[size-8]);
                        }

                        fwrite (data , sizeof(char), static_cast<size_t>(size), pFiles[sID][ssID]);
                        delete[] data;
                        packetsReceived[sID][ssID] += size;

                        /* Free memory */
                        if(hs_datalog_free(sub_name) != ST_HS_DATALOG_OK)
                        {
                            cout << "Error occurred while freeing memory\n";
                            cout << "Press any key to exit \n";
                            getchar();
                            return -1;
                        }
                    }

                }
            }

        }
        cout << "+-------------------------------------------+"  << endl;

        if ((mlc_sID != (unsigned int)(-1)) && (mlc_ssID != (unsigned int)(-1)))
        {

            if( SubsensorIsActive[mlc_sID][mlc_ssID] == true)
            {
                int MLC_counter;
                cout << "| MLC 1 Status: " << left << setw(5) << static_cast<int>(MLC_val[0]);
                stringstream tmp;
                tmp << " Timestamp: " << round(MLC_time) << "s |" << endl;
                cout << right << setw(25) << tmp.str();

                for (MLC_counter = 1; MLC_counter < 8; MLC_counter++)
                {
                    cout << "| MLC " << static_cast<int>(MLC_counter+1) << " Status: " << left << setw(5) << static_cast<int>(MLC_val[MLC_counter]) << right << setw(24) << " |"<< endl;
                }
            }
        }

        char key;
        int num_tag=-1;
        if(getInput(&key))
        {
            if(key == 0x1B || key == 'q')  /* Press Esc or q to exit */
            {
                exit = true;
            }
            else if (key >= '0' && key <= '9')
            {
                num_tag = key-'0';
            }
        }


        if(taggingEnabled)
        {
            auto result = std::find_if(
                      tags.begin(),
                      tags.end(),
                      [num_tag](const std::pair<std::tuple<int,string>, bool>& mo) {return std::get<0>(mo.first) == num_tag; });

            if(result != tags.end())
            {
                if(!result->second)
                {
                    hs_datalog_set_on_sw_tag(deviceID,std::get<0>(result->first));
                    result->second=true;
                }
                else
                {
                    hs_datalog_set_off_sw_tag(deviceID,std::get<0>(result->first));
                    result->second=false;
                }

            }

            auto it = tags.begin();
            auto end = tags.end();

            cout << "+----------------Tag labels-----------------+" << endl;
            while(it != end)
            {
                char a = it->second==0?' ':static_cast<char>(254);
                cout <<  "| -" << std::get<0>(it->first) << "- (";
                cout << a;
                cout << ") " << left << setw(34) << std::get<1>(it->first) << "|" << endl;
                it++;
            }
            cout << "+-------------------------------------------+" << endl;
            cout << "Press the corresponding number to activate/deactivate a tag. ";
        }
        cout << "ESC to exit!" << std::endl;
    }


    /* -------------------- Stop logging, close files and connection -------------------- */

    /* Stop logging */
    hs_datalog_stop_log(deviceID);

    for (sID = 0; sID < static_cast<unsigned int>(nSensors); sID++)
    {
        int MaxSub;
        hs_datalog_get_sub_sensor_number(deviceID, static_cast<int>(sID), &MaxSub);
        for (ssID = 0; ssID < static_cast<unsigned int>(MaxSub) ; ssID++)
        {
            if(SubsensorIsActive[sID][ssID])
            {
                 fclose(pFiles[sID][ssID]);
            }

        }
    }

    if(hs_datalog_get_device(deviceID, &tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while retrieving device \n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    auto json_final = nlohmann::json::parse(tempDev);
    string jsonS = json_final.dump(1,'\t'); // pretty format

    char * jsonFileName = new char[strlen(time_buff) + strlen("DeviceConfig.json") + 1];
    snprintf(jsonFileName, strlen(time_buff) + strlen("DeviceConfig.json") + 2, "%s/DeviceConfig.json", time_buff);

    FILE * jsonFile = fopen(jsonFileName, "wt");
    fwrite(jsonS.c_str(), sizeof(char), jsonS.size(), jsonFile);
    fclose(jsonFile);

    /* Free memory */
    if(hs_datalog_free(tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while freeing memory\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    if(hs_datalog_get_acquisition_info(deviceID, &tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while retrieving device \n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    auto json_acq = nlohmann::json::parse(tempDev);
    string jsonS_acq = json_acq.dump(1,'\t'); // pretty format

    char * jsonFileName_acq = new char[strlen(time_buff) + strlen("AcquisitionInfo.json") + 1];
    snprintf(jsonFileName_acq, strlen(time_buff) + strlen("AcquisitionInfo.json") + 2, "%s/AcquisitionInfo.json", time_buff);

    FILE * jsonFile_acq = fopen(jsonFileName_acq, "wt");
    fwrite(jsonS_acq.c_str(), sizeof(char), jsonS_acq.size(), jsonFile_acq);
    fclose(jsonFile_acq);

    /* Free memory */
    if(hs_datalog_free(tempDev) != ST_HS_DATALOG_OK)
    {
        cout << "Error occurred while freeing memory\n";
        cout << "Press any key to exit \n";
        getchar();
        return -1;
    }

    hs_datalog_close();

    return 0;
}

#ifdef __linux__
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif
