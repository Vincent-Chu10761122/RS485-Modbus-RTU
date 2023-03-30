#include "ros/ros.h"
#include <signal.h>
#include <iostream>
#include <boost/thread.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <map>
#include <modbus/modbus.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <gpm_msgs/VerticalCommand.h>
#include "gpm_msgs/NavigationState.h"
#include "gpm_msgs/SimpleReq.h"
#include <std_msgs/UInt8.h>
#include "filelog.h"


//time //2022.10.06
#include <signal.h>//time
#include <stdlib.h>//time
#include <unistd.h>//time
//#include <time.h>
#include<sys/time.h>


/*--------------------*/
/*      Set Flag      */
/*--------------------*/
#define CONNECT_MODE    0       /* 0:RS485  , 1:TCP */
#define TEST_SKIP       0       /* 0:Normal , 1:Skip */


/*--------------------*/
/*      For RS485     */
/*--------------------*/
#define MODBUS_DEVICE               "/dev/ttyS1"
#define MODBUS_BAUDRATE             9600
#define MODBUS_PARITY               'N'
#define MODBUS_DATA_BIT             8
#define MODBUS_STOP_BIT             1
#define MODBUS_TIMEOUT_SEC          0             
#define MODBUS_TIMEOUT_USEC         200000
#define MODBUS_RES_TIMEOUT_SEC      0             
#define MODBUS_RES_TIMEOUT_USEC     200000
#define MODBUS_DEVICE_ID            1
#define MODBUS_DEBUG                OFF

#define MODBUS_RO_BITS              32
#define MODBUS_RW_BITS              32
#define MODBUS_RO_REGISTERS         64
#define MODBUS_RW_REGISTERS         64


/*--------------------*/
/*      For tcp       */
/*--------------------*/
#if CONNECT_MODE
#define MODBUS_SERVER_IP            "111.70.15.55"
#define MODBUS_SERVER_PORT          500
#define MODBUS_DEVICE_ID            1
#define MODBUS_TIMEOUT_SEC          10//3
#define MODBUS_TIMEOUT_USEC         0
#define MODBUS_DEBUG                OFF
#endif

using namespace std;
modbus_t *ctx;
modbus_mapping_t* data_mapping;
map<string, string>::iterator iter_r;
map <string, string> point_mapping;
ros::ServiceServer service_agv_modbus_cmd_action;
ros::ServiceClient client_agv_modbus_done_action;
ros::Publisher instrument_status_pub;                   /* 210428jimc */

string sConfigPath = "/home/gpm/catkin_ws/src/gpm_project/agv_modbus/config/agv_point_map.txt";
 

std_msgs::Int32 run_status;							    
/* 210428jimc */

bool autoinsp = false;       /* 210527jimc */
bool init_end;
bool instrument_error = false;
bool instrument_finish = false;
int currentID = -1;
int currentTagID=0;//=====2023.02.08
int goalID = -2;
int movebase_status = 0;

bool pre_instrument_error = false;
bool pre_instrument_finish = false;
int pre_currentID = 0;
bool Recieve_cmd = false;   //GU
int DoubleCheck = 0;
time_t WaitStart = 0;
time_t WaitEnd = 0;
int pre_goalID = 0;
int pre_movebase_status = 0;

int compareArray[MODBUS_RW_REGISTERS];
int iInstrumentType =0 ;//0 MiTAP 1:Particle count
GFileLog *m_log;
GFileLog *m_logModbusData;//2023.01.06
std::ostringstream logStr;

class modbus_init
{
    public:
        modbus_init()
        {

        }
        int Initial()
        {
            /*--------------------*/
            /*      For RS485     */
            /*--------------------*/
            ctx = modbus_new_rtu( MODBUS_DEVICE, 
                                  MODBUS_BAUDRATE, 
                                  MODBUS_PARITY, 
                                  MODBUS_DATA_BIT, 
                                  MODBUS_STOP_BIT);
            if (ctx == NULL) 
            {
                ROS_INFO("agv_Modbus : Unable to create the libmodbus context\n");
                return(-1);
            }

            /* set slave device ID */
            modbus_set_slave(ctx, MODBUS_DEVICE_ID);

            /* Debug mode */
            modbus_set_debug(ctx, MODBUS_DEBUG);

            data_mapping = modbus_mapping_new( MODBUS_RO_BITS, 
                                               MODBUS_RW_BITS,
                                               MODBUS_RO_REGISTERS,
                                               MODBUS_RW_REGISTERS);
 
            if (data_mapping == NULL) 
            {
                ROS_INFO("agv_Modbus : Failed to allocate the mapping: %s\n",modbus_strerror(errno));
                modbus_free(ctx);
                return(-1);
            }

            modbus_rtu_set_serial_mode(ctx,MODBUS_RTU_RS485);
            /* set timeout */
            timeout.tv_sec = MODBUS_TIMEOUT_SEC;
            timeout.tv_usec = MODBUS_TIMEOUT_USEC;
            modbus_set_byte_timeout(ctx, &timeout);
            
            timeout.tv_sec = MODBUS_RES_TIMEOUT_SEC;
            timeout.tv_usec = MODBUS_RES_TIMEOUT_USEC;
            modbus_set_response_timeout(ctx, &timeout);


            /* open serial interface */
            if (modbus_connect(ctx) == -1) 
            {
                ROS_INFO("agv_Modbus : Connection failed: %s\n",modbus_strerror(errno));
                modbus_free(ctx);
                return(-1);
            }
            data_mapping->tab_input_registers[19] = 1; //for Particle count //2023.01.06
            return(1);
            
            /*--------------------*/
            /*      For tcp       */
            /*--------------------*/
#if CONNECT_MODE            
            ctx = modbus_new_tcp(MODBUS_SERVER_IP, MODBUS_SERVER_PORT);
            /* set device ID */
            modbus_set_slave(ctx, MODBUS_DEVICE_ID);
            /* Debug mode */
            modbus_set_debug(ctx, MODBUS_DEBUG);
            /* set timeout */
            timeout.tv_sec = MODBUS_TIMEOUT_SEC;
            timeout.tv_usec = MODBUS_TIMEOUT_USEC;
            modbus_get_byte_timeout(ctx, &timeout);
            timeout.tv_sec = MODBUS_TIMEOUT_SEC;
            timeout.tv_usec = MODBUS_TIMEOUT_USEC;
            modbus_set_response_timeout(ctx, &timeout);
            if (modbus_connect(ctx) == -1) {
                fprintf(stderr, "Connection failed: %s\n",
                        modbus_strerror(errno));
                modbus_free(ctx);
                exit(-1);
            }
#endif
        }
        
    private:
        struct timeval timeout;
};

void Log(string sMsg  )
{
    m_log->Log(sMsg  );
}
 
void LogModbusData(string sMsg  )//2023.01.06
{
    m_logModbusData->Log(sMsg  );
}

void shutdownProcess(int sig)
{
    //ros::NodeHandle n;
    ROS_INFO("Quit agv_modbus_comm : %s",modbus_strerror(errno));
    logStr.str("");
    logStr << "Quit agv_modbus_comm : " <<  modbus_strerror(errno);
    Log(logStr.str());
    m_log->Active(false);
    delete m_log;
    m_log = NULL;

    m_logModbusData->Active(false);//2023.01.06
    delete m_logModbusData;//2023.01.06
    m_logModbusData = NULL;//2023.01.06

    ros::shutdown();
}

bool Agv_Modbus_address_write( gpm_msgs::SimpleReq::Request  &req,
                                gpm_msgs::SimpleReq::Response &res)
{
    data_mapping->tab_input_registers[11] = 0x41;
    data_mapping->tab_input_registers[12] = 0x42;
    data_mapping->tab_input_registers[13] = 0x43;
    data_mapping->tab_input_registers[14] = 0x44;
    data_mapping->tab_input_registers[15] = 0x30;
    data_mapping->tab_input_registers[16] = 0x31;
    data_mapping->tab_input_registers[18] = 1;
    res.response = 1;
    return true;
}

bool Agv_Modbus_Cmd_Action( gpm_msgs::VerticalCommand::Request  &req,
                             gpm_msgs::VerticalCommand::Response &res)
{
    int k;
    bool ret = true;
 
     //=====2023.02.7 mark
    //double diff = 0;
    //Recieve_cmd = true;   //GU
    //WaitStart = time(NULL);
    //while(DoubleCheck<10 && diff<5)
    //{
    //  WaitEnd = time(NULL);
    //  diff = difftime(WaitEnd,WaitStart);  
    //}
    //ROS_INFO("agv_Modbus: DoubleCheck: %d, diff= %f",DoubleCheck,diff);

    currentTagID = (int)req.target; //=====2023.02.7
    string get_command = req.command;

    if( get_command == "stop" ) /* This command not use in miniAGV */
    {
        res.confirm = true;
        return true;
    }

   printf("agv_Modbus : Received command %f , AA:%d\n" ,req.target,currentTagID);
   ROS_INFO( "agv_Modbus : Received command : %s  TagID :%s", get_command.c_str() ,to_string(currentTagID)); //=====2023.02.7
    logStr.str("");
    logStr << "agv_Modbus : Received command : " << get_command << " TagID: "<< currentTagID ;   //=====2023.02.7
    Log(logStr.str());

    if( get_command == "orig" )
    {
        if( instrument_error )
        {
            ret = false;
        }
    }
    else if( get_command == "pose" )
    {
        if( !instrument_error )
        {
            iter_r = point_mapping.find( to_string(currentTagID) );//=====2023.02.7

            if(iter_r != point_mapping.end())
            {
                ROS_INFO( "agv_Modbus : Find, the point mapping" );
                logStr.str("");
                logStr << "agv_Modbus : Find, the point mapping";
                Log(logStr.str());

                for( int i = 0 ; i < 6 ;i++)
                {
                    k = (int)iter_r->second[i];
                    data_mapping->tab_input_registers[11+i] = k;
                    ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[%d] : %d",(11+i) , k );
                    logStr.str("");
                    logStr << "agv_Modbus_data_mapping->tab_input_registers[" << (11+i) << "] :" << k;
                    Log(logStr.str());
                }
                ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[18] : 1" );
                logStr.str("");
                logStr << "agv_Modbus_data_mapping->tab_input_registers[18] : 1";
                Log(logStr.str());

                data_mapping->tab_input_registers[18] = 1;    //In position

                //2022.10.06
                //
               struct timeval tv;
               struct timezone tz;
               struct tm *tt;
               gettimeofday(&tv, &tz);

    	       tt = localtime(&tv.tv_sec);

               data_mapping->tab_input_registers[48] =  1900 + tt->tm_year;

               data_mapping->tab_input_registers[49] = 1+ tt->tm_mon; 

               data_mapping->tab_input_registers[50] =  tt->tm_mday; 

               data_mapping->tab_input_registers[51] = tt->tm_hour; 

               data_mapping->tab_input_registers[52] =  tt->tm_min; 

               data_mapping->tab_input_registers[53] =  tt->tm_sec;

               ROS_INFO("y:%d m:%d d:%d h:%d m:%d s:%d",data_mapping->tab_input_registers[48],data_mapping->tab_input_registers[49],data_mapping->tab_input_registers[50],data_mapping->tab_input_registers[51],data_mapping->tab_input_registers[52],data_mapping->tab_input_registers[53]);

                logStr.str("DateTime");
                for( int i = 0 ; i < 6 ;i++)
                {
                    logStr << " " << fIntToStr(data_mapping->tab_input_registers[48+i]);
                }
                Log(logStr.str());


                run_status.data = 1;                    /* 210428jimc */
                autoinsp = true;                        /* 210527jimc */
            }
            else
            {
                ROS_INFO( "agv_Modbus : The point mapping not find" );
                logStr.str("");
                logStr << "agv_Modbus : The point mapping not find" ;
                Log(logStr.str());
                ret = false;
                run_status.data = 0;                    /* 210428jimc */
            }
        }
        else
        {
            ROS_INFO( "agv_Modbus : Instrument error do not inspection" );
            logStr.str("");
            logStr << "agv_Modbus : Instrument error do not inspection" ;
            Log(logStr.str());
            ret = false;
            run_status.data = 0;                        /* 210428jimc */
        }
        instrument_status_pub.publish(run_status);	    /* 210428jimc */
    }
#if TEST_SKIP
    else if(get_command == "ff")
    {
        instrument_finish = !instrument_finish;
    }
    else if(get_command == "ee")
    {
        instrument_error = !instrument_error;
    }
#endif

    res.confirm = ret;
    DoubleCheck = 0;
    Recieve_cmd = false;    //GU
    return true;
}

bool compare_array_diff(void)
{
    int i,j,k;
    string sID="";//2023.01.06
    std::ostringstream logString;//2023.01.06 
    logString.str("");//2023.01.06 
    logString<< ","<<currentID <<",";//2022.10.06
    for( i = 0 ; i < 60 ; i++ )//2022.10.06
    {
        if( data_mapping->tab_registers[i] != compareArray[i] )
        {
            logStr.str("");
            
            for( j = 0 ; j < 60 ; j++ )//2022.10.06
            {
                compareArray[j] = data_mapping->tab_registers[j];
                if(j>=11 && j<=16)//2023.01.06
                {
                   logString << char(compareArray[j]);
 
                }
               
                logStr << compareArray[j] << "," ;
            }
            Log(logStr.str());
            logString << "," <<logStr.str()  ;//2023.01.06
            //LogModbusData(logString.str());//2023.01.06
            break;
        }
    }
    
}
bool ModbusData_save(void)//=====2023.02.03
{
    int i,j,k;
    string sID="";//2023.01.06
    std::ostringstream logString;//2023.01.06 
    logString.str("");//2023.01.06 
    logString<< ","<<currentTagID <<",";//2022.10.06 //=====2023.02.08

            logStr.str("");
            
            for( j = 0 ; j < 60 ; j++ )//2022.10.06
            {
                compareArray[j] = data_mapping->tab_registers[j];
                if(j>=11 && j<=16)//2023.01.06
                {
                   logString << char(compareArray[j]);
 
                }
               
                logStr << compareArray[j] << "," ;
            }
            logString << "," <<logStr.str()  ;//2023.01.06
            LogModbusData(logString.str());//2023.01.06
            

    
}
void modbus_instrument_check_timer(const ros::TimerEvent&)
{
    gpm_msgs::VerticalCommand srv;

#if !TEST_SKIP
    if( data_mapping->tab_registers[17] == 2 )
    {
        instrument_finish = true;
    }
    else
    {
        instrument_finish = false;
    }

    if(    ( data_mapping->tab_registers[17] == 3 )
        || ( data_mapping->tab_registers[17] == 4 )     /* 210525jimc */
        || ( data_mapping->tab_registers[17] == 5 )     /* 210907jimc */
      )
    {
        instrument_error = true;
    }
    else
    {
        instrument_error = false;
    }
#endif

    if( instrument_finish != pre_instrument_finish )
    {
        pre_instrument_finish = instrument_finish;
        

        if(instrument_finish)
        {
            ROS_INFO( "agv_Modbus : Instrument finish" );
            logStr.str("");
            logStr << "agv_Modbus : Instrument finish";
            Log(logStr.str());
            ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[18] : 0" );
            logStr.str("");
            logStr << "agv_Modbus_data_mapping->tab_input_registers[18] : 0";
            Log(logStr.str());
            ModbusData_save();//=====2023.01.03

            data_mapping->tab_registers[17] = 0;
            data_mapping->tab_input_registers[18] = 0;    //Not in position

            srv.request.command = "done";
            if( client_agv_modbus_done_action.call(srv) )
            {
                ROS_INFO("agv_Modbus Done_action confirm : %d", (int)srv.response.confirm);
                logStr.str("");
                logStr << "agv_Modbus Done_action confirm :" << (int)srv.response.confirm;
                Log(logStr.str());
            }
            else
            {
                ROS_ERROR("agv_Modbus Done_action confirm : Failed to call service Agv_Modbus_Done_Action");
                logStr.str("");
                logStr << "agv_Modbus Done_action confirm : Failed to call service Agv_Modbus_Done_Action";
                Log(logStr.str());
            }
/* 210527jimc add */
#if TEST_SKIP
            instrument_finish = false;
            autoinsp = false;
#endif
/* 210527jimc add end */
            run_status.data = 0;                            /* 210428jimc */
            instrument_status_pub.publish(run_status);	    /* 210428jimc */
        }
    }

    if( instrument_error != pre_instrument_error )
    {
        pre_instrument_error = instrument_error;
        ROS_INFO( "agv_Modbus : Instrument error" );
        logStr.str("");
        logStr << "agv_Modbus : Instrument error";
        Log(logStr.str());
        ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[18] : 0" );
        logStr.str("");
        logStr << "agv_Modbus_data_mapping->tab_input_registers[18] : 0";
        Log(logStr.str());
        data_mapping->tab_input_registers[18] = 0;    //Not in position

        if(instrument_error)
        {
            srv.request.command = "error";
            /* 210528jimc add */
            if( data_mapping->tab_registers[17] == 3 )
            {
                srv.request.target = 3;
            }
            else if( data_mapping->tab_registers[17] == 4 )
            {
                srv.request.target = 4;
            }
            /* 210528jimc add end */
            if( client_agv_modbus_done_action.call(srv) )
            {
                ROS_INFO("agv_Modbus Done_action confirm : %d", (int)srv.response.confirm);
                logStr.str("");
                logStr << "agv_Modbus Done_action confirm :" << (int)srv.response.confirm;
                Log(logStr.str());
            }
            else
            {
                ROS_ERROR("agv_Modbus Done_action confirm : Failed to call service Agv_Modbus_Done_Action");
                logStr.str("");
                logStr << "agv_Modbus Done_action confirm : Failed to call service Agv_Modbus_Done_Action";
                Log(logStr.str());
            }
            run_status.data = 0;                            /* 210428jimc */
            instrument_status_pub.publish(run_status);	    /* 210428jimc */
        }
    }


}

void read_point_mapping()
{
    string key;
    string value;
    
    const char *fileName = sConfigPath.c_str();
    ifstream paramFile;
    paramFile.open(fileName);
    while ( paramFile >> key >> value )
    {
      point_mapping[key] = value;
    }
    paramFile.close();
    /*
    for(iter_r = point_mapping.begin(); iter_r != point_mapping.end(); iter_r++)
    {
        cout<<iter_r->first<<" "<<iter_r->second<<endl;
    }
    */
}
string read_HeaderString()//2023.01.06
{
    string sHeaderString="NoData";
    if ( iInstrumentType==0)
    {
        sHeaderString ="DataTime,Tag,StationID,4G-CSQ,AirQ-L,AirQ-H,AIO-L,AI1-D,AI2-T,AI3-H,AI4,AI5,AI6,AI7,ID1,ID2,ID3,ID4,ID5,ID6,MiTAP-State,AGV-State,AGV-S19,MiTAP-Count,MiTAP-Tvoc,MiTAP-Acetone,MiTAP-IPA,MiTAP-S24,MiTAP-S25,MiTAP-S26,MiTAP-S27,MiTAP-S28,MiTAP-S29,MiTAP-S30,MiTAP-S31,MiTAP-S32,MiTAP-S33,MiTAP-S34,MiTAP-S35,MiTAP-S36,MiTAP-S37,MiTAP-S38,MiTAP-S39,MiTAP-S40,MiTAP-Status,MiTAP-Trigger,MiTAP-s43,AGV-Comm,MiTAP-Comm,S46,S47,AGV-year,AGV-Month,AGV-Date,AGV-Hour,AGV-Min,AGV-Sec,4G-year,4G-Month,4G-Date,4G-Hour,4G-Min,4G-Sec";
        

    }
    else if ( iInstrumentType==1)
    {
        sHeaderString ="DataTime,Tag,StationID,4G-CSQ,AirQ-L,AirQ-H,AIO-L,AI1-D,AI2-T,AI3-H,AI4,AI5,AI6,AI7,ID1,ID2,ID3,ID4,ID5,ID6,Pat-State,AGV-State,AGV-E/N,Pat-Size1,Pat-Size1,Pat-Size2,Pat-Size2,Pat-Size3,Pat-Size3,Pat-Size4,Pat-Size4,Pat-Size5,Pat-Size5,Pat-Size6,Pat-Size6,Pat-S32,Pat-S33,Pat-S34,Pat-S35,Pat-S36,Pat-S37,Pat-Mode,Pat-Alarm,Pat-Alarm,Pat-Status,Pat-Sample,Pat-Step,AGV-Comm,MiTAP-Comm,S46,S47,AGV-year,AGV-Month,AGV-Date,AGV-Hour,AGV-Min,AGV-Sec,4G-year,4G-Month,4G-Date,4G-Hour,4G-Min,4G-Sec";
  

    }
 
    return sHeaderString;
}
void agv_Modbus_inspection_write_Callback( const actionlib_msgs::GoalStatusArray &msg )
{
    int k;
    if( msg.status_list.size() > 0 )
    {
        movebase_status = msg.status_list[ ( msg.status_list.size() - 1) ].status;
    }
    
    if(movebase_status != pre_movebase_status)
    {
        pre_movebase_status = movebase_status;

        ROS_INFO( "agv_Modbus_address_write status : %d" , movebase_status );
        logStr.str("");
        logStr << "agv_Modbus_address_write status : " <<  movebase_status;
        Log(logStr.str());

        if( movebase_status == 1 )                                      //1 # The goal is currently being processed by the action server
        {
            ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[18] : 0" );
            logStr.str("");
            logStr << "agv_Modbus_data_mapping->tab_input_registers[18] : 0";
            Log(logStr.str());
            data_mapping->tab_input_registers[18] = 0;    //Not in position
        }
        else if( ( movebase_status == 3 ) && ( goalID == currentID ) )  //3 # The goal was achieved successfully by the action server (Terminal State)
        {
            iter_r = point_mapping.find( to_string(goalID) );

            if(iter_r != point_mapping.end())
            {
                ROS_INFO( "agv_Modbus : Find, the point mapping" );
                logStr.str("");
                logStr << "agv_Modbus : Find, the point mapping";
                Log(logStr.str());

                for( int i = 0 ; i < 6 ;i++)
                {
                    k = (int)iter_r->second[i];
                    data_mapping->tab_input_registers[11+i] = k;
                    ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[%d] : %d",(11+i) , k );
                    logStr.str("");
                    logStr << "agv_Modbus_data_mapping->tab_input_registers[" << (11+i) << "] :" << k;
                    Log(logStr.str());
                }
                ROS_INFO( "agv_Modbus_data_mapping->tab_input_registers[18] : 1" );
                logStr.str("");
                logStr << "agv_Modbus_data_mapping->tab_input_registers[18] : 1";
                Log(logStr.str());

                data_mapping->tab_input_registers[18] = 1;    //In position
            }
            else
            {
                ROS_INFO( "agv_Modbus : The point mapping not find" );
                logStr.str("");
                logStr << "agv_Modbus : The point mapping not find";
                Log(logStr.str());
            }
        }
    }
}

void modbus_goalID_sub_CB(const std_msgs::UInt8 &msg)
{
    goalID = msg.data;
    if(pre_goalID != goalID)
    {
        ROS_INFO( "agv_Modbus_modbus_goalID : %d" , goalID );
        logStr.str("");
        logStr << "agv_Modbus_modbus_goalID : " <<  goalID;
        Log(logStr.str());
        pre_goalID = goalID;
    }
}

void nav_state_sub_CB(const gpm_msgs::NavigationState &msg)
{
    currentID = msg.lastVisitedNode.data;
    if(Recieve_cmd == true)   //GU
    {
       DoubleCheck++;
    }
    else
    {
       DoubleCheck = 0;
    }

    if( pre_currentID != currentID )
    {
        ROS_INFO( "agv_Modbus_modbus_currentID : %d" , currentID );
        logStr.str("");
        logStr << "agv_Modbus_modbus_currentID : " <<  currentID;
        Log(logStr.str());
        pre_currentID = currentID;
    }
}
 /* 211020 wei add */
/*void modbus_compensation_value()
{
    int ival;
    int count = 0;
    char key_str[10];

    for (int i =46;i<55;i++)
    {
        
	if(i !=54)
	    {
		ival = 0;
		sprintf(key_str,"AI%d",count);
		ival = ReadIniKeyString("/catkin_ws/src/commlib/AI.ini","AI",key_str);
		ROS_INFO("ReadIniKeyString:%d\n",ival);
		data_mapping -> tab_input_registers[i] =ival;
	    }
	else
	    {
		ival = 99;
		data_mapping -> tab_input_registers[i] =ival;
	    }
		count++;
    }
}
*/
/* 211020 wei add end */

void modbus_query_server()
{
    uint8_t query[MODBUS_RTU_MAX_ADU_LENGTH];
    int ret;
    ros::Rate r1(1000);
    
    while( ros::ok() && (init_end == true) )
    {
        ret = modbus_receive(ctx, query);
	//ROS_INFO("RET = %d" , ret);
        if (ret > 0) 
        {
            if(query[0] == 1 )
            {
                modbus_reply(ctx, query, ret, data_mapping);
                compare_array_diff();
            }
        }
        r1.sleep();
    }
}

/* 210527jimc add */
void modbus_instrument_auto_check_timer(const ros::TimerEvent&)
{
    if(autoinsp)
    {
        instrument_finish = true;
    }
}
/* 210527jimc add end */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_modbus_comm", ros::init_options::NoSigintHandler);       /* 210429jimc */  
    signal(SIGINT,shutdownProcess);
    ros::NodeHandle n;

    init_end = false;
    
    m_log = new GFileLog("agv_modbus_comm",true); 
    m_logModbusData = new GFileLog("agv_modbus_Data",false); //2023.01.06
    m_logModbusData->TimeStyle(GFileLog::_enTSyyyymmddhhmmsss_);//2023.01.06

    //read 
    m_logModbusData->HeaderString(read_HeaderString());//2023.01.06
    m_logModbusData->SubFilename(".csv");
    m_logModbusData->Active(true);
    
    /*------------------*/
    /*  Service server  */
    /*------------------*/
    service_agv_modbus_cmd_action = n.advertiseService("command_action", Agv_Modbus_Cmd_Action);
    //ros::ServiceServer service_w = n.advertiseService("Agv_Modbus_IO_write", Agv_Modbus_address_write);   //simulation test

    /*------------------*/
    /*  Service client  */
    /*------------------*/
    client_agv_modbus_done_action = n.serviceClient<gpm_msgs::VerticalCommand>("done_action");

    /*-------------*/
    /*  Publisher  */
    /*-------------*/
    instrument_status_pub = n.advertise<std_msgs::Int32>("/instrument_status",1);                           /* 210428jimc */

    /*-------------*/
    /*  Subscribe  */
    /*-------------*/
    ros::Subscriber navigation_state_sub = n.subscribe("navigation_state",1,nav_state_sub_CB);
    //ros::Subscriber sub = n.subscribe("/move_base/status", 1, &agv_Modbus_inspection_write_Callback);    //simulation test
    //ros::Subscriber modbus_goalID_sub = n.subscribe("modbus_goalID",1,modbus_goalID_sub_CB);              //simulation test

    /*-----------*/
    /*   Timer   */
    /*-----------*/
    ros::Timer insptimer = n.createTimer(ros::Duration(0.5), &modbus_instrument_check_timer, false);

/* 210527jimc add */
#if TEST_SKIP
    ros::Timer autoinsptimer = n.createTimer(ros::Duration(5), &modbus_instrument_auto_check_timer, false);
#endif
/* 210527jimc add end */

    read_point_mapping();
    modbus_init mbus = modbus_init();

    if (mbus.Initial() >= 0)
    {
	//modbus_compensation_value();   /* 211020 wei */ 
        ROS_INFO("agv_modbus_comm receiving...");
        logStr.str("");
        logStr << "agv_modbus_comm receiving...";
        Log(logStr.str());
        init_end = true;
        modbus_flush(ctx);
        boost::thread server(modbus_query_server);
    }

    ros::spin();
    return 0;
}

