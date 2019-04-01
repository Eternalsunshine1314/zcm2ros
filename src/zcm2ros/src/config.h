#ifndef CONFIG_H
#define CONFIG_H


#include "vgio_keyfile_schema.h"

static constexpr auto keyfile_heap = "# --- Track fusing ---\n\n";


//=======================================================================================
class Config
{
public:

    static Config load( const std::string &fname );
    static std::string get ();


    struct ZcmReceive
    {
        std::string target;
        std::string jpeg_channel;
        std::string lidar_cluster_channel;
        std::string lidar_cluster_channel2;

        int max_slippage_us;   // максимальное проскальзывание между service метками.

        bool need_trace;
    } zcm_receive;

    //-----------------------------------------------------------------------------------

private:
    struct Schema
    {
        Schema( Config *conf )
        {
            auto& sh = schema;

            sh.set_current_group( "ZCM_Receive" );

            sh.append( "target",
                       &conf->zcm_receive.target, "udpm://239.255.76.67:7667?ttl=10" );

            sh.append( "jpeg_channel",
                       &conf->zcm_receive.jpeg_channel, "Homography" );
            sh.append( "lidar_cluster_channel",
                       &conf->zcm_receive.lidar_cluster_channel, "lidar_cluster_channel" );

            sh.append( "lidar_cluster_channel2",
                       &conf->zcm_receive.lidar_cluster_channel2, "lidar_cluster_channel2" );

            sh.append( "max_slippage_us",
                       &conf->zcm_receive.max_slippage_us, 100 );

            sh.append( "need_trace", &conf->zcm_receive.need_trace, false );

        }

        vgio::KeyFile_Schema schema;
    }; // Schema
};
//=======================================================================================


#endif // CONFIG_H
