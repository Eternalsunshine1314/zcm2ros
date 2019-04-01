#ifndef TRACKS_PACK_COLLECTOR_H
#define TRACKS_PACK_COLLECTOR_H


#include "zcm_deploy.h"
#include "vzcm_subscriber.h"
#include "config.h"

//=======================================================================================
class Tracks_Pack_Collector
{
public:
    VSignal<Tracks_Pack> received;

    Tracks_Pack_Collector( zcm::ZCM* zcm, Config::ZcmReceive conf );

private:
    void _check_emit();

    VZCM_Subscriber<ZCM_BaslerJpeg> _jpegFrame;
    VZCM_Subscriber<ZCM_LidarClustersList> _clustersList;

    std::chrono::microseconds _max_slippage;
    bool _need_trace;

    Tracks_Pack _pack;
};
//=======================================================================================


#endif // TRACKS_PACK_COLLECTOR_H
